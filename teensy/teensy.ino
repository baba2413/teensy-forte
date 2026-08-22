#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <QNEthernet.h>

using namespace qindesign::network;

// -------------------------------------------------------------
// GOAL-FOLLOWING FIRMWARE -- single (slave) arm only, Ethernet UDP control.
//
// No master arm, no bilateral teleop, no haptic feedback. This build exists purely to let a
// host stream goal joint positions to the arm (e.g. from a trained policy during evaluation).
// For teleoperated data collection, flash the teleop-bi-p-t branch instead -- each branch in
// this repo is a standalone snapshot for one job, not layers meant to be combined.
//
// Hybrid transport, deliberately -- referenced from teensy-forte's isaacsim-udp branch, which
// already solved the "one channel, two different jobs" problem we hit when 'g' lived on serial:
//   USB serial : 'c' (calibrate zero) / 'd' (disable) -- single-char, human-supervised, no
//                payload, no line parsing, no state machine to get stuck in. Typed by a person at
//                minicom/screen directly -- the host's Python side never opens this port at all
//                any more (see lerobot_robot_forte_arm's teensy_link.TeensyGoalLink), same
//                reasoning as this repo's teleop-bi-c branch.
//   Ethernet UDP: two independent one-way streams, both fire-and-forget datagrams (no
//                "only one process can hold the port" constraint like serial has, no
//                terminator-byte ambiguity -- each packet is a complete message):
//                  host -> Teensy (UDP_PORT):           the continuous goal-position stream.
//                  Teensy -> host (TELEMETRY_UDP_PORT):  periodic status line, mirroring what
//                                                         teleop-bi-c already does with
//                                                         sendTelemetryLine() -- lets the host
//                                                         read positions without ever touching
//                                                         serial, so that port stays free for a
//                                                         human at minicom/screen the whole
//                                                         session.
//
// Network: static IP, direct cable to the host (see IPAddress config below -- matches
// isaacsim-udp and its proven host-side scripts, isaacsim_script/arm-ik/{test_udp,
// multiple_ik_udp}.py). UDP payload is plain CSV, no leading type marker (unlike
// isaacsim-udp's "P,..." -- this port only ever carries one kind of packet, so there's nothing
// to disambiguate):
//   "<yaw>,<pitch>,<roll>,<elbow>"   raw motor radians, kinematic order (motor ids 11,13,12,14
//                                    -- NOT CAN-wiring order, which would be 11,12,13,14)
//
// Every packet both sets the 4 joint targets and enters/refreshes GOAL mode (see
// enterGoalModeIfNeeded()). Absolute targets, not baseline+delta like isaacsim-udp -- that
// scheme exists there to reconcile IsaacSim's simulation coordinate frame with the real motor's
// arbitrary power-on encoder value, a problem we don't have (our dataset and this arm are
// already in the same raw-motor-radian units, see lerobot_robot_forte_arm's SMOLVLA_GUIDE.md
// §2). No gear ratio anywhere either, for the same reason.
// -------------------------------------------------------------

// -------------------------------------------------------------
// 1. Motor and communication configuration parameters
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2;
const uint8_t SLV_IDS_CAN1[NUM_MOTORS_CAN1] = {11, 12}; // shoulder_yaw, shoulder_roll

const uint8_t NUM_MOTORS_CAN2 = 2;
const uint8_t SLV_IDS_CAN2[NUM_MOTORS_CAN2] = {13, 14}; // shoulder_pitch, elbow_pitch

const uint8_t HOST_ID = 253;

// Position tracking gains (starting from the same values as teleop-bi-p-t's SLV_KP/KD)
const float SLV_KP = 10.0f;
const float SLV_KD = 0.1f;

// GOAL mode configuration. Same 500ms as isaacsim-udp's WATCHDOG_TIMEOUT_MS -- UDP has
// different packet-loss characteristics than a wired serial byte stream, so this leaves more
// margin than the previous serial version's 150ms.
const uint32_t GOAL_TIMEOUT_MS = 500;

// For this long (ms) right after 'd', ignore all incoming UDP goal packets entirely. UDP (the
// goal stream) and serial ('d') are completely separate channels with no synchronization between
// them, so a goal packet the host already sent just before 'd' can arrive *after* 'd' is
// processed -- handleGoalPacket() unconditionally calls enterGoalModeIfNeeded() for any valid
// packet, so that one stray packet would invalidate the 'd' that was just sent (even without
// calibration) and re-enable the motors. This was a problem reproduced in real testing -- since
// 'd' needs to act as an instant kill switch, a brief ignore window blocks this race.
const uint32_t DISABLE_IGNORE_MS = 300;
uint32_t ignore_goal_packets_until_ms = 0;

// Network configuration (same as the isaacsim-udp branch -- reuses settings already verified
// on this hardware). Assumes a direct cable connection, so there's no gateway (0.0.0.0).
IPAddress staticIP(192, 168, 1, 15);
IPAddress subnetMask(255, 255, 255, 0);
const uint16_t UDP_PORT = 5005;
EthernetUDP udp;

// Telemetry: host -> Teensy is UDP_PORT above; this is the Teensy -> host direction, same pattern
// as teleop-bi-c's sendTelemetryLine(). Host address matches that branch's telemetryHostIP.
IPAddress telemetryHostIP(192, 168, 1, 10);
const uint16_t TELEMETRY_UDP_PORT = 5006;
EthernetUDP telemetryUdp;

void sendTelemetryLine(const char* line, size_t len) {
  if (len == 0) return;
  telemetryUdp.beginPacket(telemetryHostIP, TELEMETRY_UDP_PORT);
  telemetryUdp.write((const uint8_t*)line, len);
  telemetryUdp.endPacket();
}

// 'c' software zero + per-joint travel limits (ported from the "modify" commit on the
// teensy-forte teleop-bi branch, reworked in terms of raw motor radians) -- this firmware applies
// no gear ratio at all (see §2 of lerobot_robot_forte_arm's SMOLVLA_GUIDE.md), so the original's
// joint-space/gear-ratio division was not carried over. The current raw position captured by 'c'
// becomes that motor's zero, and subsequent targets are clamped to the range
// [zero + MIN, zero + MAX].
// J1 (Yaw)   : [-2.2086 rad, +2.2086 rad] (reduced by 0.2 from the previous 2.4086)
// J2 (Roll)  : [-2.2800 rad, +2.2800 rad] (reduced by 0.2 from the previous 2.4800)
// J3 (Pitch) : [-2.3000 rad, +0.1000 rad] (negative-direction motion, limit reduced by 0.2 with +0.1 margin on the initial point)
// J4 (Pitch) : [-0.1000 rad, +0.9500 rad] (positive-direction motion, limit reduced by 0.2 with -0.1 margin on the initial point)
const float JOINT_LIMIT_MIN_CAN1[NUM_MOTORS_CAN1] = {-2.2086f, -2.2800f};
const float JOINT_LIMIT_MAX_CAN1[NUM_MOTORS_CAN1] = { 2.2086f,  2.2800f};

const float JOINT_LIMIT_MIN_CAN2[NUM_MOTORS_CAN2] = {-2.3000f, -0.1000f};
const float JOINT_LIMIT_MAX_CAN2[NUM_MOTORS_CAN2] = { 0.1000f,  0.9500f};

// Uses Teensy 4.0/4.1 CAN1, CAN2 (motors 11,12 wired to CAN1 / 13,14 wired to CAN2)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

// -------------------------------------------------------------
// 2. Robstride protocol physical limit values
// -------------------------------------------------------------
const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;

// Unidirectional motor absolute hardware/software limit (-12.4 ~ +12.4 rad)
const float RAW_LIMIT_MIN = -12.4f;
const float RAW_LIMIT_MAX = 12.4f;

// -------------------------------------------------------------
// 3. Control period configuration (300Hz / dt = 3.333ms)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 3333;
elapsedMicros controlTimer;

const uint32_t LOG_PERIOD = 1000;

// -------------------------------------------------------------
// 4. Real-time state and control flags
// -------------------------------------------------------------
bool goal_mode_enabled = false; // Entered via UDP packet, cleared with 'd'
bool is_calibrated = false;     // Entered via 'c' -- whether the zero (zero_offset_*) is currently valid

volatile float slave_pos_can1[NUM_MOTORS_CAN1] = {};
volatile float slave_trq_can1[NUM_MOTORS_CAN1] = {};
volatile bool slave_valid_can1[NUM_MOTORS_CAN1] = {}; // Whether CAN feedback has been received at least once
volatile float slave_pos_can2[NUM_MOTORS_CAN2] = {};
volatile float slave_trq_can2[NUM_MOTORS_CAN2] = {};
volatile bool slave_valid_can2[NUM_MOTORS_CAN2] = {};

// Zero captured via 'c' (raw motor radians). JOINT_LIMIT_MIN/MAX_CAN1/CAN2 are ranges relative to this value.
float zero_offset_can1[NUM_MOTORS_CAN1] = {};
float zero_offset_can2[NUM_MOTORS_CAN2] = {};

// Target raw radians to track in GOAL mode. Indices follow the same order as the
// SLV_IDS_CAN1/CAN2 arrays (CAN wiring order: goal_target_can1[0]=11, [1]=12,
// goal_target_can2[0]=13, [1]=14).
// Note: this is CAN wiring order, not the UDP packet's payload order (yaw,pitch,roll,elbow) --
// handleGoalPacket() maps between the two.
float goal_target_can1[NUM_MOTORS_CAN1] = {};
float goal_target_can2[NUM_MOTORS_CAN2] = {};
uint32_t last_goal_rx_time = 0; // Timestamp of the last valid UDP goal packet received (for the GOAL_TIMEOUT_MS watchdog)

// Stores fault and mode state from Type 2 feedback (indexed directly by motor_id)
volatile uint8_t fault_bits_can1[256] = {0};
volatile uint8_t fault_bits_can2[256] = {0};
volatile uint8_t motor_mode_can1[256] = {0};
volatile uint8_t motor_mode_can2[256] = {0};
volatile bool fault_changed_can1[256] = {false};
volatile bool fault_changed_can2[256] = {false};

// -------------------------------------------------------------
// 5. Data scaling and diagnostic helper functions
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

void printFaultBits(const char* can_name, uint8_t motor_id, uint8_t fault_bits) {
  Serial.printf("[%s FAULT] Motor %d | Raw: 0x%02X", can_name, motor_id, fault_bits);

  if (fault_bits == 0) {
    Serial.println(" | CLEARED");
    return;
  }

  if (fault_bits & (1 << 0)) Serial.print(" | UNDERVOLTAGE");
  if (fault_bits & (1 << 1)) Serial.print(" | THREE_PHASE_OVERCURRENT");
  if (fault_bits & (1 << 2)) Serial.print(" | OVERTEMPERATURE");
  if (fault_bits & (1 << 3)) Serial.print(" | MAGNETIC_ENCODER_FAULT");
  if (fault_bits & (1 << 4)) Serial.print(" | STALL_OVERLOAD");
  if (fault_bits & (1 << 5)) Serial.print(" | UNCALIBRATED");

  Serial.println();
}

// -------------------------------------------------------------
// 6. Robstride CAN1 transmit control functions
// -------------------------------------------------------------
void enableMotorCan1(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  memset(mode_msg.buf, 0, 8);
  mode_msg.buf[0] = 0x05;
  mode_msg.buf[1] = 0x70;

  Can1.write(mode_msg);
  delay(20);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  memset(enable_msg.buf, 0, 8);

  Can1.write(enable_msg);
  Serial.printf("[Teensy CAN1] Motor %d Enabled successfully!\r\n", motor_id);
}

void disableMotorCan1(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  memset(msg.buf, 0, 8);

  Can1.write(msg);
  Serial.printf("[Teensy CAN1] Motor %d Disabled.\r\n", motor_id);
}

CAN_message_t operationControlCan1(uint8_t motor_id, float feed_forward, float pos,
                                   float vel, float kp, float kd) {
  CAN_message_t msg;

  // Drop packets whose raw radian target falls outside the -12.4 ~ 12.4 rad limit
  if (pos < RAW_LIMIT_MIN || pos > RAW_LIMIT_MAX) {
    static uint32_t lastWarnMs[256] = {0};
    if (millis() - lastWarnMs[motor_id] > 200) {
      lastWarnMs[motor_id] = millis();
      Serial.printf("[CAN1 WARN] Motor %d target %.3f rad OUT OF RANGE (%.1f~%.1f) - command DROPPED\r\n",
                    motor_id, pos, RAW_LIMIT_MIN, RAW_LIMIT_MAX);
    }
    return msg;
  }

  uint16_t p_int  = floatToUint(pos, P_MIN, P_MAX, 16);
  uint16_t v_int  = floatToUint(vel, V_MIN, V_MAX, 16);
  uint16_t kp_int = floatToUint(kp, 0.0f, KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd, 0.0f, KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX, 16);

  msg.flags.extended = 1;
  msg.id = (1UL << 24) | ((uint32_t)t_int << 8) | motor_id;
  msg.len = 8;

  msg.buf[0] = (p_int >> 8) & 0xFF;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;
  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF;
  msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF;
  msg.buf[7] = kd_int & 0xFF;

  Can1.write(msg);
  return msg;
}

// -------------------------------------------------------------
// 7. Robstride CAN2 transmit control functions
// -------------------------------------------------------------
void enableMotorCan2(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  memset(mode_msg.buf, 0, 8);
  mode_msg.buf[0] = 0x05;
  mode_msg.buf[1] = 0x70;

  Can2.write(mode_msg);
  delay(20);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  memset(enable_msg.buf, 0, 8);

  Can2.write(enable_msg);
  Serial.printf("[Teensy CAN2] Motor %d Enabled successfully!\r\n", motor_id);
}

void disableMotorCan2(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  memset(msg.buf, 0, 8);

  Can2.write(msg);
  Serial.printf("[Teensy CAN2] Motor %d Disabled.\r\n", motor_id);
}

CAN_message_t operationControlCan2(uint8_t motor_id, float feed_forward, float pos,
                                   float vel, float kp, float kd) {
  CAN_message_t msg;

  // Drop packets whose raw radian target falls outside the -12.4 ~ 12.4 rad limit
  if (pos < RAW_LIMIT_MIN || pos > RAW_LIMIT_MAX) {
    static uint32_t lastWarnMs[256] = {0};
    if (millis() - lastWarnMs[motor_id] > 200) {
      lastWarnMs[motor_id] = millis();
      Serial.printf("[CAN2 WARN] Motor %d target %.3f rad OUT OF RANGE (%.1f~%.1f) - command DROPPED\r\n",
                    motor_id, pos, RAW_LIMIT_MIN, RAW_LIMIT_MAX);
    }
    return msg;
  }

  uint16_t p_int  = floatToUint(pos, P_MIN, P_MAX, 16);
  uint16_t v_int  = floatToUint(vel, V_MIN, V_MAX, 16);
  uint16_t kp_int = floatToUint(kp, 0.0f, KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd, 0.0f, KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX, 16);

  msg.flags.extended = 1;
  msg.id = (1UL << 24) | ((uint32_t)t_int << 8) | motor_id;
  msg.len = 8;

  msg.buf[0] = (p_int >> 8) & 0xFF;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;
  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF;
  msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF;
  msg.buf[7] = kd_int & 0xFF;

  Can2.write(msg);
  return msg;
}

// -------------------------------------------------------------
// 8. CAN receive interrupt callback
// -------------------------------------------------------------
void rxCallbackCan1(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;

  if (mode == 2) {
    uint8_t motor_id = (msg.id >> 8) & 0xFF;
    uint8_t new_fault_bits = (msg.id >> 16) & 0x3F;
    uint8_t new_motor_mode = (msg.id >> 22) & 0x03;

    uint16_t p_raw = ((uint16_t)msg.buf[0] << 8) | msg.buf[1];
    uint16_t t_raw = ((uint16_t)msg.buf[4] << 8) | msg.buf[5];
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);
    float current_trq = uintToFloat(t_raw, T_MIN, T_MAX);

    if (new_fault_bits != fault_bits_can1[motor_id]) {
      fault_bits_can1[motor_id] = new_fault_bits;
      fault_changed_can1[motor_id] = true;
    }
    motor_mode_can1[motor_id] = new_motor_mode;

    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (motor_id == SLV_IDS_CAN1[i]) {
        slave_pos_can1[i] = current_pos;
        slave_trq_can1[i] = current_trq;
        slave_valid_can1[i] = true;
        break;
      }
    }
  }
}

void rxCallbackCan2(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;

  if (mode == 2) {
    uint8_t motor_id = (msg.id >> 8) & 0xFF;
    uint8_t new_fault_bits = (msg.id >> 16) & 0x3F;
    uint8_t new_motor_mode = (msg.id >> 22) & 0x03;

    uint16_t p_raw = ((uint16_t)msg.buf[0] << 8) | msg.buf[1];
    uint16_t t_raw = ((uint16_t)msg.buf[4] << 8) | msg.buf[5];
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);
    float current_trq = uintToFloat(t_raw, T_MIN, T_MAX);

    if (new_fault_bits != fault_bits_can2[motor_id]) {
      fault_bits_can2[motor_id] = new_fault_bits;
      fault_changed_can2[motor_id] = true;
    }
    motor_mode_can2[motor_id] = new_motor_mode;

    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (motor_id == SLV_IDS_CAN2[i]) {
        slave_pos_can2[i] = current_pos;
        slave_trq_can2[i] = current_trq;
        slave_valid_can2[i] = true;
        break;
      }
    }
  }
}

// -------------------------------------------------------------
// 8b. 'c' software zero calibration
// -------------------------------------------------------------
void calibrateZero() {
  Serial.println("[Teensy] Calibrating zero offset from current pose...");

  bool all_ok = true;
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    if (!slave_valid_can1[i]) {
      Serial.printf("[CALIB] Motor %d: no CAN feedback yet -- FAILED\r\n", SLV_IDS_CAN1[i]);
      all_ok = false;
      continue;
    }
    zero_offset_can1[i] = slave_pos_can1[i];
    // format: radian_after_calibration (original_radian) -- always 0.000 right here (the new
    // zero IS the current pose by definition), but kept consistent with the status-print format
    // below so the same convention holds everywhere a calibration-relative reading is logged.
    Serial.printf("[CALIB] Motor %d zero set: %.3f (%.3f) rad\r\n",
                  SLV_IDS_CAN1[i], slave_pos_can1[i] - zero_offset_can1[i], slave_pos_can1[i]);
  }
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    if (!slave_valid_can2[i]) {
      Serial.printf("[CALIB] Motor %d: no CAN feedback yet -- FAILED\r\n", SLV_IDS_CAN2[i]);
      all_ok = false;
      continue;
    }
    zero_offset_can2[i] = slave_pos_can2[i];
    Serial.printf("[CALIB] Motor %d zero set: %.3f (%.3f) rad\r\n",
                  SLV_IDS_CAN2[i], slave_pos_can2[i] - zero_offset_can2[i], slave_pos_can2[i]);
  }

  is_calibrated = all_ok;
  if (all_ok) {
    Serial.println("[Teensy] Calibration complete. Per-joint limits now active relative to this pose.");
  } else {
    Serial.println("[Teensy] Calibration INCOMPLETE -- missing feedback for some motor(s). NOT calibrated.");
  }
}

// -------------------------------------------------------------
// 9. GOAL mode (UDP only): goal position packet parsing and mode entry handling
// -------------------------------------------------------------
void enterGoalModeIfNeeded() {
  if (goal_mode_enabled) return;

  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    enableMotorCan1(SLV_IDS_CAN1[i]);
    delay(20);
  }
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    enableMotorCan2(SLV_IDS_CAN2[i]);
    delay(20);
  }

  goal_mode_enabled = true;
  Serial.println("[Teensy] GOAL mode ENABLED via UDP. Send \"<yaw>,<pitch>,<roll>,<elbow>\" "
                  "(raw rad) to move. Send 'd' (serial) to stop.");
  if (!is_calibrated) {
    Serial.println("[Teensy] WARNING: entering GOAL mode without calibration ('c') -- only the "
                    "wide +-12.4 rad protocol limit is active, no real per-joint limit. Send 'd' "
                    "then 'c' first if you want the per-joint clamp enforced.");
  }
}

// buf is a NUL-terminated string. Each UDP packet is expected to be exactly in the form
// "<v0>,<v1>,<v2>,<v3>" -- there's no concept of the serial version's "bare command with just a
// newline, no value": the UDP side is always a Python script, and sending the complete 4 values
// is natural (if the caller wants to hold the current position, it just sends that same value
// back).
void handleGoalPacket(char* buf) {
  if (millis() < ignore_goal_packets_until_ms) {
    // Ignore window right after 'd' -- see DISABLE_IGNORE_MS. Prevents a stray packet already
    // sent before 'd' from arriving late and invalidating the disable that was just issued.
    Serial.println("[Teensy] GOAL packet ignored (just disabled, dropping stragglers briefly).");
    return;
  }

  float values[4];
  int i = 0;
  char* token = strtok(buf, ",");
  while (token != NULL && i < 4) {
    values[i++] = atof(token);
    token = strtok(NULL, ",");
  }

  bool all_finite = true;
  for (int j = 0; j < i; j++) {
    if (!isfinite(values[j])) all_finite = false;
  }

  if (i != 4 || !all_finite) {
    Serial.printf("[Teensy] GOAL packet REJECTED (need exactly 4 finite comma-separated floats, "
                  "got %d)\r\n", i);
    return; // UDP is packet-based, so unlike a serial line there's no risk of getting stuck in a
             // "partially received" state -- there's no need to refresh the watchdog even for a
             // bad packet, since it will be refreshed once the next valid packet arrives.
  }

  // Payload order is kinematic (yaw, pitch, roll, elbow), not CAN-wiring order --
  // motors 12 (roll) and 13 (pitch) are swapped relative to their CAN bus grouping.
  goal_target_can1[0] = values[0]; // 11 shoulder_yaw
  goal_target_can2[0] = values[1]; // 13 shoulder_pitch
  goal_target_can1[1] = values[2]; // 12 shoulder_roll
  goal_target_can2[1] = values[3]; // 14 elbow_pitch

  enterGoalModeIfNeeded();
  last_goal_rx_time = millis();
}

// -------------------------------------------------------------
// 10. Main loop structure
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== GOAL-following firmware (single arm, Ethernet UDP control) ===");

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(64);
  Can1.setMBFilter(ACCEPT_ALL);
  Can1.distribute();
  Can1.enableMBInterrupts();
  Can1.onReceive(rxCallbackCan1);

  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.setMaxMB(64);
  Can2.setMBFilter(ACCEPT_ALL);
  Can2.distribute();
  Can2.enableMBInterrupts();
  Can2.onReceive(rxCallbackCan2);

  Serial.println("CAN1/CAN2 initialized.");

  Ethernet.begin(staticIP, subnetMask, IPAddress(0, 0, 0, 0)); // Direct cable, no gateway
  udp.begin(UDP_PORT);
  telemetryUdp.begin(TELEMETRY_UDP_PORT);
  Serial.printf("Ethernet up: %d.%d.%d.%d, goal UDP port %d, telemetry UDP port %d\r\n",
                staticIP[0], staticIP[1], staticIP[2], staticIP[3], UDP_PORT, TELEMETRY_UDP_PORT);

  Serial.println("-> Serial: 'c' to calibrate zero (recommended first), 'd' to disable.");
  Serial.println("-> UDP: send \"<yaw>,<pitch>,<roll>,<elbow>\" (raw rad) to move.");
  controlTimer = 0;
}

void loop() {
  Can1.events();
  Can2.events();

  // Handle UDP receive -- the network stack already reassembles each packet as a whole, so
  // unlike the serial version, no byte-by-byte accumulating state machine is needed.
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[64];
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      handleGoalPacket(packetBuffer);
    }
  }

  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    uint8_t motor_id = SLV_IDS_CAN1[i];
    if (fault_changed_can1[motor_id]) {
      noInterrupts();
      uint8_t fault = fault_bits_can1[motor_id];
      fault_changed_can1[motor_id] = false;
      interrupts();

      printFaultBits("CAN1", motor_id, fault);
    }
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    uint8_t motor_id = SLV_IDS_CAN2[i];
    if (fault_changed_can2[motor_id]) {
      noInterrupts();
      uint8_t fault = fault_bits_can2[motor_id];
      fault_changed_can2[motor_id] = false;
      interrupts();

      printFaultBits("CAN2", motor_id, fault);
    }
  }

  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= LOG_PERIOD) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 3.333ms period control loop (300Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // GOAL mode watchdog: if no new UDP goal packet arrives within the grace period, automatically exit the mode (stop).
    if (goal_mode_enabled && (millis() - last_goal_rx_time > GOAL_TIMEOUT_MS)) {
      goal_mode_enabled = false;
      Serial.println("[Teensy] GOAL mode WATCHDOG TIMEOUT (no UDP packet received) - motors "
                      "stopped. Send a new goal packet to resume.");
    }

    // --- CAN1 control ---
    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (goal_mode_enabled) {
        float target = goal_target_can1[i];
        if (is_calibrated) {
          float lo = zero_offset_can1[i] + JOINT_LIMIT_MIN_CAN1[i];
          float hi = zero_offset_can1[i] + JOINT_LIMIT_MAX_CAN1[i];
          if (target < lo || target > hi) {
            static uint32_t lastJointLimitWarnMs[NUM_MOTORS_CAN1] = {0};
            if (millis() - lastJointLimitWarnMs[i] > 200) {
              lastJointLimitWarnMs[i] = millis();
              Serial.printf("[CAN1 JOINT LIMIT] Motor %d target %.3f rad clamped to [%.3f, %.3f]\r\n",
                            SLV_IDS_CAN1[i], target, lo, hi);
            }
            target = target < lo ? lo : hi;
          }
        }
        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, target, 0.0f, SLV_KP, SLV_KD);
      } else {
        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    // --- CAN2 control ---
    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (goal_mode_enabled) {
        float target = goal_target_can2[i];
        if (is_calibrated) {
          float lo = zero_offset_can2[i] + JOINT_LIMIT_MIN_CAN2[i];
          float hi = zero_offset_can2[i] + JOINT_LIMIT_MAX_CAN2[i];
          if (target < lo || target > hi) {
            static uint32_t lastJointLimitWarnMs[NUM_MOTORS_CAN2] = {0};
            if (millis() - lastJointLimitWarnMs[i] > 200) {
              lastJointLimitWarnMs[i] = millis();
              Serial.printf("[CAN2 JOINT LIMIT] Motor %d target %.3f rad clamped to [%.3f, %.3f]\r\n",
                            SLV_IDS_CAN2[i], target, lo, hi);
            }
            target = target < lo ? lo : hi;
          }
        }
        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, target, 0.0f, SLV_KP, SLV_KD);
      } else {
        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    // Print status monitoring every 1000ms
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= LOG_PERIOD) {
      lastPrint = millis();

      const char* mode_str = goal_mode_enabled ? "GOAL" : "DISABLED";
      const char* calib_str = is_calibrated ? "CALIB_OK" : "NO_CALIB";

      // format: radian_after_calibration (original_radian) -- lets you read a position directly
      // against JOINT_LIMIT_MIN/MAX_CAN1/CAN2 (defined relative to zero_offset_*) without doing
      // the subtraction in your head. When NOT calibrated, zero_offset_* is still 0.0 (never
      // written), so both numbers come out equal -- that's expected, not a bug.
      // Built into a buffer once per line so the exact same text goes out both Serial (human at
      // minicom/screen) and telemetry UDP (host's TeensyGoalLink) -- same pattern as teleop-bi-c.
      char line_buf[160];
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        int len = snprintf(line_buf, sizeof(line_buf),
                      "[CAN1] Slave %d Pos: %.3f (%.3f) rad | Target: %.3f (%.3f) rad (%s / %s) | Trq: %.2f Nm\r\n",
                      SLV_IDS_CAN1[i],
                      slave_pos_can1[i] - zero_offset_can1[i], slave_pos_can1[i],
                      goal_target_can1[i] - zero_offset_can1[i], goal_target_can1[i],
                      mode_str, calib_str, slave_trq_can1[i]);
        Serial.print(line_buf);
        sendTelemetryLine(line_buf, (size_t)len);
      }
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        int len = snprintf(line_buf, sizeof(line_buf),
                      "[CAN2] Slave %d Pos: %.3f (%.3f) rad | Target: %.3f (%.3f) rad (%s / %s) | Trq: %.2f Nm\r\n",
                      SLV_IDS_CAN2[i],
                      slave_pos_can2[i] - zero_offset_can2[i], slave_pos_can2[i],
                      goal_target_can2[i] - zero_offset_can2[i], goal_target_can2[i],
                      mode_str, calib_str, slave_trq_can2[i]);
        Serial.print(line_buf);
        sendTelemetryLine(line_buf, (size_t)len);
      }

      if (goal_mode_enabled) {
        Serial.printf("[GOAL] last UDP packet %lums ago (timeout %lums)\r\n",
                      (unsigned long)(millis() - last_goal_rx_time), (unsigned long)GOAL_TIMEOUT_MS);
      }

      Serial.println();
    }
  }
}

// -------------------------------------------------------------
// 11. Serial command receive interrupt -- only 'c'/'d', both single-character immediate
// commands, so no line buffering or state machine is needed at all (unlike when 'g' existed
// before being moved to UDP).
// -------------------------------------------------------------
void serialEvent() {
  while (Serial.available()) {
    char ch = Serial.read();

    if (ch == 'c' || ch == 'C') {
      calibrateZero();
    }
    else if (ch == 'd' || ch == 'D') {
      goal_mode_enabled = false;
      is_calibrated = false; // full stop invalidates the zero reference -- re-'c' before trusting it again
      ignore_goal_packets_until_ms = millis() + DISABLE_IGNORE_MS; // drop stragglers, see DISABLE_IGNORE_MS

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        disableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        disableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }

      Serial.println("[Teensy] Motors Disabled. Send 'c' to re-calibrate before sending goal packets again.");
    }
    // Any other character is not a command, so it's ignored.
  }
}
