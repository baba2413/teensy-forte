#include <Arduino.h>
#include <QNEthernet.h>
#include <FlexCAN_T4.h>
#include <math.h>

using namespace qindesign::network;

struct Vec3 { float x, y, z; };
struct Mat3 { float m[3][3]; };


// -------------------------------------------------------------
// 1. Joint configuration (4 motors currently implemented: yaw / pitch / roll / elbow)
// -------------------------------------------------------------
const uint8_t NUM_JOINTS = 4;
const uint8_t HOST_ID = 253; // Teensy host ID

// index order = Isaac Sim joint order (shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch)
// CAN IDs follow the actual hardware wiring. Unlike the original design, shoulder_roll=2 and shoulder_pitch=3 are wired this way.
const uint8_t JOINT_CAN_IDS[NUM_JOINTS] = {11, 13, 12, 14};

// -------------------------------------------------------------
// 2. Motor:link rotation ratio (an external reduction stage separate from the motor's own internal gear ratio)
//    motor_angle = link_angle * GEAR_RATIO
//    Matched using the same index order as JOINT_CAN_IDS.
// -------------------------------------------------------------
const float GEAR_RATIO[NUM_JOINTS] = {4.8077f, 3.180f, 1.0f, 1.0f};

// -------------------------------------------------------------
// 3. Joint rotation direction (relative to the link coordinate frame). If a joint's actual
//    rotation direction is reversed, it must be changed to -1.0f directly here (not something
//    computed automatically).
//
//    The absolute position offset (the old calib[].offset / computeCalibOffsets()) is no longer
//    used. Instead, a baseline is snapshotted at the moment of 'e' (enable), and from then on
//    only the "amount of change (delta)" of the UDP target relative to that baseline is added to
//    the current motor position to track it (see captureBaseline()). Regardless of whatever
//    absolute position the sim was in at the moment of enable, the delta of the first command is
//    always 0, so there is no room for a sudden jump.
// -------------------------------------------------------------
const float JOINT_SIGN[NUM_JOINTS] = {
  1.0f, // Joint 0 (shoulder_yaw,   CAN 11)
  1.0f, // Joint 1 (shoulder_pitch, CAN 13)
  1.0f, // Joint 2 (shoulder_roll,  CAN 12)
  1.0f, // Joint 3 (elbow_pitch,    CAN 14)
};

const char* JOINT_NAMES[NUM_JOINTS] = {"shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_pitch"};

// The value of multiple_ik_udp.py's MyCustomSceneCfg.robot.init_state.joint_pos (link coordinate frame, rad)
// Currently only used as the center value for the UDP input safety clamp (TEST_LINK_RANGE_LIMIT).
const float SIM_DEFAULT_LINK_POS[NUM_JOINTS] = {0.0f, -1.0472f, 0.0f, 2.0944f};

// -------------------------------------------------------------
// 3b. Safety range limit for initial real-hardware operation checks (link coordinate frame, +- around SIM_DEFAULT_LINK_POS)
//     Currently set as narrow as possible since this is only the "confirm it moves" stage.
//     Whatever value comes in over UDP, anything outside this range gets clamped.
// -------------------------------------------------------------
const float TEST_LINK_RANGE_LIMIT = 0.5f; // rad (about 8.6deg)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// -------------------------------------------------------------
// 4. Robstride protocol physical limit values (for MIT mode packing)
// -------------------------------------------------------------
const float P_MIN = -12.5f;   const float P_MAX = 12.5f;
const float V_MIN = -45.0f;   const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;  const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;   const float T_MAX = 18.0f;

// A safety margin slightly narrower than the protocol limits (P_MIN/P_MAX). As with
// teensy-forte teleop-bi, commands outside this range are not clamped but dropped entirely (never sent to the motor).
const float RAW_LIMIT_MIN = -12.4f;
const float RAW_LIMIT_MAX = 12.4f;

// -------------------------------------------------------------
// 5. Network and UDP control period configuration
// -------------------------------------------------------------
const uint16_t UDP_PORT = 5005;
EthernetUDP udp;

IPAddress staticIP(192, 168, 1, 15); // Teensy static IP (must match TEENSY_IP on the Isaac side)
IPAddress subnet(255, 255, 255, 0);

const uint32_t CONTROL_PERIOD_US = 20000; // 50Hz (20ms)
elapsedMicros controlTimer;

// -------------------------------------------------------------
// 6. External input buffer (received from Isaac Sim, link coordinate frame rad)
// -------------------------------------------------------------
float ext_target_pos[NUM_JOINTS] = {0.0f};
bool ext_control_active = false;
uint32_t last_packet_time = 0;
const uint32_t WATCHDOG_TIMEOUT_MS = 500; // Safety stop if no packet received for 0.5s

// -------------------------------------------------------------
// 6a. Baseline at the moment of enable (the core of the delta-tracking approach)
//     Snapshots the measured motor position and the UDP target value at the instant 'e' is
//     pressed. From then on, every tick computes only "how much the UDP target has changed
//     relative to the baseline (delta)" and adds that to the baseline motor position. Regardless
//     of whatever absolute position the sim was in at the moment of enable, the delta of the
//     first command is always 0, so there is never a jump at the moment of enable.
// -------------------------------------------------------------
float baseline_motor_pos[NUM_JOINTS] = {0.0f};   // Snapshot of fb[i].pos at the moment of 'e'
float baseline_link_target[NUM_JOINTS] = {0.0f}; // Snapshot of ext_target_pos at the moment of 'e'
bool baseline_ready = false;

// -------------------------------------------------------------
// 6b. Motor feedback buffer (always kept up to date, continuously refreshed by zero-resistance probes even while idle)
// -------------------------------------------------------------
struct MotorFeedback {
  float pos;
  float vel;
  float torque;
  bool updated;
  uint8_t fault_bits;   // Bits 16-21 (6 bits) of the Type2 feedback ID, same layout as teleop-bi a483a4c
  uint8_t run_mode;     // Bits 22-23 (2 bits) of the Type2 feedback ID: 0 reset, 1 cali, 2 motor run
  bool fault_changed;   // Flag so loop() logs exactly once, only when the value differs from the previous one
};
MotorFeedback fb[NUM_JOINTS];

// Counters/raw values for CAN receive diagnostics. Added to directly confirm, via logging,
// whether rxCallback() is even being called at all, whether mode==2 frames are arriving, and
// whether motor_id matching is failing (added to narrow down, by measurement rather than
// guesswork, why getpos.txt works but this code doesn't).
volatile uint32_t can_rx_total = 0;      // Total number of times rxCallback was entered (regardless of mode)
volatile uint32_t can_rx_mode2 = 0;      // Total number of mode==2 (feedback) frames
volatile uint32_t can_rx_unmatched = 0;  // Count of mode==2 frames whose motor_id didn't match JOINT_CAN_IDS
volatile uint32_t last_raw_can_id = 0;
volatile uint8_t  last_raw_can_len = 0;
volatile uint32_t last_raw_can_ms = 0;

// Meaning of the fault bits carried in the Type2 feedback CAN ID (per teensy-forte teleop-bi a483a4c, verified)
void printFaultBits(int idx, uint8_t fault_bits) {
  Serial.printf("[FAULT] Joint %d %-14s (CAN %2d) | Raw: 0x%02X",
                idx, JOINT_NAMES[idx], JOINT_CAN_IDS[idx], fault_bits);

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
// 6c. Status logging (teensy-forte teleop-bi style)
//     Every LOG_PERIOD_MS, prints the command values (link/motor axis) and real-time feedback (raw).
// -------------------------------------------------------------
const uint32_t NORMAL_LOG_PERIOD_MS = 1000;
// When enable_debug=true, enabling via 'e' lowers the period to log this densely (same period
// as the control loop, i.e. every tick). Since logs accumulate quickly, pause with 'p' to inspect a specific moment.
const uint32_t DEBUG_LOG_PERIOD_MS = 700;

// Manual debug mode switch. Change true/false directly in the code and re-upload.
// If true, enabling via 'e' lowers LOG_PERIOD_MS to DEBUG_LOG_PERIOD_MS, so logs print much more frequently.
bool enable_debug = true;

uint32_t LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS;
uint32_t lastLogMs = 0;
bool serialLogPaused = false; // Toggled by the 'p' command (pause to avoid flooding the serial output)

// Toggled by the 'g' command. When on, prints a one-line CSV (PLOT,...) that plot_positions.py
// can parse on every control loop tick (50Hz). Separate from logStatus()'s human-readable summary.
bool plotStreamEnabled = false;

float last_cmd_link_pos[NUM_JOINTS] = {0.0f};
float last_cmd_motor_pos[NUM_JOINTS] = {0.0f};
float last_cmd_ff_torque[NUM_JOINTS] = {0.0f};
float last_cmd_delta_link[NUM_JOINTS] = {0.0f};

// Counts of TEST_LINK_RANGE_LIMIT clamps / RAW_LIMIT drops. Logging on every packet would flood
// the serial output due to Isaac Sim's large motion range, so these are only aggregated and printed in the LOG_PERIOD_MS summary.
uint32_t range_clamp_count[NUM_JOINTS] = {0};
uint32_t raw_drop_count[NUM_JOINTS] = {0};

// The last raw UDP packet received (kept separately for logging since strtok() destroys the original buffer)
char last_udp_raw[64] = "";
uint32_t last_udp_recv_ms = 0;

// The raw UDP requested value "before" the safety clamp (TEST_LINK_RANGE_LIMIT) is applied. Logged
// side by side with the post-clamp value (ext_target_pos) to distinguish "what UDP actually requested" vs "what was used after clamping".
float last_udp_target_raw[NUM_JOINTS] = {0.0f};

// The last CAN command frame actually sent to the motor (filled in by operationControl())
uint32_t last_can_id[NUM_JOINTS] = {0};
uint8_t last_can_buf[NUM_JOINTS][8] = {{0}};
bool last_can_sent[NUM_JOINTS] = {false};

// -------------------------------------------------------------
// 7. Data quantization and motor control command functions
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

int getJointIndex(uint8_t motor_id) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (JOINT_CAN_IDS[i] == motor_id) return i;
  }
  return -1;
}

// CAN feedback (Type 2) receive callback: parses the motor's current position/velocity/torque
void rxCallback(const CAN_message_t &msg) {
  can_rx_total++;
  last_raw_can_id = msg.id;
  last_raw_can_len = msg.len;
  last_raw_can_ms = millis();

  uint8_t mode = (msg.id >> 24) & 0x1F;
  if (mode != 2) return;
  can_rx_mode2++;

  uint8_t motor_id = (msg.id >> 8) & 0xFF;
  int idx = getJointIndex(motor_id);
  if (idx < 0) {
    // Fallback verified via getpos.txt: some feedback frames carry the motor ID in the low
    // byte (bit0-7) instead of bit8-15. If it's not found here either, matching has genuinely failed.
    motor_id = msg.id & 0xFF;
    idx = getJointIndex(motor_id);
  }
  if (idx < 0) { can_rx_unmatched++; return; }

  uint8_t new_fault_bits = (msg.id >> 16) & 0x3F;
  uint8_t new_run_mode = (msg.id >> 22) & 0x03;
  if (new_fault_bits != fb[idx].fault_bits) {
    fb[idx].fault_bits = new_fault_bits;
    fb[idx].fault_changed = true; // Actual printing happens in loop() (to avoid spamming Serial.printf inside the callback)
  }
  fb[idx].run_mode = new_run_mode;

  uint16_t p_raw = (msg.buf[0] << 8) | msg.buf[1];
  uint16_t v_raw = (msg.buf[2] << 8) | msg.buf[3];
  uint16_t t_raw = (msg.buf[4] << 8) | msg.buf[5];

  fb[idx].pos = uintToFloat(p_raw, P_MIN, P_MAX);
  fb[idx].vel = uintToFloat(v_raw, V_MIN, V_MAX);
  fb[idx].torque = uintToFloat(t_raw, T_MIN, T_MAX);
  fb[idx].updated = true;
}

void enableMotor(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12 << 24) | (HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  mode_msg.buf[0] = 0x05; mode_msg.buf[1] = 0x70; // Run Mode address (0x7005)
  mode_msg.buf[4] = 0x00; // MIT operation control mode (0)
  Can0.write(mode_msg);
  delay(50);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3 << 24) | (HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  for (int i = 0; i < 8; i++) enable_msg.buf[i] = 0;
  Can0.write(enable_msg);
  Serial.printf("[Teensy] Motor ID %d Enabled.\r\n", motor_id);
}

void disableMotor(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4 << 24) | (HOST_ID << 8) | motor_id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;
  Can0.write(msg);
  Serial.printf("[Teensy] Motor ID %d Disabled.\r\n", motor_id);
}

// Called when the motor has entered a fault state (overcurrent/overtemperature/stall/etc.) and
// stops accepting position commands. The theory that the Type4 frame has a data byte for
// fault-clear (buf[0]=1) was an unverified guess with no precedent of use anywhere in this repo's
// branch history (the firmware likely ignores it). Instead, this reuses as-is the restart
// sequence that 'e' already performs every time and that has actually been verified (stop ->
// re-write Run Mode -> enable): re-evaluating/clearing the fault on Run Mode re-entry is standard
// controller behavior, and since no position command is carried, the arm won't suddenly move.
void clearFault(uint8_t motor_id) {
  disableMotor(motor_id);
  delay(20);
  enableMotor(motor_id);
}

void operationControl(uint8_t motor_id, float feed_forward, float pos, float vel, float kp, float kd) {
  int idx = getJointIndex(motor_id);

  if (pos < RAW_LIMIT_MIN || pos > RAW_LIMIT_MAX) {
    // Logging on every packet would flood the serial output, so just count and print in the LOG_PERIOD_MS summary
    if (idx >= 0) {
      raw_drop_count[idx]++;
      last_can_sent[idx] = false; // Reflect in the logging that this one wasn't actually sent
    }
    return; // Drop the packet and don't send over CAN
  }

  uint16_t p_int  = floatToUint(pos,          P_MIN, P_MAX,  16);
  uint16_t v_int  = floatToUint(vel,          V_MIN, V_MAX,  16);
  uint16_t kp_int = floatToUint(kp,           0.0f,  KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd,           0.0f,  KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX,  16);

  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (1 << 24) | (t_int << 8) | motor_id;
  msg.len = 8;
  msg.buf[0] = (p_int >> 8) & 0xFF;  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF; msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF; msg.buf[7] = kd_int & 0xFF;
  Can0.write(msg);

  if (idx >= 0) {
    last_can_id[idx] = msg.id;
    memcpy(last_can_buf[idx], msg.buf, 8);
    last_can_sent[idx] = true;
  }
}

// Called while processing 'e' (enable): snapshots the measured motor position and the UDP
// target value at that instant as the baseline. fb[]/ext_target_pos are always kept up to date
// even while idle (the zero-resistance probe / UDP parsing keeps running), so there's no need to
// wait separately. Subsequent control only adds the "amount of change (delta)" of the UDP target
// relative to the baseline to the current motor position, so regardless of whatever absolute pose
// the sim was in at this moment, the first command's delta is always 0 -> it never jumps.
void captureBaseline() {
  Serial.println("--- [BASELINE] Captured at enable ---");
  for (int i = 0; i < NUM_JOINTS; i++) {
    baseline_motor_pos[i] = fb[i].pos;
    baseline_link_target[i] = ext_target_pos[i];
    if (!fb[i].updated) {
      Serial.printf("  Joint %d %-14s (CAN %2d): WARNING - no motor feedback received yet, baseline motor pos=0\r\n",
                    i, JOINT_NAMES[i], JOINT_CAN_IDS[i]);
    } else {
      Serial.printf("  Joint %d %-14s (CAN %2d) motor_baseline=%7.4f rad  link_baseline=%7.4f rad\r\n",
                    i, JOINT_NAMES[i], JOINT_CAN_IDS[i], baseline_motor_pos[i], baseline_link_target[i]);
    }
  }
  baseline_ready = true;
  Serial.println("[BASELINE] motor_pos = motor_baseline + (udp_target - link_baseline) * sign * gear_ratio");
}

// -------------------------------------------------------------
// 8b. Gravity compensation (feed-forward torque)
//     Builds forward kinematics from the mass/COM/joint origins in
//     workspace2/urdf/forteme_collision_fixed.urdf, and computes the gravity torque on
//     shoulder_pitch/shoulder_roll/elbow_pitch using the virtual work method.
//       tau_j = sum_k  m_k * g_vec . (axis_j x (com_k - p_j))   [summed over distal links k]
//     shoulder_yaw is excluded from the computation since its rotation axis (Z) is always
//     parallel to gravity, making it always 0.
//
//     [Approximation/limitation] lower_arm_roll and wrist_pitch have no motor connected and hang
//     freely (backdriven), so their actual angles are unknown. Here, these two joints are assumed
//     to hold the URDF zero (0rad) pose and are treated as if rigidly attached to elbow_link. If
//     the actual sag is large this can introduce error, so when first powering on, be sure to
//     check the direction (sign) and magnitude first using small kp/kd.
// -------------------------------------------------------------
const float GRAVITY = 9.81f;


Vec3 vadd(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 vsub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 vcross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Mat3 matMul(const Mat3 &A, const Mat3 &B) {
  Mat3 r;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      r.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
    }
  }
  return r;
}

Vec3 matVec(const Mat3 &A, Vec3 v) {
  return {
    A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z,
    A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z,
    A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z,
  };
}

Mat3 rotX(float a) { float c = cosf(a), s = sinf(a); Mat3 r = {{{1,0,0},{0,c,-s},{0,s,c}}}; return r; }
Mat3 rotY(float a) { float c = cosf(a), s = sinf(a); Mat3 r = {{{c,0,s},{0,1,0},{-s,0,c}}}; return r; }
Mat3 rotZ(float a) { float c = cosf(a), s = sinf(a); Mat3 r = {{{c,-s,0},{s,c,0},{0,0,1}}}; return r; }

// URDF joint origins (relative to the parent link frame, meters)
const Vec3 P_SHOULDER_YAW   = {0.0f,   0.0f,    0.018f};  // base_link -> shoulder_yaw
const Vec3 P_SHOULDER_PITCH = {0.0f,  -0.07f,   0.151f};  // shoulder_yaw_link -> shoulder_pitch
const Vec3 P_SHOULDER_ROLL  = {0.002f, 0.07f,   0.0f};    // shoulder_pitch_link -> shoulder_roll
const Vec3 P_ELBOW          = {0.358f,-0.0345f, 0.0f};    // upper_arm_link -> elbow_pitch
const Vec3 P_LOWER_ARM_ROLL = {0.118f, 0.0335f, 0.02f};   // elbow_link -> lower_arm_roll (unactuated, treated as fixed at 0rad)
const Vec3 P_WRIST_PITCH    = {0.247f,-0.0285f, 0.0f};    // lower_arm_link -> wrist_pitch (unactuated, treated as fixed at 0rad)

// URDF link mass/center of mass (relative to each link's own frame, meters/kg)
const float M_SHOULDER_PITCH_LINK  = 0.5612f;
const Vec3  COM_SHOULDER_PITCH_LINK = {0.0082f, 0.0856f, -0.0002f};
const float M_UPPER_ARM_LINK       = 0.8932f;
const Vec3  COM_UPPER_ARM_LINK      = {0.1441f, 0.00004f, 0.00003f};
const float M_ELBOW_LINK           = 0.4441f;
const Vec3  COM_ELBOW_LINK          = {0.0235f, 0.0334f, 0.0173f};
const float M_LOWER_ARM_LINK       = 0.5071f;
const Vec3  COM_LOWER_ARM_LINK      = {0.0889f, -0.000003f, -0.00005f};
const float M_WRIST_LINK           = 0.3111f;
const Vec3  COM_WRIST_LINK          = {0.0466f, 0.0281f, 0.0268f};

const Vec3 GRAVITY_VEC = {0.0f, 0.0f, -GRAVITY};

// Whether gravity compensation is enabled (set to false if needed to turn off feed-forward torque)
bool GRAVITY_COMPENSATION_ENABLED = false;

// Safety cap: limits torque so that even if the computation is wrong during the first
// real-hardware verification stage, excessive torque isn't sent out. The theoretical maximum
// obtained by exhaustively sweeping the full joint range (+-pi, URDF limits) is
// pitch 8.17Nm, roll 2.39Nm, elbow 2.39Nm (yaw is always 0, so not applicable).
// Since pitch is more than 3x larger than roll/elbow, sharing a single cap would mean pitch keeps
// getting clipped or roll/elbow's margin becomes excessively loose, so each joint gets its own cap
// (10-15% margin below its maximum, sufficiently below T_MAX=18Nm).
const float GRAVITY_FF_LIMIT_PITCH_NM = 3.0f; // Nm (link axis)
const float GRAVITY_FF_LIMIT_ROLL_NM  = 1.0f; // Nm (link axis)
const float GRAVITY_FF_LIMIT_ELBOW_NM = 1.0f; // Nm (link axis)

float clampTorque(float v, float limit) {
  if (v < -limit) return -limit;
  if (v > limit) return limit;
  return v;
}

float gravityContrib(float m, Vec3 com, Vec3 p_j, Vec3 axis_j) {
  Vec3 r = vsub(com, p_j);
  Vec3 cr = vcross(axis_j, r);
  return m * vdot(GRAVITY_VEC, cr);
}

// q_yaw/q_pitch/q_roll/q_elbow: link (sim) coordinate frame radians (post-calibration values)
// tau_link_out[4]: order {yaw, pitch, roll, elbow}, Nm relative to the link axis (yaw is always 0)
void computeGravityTorques(float q_yaw, float q_pitch, float q_roll, float q_elbow, float tau_link_out[4]) {
  Mat3 R_yaw   = rotZ(q_yaw);
  Mat3 R_pitch = matMul(R_yaw,   rotY(q_pitch));
  Mat3 R_roll  = matMul(R_pitch, rotX(q_roll));
  Mat3 R_elbow = matMul(R_roll,  rotY(q_elbow));

  Vec3 p_yaw   = P_SHOULDER_YAW;
  Vec3 p_pitch = vadd(p_yaw,   matVec(R_yaw,   P_SHOULDER_PITCH));
  Vec3 p_roll  = vadd(p_pitch, matVec(R_pitch, P_SHOULDER_ROLL));
  Vec3 p_elbow = vadd(p_roll,  matVec(R_roll,  P_ELBOW));
  Vec3 p_lower_arm_roll = vadd(p_elbow,          matVec(R_elbow, P_LOWER_ARM_ROLL));
  Vec3 p_wrist_pitch    = vadd(p_lower_arm_roll, matVec(R_elbow, P_WRIST_PITCH)); // unactuated=assumed 0rad

  Vec3 com_shoulder_pitch = vadd(p_pitch, matVec(R_pitch, COM_SHOULDER_PITCH_LINK));
  Vec3 com_upper_arm      = vadd(p_roll,  matVec(R_roll,  COM_UPPER_ARM_LINK));
  Vec3 com_elbow          = vadd(p_elbow, matVec(R_elbow, COM_ELBOW_LINK));
  Vec3 com_lower_arm      = vadd(p_lower_arm_roll, matVec(R_elbow, COM_LOWER_ARM_LINK));
  Vec3 com_wrist          = vadd(p_wrist_pitch,    matVec(R_elbow, COM_WRIST_LINK));

  Vec3 axis_pitch = matVec(R_yaw,   {0, 1, 0});
  Vec3 axis_roll  = matVec(R_pitch, {1, 0, 0});
  Vec3 axis_elbow = matVec(R_roll,  {0, 1, 0});

  float tau_elbow = gravityContrib(M_ELBOW_LINK,      com_elbow,     p_elbow, axis_elbow)
                   + gravityContrib(M_LOWER_ARM_LINK, com_lower_arm, p_elbow, axis_elbow)
                   + gravityContrib(M_WRIST_LINK,     com_wrist,     p_elbow, axis_elbow);

  float tau_roll = gravityContrib(M_UPPER_ARM_LINK,  com_upper_arm, p_roll, axis_roll)
                  + gravityContrib(M_ELBOW_LINK,      com_elbow,     p_roll, axis_roll)
                  + gravityContrib(M_LOWER_ARM_LINK,  com_lower_arm, p_roll, axis_roll)
                  + gravityContrib(M_WRIST_LINK,      com_wrist,     p_roll, axis_roll);

  float tau_pitch = gravityContrib(M_SHOULDER_PITCH_LINK, com_shoulder_pitch, p_pitch, axis_pitch)
                   + gravityContrib(M_UPPER_ARM_LINK,      com_upper_arm,      p_pitch, axis_pitch)
                   + gravityContrib(M_ELBOW_LINK,          com_elbow,          p_pitch, axis_pitch)
                   + gravityContrib(M_LOWER_ARM_LINK,      com_lower_arm,      p_pitch, axis_pitch)
                   + gravityContrib(M_WRIST_LINK,          com_wrist,          p_pitch, axis_pitch);

  tau_link_out[0] = 0.0f; // shoulder_yaw: axis is parallel to gravity -> always 0
  tau_link_out[1] = clampTorque(tau_pitch, GRAVITY_FF_LIMIT_PITCH_NM);
  tau_link_out[2] = clampTorque(tau_roll,  GRAVITY_FF_LIMIT_ROLL_NM);
  tau_link_out[3] = clampTorque(tau_elbow, GRAVITY_FF_LIMIT_ELBOW_NM);
}

const char* runModeStr(uint8_t mode) {
  switch (mode) {
    case 0: return "RESET (idle)";
    case 1: return "CALIBRATING";
    case 2: return "RUNNING";
    default: return "UNKNOWN";
  }
}

// Converts fault_bits into a human-readable list of names (uses the same bit meanings as printFaultBits())
void faultBitsToStr(uint8_t bits, char* out, size_t outSize) {
  if (bits == 0) { snprintf(out, outSize, "none"); return; }
  out[0] = '\0';
  const char* names[6] = {
    "UNDERVOLTAGE", "THREE_PHASE_OVERCURRENT", "OVERTEMPERATURE",
    "MAGNETIC_ENCODER_FAULT", "STALL_OVERLOAD", "UNCALIBRATED"
  };
  for (int b = 0; b < 6; b++) {
    if (!(bits & (1 << b))) continue;
    if (out[0] != '\0') strncat(out, "+", outSize - strlen(out) - 1);
    strncat(out, names[b], outSize - strlen(out) - 1);
  }
}

// Called every LOG_PERIOD_MS: per joint, prints in this order:
//   1. Current motor position (measured) -> 2. Target position from the UDP packet -> 3. This motor's offset ->
//   4. Gear ratio -> 5. Final position with offset+gear ratio applied -> 6. Final position as packed into CAN (compare with 5)
// Also summarizes the number of clamps/drops that occurred in between.
void logStatus() {
  uint32_t now = millis();
  Serial.println("=======================================================================");
  Serial.printf("STATUS  t=%lums  control_active=%s\r\n",
                (unsigned long)now, ext_control_active ? "YES" : "NO (watchdog timeout / not enabled)");

  if (last_udp_recv_ms == 0) {
    Serial.println("UDP packet : none received yet");
  } else {
    Serial.printf("UDP packet : \"%s\"  (received %lums ago, watchdog timeout=%lums)\r\n",
                  last_udp_raw, (unsigned long)(now - last_udp_recv_ms), (unsigned long)WATCHDOG_TIMEOUT_MS);
  }

  // CAN receive diagnostics: whether rxCallback is actually being called (any mode), how many of
  // those are mode==2 (feedback), how many of those failed motor_id matching, and the last raw CAN
  // ID/length seen. If all these values are 0, the interrupt itself isn't firing (wiring/bus
  // issue); if can_rx_total>0 but can_rx_mode2=0, the mode field is being interpreted wrong; if
  // can_rx_unmatched>0, the motor_id position is wrong -- these numbers narrow down the cause
  // without guesswork.
  Serial.printf("CAN RX diag : total=%lu  mode2(feedback)=%lu  unmatched_id=%lu  last_raw_id=0x%08lX len=%u (%lums ago)\r\n",
                (unsigned long)can_rx_total, (unsigned long)can_rx_mode2, (unsigned long)can_rx_unmatched,
                (unsigned long)last_raw_can_id, (unsigned)last_raw_can_len,
                (unsigned long)(last_raw_can_ms == 0 ? 0 : now - last_raw_can_ms));

  for (int i = 0; i < NUM_JOINTS; i++) {
    char faultStr[96];
    faultBitsToStr(fb[i].fault_bits, faultStr, sizeof(faultStr));

    Serial.printf("\r\n[%s] (CAN ID %d)  motor state=%s  fault=%s\r\n",
                  JOINT_NAMES[i], JOINT_CAN_IDS[i], runModeStr(fb[i].run_mode), faultStr);

    if (fb[i].updated) {
      Serial.printf("  1. Current motor position (measured)     : %8.4f rad\r\n", fb[i].pos);
      Serial.printf("     current velocity / torque             : %6.2f rad/s  /  %6.2f Nm\r\n", fb[i].vel, fb[i].torque);
    } else {
      Serial.println("  1. Current motor position (measured)     : no feedback received yet");
    }

    Serial.printf("  2. Target position from UDP (link axis)  : %8.4f rad  (clamped %lu time(s) this period, raw request=%.4f)\r\n",
                  ext_target_pos[i], (unsigned long)range_clamp_count[i], last_udp_target_raw[i]);
    Serial.printf("  3. Baseline @ last enable  motor=%8.4f rad  link=%8.4f rad  (sign=%+.1f)\r\n",
                  baseline_motor_pos[i], baseline_link_target[i], JOINT_SIGN[i]);
    Serial.printf("  4. Delta from baseline (link axis, gear=x%.4f) : %8.4f rad\r\n",
                  GEAR_RATIO[i], last_cmd_delta_link[i]);
    Serial.printf("  5. Final position (baseline+delta*gear)  : %8.4f rad (motor axis)   <- computed, full precision\r\n",
                  last_cmd_motor_pos[i]);

    if (last_can_sent[i]) {
      uint16_t p_raw = (last_can_buf[i][0] << 8) | last_can_buf[i][1];
      float encoded_pos = uintToFloat(p_raw, P_MIN, P_MAX);
      Serial.printf("  6. Final position as packed into CAN     : %8.4f rad (motor axis)   <- compare with #5 (quantization)  [SENT]\r\n",
                    encoded_pos);
    } else {
      Serial.printf("  6. Final position as packed into CAN     : DROPPED, not sent  (outside +-%.1f rad raw limit, %lu drop(s) this period)\r\n",
                    RAW_LIMIT_MAX, (unsigned long)raw_drop_count[i]);
    }

    range_clamp_count[i] = 0;
    raw_drop_count[i] = 0;
  }
  Serial.println("=======================================================================\r\n");
}

// -------------------------------------------------------------
// 8. Main loop
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps
  Can0.setMaxMB(64);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();
  Can0.enableMBInterrupts();
  Can0.onReceive(rxCallback);

  Ethernet.begin(staticIP, subnet, IPAddress(0, 0, 0, 0));
  udp.begin(UDP_PORT);

  IPAddress ip = Ethernet.localIP();
  Serial.println("==================================================");
  Serial.println("[Teensy 4.1] 4-Joint Arm Control (Ethernet-CAN)");
  Serial.printf("Joints (idx->CAN): yaw->%d, pitch->%d, roll->%d, elbow->%d\r\n",
                JOINT_CAN_IDS[0], JOINT_CAN_IDS[1], JOINT_CAN_IDS[2], JOINT_CAN_IDS[3]);
  Serial.printf("Static IP       : %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
  Serial.printf("UDP Port        : %d\r\n", UDP_PORT);
  Serial.printf("Debug mode      : %s (LOG_PERIOD after 'e' enable: %lums)\r\n",
                enable_debug ? "ON" : "OFF",
                (unsigned long)(enable_debug ? DEBUG_LOG_PERIOD_MS : NORMAL_LOG_PERIOD_MS));
  Serial.println("Serial commands : d=disable");
  Serial.println("                  e=enable all motors + capture baseline (current motor pos + current");
  Serial.println("                    UDP target). From then on, motor follows ONLY the delta of the UDP");
  Serial.println("                    target from that baseline -> no jump at enable, regardless of sim pose.");
  Serial.println("                  p=pause/resume periodic status log");
  Serial.println("                  g=toggle real-time PLOT,... CSV stream (for plot_positions.py)");
  Serial.println("                  f=attempt fault reset on all motors (stop->re-enable->stop)");
  Serial.println("==================================================");

  controlTimer = 0;
}

void loop() {
  Can0.events();

  // Print immediately when the fault state changes (without waiting for LOG_PERIOD_MS, only once when the value changes)
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (fb[i].fault_changed) {
      fb[i].fault_changed = false;
      printFaultBits(i, fb[i].fault_bits);
    }
  }

  // Handle UDP receive
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[64];
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';

      // Format parsing: P,yaw,pitch,roll,elbow (link coordinate frame rad)
      if (packetBuffer[0] == 'P') {
        // strtok() destroys packetBuffer below, so copy the original for logging beforehand
        strncpy(last_udp_raw, packetBuffer, sizeof(last_udp_raw) - 1);
        last_udp_raw[sizeof(last_udp_raw) - 1] = '\0';
        last_udp_recv_ms = millis();

        char* token = strtok(packetBuffer, ",");
        int i = 0;
        while (token != NULL && i < NUM_JOINTS) {
          token = strtok(NULL, ",");
          if (token != NULL) {
            float v = atof(token);
            last_udp_target_raw[i] = v; // Original value before clamping (for logging)

            // Safety measure for the initial real-hardware verification stage: clamp to
            // +-TEST_LINK_RANGE_LIMIT around SIM_DEFAULT_LINK_POS
            // (Isaac Sim's actual motion range is much larger than this, so it's normal for
            //  almost every packet to get clamped. Logging every time would flood the serial
            //  output, so just count and print in the LOG_PERIOD_MS summary)
            float lo = SIM_DEFAULT_LINK_POS[i] - TEST_LINK_RANGE_LIMIT;
            float hi = SIM_DEFAULT_LINK_POS[i] + TEST_LINK_RANGE_LIMIT;
            if (v < lo || v > hi) {
              range_clamp_count[i]++;
              v = (v < lo) ? lo : hi;
            }

            ext_target_pos[i] = v;
            i++;
          }
        }
        ext_control_active = true;
        last_packet_time = millis(); // Reset the watchdog
      }
    }
  }

  // 20ms control loop (50Hz). Same structure as teensy-forte teleop-bi's main loop:
  // every tick, unconditionally sends exactly one of either "the actual command if under active
  // control, or a zero-resistance (kp=kd=0) probe otherwise" per joint (regardless of enable
  // state). Since this single Type1 frame is simultaneously the command and a feedback request,
  // fb[] is always kept up to date.
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // fb[i].updated is not reset here. The response to the command just sent in this tick only
    // arrives via rxCallback() on a later tick, so clearing it to false at the start of every tick
    // would give logStatus() right below no chance at all to ever read that value (observed in
    // practice: even though mode2 feedback kept arriving, the log always printed "no feedback
    // received yet" -- this was a bug). It's enough for rxCallback() to set it to true only when
    // an actual response arrives, and keep it true from then on.
    if (ext_control_active && (millis() - last_packet_time < WATCHDOG_TIMEOUT_MS)) {

      if (!baseline_ready) {
        // 'e' hasn't been pressed yet so there's no baseline -> keep only the zero-resistance probe (no motion)
        for (int i = 0; i < NUM_JOINTS; i++) {
          operationControl(JOINT_CAN_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
      } else {
        float target_vel = 0.0f;
        float kp = 15.0f;
        float kd = 1.0f;

        // 1. Compute only the amount of change (delta, link axis) of the UDP target relative to the baseline - absolute position is not used
        float delta_link[NUM_JOINTS];
        float link_pos_est[NUM_JOINTS]; // Estimated absolute angle from the sim's own perspective (for gravity compensation reference)
        for (int i = 0; i < NUM_JOINTS; i++) {
          delta_link[i] = (ext_target_pos[i] - baseline_link_target[i]) * JOINT_SIGN[i];
          link_pos_est[i] = ext_target_pos[i] * JOINT_SIGN[i];
        }

        // 2. Compute gravity compensation torque based on the current target pose (link axis, Nm)
        float tau_gravity_link[NUM_JOINTS];
        computeGravityTorques(link_pos_est[0], link_pos_est[1], link_pos_est[2], link_pos_est[3], tau_gravity_link);

        for (int i = 0; i < NUM_JOINTS; i++) {
          // 3. Convert delta to the motor axis (=CAN command) coordinate frame by multiplying by the motor:link rotation ratio, then add to the baseline
          float motor_pos = baseline_motor_pos[i] + delta_link[i] * GEAR_RATIO[i];
          // Torque, unlike position, is divided by the ratio (since the external reduction stage amplifies torque by the ratio)
          float feed_forward_torque = GRAVITY_COMPENSATION_ENABLED ? (tau_gravity_link[i] / GEAR_RATIO[i]) : 0.0f;

          last_cmd_delta_link[i] = delta_link[i];
          last_cmd_link_pos[i] = link_pos_est[i];
          last_cmd_motor_pos[i] = motor_pos;
          last_cmd_ff_torque[i] = feed_forward_torque;

          operationControl(JOINT_CAN_IDS[i], feed_forward_torque, motor_pos, target_vel, kp, kd);
        }

        if (plotStreamEnabled) {
          // PLOT,millis,link_yaw,link_pitch,link_roll,link_elbow,motor_yaw,motor_pitch,motor_roll,motor_elbow
          Serial.printf("PLOT,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                        (unsigned long)millis(),
                        last_cmd_link_pos[0], last_cmd_link_pos[1], last_cmd_link_pos[2], last_cmd_link_pos[3],
                        last_cmd_motor_pos[0], last_cmd_motor_pos[1], last_cmd_motor_pos[2], last_cmd_motor_pos[3]);
        }
      }

    } else {
      // Handle watchdog timeout
      if (ext_control_active) {
        Serial.println("[EMERGENCY] UDP Timeout! Disabling all motors.");
        ext_control_active = false;
        baseline_ready = false; // The next active control must re-capture the baseline with a fresh 'e'
        for (int i = 0; i < NUM_JOINTS; i++) {
          disableMotor(JOINT_CAN_IDS[i]);
        }
        LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS; // On disable, release the dense debug log period and return to the normal period
      }

      // Even when not under active control, continue requesting feedback via the zero-resistance probe (same as teleop-bi)
      for (int i = 0; i < NUM_JOINTS; i++) {
        operationControl(JOINT_CAN_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    if (!serialLogPaused && millis() - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = millis();
      logStatus();
    }
  }
}

// Manual control via serial commands
// E: Enable all motors (MIT run mode) + capture baseline (current motor position + current UDP target).
//    From then on, control only tracks the delta of the UDP target relative to this baseline.
// D: Turn everything off (invalidates the baseline until the next enable)
// P: Toggle pause/resume of the periodic status log (logStatus) output (does not affect motor control/UDP receive)
// G: Toggle the real-time position CSV (PLOT,...) stream (plot_positions.py parses this output)
// F: Attempt fault reset on all motors (stop->re-write Run Mode->enable->stop, requires re-enabling with e afterward)
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd' || ch == 'D') {
      baseline_ready = false;
      for (int i = 0; i < NUM_JOINTS; i++) {
        disableMotor(JOINT_CAN_IDS[i]);
      }
      LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS; // On disable, release the dense debug log period and return to the normal period
    } else if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_JOINTS; i++) {
        enableMotor(JOINT_CAN_IDS[i]);
        delay(20); // Small delay for stability of consecutive commands on the CAN bus
      }
      captureBaseline(); // Snapshot the motor position + UDP target value at this instant as the baseline
      LOG_PERIOD_MS = enable_debug ? DEBUG_LOG_PERIOD_MS : NORMAL_LOG_PERIOD_MS;
      if (enable_debug) {
        Serial.printf("[Teensy] enable_debug=true -> LOG_PERIOD_MS=%lums (dense logging). Press 'p' to pause and inspect.\r\n",
                      (unsigned long)LOG_PERIOD_MS);
      }
    } else if (ch == 'p' || ch == 'P') {
      serialLogPaused = !serialLogPaused;
      Serial.printf("[Teensy] Status log %s.\r\n", serialLogPaused ? "PAUSED" : "RESUMED");
    } else if (ch == 'g' || ch == 'G') {
      plotStreamEnabled = !plotStreamEnabled;
      Serial.printf("[Teensy] Plot CSV stream %s.\r\n", plotStreamEnabled ? "ON" : "OFF");
    } else if (ch == 'f' || ch == 'F') {
      ext_control_active = false; // After a fault reset, UDP control does not resume until re-enabled ('e')
      baseline_ready = false; // The baseline is also invalidated until re-enabled
      for (int i = 0; i < NUM_JOINTS; i++) {
        clearFault(JOINT_CAN_IDS[i]); // stop -> re-write Run Mode -> enable (induces fault re-evaluation via restart)
        delay(20);
        // How the motor behaves right after enable, before any position command (Type1) has been
        // sent, hasn't been verified, so it's safely returned to a definite stop state again.
        disableMotor(JOINT_CAN_IDS[i]);
        delay(20);
      }
      LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS; // On disable, release the dense debug log period and return to the normal period
    }
  }
}
