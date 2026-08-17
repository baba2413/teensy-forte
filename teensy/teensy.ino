#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 1. Motor and Communication Configuration Parameters
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2;
const uint8_t MST_IDS_CAN1[NUM_MOTORS_CAN1 > 0 ? NUM_MOTORS_CAN1 : 1] = {1, 3};
const uint8_t SLV_IDS_CAN1[NUM_MOTORS_CAN1 > 0 ? NUM_MOTORS_CAN1 : 1] = {11, 13};

const uint8_t NUM_MOTORS_CAN2 = 2;
const uint8_t MST_IDS_CAN2[NUM_MOTORS_CAN2 > 0 ? NUM_MOTORS_CAN2 : 1] = {2, 4};
const uint8_t SLV_IDS_CAN2[NUM_MOTORS_CAN2 > 0 ? NUM_MOTORS_CAN2 : 1] = {12, 14};

const uint8_t HOST_ID = 253;

#define SAFE_BUF_SIZE(n) ((n) > 0 ? (n) : 1)

// Master motor haptic feedback gains (for bilateral control)
const float KP = 10.0f;
const float KD = 0.1f;
const float SLV_KP = 10.0f;
const float SLV_KD = 0.1f;

// Using Teensy 4.0/4.1 CAN1 and CAN2
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

// -------------------------------------------------------------
// 2. Robstride Protocol Physical Limit Constants
// -------------------------------------------------------------
const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;

// Unidirectional motor absolute hardware/software limits (-12.4 ~ +12.4 rad)
const float RAW_LIMIT_MIN = -12.4f;
const float RAW_LIMIT_MAX = 12.4f;

// -------------------------------------------------------------
// 3. Control Loop Period Configuration (3333 us / ~300 Hz)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 3333;
elapsedMicros controlTimer;

const uint32_t LOG_PERIOD = 2000;

// -------------------------------------------------------------
// 4. Real-time Status, Offset Variables, and Control Flags
// -------------------------------------------------------------
bool system_enabled = false; // Control enable flag (triggered via 'e' command)

volatile float master_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float slave_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
float pos_offset_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]          = {};

volatile float master_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile float slave_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
float pos_offset_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]          = {};

// Feedback reception validation flags before offset calculation
volatile bool master_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile bool slave_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile bool master_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile bool slave_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

bool offset_ready_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
bool offset_ready_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// Storage for fault bits and motor mode from Type 2 feedback
volatile uint8_t fault_bits_can1[256] = {0};
volatile uint8_t fault_bits_can2[256] = {0};
volatile uint8_t motor_mode_can1[256] = {0};
volatile uint8_t motor_mode_can2[256] = {0};
volatile bool fault_changed_can1[256] = {false};
volatile bool fault_changed_can2[256] = {false};

// -------------------------------------------------------------
// 5. Data Scaling and Diagnostic Helper Functions
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
// 6. Robstride CAN1 Transmission Functions
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

  // Drop packet if target angle exceeds the raw limit range (-12.4 ~ 12.4 rad)
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
// 7. Robstride CAN2 Transmission Functions
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

  // Drop packet if target angle exceeds the raw limit range (-12.4 ~ 12.4 rad)
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
// 8. CAN Receive Interrupt Callbacks
// -------------------------------------------------------------
void rxCallbackCan1(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;

  if (mode == 2) {
    uint8_t motor_id = (msg.id >> 8) & 0xFF;
    uint8_t new_fault_bits = (msg.id >> 16) & 0x3F;
    uint8_t new_motor_mode = (msg.id >> 22) & 0x03;

    uint16_t p_raw = ((uint16_t)msg.buf[0] << 8) | msg.buf[1];
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);

    if (new_fault_bits != fault_bits_can1[motor_id]) {
      fault_bits_can1[motor_id] = new_fault_bits;
      fault_changed_can1[motor_id] = true;
    }
    motor_mode_can1[motor_id] = new_motor_mode;

    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (motor_id == MST_IDS_CAN1[i]) {
        master_pos_can1[i] = current_pos;
        master_valid_can1[i] = true;
        break;
      } else if (motor_id == SLV_IDS_CAN1[i]) {
        slave_pos_can1[i] = current_pos;
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
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);

    if (new_fault_bits != fault_bits_can2[motor_id]) {
      fault_bits_can2[motor_id] = new_fault_bits;
      fault_changed_can2[motor_id] = true;
    }
    motor_mode_can2[motor_id] = new_motor_mode;

    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (motor_id == MST_IDS_CAN2[i]) {
        master_pos_can2[i] = current_pos;
        master_valid_can2[i] = true;
        break;
      } else if (motor_id == SLV_IDS_CAN2[i]) {
        slave_pos_can2[i] = current_pos;
        slave_valid_can2[i] = true;
        break;
      }
    }
  }
}

// -------------------------------------------------------------
// 9. Initial Position Offset Measurement Helper Function
// -------------------------------------------------------------
void setupOffset() {
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    master_valid_can1[i] = false;
    slave_valid_can1[i] = false;
    offset_ready_can1[i] = false;
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    master_valid_can2[i] = false;
    slave_valid_can2[i] = false;
    offset_ready_can2[i] = false;
  }

  // Send dummy commands to request motor feedback responses
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    operationControlCan1(MST_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    operationControlCan1(SLV_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    operationControlCan2(MST_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    operationControlCan2(SLV_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  uint32_t waitStart = millis();
  while (millis() - waitStart < 100) {
    Can1.events();
    Can2.events();
  }

  // Calculate CAN1 offsets
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    uint8_t master_id = MST_IDS_CAN1[i];
    uint8_t slave_id = SLV_IDS_CAN1[i];

    if (!master_valid_can1[i] || !slave_valid_can1[i]) {
      Serial.printf("[SETUP CAN1] Pair %d Offset FAILED: feedback missing "
                    "(Master %d: %s, Slave %d: %s)\r\n",
                    i + 1,
                    master_id, master_valid_can1[i] ? "OK" : "NO",
                    slave_id, slave_valid_can1[i] ? "OK" : "NO");
      continue;
    }

    if (fault_bits_can1[master_id] != 0 || fault_bits_can1[slave_id] != 0) {
      Serial.printf("[SETUP CAN1] Pair %d Offset FAILED: motor fault "
                    "(Master %d: 0x%02X, Slave %d: 0x%02X)\r\n",
                    i + 1,
                    master_id, fault_bits_can1[master_id],
                    slave_id, fault_bits_can1[slave_id]);
      continue;
    }

    if (!isfinite(master_pos_can1[i]) || !isfinite(slave_pos_can1[i])) {
      Serial.printf("[SETUP CAN1] Pair %d Offset FAILED: invalid position value\r\n",
                    i + 1);
      continue;
    }

    pos_offset_can1[i] = slave_pos_can1[i] - master_pos_can1[i];
    offset_ready_can1[i] = true;

    Serial.printf("[SETUP CAN1] Pair %d Offset Calculated: %.3f rad "
                  "(Master %d: %.3f, Slave %d: %.3f, Mode %d/%d)\r\n",
                  i + 1, pos_offset_can1[i],
                  master_id, master_pos_can1[i],
                  slave_id, slave_pos_can1[i],
                  motor_mode_can1[master_id], motor_mode_can1[slave_id]);
  }

  // Calculate CAN2 offsets
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    uint8_t master_id = MST_IDS_CAN2[i];
    uint8_t slave_id = SLV_IDS_CAN2[i];
    int pair_num = i + 1 + NUM_MOTORS_CAN1;

    if (!master_valid_can2[i] || !slave_valid_can2[i]) {
      Serial.printf("[SETUP CAN2] Pair %d Offset FAILED: feedback missing "
                    "(Master %d: %s, Slave %d: %s)\r\n",
                    pair_num,
                    master_id, master_valid_can2[i] ? "OK" : "NO",
                    slave_id, slave_valid_can2[i] ? "OK" : "NO");
      continue;
    }

    if (fault_bits_can2[master_id] != 0 || fault_bits_can2[slave_id] != 0) {
      Serial.printf("[SETUP CAN2] Pair %d Offset FAILED: motor fault "
                    "(Master %d: 0x%02X, Slave %d: 0x%02X)\r\n",
                    pair_num,
                    master_id, fault_bits_can2[master_id],
                    slave_id, fault_bits_can2[slave_id]);
      continue;
    }

    if (!isfinite(master_pos_can2[i]) || !isfinite(slave_pos_can2[i])) {
      Serial.printf("[SETUP CAN2] Pair %d Offset FAILED: invalid position value\r\n",
                    pair_num);
      continue;
    }

    pos_offset_can2[i] = slave_pos_can2[i] - master_pos_can2[i];
    offset_ready_can2[i] = true;

    Serial.printf("[SETUP CAN2] Pair %d Offset Calculated: %.3f rad "
                  "(Master %d: %.3f, Slave %d: %.3f, Mode %d/%d)\r\n",
                  pair_num, pos_offset_can2[i],
                  master_id, master_pos_can2[i],
                  slave_id, slave_pos_can2[i],
                  motor_mode_can2[master_id], motor_mode_can2[slave_id]);
  }
}

// -------------------------------------------------------------
// 10. Main Loop Structure
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== Robstride Bilateral Teleoperation ===");

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

  Serial.println("Teensy CAN1/CAN2 initialized.");
  Serial.println("-> Send 'e' to enable motors.");
  controlTimer = 0;
}

void loop() {
  Can1.events();
  Can2.events();

  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    uint8_t ids[2] = {MST_IDS_CAN1[i], SLV_IDS_CAN1[i]};

    for (int j = 0; j < 2; j++) {
      uint8_t motor_id = ids[j];

      if (fault_changed_can1[motor_id]) {
        noInterrupts();
        uint8_t fault = fault_bits_can1[motor_id];
        fault_changed_can1[motor_id] = false;
        interrupts();

        printFaultBits("CAN1", motor_id, fault);
      }
    }
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    uint8_t ids[2] = {MST_IDS_CAN2[i], SLV_IDS_CAN2[i]};

    for (int j = 0; j < 2; j++) {
      uint8_t motor_id = ids[j];

      if (fault_changed_can2[motor_id]) {
        noInterrupts();
        uint8_t fault = fault_bits_can2[motor_id];
        fault_changed_can2[motor_id] = false;
        interrupts();

        printFaultBits("CAN2", motor_id, fault);
      }
    }
  }

  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= LOG_PERIOD) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Periodic control loop execution
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // Bilateral control gain configuration
    float master_kp = KP; // Global parameter
    float master_kd = KD; // Global parameter
    float slave_kp  = SLV_KP;
    float slave_kd  = SLV_KD;

    // --- CAN1 Bilateral Control ---
    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (system_enabled && offset_ready_can1[i]) {
        // Calculate tracking target angle (based on raw position)
        float master_target_raw = slave_pos_can1[i] - pos_offset_can1[i];
        float slave_target_raw  = master_pos_can1[i] + pos_offset_can1[i];

        // TEST
        master_target_raw = master_pos_can1[i];

        operationControlCan1(MST_IDS_CAN1[i], 0.0f, master_target_raw, 0.0f, master_kp, master_kd);
        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, slave_target_raw, 0.0f, slave_kp, slave_kd);
      } else {
        operationControlCan1(MST_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    // --- CAN2 Bilateral Control ---
    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (system_enabled && offset_ready_can2[i]) {
        float master_target_raw = slave_pos_can2[i] - pos_offset_can2[i];
        float slave_target_raw  = master_pos_can2[i] + pos_offset_can2[i];

        operationControlCan2(MST_IDS_CAN2[i], 0.0f, master_target_raw, 0.0f, master_kp, master_kd);
        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, slave_target_raw, 0.0f, slave_kp, slave_kd);
      } else {
        operationControlCan2(MST_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    // Status monitoring printout every 1000 ms
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        Serial.printf("[CAN1] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad | Offset: %.3f (%s)\r\n",
                      MST_IDS_CAN1[i], master_pos_can1[i],
                      SLV_IDS_CAN1[i], slave_pos_can1[i],
                      pos_offset_can1[i],
                      (system_enabled && offset_ready_can1[i]) ? "ENABLED" : "DISABLED");
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        Serial.printf("[CAN2] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad | Offset: %.3f (%s)\r\n",
                      MST_IDS_CAN2[i], master_pos_can2[i],
                      SLV_IDS_CAN2[i], slave_pos_can2[i],
                      pos_offset_can2[i],
                      (system_enabled && offset_ready_can2[i]) ? "ENABLED" : "DISABLED");
      }

      Serial.println();
    }
  }
}

// -------------------------------------------------------------
// 11. Serial Command Reception Handler
// -------------------------------------------------------------
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        enableMotorCan1(MST_IDS_CAN1[i]); delay(20);
        enableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        enableMotorCan2(MST_IDS_CAN2[i]); delay(20);
        enableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }

      setupOffset();
      system_enabled = true;
      Serial.println("[Teensy] All Motors Enabled & Offset Calculated.");
    } 
    else if (ch == 'd' || ch == 'D') {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        disableMotorCan1(MST_IDS_CAN1[i]); delay(20);
        disableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        disableMotorCan2(MST_IDS_CAN2[i]); delay(20);
        disableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }

      system_enabled = false;
      Serial.println("[Teensy] All Motors Disabled. Press 'e' to run again.");
    }
  }
}