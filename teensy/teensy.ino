#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 1. Motor and communication configuration parameters
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2;
const uint8_t MST_IDS_CAN1[NUM_MOTORS_CAN1 > 0 ? NUM_MOTORS_CAN1 : 1] = {1, 2};
const uint8_t SLV_IDS_CAN1[NUM_MOTORS_CAN1 > 0 ? NUM_MOTORS_CAN1 : 1] = {11, 12};

const uint8_t NUM_MOTORS_CAN2 = 0;
const uint8_t MST_IDS_CAN2[NUM_MOTORS_CAN2 > 0 ? NUM_MOTORS_CAN2 : 1] = {};
const uint8_t SLV_IDS_CAN2[NUM_MOTORS_CAN2 > 0 ? NUM_MOTORS_CAN2 : 1] = {};

const uint8_t HOST_ID = 253;

const float KP = 3.0f;
const float KD = 0.0f;

// Uses Teensy 4.0/4.1 CAN1, CAN2
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

// -------------------------------------------------------------
// 3. Control period configuration (500 Hz for teleoperation / dt = 0.002s)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000;
elapsedMicros controlTimer;

// Position jump detection margin (actual allowed change = V_MAX * elapsed time + margin)
const float POSITION_JUMP_MARGIN_RAD = 0.20f;

// -------------------------------------------------------------
// 4. Real-time state and offset variables (supports variable sizes)
// -------------------------------------------------------------
#define SAFE_BUF_SIZE(n) ((n) > 0 ? (n) : 1)

volatile float master_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float slave_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
float pos_offset_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]          = {};

volatile float master_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile float slave_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
float pos_offset_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]          = {};

// Tracks whether new feedback has been received, before offset calculation
volatile bool master_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile bool slave_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile bool master_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile bool slave_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

bool offset_ready_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
bool offset_ready_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// Stores fault and mode state from Type 2 feedback (mapped across the full ID range 0-255)
volatile uint8_t fault_bits_can1[256] = {0};
volatile uint8_t fault_bits_can2[256] = {0};
volatile uint8_t motor_mode_can1[256] = {0};
volatile uint8_t motor_mode_can2[256] = {0};
volatile bool fault_changed_can1[256] = {false};
volatile bool fault_changed_can2[256] = {false};

// Position jump and position range wraparound monitoring
volatile bool position_initialized_can1[256] = {false};
volatile bool position_initialized_can2[256] = {false};
volatile float previous_position_can1[256] = {0.0f};
volatile float previous_position_can2[256] = {0.0f};
volatile uint32_t previous_position_us_can1[256] = {0};
volatile uint32_t previous_position_us_can2[256] = {0};
volatile bool position_jump_can1[256] = {false};
volatile bool position_jump_can2[256] = {false};
volatile float position_jump_delta_can1[256] = {0.0f};
volatile float position_jump_delta_can2[256] = {0.0f};
volatile bool position_wrap_can1[256] = {false};
volatile bool position_wrap_can2[256] = {false};

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

float wrapPosition(float pos) {
  const float range = P_MAX - P_MIN;

  while (pos > P_MAX) pos -= range;
  while (pos < P_MIN) pos += range;

  return pos;
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

bool checkPositionJump(uint8_t motor_id, float current_pos,
                       volatile bool* initialized,
                       volatile float* previous_position,
                       volatile uint32_t* previous_time_us,
                       volatile bool* jump_flag,
                       volatile float* jump_delta,
                       volatile bool* wrap_flag) {
  uint32_t now_us = micros();

  if (!initialized[motor_id]) {
    initialized[motor_id] = true;
    previous_position[motor_id] = current_pos;
    previous_time_us[motor_id] = now_us;
    return true;
  }

  float raw_delta = current_pos - previous_position[motor_id];
  const float range = P_MAX - P_MIN;

  if (raw_delta > range * 0.5f || raw_delta < -range * 0.5f) {
    wrap_flag[motor_id] = true;
  }

  uint32_t dt_us = now_us - previous_time_us[motor_id];
  float dt = (float)dt_us * 1.0e-6f;
  float allowed_delta = V_MAX * dt + POSITION_JUMP_MARGIN_RAD;

  if (fabsf(raw_delta) > allowed_delta) {
    jump_delta[motor_id] = raw_delta;
    jump_flag[motor_id] = true;
    return false;
  }

  previous_position[motor_id] = current_pos;
  previous_time_us[motor_id] = now_us;
  return true;
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
  uint16_t p_int  = floatToUint(pos, P_MIN, P_MAX, 16);
  uint16_t v_int  = floatToUint(vel, V_MIN, V_MAX, 16);
  uint16_t kp_int = floatToUint(kp, 0.0f, KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd, 0.0f, KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX, 16);

  CAN_message_t msg;
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
  uint16_t p_int  = floatToUint(pos, P_MIN, P_MAX, 16);
  uint16_t v_int  = floatToUint(vel, V_MIN, V_MAX, 16);
  uint16_t kp_int = floatToUint(kp, 0.0f, KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd, 0.0f, KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX, 16);

  CAN_message_t msg;
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
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);

    if (!checkPositionJump(motor_id, current_pos,
                           position_initialized_can1,
                           previous_position_can1,
                           previous_position_us_can1,
                           position_jump_can1,
                           position_jump_delta_can1,
                           position_wrap_can1)) {
      return;
    }

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

    if (!checkPositionJump(motor_id, current_pos,
                           position_initialized_can2,
                           previous_position_can2,
                           previous_position_us_can2,
                           position_jump_can2,
                           position_jump_delta_can2,
                           position_wrap_can2)) {
      return;
    }

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
// 9. Initial position offset measurement helper function
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

  // Send dummy command to induce motor feedback
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

  // CAN1 offset calculation
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

  // CAN2 offset calculation (Pair number accounts for the CAN1 count)
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
// 10. Main loop structure
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== Robstride Unidirectional Teleoperation with Initial Offset ===");

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
  delay(1000);

  setupOffset();
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

      if (position_wrap_can1[motor_id]) {
        noInterrupts();
        position_wrap_can1[motor_id] = false;
        interrupts();

        Serial.printf("[CAN1 WRAP] Motor %d position crossed P_MIN/P_MAX boundary\r\n",
                      motor_id);
      }

      if (position_jump_can1[motor_id]) {
        noInterrupts();
        float delta = position_jump_delta_can1[motor_id];
        position_jump_can1[motor_id] = false;
        interrupts();

        Serial.printf("[CAN1 JUMP] Motor %d unexpected position delta: %.3f rad\r\n",
                      motor_id, delta);
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

      if (position_wrap_can2[motor_id]) {
        noInterrupts();
        position_wrap_can2[motor_id] = false;
        interrupts();

        Serial.printf("[CAN2 WRAP] Motor %d position crossed P_MIN/P_MAX boundary\r\n",
                      motor_id);
      }

      if (position_jump_can2[motor_id]) {
        noInterrupts();
        float delta = position_jump_delta_can2[motor_id];
        position_jump_can2[motor_id] = false;
        interrupts();

        Serial.printf("[CAN2 JUMP] Motor %d unexpected position delta: %.3f rad\r\n",
                      motor_id, delta);
      }
    }
  }

  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= 1000) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 2ms period control loop (500Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    float slave_kp = 25.0f;
    float slave_kd = 1.0f;

    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      operationControlCan1(MST_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      if (offset_ready_can1[i]) {
        float slave_target_pos = wrapPosition(master_pos_can1[i] + pos_offset_can1[i]);
        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, slave_target_pos, 0.0f, slave_kp, slave_kd);
      }
    }

    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      operationControlCan2(MST_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      if (offset_ready_can2[i]) {
        float slave_target_pos = wrapPosition(master_pos_can2[i] + pos_offset_can2[i]);
        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, slave_target_pos, 0.0f, slave_kp, slave_kd);
      }
    }

    // Print status monitoring every 500ms
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        float slave_target_pos = wrapPosition(master_pos_can1[i] + pos_offset_can1[i]);
        Serial.printf("[CAN1] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad "
                      "(Target: %.3f, Offset: %s)\r\n",
                      MST_IDS_CAN1[i], master_pos_can1[i],
                      SLV_IDS_CAN1[i], slave_pos_can1[i], slave_target_pos,
                      offset_ready_can1[i] ? "READY" : "NOT READY");
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        float slave_target_pos = wrapPosition(master_pos_can2[i] + pos_offset_can2[i]);
        Serial.printf("[CAN2] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad "
                      "(Target: %.3f, Offset: %s)\r\n",
                      MST_IDS_CAN2[i], master_pos_can2[i],
                      SLV_IDS_CAN2[i], slave_pos_can2[i], slave_target_pos,
                      offset_ready_can2[i] ? "READY" : "NOT READY");
      }

      Serial.println();
    }
  }
}

// -------------------------------------------------------------
// 11. Serial command receive interrupt
// -------------------------------------------------------------
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == 'd' || ch == 'D') {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        disableMotorCan1(MST_IDS_CAN1[i]); delay(20);
        disableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        disableMotorCan2(MST_IDS_CAN2[i]); delay(20);
        disableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }

      Serial.println("[Teensy] All Motors Disabled.");

    } else if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        enableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        enableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }

      setupOffset();
      Serial.println("[Teensy] All Motors Enabled & Offset Reset.");
    }
  }
}