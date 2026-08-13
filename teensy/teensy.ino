#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 0. 안전한 배열 크기 매크로 (0개 설정 시 컴파일 에러 방지)
// -------------------------------------------------------------
#define SAFE_BUF_SIZE(n) ((n) > 0 ? (n) : 1)

// -------------------------------------------------------------
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 1;
const uint8_t MST_IDS_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {127};
const uint8_t SLV_IDS_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {1};

const uint8_t NUM_MOTORS_CAN2 = 0;
const uint8_t MST_IDS_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
const uint8_t SLV_IDS_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

const uint8_t HOST_ID = 253;

const float KP = 3.0f;
const float KD = 0.0f;

// Teensy 4.0/4.1 CAN1, CAN2 사용
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

// -------------------------------------------------------------
// 2. Robstride 프로토콜 물리적 제한 한계값
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
// 3. 슬레이브 가동 범위 설정 (임시: 프로토콜 이론상 최고/최저값)
// -------------------------------------------------------------
const float SLAVE_LIMIT_MIN_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {-12.5f}; // J1, J2
const float SLAVE_LIMIT_MAX_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = { 12.5f};

const float SLAVE_LIMIT_MIN_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {}; // J3, J4
const float SLAVE_LIMIT_MAX_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// 안전 및 왓치독 파라미터
const float MAX_SAFE_TORQUE = 2.0f;       // 작업자 손목 보호용 최대 피드백 토크 (Nm)
const float K_WALL = 15.0f;               // 슬레이브 한계 가상벽 반발 강도 (Nm/rad)
const float MASTER_KD = 0.0f;            // 마스터 모터 능동 댐핑
const uint32_t WATCHDOG_TIMEOUT_MS = 100; // 통신 끊김 판정 기준 (100ms)

// -------------------------------------------------------------
// 4. 제어 주기 및 진단 설정
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz (dt = 0.002초)
elapsedMicros controlTimer;

// 위치 점프 판정 여유값 (실제 허용 변화량 = V_MAX * 경과시간 + 여유값)
const float POSITION_JUMP_MARGIN_RAD = 0.20f;

// -------------------------------------------------------------
// 5. 실시간 상태, 오프셋 및 진단 변수 (가변 크기 대응)
// -------------------------------------------------------------
volatile float master_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float slave_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile float slave_trq_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
float pos_offset_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]          = {};

volatile float master_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile float slave_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
volatile float slave_trq_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
float pos_offset_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]          = {};

volatile bool master_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile bool slave_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile bool master_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile bool slave_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

bool offset_ready_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
bool offset_ready_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// Type 2 피드백의 fault 및 mode 상태 저장 (전체 ID 0~255 매핑)
volatile uint8_t fault_bits_can1[256] = {0};
volatile uint8_t fault_bits_can2[256] = {0};
volatile uint8_t motor_mode_can1[256] = {0};
volatile uint8_t motor_mode_can2[256] = {0};
volatile bool fault_changed_can1[256] = {false};
volatile bool fault_changed_can2[256] = {false};

// 위치 점프 및 위치 범위 순환 감시
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

// 통신 상태 및 왓치독 타임스탬프
volatile uint32_t last_rx_time_mst_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile uint32_t last_rx_time_slv_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile uint32_t last_rx_time_mst_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile uint32_t last_rx_time_slv_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// 마스터 토크 급변 제한(Slew Rate Limiter)용 이전 출력 상태
struct MasterTorqueState {
  float current_output_trq = 0.0f;
};

MasterTorqueState mst_trq_state_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
MasterTorqueState mst_trq_state_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// -------------------------------------------------------------
// 6. 데이터 스케일링 및 진단 헬퍼 함수
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

// 슬레이브 충돌 토크 안전 정화 함수 (Deadzone + Slew Rate Limiter)
float processSlaveTorqueSafety(float slave_trq, MasterTorqueState &state) {
  float filtered_trq = slave_trq;
  if (fabsf(filtered_trq) < 0.15f) {
    filtered_trq = 0.0f;
  } else if (filtered_trq > 0) {
    filtered_trq -= 0.15f;
  } else {
    filtered_trq += 0.15f;
  }

  float target_trq = -1.0f * filtered_trq * 0.4f;

  const float MAX_TORQUE_STEP = 0.10f; // 2ms당 최대 변화량
  float trq_delta = target_trq - state.current_output_trq;

  if (trq_delta > MAX_TORQUE_STEP) {
    target_trq = state.current_output_trq + MAX_TORQUE_STEP;
  } else if (trq_delta < -MAX_TORQUE_STEP) {
    target_trq = state.current_output_trq - MAX_TORQUE_STEP;
  }

  state.current_output_trq = target_trq;
  return target_trq;
}

// -------------------------------------------------------------
// 7. Robstride CAN1/CAN2 송신 제어 함수군
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
// 8. CAN 수신 인터럽트 콜백
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
    uint32_t now = millis();

    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (motor_id == MST_IDS_CAN1[i]) {
        master_pos_can1[i] = current_pos;
        master_valid_can1[i] = true;
        last_rx_time_mst_can1[i] = now;
        break;
      } else if (motor_id == SLV_IDS_CAN1[i]) {
        slave_pos_can1[i] = current_pos;
        slave_trq_can1[i] = current_trq;
        slave_valid_can1[i] = true;
        last_rx_time_slv_can1[i] = now;
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
    uint32_t now = millis();

    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (motor_id == MST_IDS_CAN2[i]) {
        master_pos_can2[i] = current_pos;
        master_valid_can2[i] = true;
        last_rx_time_mst_can2[i] = now;
        break;
      } else if (motor_id == SLV_IDS_CAN2[i]) {
        slave_pos_can2[i] = current_pos;
        slave_trq_can2[i] = current_trq;
        slave_valid_can2[i] = true;
        last_rx_time_slv_can2[i] = now;
        break;
      }
    }
  }
}

// -------------------------------------------------------------
// 9. 초기 위치 오프셋 측정 헬퍼 함수
// -------------------------------------------------------------
void setupOffset() {
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    master_valid_can1[i] = false;
    slave_valid_can1[i] = false;
    offset_ready_can1[i] = false;
    last_rx_time_mst_can1[i] = 0;
    last_rx_time_slv_can1[i] = 0;
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    master_valid_can2[i] = false;
    slave_valid_can2[i] = false;
    offset_ready_can2[i] = false;
    last_rx_time_mst_can2[i] = 0;
    last_rx_time_slv_can2[i] = 0;
  }

  // 모터 피드백 유도를 위한 Dummy 명령 전송
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

  // CAN1 오프셋 계산
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

  // CAN2 오프셋 계산 (CAN1 개수를 반영하여 Pair 번호 자동 계산)
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
// 10. 관절 쌍 연동 제어 및 왓치독
// -------------------------------------------------------------
void controlJointPair(uint8_t pair_idx, uint8_t mst_id, uint8_t slv_id, float master_pos, float slave_trq,
                      float offset, float slv_min, float slv_max,
                      MasterTorqueState &trq_state, bool is_can1) {

  uint32_t now = millis();
  uint32_t mst_last_rx = is_can1 ? last_rx_time_mst_can1[pair_idx] : last_rx_time_mst_can2[pair_idx];
  uint32_t slv_last_rx = is_can1 ? last_rx_time_slv_can1[pair_idx] : last_rx_time_slv_can2[pair_idx];

  // [왓치독 검사] 통신이 끊긴 경우 토크 출력을 안전하게 차단 (0 Nm)
  if ((now - mst_last_rx > WATCHDOG_TIMEOUT_MS) || (now - slv_last_rx > WATCHDOG_TIMEOUT_MS)) {
    if (is_can1) {
      operationControlCan1(slv_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      operationControlCan1(mst_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    } else {
      operationControlCan2(slv_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      operationControlCan2(mst_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    return;
  }

  // 1. 오프셋이 적용된 Raw 슬레이브 목표 위치
  float raw_slave_target = master_pos + offset;

  // 2. 슬레이브 목표 위치 Clamp 및 가상벽 침범량 계산
  float safe_slave_target = raw_slave_target;
  float limit_penetration = 0.0f;

  if (raw_slave_target > slv_max) {
    safe_slave_target = slv_max;                     // 슬레이브 명령 위치 고정
    limit_penetration = raw_slave_target - slv_max;  // 상한 초과 깊이 (+)
  } else if (raw_slave_target < slv_min) {
    safe_slave_target = slv_min;                     // 슬레이브 명령 위치 고정
    limit_penetration = raw_slave_target - slv_min;  // 하한 초과 깊이 (-)
  }

  // 범위 Wrapping 환산 (안전장치)
  safe_slave_target = wrapPosition(safe_slave_target);

  // 3. 슬레이브 모터에 위치 명령 전달 (Safe Target)
  float slave_kp = 25.0f;
  float slave_kd = 1.0f;
  if (is_can1) {
    operationControlCan1(slv_id, 0.0f, safe_slave_target, 0.0f, slave_kp, slave_kd);
  } else {
    operationControlCan2(slv_id, 0.0f, safe_slave_target, 0.0f, slave_kp, slave_kd);
  }

  // 4. 마스터 피드백 토크 연산
  float master_trq = processSlaveTorqueSafety(slave_trq, trq_state);

  // 슬레이브 한계점 도달 시 마스터 가상벽 반력 추가
  if (limit_penetration != 0.0f) {
    master_trq -= K_WALL * limit_penetration;
  }

  // 최종 마스터 피드백 토크 Clamping (최대 2.0 Nm)
  if (master_trq > MAX_SAFE_TORQUE)  master_trq = MAX_SAFE_TORQUE;
  if (master_trq < -MAX_SAFE_TORQUE) master_trq = -MAX_SAFE_TORQUE;

  // 5. 마스터 모터 토크 전송
  if (is_can1) {
    operationControlCan1(mst_id, master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
  } else {
    operationControlCan2(mst_id, master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
  }
}

// -------------------------------------------------------------
// 11. 메인 루프 구조
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== Robstride Bilateral Teleoperation with Full Safety & Diagnostics ===");

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

  // 모든 모터 활성화
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    enableMotorCan1(MST_IDS_CAN1[i]); delay(20);
    enableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    enableMotorCan2(MST_IDS_CAN2[i]); delay(20);
    enableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
  }

  setupOffset();
  controlTimer = 0;
}

void loop() {
  Can1.events();
  Can2.events();

  // 1. Fault, Position Wrap, Position Jump 상태 로깅
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

  // 2. 1초(1000ms)마다 내장 LED 토글
  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= 1000) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 3. 2ms 주기 제어 루프 (500Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // CAN1 제어
    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (offset_ready_can1[i]) {
        controlJointPair(i, MST_IDS_CAN1[i], SLV_IDS_CAN1[i], 
                         master_pos_can1[i], slave_trq_can1[i], pos_offset_can1[i], 
                         SLAVE_LIMIT_MIN_CAN1[i], SLAVE_LIMIT_MAX_CAN1[i], 
                         mst_trq_state_can1[i], true);
      }
    }

    // CAN2 제어
    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (offset_ready_can2[i]) {
        controlJointPair(i, MST_IDS_CAN2[i], SLV_IDS_CAN2[i], 
                         master_pos_can2[i], slave_trq_can2[i], pos_offset_can2[i], 
                         SLAVE_LIMIT_MIN_CAN2[i], SLAVE_LIMIT_MAX_CAN2[i], 
                         mst_trq_state_can2[i], false);
      }
    }

    // 4. 500ms마다 상태 모니터링 출력
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        float slave_target_pos = wrapPosition(master_pos_can1[i] + pos_offset_can1[i]);
        Serial.printf("[CAN1 J%d] MST: %.3f rad | SLV: %.3f rad (Target: %.3f, Offset: %s) | SLV Trq: %.2f Nm | MST FB: %.2f Nm\r\n",
                      i + 1, master_pos_can1[i], slave_pos_can1[i], slave_target_pos,
                      offset_ready_can1[i] ? "READY" : "NOT READY",
                      slave_trq_can1[i], mst_trq_state_can1[i].current_output_trq);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        float slave_target_pos = wrapPosition(master_pos_can2[i] + pos_offset_can2[i]);
        Serial.printf("[CAN2 J%d] MST: %.3f rad | SLV: %.3f rad (Target: %.3f, Offset: %s) | SLV Trq: %.2f Nm | MST FB: %.2f Nm\r\n",
                      i + 1 + NUM_MOTORS_CAN1, master_pos_can2[i], slave_pos_can2[i], slave_target_pos,
                      offset_ready_can2[i] ? "READY" : "NOT READY",
                      slave_trq_can2[i], mst_trq_state_can2[i].current_output_trq);
      }

      Serial.println();
    }
  }
}

// -------------------------------------------------------------
// 12. 시리얼 명령 수신 인터럽트
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
        enableMotorCan1(MST_IDS_CAN1[i]); delay(20);
        enableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        enableMotorCan2(MST_IDS_CAN2[i]); delay(20);
        enableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }

      setupOffset();
      Serial.println("[Teensy] All Motors Enabled & Offset Reset.");
    }
  }
}