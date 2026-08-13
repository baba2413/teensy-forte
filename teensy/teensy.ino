#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2;
const uint8_t MST_IDS_CAN1[NUM_MOTORS_CAN1 > 0 ? NUM_MOTORS_CAN1 : 1] = {1, 2};
const uint8_t SLV_IDS_CAN1[NUM_MOTORS_CAN1 > 0 ? NUM_MOTORS_CAN1 : 1] = {11, 12};

const uint8_t NUM_MOTORS_CAN2 = 2;
const uint8_t MST_IDS_CAN2[NUM_MOTORS_CAN2 > 0 ? NUM_MOTORS_CAN2 : 1] = {3, 4};
const uint8_t SLV_IDS_CAN2[NUM_MOTORS_CAN2 > 0 ? NUM_MOTORS_CAN2 : 1] = {13, 14};

const uint8_t HOST_ID = 253;

#define SAFE_BUF_SIZE(n) ((n) > 0 ? (n) : 1)

// --- CAN1 모터 Pair별 이동 범위 및 제한 사용여부 설정 ---
const bool USE_LIMIT_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {false, false};  // 페어별 제한 로직 ON/OFF
const float LIMIT_MIN_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {-1.5f, -2.0f}; // 페어별 최소 이동 범위 (영점 기준 음수 rad)
const float LIMIT_MAX_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = { 2.5f,  2.0f}; // 페어별 최대 이동 범위 (영점 기준 양수 rad)

// --- CAN2 모터 Pair별 이동 범위 및 제한 사용여부 설정 ---
const bool USE_LIMIT_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {false, true};
const float LIMIT_MIN_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {-6.0f, -6.0f};
const float LIMIT_MAX_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = { 6.0f,  2.2f};

// 슬레이브 모터 위치 추종 게인
const float SLV_KP = 24.0f;
const float SLV_KD = 0.2f;

// 마스터 모터 햅틱 피드백 토크 파라미터 (position-torque 방식)
const float MAX_SAFE_TORQUE = 2.0f;       // 작업자 손목 보호용 최대 피드백 토크 (Nm)
const float MASTER_KD = 0.0f;             // 마스터 모터 능동 댐핑
const uint32_t WATCHDOG_TIMEOUT_MS = 100; // 통신 끊김 판정 기준 (100ms)

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

// 단방향 모터 절대 하드웨어 소프트웨어 제한 (-12.4 ~ +12.4 rad)
const float RAW_LIMIT_MIN = -12.4f;
const float RAW_LIMIT_MAX = 12.4f;

// -------------------------------------------------------------
// 3. 제어 주기 설정 (텔레오퍼레이션용 500 Hz / dt = 0.002초)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000;
elapsedMicros controlTimer;

const uint32_t LOG_PERIOD = 2000;

// -------------------------------------------------------------
// 4. 실시간 상태, 오프셋, 영점 변수 및 제어 플래그
// -------------------------------------------------------------
bool system_zeroed = false;  // 'c' 버튼 영점 설정 완료 여부
bool system_enabled = false; // 'e' 버튼 제어 활성화 여부

volatile float master_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float slave_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile float slave_trq_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
float pos_offset_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]          = {};

volatile float master_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile float slave_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
volatile float slave_trq_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
float pos_offset_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]          = {};

// 통신 상태 왓치독 타임스탬프
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

// 영점 위치 (Raw Radian 기준)
volatile float mst_zero_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float slv_zero_pos_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float mst_zero_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile float slv_zero_pos_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// 오프셋 계산 전 새 피드백 수신 여부 확인
volatile bool master_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile bool slave_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile bool master_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile bool slave_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

bool offset_ready_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
bool offset_ready_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// Type 2 피드백의 fault 및 mode 상태 저장
volatile uint8_t fault_bits_can1[256] = {0};
volatile uint8_t fault_bits_can2[256] = {0};
volatile uint8_t motor_mode_can1[256] = {0};
volatile uint8_t motor_mode_can2[256] = {0};
volatile bool fault_changed_can1[256] = {false};
volatile bool fault_changed_can2[256] = {false};

// -------------------------------------------------------------
// 5. 데이터 스케일링 및 진단 헬퍼 함수
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
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
// 6. Robstride CAN1 송신 제어 함수군
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

  // Raw Radian이 -12.4 ~ 12.4 rad 한계를 벗어나는 패킷은 버림
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
// 7. Robstride CAN2 송신 제어 함수군
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

  // Raw Radian이 -12.4 ~ 12.4 rad 한계를 벗어나는 패킷은 버림
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
// 9. 영점 설정 함수 ('c' 버튼 동작)
// -------------------------------------------------------------
void setZeroPosition() {
  // 최신 위치 피드백 갱신을 위해 Dummy 명령 전송
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

  Serial.println("\r\n=== Zero Position Set ===");
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    mst_zero_pos_can1[i] = master_pos_can1[i];
    slv_zero_pos_can1[i] = slave_pos_can1[i];

    float mst_zeroed = master_pos_can1[i] - mst_zero_pos_can1[i]; // 0.000 rad
    float slv_zeroed = slave_pos_can1[i] - slv_zero_pos_can1[i]; // 0.000 rad

    Serial.printf("[CAN1 Zero] Master %d: %.3f rad (raw: %.3f) [Limit: %s (%.2f ~ %.2f rad)]\r\n",
                  MST_IDS_CAN1[i], mst_zeroed, master_pos_can1[i],
                  USE_LIMIT_CAN1[i] ? "ON" : "OFF",
                  LIMIT_MIN_CAN1[i], LIMIT_MAX_CAN1[i]);
    Serial.printf("[CAN1 Zero] Slave %d: %.3f rad (raw: %.3f) [Limit: %s (%.2f ~ %.2f rad)]\r\n",
                  SLV_IDS_CAN1[i], slv_zeroed, slave_pos_can1[i],
                  USE_LIMIT_CAN1[i] ? "ON" : "OFF",
                  LIMIT_MIN_CAN1[i], LIMIT_MAX_CAN1[i]);
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    mst_zero_pos_can2[i] = master_pos_can2[i];
    slv_zero_pos_can2[i] = slave_pos_can2[i];

    float mst_zeroed = master_pos_can2[i] - mst_zero_pos_can2[i];
    float slv_zeroed = slave_pos_can2[i] - slv_zero_pos_can2[i];

    Serial.printf("[CAN2 Zero] Master %d: %.3f rad (raw: %.3f) [Limit: %s (%.2f ~ %.2f rad)]\r\n",
                  MST_IDS_CAN2[i], mst_zeroed, master_pos_can2[i],
                  USE_LIMIT_CAN2[i] ? "ON" : "OFF",
                  LIMIT_MIN_CAN2[i], LIMIT_MAX_CAN2[i]);
    Serial.printf("[CAN2 Zero] Slave %d: %.3f rad (raw: %.3f) [Limit: %s (%.2f ~ %.2f rad)]\r\n",
                  SLV_IDS_CAN2[i], slv_zeroed, slave_pos_can2[i],
                  USE_LIMIT_CAN2[i] ? "ON" : "OFF",
                  LIMIT_MIN_CAN2[i], LIMIT_MAX_CAN2[i]);
  }

  system_zeroed = true;
  Serial.println("Zeroing completed! Now press 'e' to enable motors.\r\n");
}

// -------------------------------------------------------------
// 10. 초기 위치 오프셋 측정 헬퍼 함수
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

  // CAN2 오프셋 계산
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
// 11. 메인 루프 구조
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== Robstride Bilateral Teleoperation with Zeroing & Limits ===");

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
  Serial.println("-> Send 'c' to set Zero position before enabling!");
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

  // 2ms 주기 제어 루프 (500Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // 슬레이브 위치 추종 게인
    float slave_kp = SLV_KP;
    float slave_kd = SLV_KD;

    // --- CAN1 Position-Torque 제어 ---
    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      bool watchdog_ok = (millis() - last_rx_time_mst_can1[i] <= WATCHDOG_TIMEOUT_MS) &&
                          (millis() - last_rx_time_slv_can1[i] <= WATCHDOG_TIMEOUT_MS);

      if (system_enabled && offset_ready_can1[i] && watchdog_ok) {
        // 1. 슬레이브 목표 라디안 계산 (마스터를 추종, Raw 위치 기준)
        float slave_target_raw = master_pos_can1[i] + pos_offset_can1[i];

        // 2. 해당 모터 쌍의 제한 로직이 켜진 경우(true)에만 LIMIT_MIN ~ LIMIT_MAX 범위 클램핑
        if (USE_LIMIT_CAN1[i]) {
          float min_limit = LIMIT_MIN_CAN1[i];
          float max_limit = LIMIT_MAX_CAN1[i];

          float slv_min = slv_zero_pos_can1[i] + min_limit;
          float slv_max = slv_zero_pos_can1[i] + max_limit;
          if (slave_target_raw < slv_min) slave_target_raw = slv_min;
          if (slave_target_raw > slv_max) slave_target_raw = slv_max;
        }

        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, slave_target_raw, 0.0f, slave_kp, slave_kd);

        // 3. 마스터 피드백 토크 연산 (슬레이브 토크 기반 햅틱 피드백)
        float master_trq = processSlaveTorqueSafety(slave_trq_can1[i], mst_trq_state_can1[i]);
        if (master_trq > MAX_SAFE_TORQUE)  master_trq = MAX_SAFE_TORQUE;
        if (master_trq < -MAX_SAFE_TORQUE) master_trq = -MAX_SAFE_TORQUE;

        operationControlCan1(MST_IDS_CAN1[i], master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
      } else {
        operationControlCan1(MST_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    // --- CAN2 Position-Torque 제어 ---
    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      bool watchdog_ok = (millis() - last_rx_time_mst_can2[i] <= WATCHDOG_TIMEOUT_MS) &&
                          (millis() - last_rx_time_slv_can2[i] <= WATCHDOG_TIMEOUT_MS);

      if (system_enabled && offset_ready_can2[i] && watchdog_ok) {
        float slave_target_raw = master_pos_can2[i] + pos_offset_can2[i];

        if (USE_LIMIT_CAN2[i]) {
          float min_limit = LIMIT_MIN_CAN2[i];
          float max_limit = LIMIT_MAX_CAN2[i];

          float slv_min = slv_zero_pos_can2[i] + min_limit;
          float slv_max = slv_zero_pos_can2[i] + max_limit;
          if (slave_target_raw < slv_min) slave_target_raw = slv_min;
          if (slave_target_raw > slv_max) slave_target_raw = slv_max;
        }

        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, slave_target_raw, 0.0f, slave_kp, slave_kd);

        float master_trq = processSlaveTorqueSafety(slave_trq_can2[i], mst_trq_state_can2[i]);
        if (master_trq > MAX_SAFE_TORQUE)  master_trq = MAX_SAFE_TORQUE;
        if (master_trq < -MAX_SAFE_TORQUE) master_trq = -MAX_SAFE_TORQUE;

        operationControlCan2(MST_IDS_CAN2[i], master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
      } else {
        operationControlCan2(MST_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    // 500ms마다 상태 모니터링 출력
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        float master_zeroed = master_pos_can1[i] - mst_zero_pos_can1[i];
        float slave_zeroed  = slave_pos_can1[i] - slv_zero_pos_can1[i];

        Serial.printf("[CAN1] Master %d Pos: %.3f rad (raw: %.3f) | Slave %d Pos: %.3f rad (raw: %.3f) | Offset: %.3f (%s) | SLV Trq: %.2f Nm | MST FB: %.2f Nm\r\n",
                      MST_IDS_CAN1[i], master_zeroed, master_pos_can1[i],
                      SLV_IDS_CAN1[i], slave_zeroed, slave_pos_can1[i],
                      pos_offset_can1[i],
                      (system_enabled && offset_ready_can1[i]) ? "ENABLED" : "DISABLED",
                      slave_trq_can1[i], mst_trq_state_can1[i].current_output_trq);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        float master_zeroed = master_pos_can2[i] - mst_zero_pos_can2[i];
        float slave_zeroed  = slave_pos_can2[i] - slv_zero_pos_can2[i];

        Serial.printf("[CAN2] Master %d Pos: %.3f rad (raw: %.3f) | Slave %d Pos: %.3f rad (raw: %.3f) | Offset: %.3f (%s) | SLV Trq: %.2f Nm | MST FB: %.2f Nm\r\n",
                      MST_IDS_CAN2[i], master_zeroed, master_pos_can2[i],
                      SLV_IDS_CAN2[i], slave_zeroed, slave_pos_can2[i],
                      pos_offset_can2[i],
                      (system_enabled && offset_ready_can2[i]) ? "ENABLED" : "DISABLED",
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

    if (ch == 'c' || ch == 'C') {
      setZeroPosition();
    } 
    else if (ch == 'e' || ch == 'E') {
      // 'c' 키를 누르지 않은 상태에서 'e' 키 입력 차단
      if (!system_zeroed) {
        Serial.println("[ERROR] Zero position not set! Press 'c' first before enabling with 'e'.");
        return;
      }

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
      system_zeroed = false; // d를 누른 후에는 영점 재설정 필요
      Serial.println("[Teensy] All Motors Disabled. Press 'c' then 'e' to run again.");
    }
  }
}