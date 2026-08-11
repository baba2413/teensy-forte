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
const uint8_t NUM_MOTORS_CAN1 = 1; // 필요에 따라 2로 변경
const uint8_t MST_IDS_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {1, 2};
const uint8_t SLV_IDS_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {11, 12};

const uint8_t NUM_MOTORS_CAN2 = 0; // 필요에 따라 2로 변경
const uint8_t MST_IDS_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {3, 4};
const uint8_t SLV_IDS_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {13, 14};

const uint8_t HOST_ID = 253;

const float KP = 3.0f;
const float KD = 0.0f;

// Teensy 4.0/4.1 CAN1, CAN2 사용
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

// -------------------------------------------------------------
// 2. Robstride 프로토콜 및 감속비 물리적 상수
// -------------------------------------------------------------
const float GEAR_RATIO = 6.0f;              // RS06 / EL05 감속비 (6:1)
const float MOTOR_SPAN_RAD = 8.0f * M_PI;   // 8pi (약 25.1327 rad) - 오버플로우 보정치
const float ROLLOVER_THRESHOLD = 12.0f;     // +-4pi(12.566) 근처 롤오버 임계값

const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;

// -------------------------------------------------------------
// 3. 실제 로봇 관절 회전 범위 (Joint Space Limits in Radian)
// -------------------------------------------------------------
// J1: Yaw   (-138.0 deg ~ +138.0 deg)
// J2: Roll  (-142.4 deg ~ +142.4 deg)
// J3: Pitch ( 0.0 deg ~ +143.2 deg)
// J4: Pitch ( 0.0 deg ~ +65.9 deg)
const float JOINT_LIMIT_MIN_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {-2.4086f, -2.48f}; 
const float JOINT_LIMIT_MAX_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = { 2.4086f,  2.48f};

const float JOINT_LIMIT_MIN_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = { 0.0f,     0.0f}; 
const float JOINT_LIMIT_MAX_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = { 2.50f,    1.15f};

// 안전 및 왓치독 파라미터
const float MAX_SAFE_TORQUE = 2.0f;       // 작업자 보호용 최대 마스터 피드백 토크 (Nm)
const float K_WALL = 12.0f;               // 가상벽 반발 강도 (Nm/rad)
const float MASTER_KD = 0.0f;            // 마스터 모터 능동 댐핑
const uint32_t WATCHDOG_TIMEOUT_MS = 100; // 통신 끊김 판정 기준

// -------------------------------------------------------------
// 4. 제어 주기 및 위치 점프 여유값
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz (dt = 0.002초)
elapsedMicros controlTimer;

const float POSITION_JUMP_MARGIN_RAD = 0.20f;

// -------------------------------------------------------------
// 5. 모터 트래커 (롤오버 언랩핑 & 위치점프 분리 구조체)
// -------------------------------------------------------------
struct MotorTracker {
  bool initialized = false;
  float raw_prev = 0.0f;
  int32_t turn_count = 0;
  float continuous_pos = 0.0f; // 모터축 기준 누적 라디안
  uint32_t last_time_us = 0;
  
  bool jump_flag = false;
  bool rollover_flag = false;
  float jump_delta = 0.0f;
};

MotorTracker tracker_can1[256];
MotorTracker tracker_can2[256];

// 실시간 상태 및 영점 칼리브레이션 변수
volatile float master_raw_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float slave_raw_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile float slave_trq_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};

volatile float master_raw_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile float slave_raw_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
volatile float slave_trq_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

// 'c' 키로 저장되는 관절 Zero 오프셋 (모터 연속 라디안 기준)
float master_zero_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
float slave_zero_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
bool joint_calibrated_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};

float master_zero_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
float slave_zero_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
bool joint_calibrated_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

volatile bool master_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile bool slave_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile bool master_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile bool slave_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

// Fault 및 Mode 매핑
volatile uint8_t fault_bits_can1[256] = {0};
volatile uint8_t fault_bits_can2[256] = {0};
volatile uint8_t motor_mode_can1[256] = {0};
volatile uint8_t motor_mode_can2[256] = {0};
volatile bool fault_changed_can1[256] = {false};
volatile bool fault_changed_can2[256] = {false};

// 왓치독 타임스탬프
volatile uint32_t last_rx_time_mst_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile uint32_t last_rx_time_slv_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile uint32_t last_rx_time_mst_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile uint32_t last_rx_time_slv_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// 토크 제한용 구조체
struct MasterTorqueState {
  float current_output_trq = 0.0f;
};

MasterTorqueState mst_trq_state_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
MasterTorqueState mst_trq_state_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// -------------------------------------------------------------
// 6. 데이터 스케일링, 언랩핑 및 헬퍼 함수
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

// 모터 생 수치(-12.5 ~ +12.5) 범위 바인딩 함수
float wrapToMotorRaw(float pos) {
  const float range = P_MAX - P_MIN; // 25.0f
  while (pos > P_MAX) pos -= range;
  while (pos < P_MIN) pos += range;
  return pos;
}

// [핵심] 롤오버 언랩핑과 노이즈 위치점프를 명확히 분리하는 위치 업데이트
bool updateMotorTracker(uint8_t motor_id, float raw_pos, MotorTracker &tr) {
  uint32_t now_us = micros();

  if (!tr.initialized) {
    tr.initialized = true;
    tr.raw_prev = raw_pos;
    tr.continuous_pos = raw_pos;
    tr.turn_count = 0;
    tr.last_time_us = now_us;
    return true;
  }

  float raw_delta = raw_pos - tr.raw_prev;
  int turn_delta = 0;

  // 1. +-4pi 오버플로우(Rollover) 감지
  if (raw_delta < -ROLLOVER_THRESHOLD) {
    turn_delta = 1;  // +4pi -> -4pi 로 넘어감 (정회전)
  } else if (raw_delta > ROLLOVER_THRESHOLD) {
    turn_delta = -1; // -4pi -> +4pi 로 넘어감 (역회전)
  }

  // 실제 모터 회전자 이동량 (롤오버 보정 적용)
  float motor_step = raw_delta + (float)turn_delta * MOTOR_SPAN_RAD;

  // 2. 노이즈 및 위치 점프(Jump) 감지
  float dt = (float)(now_us - tr.last_time_us) * 1.0e-6f;
  if (dt <= 0.0f) dt = 0.001f;
  float allowed_step = V_MAX * dt + POSITION_JUMP_MARGIN_RAD;

  if (fabsf(motor_step) > allowed_step) {
    tr.jump_flag = true;
    tr.jump_delta = motor_step;
    return false; // 이상 수치 무시
  }

  // 정상 데이터인 경우 상태 업데이트
  if (turn_delta != 0) {
    tr.turn_count += turn_delta;
    tr.rollover_flag = true;
  }

  tr.continuous_pos = raw_pos + (float)tr.turn_count * MOTOR_SPAN_RAD;
  tr.raw_prev = raw_pos;
  tr.last_time_us = now_us;
  return true;
}

void printFaultBits(const char* can_name, uint8_t motor_id, uint8_t fault_bits) {
  Serial.printf("[%s FAULT] Motor %d | Raw: 0x%02X", can_name, motor_id, fault_bits);
  if (fault_bits == 0) { Serial.println(" | CLEARED"); return; }
  if (fault_bits & (1 << 0)) Serial.print(" | UNDERVOLTAGE");
  if (fault_bits & (1 << 1)) Serial.print(" | THREE_PHASE_OVERCURRENT");
  if (fault_bits & (1 << 2)) Serial.print(" | OVERTEMPERATURE");
  if (fault_bits & (1 << 3)) Serial.print(" | MAGNETIC_ENCODER_FAULT");
  if (fault_bits & (1 << 4)) Serial.print(" | STALL_OVERLOAD");
  if (fault_bits & (1 << 5)) Serial.print(" | UNCALIBRATED");
  Serial.println();
}

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
  const float MAX_TORQUE_STEP = 0.10f; // Slew Rate Limiter
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
// 7. CAN 송신 제어 함수군
// -------------------------------------------------------------
void enableMotorCan1(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  memset(mode_msg.buf, 0, 8);
  mode_msg.buf[0] = 0x05; mode_msg.buf[1] = 0x70;
  Can1.write(mode_msg);
  delay(20);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  memset(enable_msg.buf, 0, 8);
  Can1.write(enable_msg);
  Serial.printf("[CAN1] Motor %d Enabled.\r\n", motor_id);
}

void disableMotorCan1(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  memset(msg.buf, 0, 8);
  Can1.write(msg);
  Serial.printf("[CAN1] Motor %d Disabled.\r\n", motor_id);
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
  msg.buf[0] = (p_int >> 8) & 0xFF; msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF; msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF; msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF; msg.buf[7] = kd_int & 0xFF;
  Can1.write(msg);
  return msg;
}

void enableMotorCan2(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  memset(mode_msg.buf, 0, 8);
  mode_msg.buf[0] = 0x05; mode_msg.buf[1] = 0x70;
  Can2.write(mode_msg);
  delay(20);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  memset(enable_msg.buf, 0, 8);
  Can2.write(enable_msg);
  Serial.printf("[CAN2] Motor %d Enabled.\r\n", motor_id);
}

void disableMotorCan2(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  memset(msg.buf, 0, 8);
  Can2.write(msg);
  Serial.printf("[CAN2] Motor %d Disabled.\r\n", motor_id);
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
  msg.buf[0] = (p_int >> 8) & 0xFF; msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF; msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF; msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF; msg.buf[7] = kd_int & 0xFF;
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

    // 언랩핑 & 노이즈 검사
    if (!updateMotorTracker(motor_id, current_pos, tracker_can1[motor_id])) {
      return; // 위치 점프 검출 시 업데이트 건너뜀
    }

    if (new_fault_bits != fault_bits_can1[motor_id]) {
      fault_bits_can1[motor_id] = new_fault_bits;
      fault_changed_can1[motor_id] = true;
    }
    motor_mode_can1[motor_id] = new_motor_mode;
    uint32_t now = millis();

    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (motor_id == MST_IDS_CAN1[i]) {
        master_raw_can1[i] = current_pos;
        master_valid_can1[i] = true;
        last_rx_time_mst_can1[i] = now;
        break;
      } else if (motor_id == SLV_IDS_CAN1[i]) {
        slave_raw_can1[i] = current_pos;
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

    if (!updateMotorTracker(motor_id, current_pos, tracker_can2[motor_id])) {
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
        master_raw_can2[i] = current_pos;
        master_valid_can2[i] = true;
        last_rx_time_mst_can2[i] = now;
        break;
      } else if (motor_id == SLV_IDS_CAN2[i]) {
        slave_raw_can2[i] = current_pos;
        slave_trq_can2[i] = current_trq;
        slave_valid_can2[i] = true;
        last_rx_time_slv_can2[i] = now;
        break;
      }
    }
  }
}

// -------------------------------------------------------------
// 9. 'c' 키 기반 영점(Joint Zero) 칼리브레이션
// -------------------------------------------------------------
void calibrateJointZero() {
  Serial.println("\n[CALIBRATION] Calibrating Joint Zeros (Current Pose -> 0.0 rad)...");

  // CAN1 관절 영점 설정
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    uint8_t m_id = MST_IDS_CAN1[i];
    uint8_t s_id = SLV_IDS_CAN1[i];

    if (!master_valid_can1[i] || !slave_valid_can1[i]) {
      Serial.printf("[CALIB CAN1] Pair %d FAILED: Feedback missing!\r\n", i + 1);
      continue;
    }

    master_zero_can1[i] = tracker_can1[m_id].continuous_pos;
    slave_zero_can1[i]  = tracker_can1[s_id].continuous_pos;
    joint_calibrated_can1[i] = true;

    Serial.printf("[CALIB CAN1] Pair %d (J%d) Zero Fixed | MST Motor Zero: %.3f rad, SLV Motor Zero: %.3f rad\r\n",
                  i + 1, i + 1, master_zero_can1[i], slave_zero_can1[i]);
  }

  // CAN2 관절 영점 설정
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    uint8_t m_id = MST_IDS_CAN2[i];
    uint8_t s_id = SLV_IDS_CAN2[i];
    int j_num = i + 1 + NUM_MOTORS_CAN1;

    if (!master_valid_can2[i] || !slave_valid_can2[i]) {
      Serial.printf("[CALIB CAN2] Pair %d FAILED: Feedback missing!\r\n", j_num);
      continue;
    }

    master_zero_can2[i] = tracker_can2[m_id].continuous_pos;
    slave_zero_can2[i]  = tracker_can2[s_id].continuous_pos;
    joint_calibrated_can2[i] = true;

    Serial.printf("[CALIB CAN2] Pair %d (J%d) Zero Fixed | MST Motor Zero: %.3f rad, SLV Motor Zero: %.3f rad\r\n",
                  i + 1, j_num, master_zero_can2[i], slave_zero_can2[i]);
  }

  Serial.println("[CALIBRATION] Complete! Virtual Walls & Joint Limits Active.\n");
}

// -------------------------------------------------------------
// 10. 관절 공간(Joint Space) 기반 연동 제어 및 가상벽
// -------------------------------------------------------------
void controlJointPair(uint8_t pair_idx, uint8_t mst_id, uint8_t slv_id,
                      float j_min, float j_max,
                      MasterTorqueState &trq_state, bool is_can1) {

  uint32_t now = millis();
  uint32_t mst_last_rx = is_can1 ? last_rx_time_mst_can1[pair_idx] : last_rx_time_mst_can2[pair_idx];
  uint32_t slv_last_rx = is_can1 ? last_rx_time_slv_can1[pair_idx] : last_rx_time_slv_can2[pair_idx];

  // [왓치독] 통신 끊김 시 토크 차단
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

  // 트래커 참조
  MotorTracker &mst_tr = is_can1 ? tracker_can1[mst_id] : tracker_can2[mst_id];
  MotorTracker &slv_tr = is_can1 ? tracker_can1[slv_id] : tracker_can2[slv_id];

  float mst_zero = is_can1 ? master_zero_can1[pair_idx] : master_zero_can2[pair_idx];
  float slv_zero = is_can1 ? slave_zero_can1[pair_idx]  : slave_zero_can2[pair_idx];

  // 1. 관절 공간 상대 각도 연산 (Joint Angle in Radian)
  float q_master = (mst_tr.continuous_pos - mst_zero) / GEAR_RATIO;
  float q_slave_target = q_master; // 마스터 추종

  // 2. 가상벽 클램핑 및 관절 침범량(Penetration) 계산
  float q_slave_clamped = q_slave_target;
  float limit_penetration = 0.0f;

  if (q_slave_target > j_max) {
    q_slave_clamped = j_max;
    limit_penetration = q_slave_target - j_max; // (+) 침범
  } else if (q_slave_target < j_min) {
    q_slave_clamped = j_min;
    limit_penetration = q_slave_target - j_min; // (-) 침범
  }

  // 3. 클램핑된 관절 위치를 슬레이브 모터 생 수치(-12.5~+12.5)로 역변환
  float motor_target_cont = slv_zero + (q_slave_clamped * GEAR_RATIO);
  float motor_target_raw  = wrapToMotorRaw(motor_target_cont);

  // 4. 슬레이브 모터 제어 명령
  float slave_kp = 25.0f;
  float slave_kd = 1.0f;
  float slv_trq = is_can1 ? slave_trq_can1[pair_idx] : slave_trq_can2[pair_idx];

  if (is_can1) {
    operationControlCan1(slv_id, 0.0f, motor_target_raw, 0.0f, slave_kp, slave_kd);
  } else {
    operationControlCan2(slv_id, 0.0f, motor_target_raw, 0.0f, slave_kp, slave_kd);
  }

  // 5. 마스터 반력 토크 연산
  float master_trq = processSlaveTorqueSafety(slv_trq, trq_state);

  // 가상벽 침범 시 마스터 반력 추가 (감속비 반영)
  if (limit_penetration != 0.0f) {
    master_trq -= (K_WALL * limit_penetration) / GEAR_RATIO;
  }

  // 마스터 피드백 토크 제한
  if (master_trq > MAX_SAFE_TORQUE)  master_trq = MAX_SAFE_TORQUE;
  if (master_trq < -MAX_SAFE_TORQUE) master_trq = -MAX_SAFE_TORQUE;

  // 6. 마스터 모터 토크 전송
  if (is_can1) {
    operationControlCan1(mst_id, master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
  } else {
    operationControlCan2(mst_id, master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
  }
}

// -------------------------------------------------------------
// 11. 메인 루프
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("=== Robstride Bilateral Teleoperation with Joint Space Calibration ===");

  Can1.begin(); Can1.setBaudRate(1000000); Can1.setMaxMB(64);
  Can1.setMBFilter(ACCEPT_ALL); Can1.distribute(); Can1.enableMBInterrupts();
  Can1.onReceive(rxCallbackCan1);

  Can2.begin(); Can2.setBaudRate(1000000); Can2.setMaxMB(64);
  Can2.setMBFilter(ACCEPT_ALL); Can2.distribute(); Can2.enableMBInterrupts();
  Can2.onReceive(rxCallbackCan2);

  Serial.println("CAN Controllers Initialized.");
  delay(500);

  // 모든 모터 활성화
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    enableMotorCan1(MST_IDS_CAN1[i]); delay(20);
    enableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
  }
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    enableMotorCan2(MST_IDS_CAN2[i]); delay(20);
    enableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
  }

  // 더미 명령으로 피드백 유도
  uint32_t waitStart = millis();
  while (millis() - waitStart < 200) {
    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      operationControlCan1(MST_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      operationControlCan1(SLV_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      operationControlCan2(MST_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      operationControlCan2(SLV_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    Can1.events(); Can2.events();
    delay(10);
  }

  Serial.println("\n>>> Press 'c' to Calibrate Current Pose as Zero Reference <<<");
  controlTimer = 0;
}

void loop() {
  Can1.events();
  Can2.events();

  // 1. Fault, Rollover, Jump 상태 진단 로깅
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    uint8_t ids[2] = {MST_IDS_CAN1[i], SLV_IDS_CAN1[i]};
    for (int j = 0; j < 2; j++) {
      uint8_t m_id = ids[j];

      if (fault_changed_can1[m_id]) {
        noInterrupts();
        uint8_t fault = fault_bits_can1[m_id];
        fault_changed_can1[m_id] = false;
        interrupts();
        printFaultBits("CAN1", m_id, fault);
      }

      if (tracker_can1[m_id].rollover_flag) {
        tracker_can1[m_id].rollover_flag = false;
        Serial.printf("[CAN1 ROLLOVER] Motor %d Rollover Handled | Turn Count: %d\r\n",
                      m_id, tracker_can1[m_id].turn_count);
      }

      if (tracker_can1[m_id].jump_flag) {
        tracker_can1[m_id].jump_flag = false;
        Serial.printf("[CAN1 JUMP DISCARDED] Motor %d Delta: %.3f rad\r\n",
                      m_id, tracker_can1[m_id].jump_delta);
      }
    }
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    uint8_t ids[2] = {MST_IDS_CAN2[i], SLV_IDS_CAN2[i]};
    for (int j = 0; j < 2; j++) {
      uint8_t m_id = ids[j];

      if (fault_changed_can2[m_id]) {
        noInterrupts();
        uint8_t fault = fault_bits_can2[m_id];
        fault_changed_can2[m_id] = false;
        interrupts();
        printFaultBits("CAN2", m_id, fault);
      }

      if (tracker_can2[m_id].rollover_flag) {
        tracker_can2[m_id].rollover_flag = false;
        Serial.printf("[CAN2 ROLLOVER] Motor %d Rollover Handled | Turn Count: %d\r\n",
                      m_id, tracker_can2[m_id].turn_count);
      }

      if (tracker_can2[m_id].jump_flag) {
        tracker_can2[m_id].jump_flag = false;
        Serial.printf("[CAN2 JUMP DISCARDED] Motor %d Delta: %.3f rad\r\n",
                      m_id, tracker_can2[m_id].jump_delta);
      }
    }
  }

  // 2. LED 토글
  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= 1000) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 3. 500Hz 실시간 제어 루프
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // CAN1 제어
    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (joint_calibrated_can1[i]) {
        controlJointPair(i, MST_IDS_CAN1[i], SLV_IDS_CAN1[i],
                         JOINT_LIMIT_MIN_CAN1[i], JOINT_LIMIT_MAX_CAN1[i],
                         mst_trq_state_can1[i], true);
      }
    }

    // CAN2 제어
    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (joint_calibrated_can2[i]) {
        controlJointPair(i, MST_IDS_CAN2[i], SLV_IDS_CAN2[i],
                         JOINT_LIMIT_MIN_CAN2[i], JOINT_LIMIT_MAX_CAN2[i],
                         mst_trq_state_can2[i], false);
      }
    }

    // 4. 모니터링 출력 (500ms)
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        uint8_t m_id = MST_IDS_CAN1[i];
        uint8_t s_id = SLV_IDS_CAN1[i];

        float q_mst = (tracker_can1[m_id].continuous_pos - master_zero_can1[i]) / GEAR_RATIO;
        float q_slv = (tracker_can1[s_id].continuous_pos - slave_zero_can1[i]) / GEAR_RATIO;

        Serial.printf("[CAN1 J%d] Joint MST: %.3f rad | Joint SLV: %.3f rad | SLV Trq: %.2f Nm | MST FB: %.2f Nm (%s)\r\n",
                      i + 1, q_mst, q_slv, slave_trq_can1[i],
                      mst_trq_state_can1[i].current_output_trq,
                      joint_calibrated_can1[i] ? "CALIB OK" : "NEED CALIB ('c')");
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        uint8_t m_id = MST_IDS_CAN2[i];
        uint8_t s_id = SLV_IDS_CAN2[i];

        float q_mst = (tracker_can2[m_id].continuous_pos - master_zero_can2[i]) / GEAR_RATIO;
        float q_slv = (tracker_can2[s_id].continuous_pos - slave_zero_can2[i]) / GEAR_RATIO;

        Serial.printf("[CAN2 J%d] Joint MST: %.3f rad | Joint SLV: %.3f rad | SLV Trq: %.2f Nm | MST FB: %.2f Nm (%s)\r\n",
                      i + 1 + NUM_MOTORS_CAN1, q_mst, q_slv, slave_trq_can2[i],
                      mst_trq_state_can2[i].current_output_trq,
                      joint_calibrated_can2[i] ? "CALIB OK" : "NEED CALIB ('c')");
      }
    }
  }
}

// -------------------------------------------------------------
// 12. 시리얼 명령어 인터럽트
// -------------------------------------------------------------
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == 'c' || ch == 'C') {
      calibrateJointZero(); // 현재 자세를 영점(0 rad)으로 저장

    } else if (ch == 'd' || ch == 'D') {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        disableMotorCan1(MST_IDS_CAN1[i]); delay(20);
        disableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        disableMotorCan2(MST_IDS_CAN2[i]); delay(20);
        disableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }
      Serial.println("[SYSTEM] All Motors Disabled.");

    } else if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        enableMotorCan1(MST_IDS_CAN1[i]); delay(20);
        enableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        enableMotorCan2(MST_IDS_CAN2[i]); delay(20);
        enableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }
      Serial.println("[SYSTEM] All Motors Enabled.");
    }
  }
}