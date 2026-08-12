#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 0. 안전한 배열 크기 매크로
// -------------------------------------------------------------
#define SAFE_BUF_SIZE(n) ((n) > 0 ? (n) : 1)

// -------------------------------------------------------------
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2; 
const uint8_t MST_IDS_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {1, 2};
const uint8_t SLV_IDS_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {11, 12};

const uint8_t NUM_MOTORS_CAN2 = 2; 
const uint8_t MST_IDS_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {3, 4};
const uint8_t SLV_IDS_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {13, 14};

const uint8_t HOST_ID = 253;

// Teensy 4.0/4.1 CAN1, CAN2
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

const uint32_t LOG_PERIOD = 1000; // ms

// -------------------------------------------------------------
// 2. Robstride 프로토콜 및 감속비 물리적 상수
// -------------------------------------------------------------
const float GEAR_RATIO = 6.0f;               // RS06 / EL05 감속비 (6:1)
const float MOTOR_SPAN_RAD = 25.0f;          // RobStride 모터 Raw 스팬 (12.5 - (-12.5) = 25.0f)
const float ROLLOVER_THRESHOLD = 12.0f;      // 롤오버 임계값

const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;

// -------------------------------------------------------------
// 3. 관절 회전 가상벽 범위 (Joint Space Limits in Radian)
// -------------------------------------------------------------
const float JOINT_LIMIT_MIN_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {-2.2086f, -2.28f}; 
const float JOINT_LIMIT_MAX_CAN1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = { 2.2086f,  2.28f};

const float JOINT_LIMIT_MIN_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {-2.3000f, -0.1000f}; 
const float JOINT_LIMIT_MAX_CAN2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = { 0.1000f,  0.9500f};

// 안전 및 파라미터
const float MAX_SAFE_TORQUE = 2.0f;        // 작업자 보호용 최대 마스터 피드백 토크 (Nm)
const float K_WALL = 12.0f;                // 가상벽 반발 강도 (Nm/rad)
const float MASTER_KD = 0.0f;             // 마스터 모터 능동 댐핑
const uint32_t WATCHDOG_TIMEOUT_MS = 100;  // 통신 끊김 판정 기준

// ★ [Pure Torque 제어용 파라미터] ★
const float SLV_TEENSY_KP = 15.0f;          // Teensy 주도 슬레이브 추종 토크 게인 (Nm/rad)
const float SLV_MOTOR_KD  = 0.8f;          // 슬레이브 모터 속도 댐핑 (모터 K_d)
const float MAX_SLV_TORQUE_CMD = 6.0f;     // 슬레이브 모터 최대 가하는 토크 한계 (Nm)

// -------------------------------------------------------------
// 4. 제어 주기 설정
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz (dt = 0.002초)
elapsedMicros controlTimer;

// -------------------------------------------------------------
// 5. 모터 트래커
// -------------------------------------------------------------
struct MotorTracker {
  bool initialized = false;
  float raw_prev = 0.0f;
  int32_t turn_count = 0;
  float continuous_pos = 0.0f; // 모터축 기준 누적 라디안
  uint32_t last_time_us = 0;
  
  bool rollover_flag = false;
};

MotorTracker tracker_can1[256];
MotorTracker tracker_can2[256];

// 실시간 센서 수치
volatile float master_raw_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile float slave_raw_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile float slave_trq_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};

volatile float master_raw_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile float slave_raw_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
volatile float slave_trq_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

// 플래그 변수
bool is_system_calibrated = false;  
bool is_system_enabled = false;     

float master_zero_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
float slave_zero_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};

float master_zero_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
float slave_zero_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

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

void disableMotorCan1(uint8_t motor_id);
void disableMotorCan2(uint8_t motor_id);

// 연속 위치 언랩핑 트래커 (패킷 버림 없이 연속 추적)
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

  if (raw_delta < -ROLLOVER_THRESHOLD) {
    turn_delta = 1;  
  } else if (raw_delta > ROLLOVER_THRESHOLD) {
    turn_delta = -1; 
  }

  tr.turn_count += turn_delta;
  if (turn_delta != 0) {
    tr.rollover_flag = true;
  }

  tr.continuous_pos = raw_pos + ((float)tr.turn_count * MOTOR_SPAN_RAD);
  tr.raw_prev = raw_pos;
  tr.last_time_us = now_us;
  return true;
}

void printFaultBits(const char* can_name, uint8_t motor_id, uint8_t fault_bits) {
  Serial.printf("[%s FAULT] Motor ID: %d | Raw: 0x%02X", can_name, motor_id, fault_bits);
  if (fault_bits == 0) { Serial.println(" | CLEARED"); return; }
  if (fault_bits & (1 << 0)) Serial.print(" | UNDERVOLTAGE");
  if (fault_bits & (1 << 1)) Serial.print(" | THREE_PHASE_OVERCURRENT");
  if (fault_bits & (1 << 2)) Serial.print(" | OVERTEMPERATURE");
  if (fault_bits & (1 << 3)) Serial.print(" | MAGNETIC_ENCODER_FAULT");
  if (fault_bits & (1 << 4)) Serial.print(" | STALL_OVERLOAD");
  if (fault_bits & (1 << 5)) Serial.print(" | UNCALIBRATED");
  Serial.println(" -> DISABLED!");
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
  const float MAX_TORQUE_STEP = 0.10f;
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
  Serial.printf("[CAN1] Motor ID: %d Enabled.\r\n", motor_id);
}

void disableMotorCan1(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  memset(msg.buf, 0, 8);
  Can1.write(msg);
  Serial.printf("[CAN1] Motor ID: %d Disabled.\r\n", motor_id);
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
  Serial.printf("[CAN2] Motor ID: %d Enabled.\r\n", motor_id);
}

void disableMotorCan2(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  memset(msg.buf, 0, 8);
  Can2.write(msg);
  Serial.printf("[CAN2] Motor ID: %d Disabled.\r\n", motor_id);
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

    updateMotorTracker(motor_id, current_pos, tracker_can1[motor_id]);

    if (new_fault_bits != 0) {
      disableMotorCan1(motor_id);
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

    updateMotorTracker(motor_id, current_pos, tracker_can2[motor_id]);

    if (new_fault_bits != 0) {
      disableMotorCan2(motor_id);
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
  Serial.println("\n[CALIBRATION] Calibrating Joint Zeros...");

  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    uint8_t m_id = MST_IDS_CAN1[i];
    uint8_t s_id = SLV_IDS_CAN1[i];

    if (!master_valid_can1[i] || !slave_valid_can1[i]) continue;

    master_zero_can1[i] = tracker_can1[m_id].continuous_pos;
    slave_zero_can1[i]  = tracker_can1[s_id].continuous_pos;

    Serial.printf("[CALIB CAN1] Pair (MST ID:%d / SLV ID:%d) Zero Fixed | MST: %.3f rad, SLV: %.3f rad\r\n",
                  m_id, s_id, master_zero_can1[i], slave_zero_can1[i]);
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    uint8_t m_id = MST_IDS_CAN2[i];
    uint8_t s_id = SLV_IDS_CAN2[i];

    if (!master_valid_can2[i] || !slave_valid_can2[i]) continue;

    master_zero_can2[i] = tracker_can2[m_id].continuous_pos;
    slave_zero_can2[i]  = tracker_can2[s_id].continuous_pos;

    Serial.printf("[CALIB CAN2] Pair (MST ID:%d / SLV ID:%d) Zero Fixed | MST: %.3f rad, SLV: %.3f rad\r\n",
                  m_id, s_id, master_zero_can2[i], slave_zero_can2[i]);
  }

  is_system_calibrated = true; 
  Serial.println("[CALIBRATION] Complete! Ready to Enable ('e').\n");
}

// -------------------------------------------------------------
// 10. 관절 연동 제어 (★ PURE TORQUE 제어 적용)
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

  MotorTracker &mst_tr = is_can1 ? tracker_can1[mst_id] : tracker_can2[mst_id];
  MotorTracker &slv_tr = is_can1 ? tracker_can1[slv_id] : tracker_can2[slv_id];

  float mst_zero = is_can1 ? master_zero_can1[pair_idx] : master_zero_can2[pair_idx];
  float slv_zero = is_can1 ? slave_zero_can1[pair_idx]  : slave_zero_can2[pair_idx];

  // 1. 관절 상대 각도 연산
  float q_master = (mst_tr.continuous_pos - mst_zero) / GEAR_RATIO;
  float q_slave_target = q_master;

  // 2. 가상벽 클램핑 및 침범량 계산
  float q_slave_clamped = q_slave_target;
  float limit_penetration = 0.0f;

  if (q_slave_target > j_max) {
    q_slave_clamped = j_max;
    limit_penetration = q_slave_target - j_max;
  } else if (q_slave_target < j_min) {
    q_slave_clamped = j_min;
    limit_penetration = q_slave_target - j_min;
  }

  // 3. 마스터 가상벽 및 슬레이브 반력 토크 연산 & 전송
  float slv_trq = is_can1 ? slave_trq_can1[pair_idx] : slave_trq_can2[pair_idx];
  float master_trq = processSlaveTorqueSafety(slv_trq, trq_state);

  if (limit_penetration != 0.0f) {
    master_trq -= (K_WALL * limit_penetration) / GEAR_RATIO;
  }

  if (master_trq > MAX_SAFE_TORQUE)  master_trq = MAX_SAFE_TORQUE;
  if (master_trq < -MAX_SAFE_TORQUE) master_trq = -MAX_SAFE_TORQUE;

  if (is_can1) {
    operationControlCan1(mst_id, master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
  } else {
    operationControlCan2(mst_id, master_trq, 0.0f, 0.0f, 0.0f, MASTER_KD);
  }

  // =========================================================
  // ★ [핵심 변경] PURE TORQUE CONTROL (슬레이브)
  // continuous_pos 공간에서 오차를 연산하고, Teensy가 직접 토크 계산!
  // 모터 Kp는 0.0f로 꺼버려 12.5rad 롤오버/클램핑 버그를 완전 소멸시킴.
  // =========================================================
  float target_motor_cont = slv_zero + (q_slave_clamped * GEAR_RATIO);
  float pos_error_cont = target_motor_cont - slv_tr.continuous_pos; // continuous 오차

  // Teensy P-토크 연산 (P_Gain * Error)
  float slave_cmd_trq = pos_error_cont * SLV_TEENSY_KP;

  // 최대 안전 토크 클램핑
  if (slave_cmd_trq > MAX_SLV_TORQUE_CMD)  slave_cmd_trq = MAX_SLV_TORQUE_CMD;
  if (slave_cmd_trq < -MAX_SLV_TORQUE_CMD) slave_cmd_trq = -MAX_SLV_TORQUE_CMD;

  // CAN 전송: feed_forward = slave_cmd_trq, pos = 0.0f, kp = 0.0f, kd = SLV_MOTOR_KD
  if (is_can1) {
    operationControlCan1(slv_id, slave_cmd_trq, 0.0f, 0.0f, 0.0f, SLV_MOTOR_KD);
  } else {
    operationControlCan2(slv_id, slave_cmd_trq, 0.0f, 0.0f, 0.0f, SLV_MOTOR_KD);
  }
}

// -------------------------------------------------------------
// 11. 메인 루프
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("=== Robstride Pure Torque Teleoperation Setup ===");

  Can1.begin(); Can1.setBaudRate(1000000); Can1.setMaxMB(64);
  Can1.setMBFilter(ACCEPT_ALL); Can1.distribute(); Can1.enableMBInterrupts();
  Can1.onReceive(rxCallbackCan1);

  Can2.begin(); Can2.setBaudRate(1000000); Can2.setMaxMB(64);
  Can2.setMBFilter(ACCEPT_ALL); Can2.distribute(); Can2.enableMBInterrupts();
  Can2.onReceive(rxCallbackCan2);

  Serial.println("CAN Controllers Initialized.");
  delay(500);

  controlTimer = 0;
}

void loop() {
  Can1.events();
  Can2.events();

  // 진단 로깅
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
        Serial.printf("[CAN1 ROLLOVER] Motor ID: %d | Turn Count: %d\r\n",
                      m_id, tracker_can1[m_id].turn_count);
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
        Serial.printf("[CAN2 ROLLOVER] Motor ID: %d | Turn Count: %d\r\n",
                      m_id, tracker_can2[m_id].turn_count);
      }
    }
  }

  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= 1000) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    if (is_system_enabled && is_system_calibrated) {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        controlJointPair(i, MST_IDS_CAN1[i], SLV_IDS_CAN1[i],
                         JOINT_LIMIT_MIN_CAN1[i], JOINT_LIMIT_MAX_CAN1[i],
                         mst_trq_state_can1[i], true);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        controlJointPair(i, MST_IDS_CAN2[i], SLV_IDS_CAN2[i],
                         JOINT_LIMIT_MIN_CAN2[i], JOINT_LIMIT_MAX_CAN2[i],
                         mst_trq_state_can2[i], false);
      }
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= LOG_PERIOD) {
      lastPrint = millis();

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        uint8_t m_id = MST_IDS_CAN1[i];
        uint8_t s_id = SLV_IDS_CAN1[i];

        float q_mst = (tracker_can1[m_id].continuous_pos - master_zero_can1[i]) / GEAR_RATIO;
        float q_slv = (tracker_can1[s_id].continuous_pos - slave_zero_can1[i]) / GEAR_RATIO;

        float raw_mst = master_raw_can1[i];
        float raw_slv = slave_raw_can1[i];

        Serial.printf("[CAN1 MST_ID:%d / SLV_ID:%d] Joint MST: %.3f rad (Raw: %.3f) | Joint SLV: %.3f rad (Raw: %.3f) | SLV Trq: %.2f Nm | MST FB: %.2f Nm (%s / %s)\r\n",
                      m_id, s_id, q_mst, raw_mst, q_slv, raw_slv, slave_trq_can1[i],
                      mst_trq_state_can1[i].current_output_trq,
                      is_system_calibrated ? "CALIB_OK" : "NO_CALIB",
                      is_system_enabled ? "ENABLED" : "DISABLED");
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        uint8_t m_id = MST_IDS_CAN2[i];
        uint8_t s_id = SLV_IDS_CAN2[i];

        float q_mst = (tracker_can2[m_id].continuous_pos - master_zero_can2[i]) / GEAR_RATIO;
        float q_slv = (tracker_can2[s_id].continuous_pos - slave_zero_can2[i]) / GEAR_RATIO;

        float raw_mst = master_raw_can2[i];
        float raw_slv = slave_raw_can2[i];

        Serial.printf("[CAN2 MST_ID:%d / SLV_ID:%d] Joint MST: %.3f rad (Raw: %.3f) | Joint SLV: %.3f rad (Raw: %.3f) | SLV Trq: %.2f Nm | MST FB: %.2f Nm (%s / %s)\r\n",
                      m_id, s_id, q_mst, raw_mst, q_slv, raw_slv, slave_trq_can2[i],
                      mst_trq_state_can2[i].current_output_trq,
                      is_system_calibrated ? "CALIB_OK" : "NO_CALIB",
                      is_system_enabled ? "ENABLED" : "DISABLED");
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
      calibrateJointZero(); 

    } else if (ch == 'e' || ch == 'E') {
      if (!is_system_calibrated) {
        Serial.println("\n[ERROR REJECTED] Press 'c' to Calibrate Zero first.\n");
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
      is_system_enabled = true;
      Serial.println("[SYSTEM] All Motors Enabled (Pure Torque Mode Active).");

    } else if (ch == 'd' || ch == 'D') {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        disableMotorCan1(MST_IDS_CAN1[i]); delay(20);
        disableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
      }
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        disableMotorCan2(MST_IDS_CAN2[i]); delay(20);
        disableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
      }

      is_system_enabled = false;
      is_system_calibrated = false; 
      Serial.println("[SYSTEM] All Motors Disabled.");
    }
  }
}