#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <stdio.h> // sscanf() for parsing 'g' goal-position lines

// -------------------------------------------------------------
// GOAL-FOLLOWING FIRMWARE -- single (slave) arm only.
//
// No master arm, no bilateral teleop, no haptic feedback. This build exists purely to let a
// host stream goal joint positions to the arm (e.g. from a trained policy during evaluation).
// For teleoperated data collection, flash the teleop-bi-p-t branch instead -- each branch in
// this repo is a standalone snapshot for one job, not layers meant to be combined.
//
// Serial protocol (single-char command, 'g' takes a payload line):
//   c                          -> calibrate: capture current pose as each motor's software zero
//                                 (offset only, not a physical move). Required for the per-joint
//                                 limit clamp (JOINT_LIMIT_MIN/MAX_CAN1/CAN2) to be enforced --
//                                 without it, only the wide +-12.4 rad protocol limit applies.
//   g\n                        -> enter GOAL mode, holding the arm's current position
//   g <p0> <p1> <p2> <p3>\n    -> enter/refresh GOAL mode, set the 4 joint targets
//                                 (raw motor radians, order = shoulder_yaw, shoulder_pitch,
//                                 shoulder_roll, elbow_pitch = motor ids 11, 13, 12, 14 --
//                                 note 12/13 are swapped relative to CAN wiring order, since
//                                 the kinematic joint order is yaw->pitch->roll->elbow, not the
//                                 order the CAN ids happen to be wired in)
//   d                          -> disable (zero-torque, stop). Also clears calibration -- 'c'
//                                 again before 'g' if you want the joint-limit clamp back.
// -------------------------------------------------------------

// -------------------------------------------------------------
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2;
const uint8_t SLV_IDS_CAN1[NUM_MOTORS_CAN1] = {11, 12}; // shoulder_yaw, shoulder_roll

const uint8_t NUM_MOTORS_CAN2 = 2;
const uint8_t SLV_IDS_CAN2[NUM_MOTORS_CAN2] = {13, 14}; // shoulder_pitch, elbow_pitch

const uint8_t HOST_ID = 253;

// 위치 추종 게인 (teleop-bi-p-t의 SLV_KP/KD와 동일한 값으로 시작)
const float SLV_KP = 24.0f;
const float SLV_KD = 0.2f;

// GOAL 모드 설정
const uint32_t GOAL_TIMEOUT_MS = 150;  // 이 시간 안에 새 'g' 라인이 없으면 자동 정지
const uint8_t GOAL_LINE_BUF_SIZE = 64; // 'g' 명령 한 줄의 최대 길이

// 'c' 소프트웨어 영점 + 관절별 이동 한계 (teensy-forte teleop-bi 브랜치의 "modify" 커밋에서
// 포팅, raw 모터 라디안 기준으로 재구성) -- 이 펌웨어는 기어비를 전혀 적용하지 않으므로
// (lerobot_robot_forte_arm의 SMOLVLA_GUIDE.md §2 참고) 원본의 관절-공간/기어비 나눗셈은
// 가져오지 않았다. 'c'로 잡은 현재 raw 위치가 그 모터의 영점이 되고, 이후 목표값은
// [영점 + MIN, 영점 + MAX] 범위로 클램프된다.
//
// TODO: 아래 값은 PLACEHOLDER (영점 기준 +-1.0 rad)다. 실제 팔의 물리적으로 검증된 안전
// J1 (Yaw)   : [-2.2086 rad, +2.2086 rad] (기존 2.4086에서 0.2 감소)
// J2 (Roll)  : [-2.2800 rad, +2.2800 rad] (기존 2.4800에서 0.2 감소)
// J3 (Pitch) : [-2.3000 rad, +0.1000 rad] (음수 동작, 한계 0.2 감소 및 초기점 +0.1 여유)
// J4 (Pitch) : [-0.1000 rad, +0.9500 rad] (양수 동작, 한계 0.2 감소 및 초기점 -0.1 여유)
const float JOINT_LIMIT_MIN_CAN1[NUM_MOTORS_CAN1] = {-2.2086f, -2.2800f}; 
const float JOINT_LIMIT_MAX_CAN1[NUM_MOTORS_CAN1] = { 2.2086f,  2.2800f};

const float JOINT_LIMIT_MIN_CAN2[NUM_MOTORS_CAN2] = {-2.3000f, -0.1000f}; 
const float JOINT_LIMIT_MAX_CAN2[NUM_MOTORS_CAN2] = { 0.1000f,  0.9500f};


// Teensy 4.0/4.1 CAN1, CAN2 사용 (모터 11,12는 CAN1 / 13,14는 CAN2 배선)
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
// 3. 제어 주기 설정 (300Hz / dt = 3.333ms)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 3333;
elapsedMicros controlTimer;

const uint32_t LOG_PERIOD = 1000;

// -------------------------------------------------------------
// 4. 실시간 상태 및 제어 플래그
// -------------------------------------------------------------
bool goal_mode_enabled = false; // 'g'로 진입, 'd'로 해제
bool is_calibrated = false;     // 'c'로 진입 -- 영점(zero_offset_*)이 현재 유효한지

volatile float slave_pos_can1[NUM_MOTORS_CAN1] = {};
volatile float slave_trq_can1[NUM_MOTORS_CAN1] = {};
volatile bool slave_valid_can1[NUM_MOTORS_CAN1] = {}; // CAN 피드백을 한 번이라도 받았는지
volatile float slave_pos_can2[NUM_MOTORS_CAN2] = {};
volatile float slave_trq_can2[NUM_MOTORS_CAN2] = {};
volatile bool slave_valid_can2[NUM_MOTORS_CAN2] = {};

// 'c'로 잡은 영점 (raw 모터 라디안). JOINT_LIMIT_MIN/MAX_CAN1/CAN2가 이 값 기준 상대 범위다.
float zero_offset_can1[NUM_MOTORS_CAN1] = {};
float zero_offset_can2[NUM_MOTORS_CAN2] = {};

// GOAL 모드에서 추종할 목표 원시 라디안. 인덱스는 SLV_IDS_CAN1/CAN2 배열과 동일한 순서
// (CAN 배선 기준: goal_target_can1[0]=11, [1]=12, goal_target_can2[0]=13, [1]=14).
// 주의: 이건 CAN 배선 순서이지 'g' 라인의 페이로드 순서(yaw,pitch,roll,elbow)와 다르다 --
// handleGoalLine()이 그 사이를 매핑한다.
float goal_target_can1[NUM_MOTORS_CAN1] = {};
float goal_target_can2[NUM_MOTORS_CAN2] = {};
uint32_t last_goal_rx_time = 0; // 마지막으로 유효한 'g' 라인을 받은 시각 (GOAL_TIMEOUT_MS 왓치독용)

// Type 2 피드백의 fault 및 mode 상태 저장 (motor_id로 바로 인덱싱)
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
// 8b. 'c' 소프트웨어 영점 캘리브레이션
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
    Serial.printf("[CALIB] Motor %d zero = %.3f rad\r\n", SLV_IDS_CAN1[i], zero_offset_can1[i]);
  }
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    if (!slave_valid_can2[i]) {
      Serial.printf("[CALIB] Motor %d: no CAN feedback yet -- FAILED\r\n", SLV_IDS_CAN2[i]);
      all_ok = false;
      continue;
    }
    zero_offset_can2[i] = slave_pos_can2[i];
    Serial.printf("[CALIB] Motor %d zero = %.3f rad\r\n", SLV_IDS_CAN2[i], zero_offset_can2[i]);
  }

  is_calibrated = all_ok;
  if (all_ok) {
    Serial.println("[Teensy] Calibration complete. Per-joint limits now active relative to this pose.");
  } else {
    Serial.println("[Teensy] Calibration INCOMPLETE -- missing feedback for some motor(s). NOT calibrated.");
  }
}

// -------------------------------------------------------------
// 9. GOAL 모드 ('g' 전용): 목표 위치 라인 파싱 및 진입 처리
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
  Serial.println("[Teensy] GOAL mode ENABLED. Send 'g <yaw> <pitch> <roll> <elbow>' (raw rad) "
                  "to move. Send 'd' to stop.");
  if (!is_calibrated) {
    Serial.println("[Teensy] WARNING: entering GOAL mode without calibration ('c') -- only the "
                    "wide +-12.4 rad protocol limit is active, no real per-joint limit. Send 'd' "
                    "then 'c' first if you want the per-joint clamp enforced.");
  }
}

// line은 NUL로 끝나는 문자열, len은 '\n'/'\r' 제외한 실제 길이 (0 가능 -- bare "g\n").
void handleGoalLine(const char* line, uint8_t len) {
  if (len == 0) {
    // "g\n" 단독: 현재 위치를 그대로 목표로 유지하며 GOAL 모드 진입/갱신.
    // (아직 한 번도 진입한 적 없다면 CAN 피드백에서 읽은 현재 값을 시드로 사용 -- 급격한 이동 방지)
    if (!goal_mode_enabled) {
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) goal_target_can1[i] = slave_pos_can1[i];
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) goal_target_can2[i] = slave_pos_can2[i];
    }
    enterGoalModeIfNeeded();
    last_goal_rx_time = millis();
    return;
  }

  float v0, v1, v2, v3;
  int parsed = sscanf(line, "%f %f %f %f", &v0, &v1, &v2, &v3);

  if (parsed != 4 || !isfinite(v0) || !isfinite(v1) || !isfinite(v2) || !isfinite(v3)) {
    Serial.printf("[Teensy] GOAL line REJECTED (need 4 finite floats): \"%s\"\r\n", line);
    // 형식이 잘못된 라인도 호스트가 살아있다는 신호이므로 왓치독은 갱신한다 (목표값은 갱신 안 함).
    last_goal_rx_time = millis();
    return;
  }

  // Payload order is kinematic (yaw, pitch, roll, elbow), not CAN-wiring order --
  // motors 12 (roll) and 13 (pitch) are swapped relative to their CAN bus grouping.
  goal_target_can1[0] = v0; // 11 shoulder_yaw
  goal_target_can2[0] = v1; // 13 shoulder_pitch
  goal_target_can1[1] = v2; // 12 shoulder_roll
  goal_target_can2[1] = v3; // 14 elbow_pitch

  enterGoalModeIfNeeded();
  last_goal_rx_time = millis();
}

// -------------------------------------------------------------
// 10. 메인 루프 구조
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== GOAL-following firmware (single arm, no bilateral) ===");

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
  Serial.println("-> Send 'c' to calibrate zero (recommended first), 'g' for goal-position mode, "
                  "'d' to disable.");
  controlTimer = 0;
}

void loop() {
  Can1.events();
  Can2.events();

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

  // 3.333ms 주기 제어 루프 (300Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // GOAL 모드 왓치독: 유예 시간 안에 새 'g' 라인이 없으면 자동으로 모드 해제(정지).
    if (goal_mode_enabled && (millis() - last_goal_rx_time > GOAL_TIMEOUT_MS)) {
      goal_mode_enabled = false;
      Serial.println("[Teensy] GOAL mode WATCHDOG TIMEOUT (no 'g' line received) - motors stopped. "
                      "Send 'g' to resume.");
    }

    // --- CAN1 제어 ---
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

    // --- CAN2 제어 ---
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

    // 1000ms마다 상태 모니터링 출력
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= LOG_PERIOD) {
      lastPrint = millis();

      const char* mode_str = goal_mode_enabled ? "GOAL" : "DISABLED";
      const char* calib_str = is_calibrated ? "CALIB_OK" : "NO_CALIB";

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        Serial.printf("[CAN1] Slave %d Pos: %.3f rad | Target: %.3f rad (%s / %s) | Trq: %.2f Nm\r\n",
                      SLV_IDS_CAN1[i], slave_pos_can1[i], goal_target_can1[i], mode_str, calib_str,
                      slave_trq_can1[i]);
      }
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        Serial.printf("[CAN2] Slave %d Pos: %.3f rad | Target: %.3f rad (%s / %s) | Trq: %.2f Nm\r\n",
                      SLV_IDS_CAN2[i], slave_pos_can2[i], goal_target_can2[i], mode_str, calib_str,
                      slave_trq_can2[i]);
      }

      if (goal_mode_enabled) {
        Serial.printf("[GOAL] last 'g' %lums ago (timeout %lums)\r\n",
                      (unsigned long)(millis() - last_goal_rx_time), (unsigned long)GOAL_TIMEOUT_MS);
      }

      Serial.println();
    }
  }
}

// -------------------------------------------------------------
// 11. 시리얼 명령 수신 인터럽트
//
// 'd'는 단일 문자 명령. 'g'는 그 뒤로 같은 줄에 4개의 float를 받을 수도 있는(또는 아무것도
// 없이 개행만 올 수도 있는) 라인형 명령이라, 'g'를 본 순간부터 '\n' 또는 '\r'을 볼 때까지
// 별도 버퍼에 누적한다. serialEvent()는 loop() 1회당 1번만 호출되는 Arduino 코어 콜백이므로, 그 안에서
// 그 시점에 도착해 있는 바이트를 전부 소진해야 (1 tick당 1바이트만 읽으면) 멀티바이트 라인의
// 수신 지연이 커진다.
// -------------------------------------------------------------
bool receiving_goal_line = false;
bool goal_line_overflowed = false;
char goal_line_buf[GOAL_LINE_BUF_SIZE];
uint8_t goal_line_len = 0;

void serialEvent() {
  while (Serial.available()) {
    char ch = Serial.read();

    if (!receiving_goal_line) {
      if (ch == 'c' || ch == 'C') {
        calibrateZero();
      }
      else if (ch == 'd' || ch == 'D') {
        goal_mode_enabled = false;
        is_calibrated = false; // full stop invalidates the zero reference -- re-'c' before trusting it again

        for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
          disableMotorCan1(SLV_IDS_CAN1[i]); delay(20);
        }
        for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
          disableMotorCan2(SLV_IDS_CAN2[i]); delay(20);
        }

        Serial.println("[Teensy] Motors Disabled. Send 'c' to re-calibrate, then 'g' to run again.");
      }
      else if (ch == 'g' || ch == 'G') {
        receiving_goal_line = true;
        goal_line_overflowed = false;
        goal_line_len = 0;
      }
      // 그 외 문자(공백, 개행 등)는 명령 시작 문자가 아니므로 무시.
    } else {
      // 'g' 라인 누적 중. '\n'과 '\r' 둘 다 라인 종료로 취급한다 -- screen 등 일부 시리얼
      // 터미널은 Enter에 '\r'만 보내고 '\n'은 절대 안 보내므로, '\r'을 종료자로 인정하지
      // 않으면 receiving_goal_line이 영영 true로 남아 이후 모든 입력(예: 'c', 'd')이
      // 커맨드로 처리되지 못하고 조용히 라인 버퍼에 먹혀버린다 (사용자가 직접 겪은 버그).
      // 진짜 CRLF("\r\n")가 오면 '\r'에서 라인이 끝나고, 뒤따라온 '\n'은
      // !receiving_goal_line 상태에서 c/d/g 어느 것과도 안 맞아 그냥 무시된다 -- 중복 처리 없음.
      if (ch == '\n' || ch == '\r') {
        goal_line_buf[goal_line_len] = '\0';
        if (goal_line_overflowed) {
          Serial.println("[Teensy] GOAL line REJECTED (too long, buffer overflow).");
        } else {
          handleGoalLine(goal_line_buf, goal_line_len);
        }
        receiving_goal_line = false;
        goal_line_overflowed = false;
        goal_line_len = 0;
      } else if (!goal_line_overflowed) {
        if (goal_line_len < GOAL_LINE_BUF_SIZE - 1) {
          goal_line_buf[goal_line_len++] = ch;
        } else {
          goal_line_overflowed = true; // 다음 '\n' 또는 '\r'까지 나머지는 버림 (resync)
        }
      }
    }
  }
}
