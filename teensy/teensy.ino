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
//                payload, no line parsing, no state machine to get stuck in.
//   Ethernet UDP: the continuous goal-position stream. Fire-and-forget datagrams, no
//                "only one process can hold the port" constraint like serial has, no
//                terminator-byte ambiguity -- each packet is a complete message.
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
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2;
const uint8_t SLV_IDS_CAN1[NUM_MOTORS_CAN1] = {11, 12}; // shoulder_yaw, shoulder_roll

const uint8_t NUM_MOTORS_CAN2 = 2;
const uint8_t SLV_IDS_CAN2[NUM_MOTORS_CAN2] = {13, 14}; // shoulder_pitch, elbow_pitch

const uint8_t HOST_ID = 253;

// 위치 추종 게인 (teleop-bi-p-t의 SLV_KP/KD와 동일한 값으로 시작)
const float SLV_KP = 10.0f;
const float SLV_KD = 0.1f;

// GOAL 모드 설정. isaacsim-udp의 WATCHDOG_TIMEOUT_MS와 동일한 500ms -- 유선 시리얼 바이트
// 스트림보다 UDP 쪽이 패킷 유실 특성이 다르므로, 이전 시리얼 버전의 150ms보다 여유를 둔다.
const uint32_t GOAL_TIMEOUT_MS = 500;

// 'd' 직후 이 시간(ms) 동안은 들어오는 UDP 목표 패킷을 전부 무시한다. UDP(목표 스트림)와
// 시리얼('d')은 서로 동기화되지 않는 완전히 별개의 채널이라, 'd'를 보내기 직전에 호스트가
// 이미 보내둔 목표 패킷이 'd' 처리 *이후*에 도착할 수 있다 -- handleGoalPacket()은 원래
// 유효한 패킷이면 무조건 enterGoalModeIfNeeded()를 부르므로, 이 stray 패킷 하나가 방금 건
// 'd'를 무효화하고 (심지어 캘리브레이션도 안 된 채로) 모터를 재활성화해버린다. 실측으로
// 재현된 문제 -- 'd'는 순간적인 킬 스위치여야 하므로, 짧은 무시 구간으로 이 경합을 막는다.
const uint32_t DISABLE_IGNORE_MS = 300;
uint32_t ignore_goal_packets_until_ms = 0;

// 네트워크 설정 (isaacsim-udp 브랜치와 동일 -- 실제로 이 하드웨어에서 검증된 설정을 그대로 사용).
// 직결 케이블 전제라 게이트웨이는 없음(0.0.0.0).
IPAddress staticIP(192, 168, 1, 15);
IPAddress subnetMask(255, 255, 255, 0);
const uint16_t UDP_PORT = 5005;
EthernetUDP udp;

// 'c' 소프트웨어 영점 + 관절별 이동 한계 (teensy-forte teleop-bi 브랜치의 "modify" 커밋에서
// 포팅, raw 모터 라디안 기준으로 재구성) -- 이 펌웨어는 기어비를 전혀 적용하지 않으므로
// (lerobot_robot_forte_arm의 SMOLVLA_GUIDE.md §2 참고) 원본의 관절-공간/기어비 나눗셈은
// 가져오지 않았다. 'c'로 잡은 현재 raw 위치가 그 모터의 영점이 되고, 이후 목표값은
// [영점 + MIN, 영점 + MAX] 범위로 클램프된다.
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
bool goal_mode_enabled = false; // UDP 패킷으로 진입, 'd'로 해제
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
// 주의: 이건 CAN 배선 순서이지 UDP 패킷의 페이로드 순서(yaw,pitch,roll,elbow)와 다르다 --
// handleGoalPacket()이 그 사이를 매핑한다.
float goal_target_can1[NUM_MOTORS_CAN1] = {};
float goal_target_can2[NUM_MOTORS_CAN2] = {};
uint32_t last_goal_rx_time = 0; // 마지막으로 유효한 UDP 목표 패킷을 받은 시각 (GOAL_TIMEOUT_MS 왓치독용)

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
// 9. GOAL 모드 (UDP 전용): 목표 위치 패킷 파싱 및 진입 처리
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

// buf는 NUL로 끝나는 문자열. UDP 패킷 하나당 정확히 "<v0>,<v1>,<v2>,<v3>" 형태를 기대한다 --
// 시리얼 버전의 "값 없이 개행만 오는 bare 명령" 개념은 없다: UDP를 쓰는 쪽은 항상 파이썬
// 스크립트이고, 완전한 4개 값을 보내는 것이 자연스럽다 (현재 위치를 유지하고 싶으면 호출자가
// 그 값을 그대로 다시 보내면 된다).
void handleGoalPacket(char* buf) {
  if (millis() < ignore_goal_packets_until_ms) {
    // 'd' 직후 무시 구간 -- DISABLE_IGNORE_MS 참고. 'd' 이전에 이미 보내진 stray 패킷이
    // 뒤늦게 도착해 방금 건 disable을 무효화하는 걸 막는다.
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
    return; // UDP는 패킷 단위라 시리얼 라인처럼 "덜 온 상태로 갇힐" 위험이 없으므로, 잘못된
             // 패킷에도 왓치독을 갱신해줄 필요가 없다 -- 다음 정상 패킷이 오면 그때 갱신된다.
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
// 10. 메인 루프 구조
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

  Ethernet.begin(staticIP, subnetMask, IPAddress(0, 0, 0, 0)); // 직결 케이블, 게이트웨이 없음
  udp.begin(UDP_PORT);
  Serial.printf("Ethernet up: %d.%d.%d.%d, UDP port %d\r\n",
                staticIP[0], staticIP[1], staticIP[2], staticIP[3], UDP_PORT);

  Serial.println("-> Serial: 'c' to calibrate zero (recommended first), 'd' to disable.");
  Serial.println("-> UDP: send \"<yaw>,<pitch>,<roll>,<elbow>\" (raw rad) to move.");
  controlTimer = 0;
}

void loop() {
  Can1.events();
  Can2.events();

  // UDP 수신 처리 -- 패킷은 네트워크 스택이 이미 통째로 재조립해주므로, 시리얼 버전처럼
  // 바이트 단위로 누적하는 상태 기계가 필요 없다.
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

  // 3.333ms 주기 제어 루프 (300Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // GOAL 모드 왓치독: 유예 시간 안에 새 UDP 목표 패킷이 없으면 자동으로 모드 해제(정지).
    if (goal_mode_enabled && (millis() - last_goal_rx_time > GOAL_TIMEOUT_MS)) {
      goal_mode_enabled = false;
      Serial.println("[Teensy] GOAL mode WATCHDOG TIMEOUT (no UDP packet received) - motors "
                      "stopped. Send a new goal packet to resume.");
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

      // format: radian_after_calibration (original_radian) -- lets you read a position directly
      // against JOINT_LIMIT_MIN/MAX_CAN1/CAN2 (defined relative to zero_offset_*) without doing
      // the subtraction in your head. When NOT calibrated, zero_offset_* is still 0.0 (never
      // written), so both numbers come out equal -- that's expected, not a bug.
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        Serial.printf("[CAN1] Slave %d Pos: %.3f (%.3f) rad | Target: %.3f (%.3f) rad (%s / %s) | Trq: %.2f Nm\r\n",
                      SLV_IDS_CAN1[i],
                      slave_pos_can1[i] - zero_offset_can1[i], slave_pos_can1[i],
                      goal_target_can1[i] - zero_offset_can1[i], goal_target_can1[i],
                      mode_str, calib_str, slave_trq_can1[i]);
      }
      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        Serial.printf("[CAN2] Slave %d Pos: %.3f (%.3f) rad | Target: %.3f (%.3f) rad (%s / %s) | Trq: %.2f Nm\r\n",
                      SLV_IDS_CAN2[i],
                      slave_pos_can2[i] - zero_offset_can2[i], slave_pos_can2[i],
                      goal_target_can2[i] - zero_offset_can2[i], goal_target_can2[i],
                      mode_str, calib_str, slave_trq_can2[i]);
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
// 11. 시리얼 명령 수신 인터럽트 -- 'c'/'d' 뿐, 둘 다 단일 문자 즉시 명령이라 라인 버퍼링이나
// 상태 기계가 전혀 필요 없다 (UDP로 옮기기 전 'g'가 있었을 때와 달리).
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
    // 그 외 문자는 명령이 아니므로 무시.
  }
}
