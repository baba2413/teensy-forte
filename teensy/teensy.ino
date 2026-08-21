#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#include <math.h>

using namespace qindesign::network;

// -------------------------------------------------------------
// UDP 텔레메트리 (읽기 전용, 'teleop-bi-c'에서 추가) -- 기존 Serial.printf() 상태 로그는 그대로
// 두고, 동일한 문자열을 UDP로도 병행 전송한다. 목적은 오직 하나: 지금까지는
// 'lerobot-record' 세션 내내 minicom을 열어두면 시리얼 포트를 파이썬(TeensyLink)과 두고
// 다퉈야 했는데(그래서 실제로는 리셋 구간에만 열고 닫아야 했다), 텔레메트리를 UDP로 옮기면
// 파이썬은 시리얼을 아예 건드리지 않게 되어 minicom을 녹화 시작부터 끝까지 계속 열어둘 수
// 있다. 시리얼 명령('e'/'c'/'d')과 CAN 제어 루프는 이 커밋에서 전혀 건드리지 않았다 --
// 오직 상태 로그 줄의 "출력 목적지"가 하나(Serial)에서 둘(Serial + UDP)로 늘었을 뿐이다.
//
// 하드코딩 유니캐스트(브로드캐스트 아님)로 결정: Teensy IP는 'goal' 브랜치와 동일한 물리
// 하드웨어이므로 그 브랜치의 정적 IP(192.168.1.15)를 그대로 재사용한다. 호스트 목적지 IP는
// RUNBOOK.md Phase 9가 이미 문서화해 둔 host static IP(192.168.1.10, 192.168.1.0/24 직결
// 케이블 서브넷)를 그대로 재사용한다. 포트는 'goal' 브랜치가 쓰는 5005(호스트->Teensy 목표
// 스트림)와 겹치지 않도록, 그리고 방향이 반대(Teensy->호스트 텔레메트리)임을 표시하기 위해
// 5006으로 분리했다. 둘 다 이 파일의 상수이므로 실제 배치 IP가 다르면 여기만 고치면 된다.
// -------------------------------------------------------------
IPAddress teensyStaticIP(192, 168, 1, 15);
IPAddress teensySubnetMask(255, 255, 255, 0);
IPAddress telemetryHostIP(192, 168, 1, 10);
const uint16_t TELEMETRY_UDP_PORT = 5006;
EthernetUDP telemetryUdp;

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

// 슬레이브 모터 위치 추종 게인
const float SLV_KP = 24.0f;
const float SLV_KD = 0.2f;

// 마스터 모터 햅틱 피드백 토크 파라미터 (position-torque 방식)
const float MAX_SAFE_TORQUE = 2.0f;       // 작업자 손목 보호용 최대 피드백 토크 (Nm)
const float K_WALL = 15.0f;               // 슬레이브 한계 가상벽 반발 강도 (Nm/rad)
const float MASTER_KD = 0.0f;             // 마스터 모터 능동 댐핑
const uint32_t WATCHDOG_TIMEOUT_MS = 100; // 통신 끊김 판정 기준 (100ms)

// 슬레이브 충돌 토크 -> 마스터 피드백 변환 파라미터 (민감도 튜닝용)
const float SLAVE_TRQ_DEADZONE = 0.08f;   // 이 값 미만의 슬레이브 토크는 노이즈로 간주해 무시 (Nm)
const float SLAVE_TRQ_GAIN = 0.8f;        // 슬레이브 토크 대비 마스터 피드백 반영 비율
const float MASTER_TRQ_MAX_STEP = 2.0f;   // 2ms 주기당 최대 토크 변화량 (Nm) - 클수록 반응이 즉각적

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
const uint32_t CONTROL_PERIOD_US = 3333;
elapsedMicros controlTimer;

const uint32_t LOG_PERIOD = 1000;

// -------------------------------------------------------------
// 4. 실시간 상태, 오프셋 변수 및 제어 플래그
// -------------------------------------------------------------
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

// 오프셋 계산 전 새 피드백 수신 여부 확인
volatile bool master_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
volatile bool slave_valid_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
volatile bool master_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
volatile bool slave_valid_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};

bool offset_ready_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
bool offset_ready_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};

// 'c' 로깅 전용 영점 (teensy-forte `goal` 브랜치와 동일한 개념이지만 용도가 다름).
// 오직 상태 출력 줄(Serial.printf)에서 빼는 값일 뿐, pos_offset_can1/2나 제어 루프
// (slave_target_raw 계산, operationControlCan1/2 호출)에는 절대 쓰이지 않는다 -- 그쪽은
// 기존에 검증된 raw 좌표계 그대로 유지해야 실제 모터 명령이 안전하다. 'c'는 순수 로깅용.
float zero_offset_master_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {};
float zero_offset_slave_can1[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)]  = {};
float zero_offset_master_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {};
float zero_offset_slave_can2[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)]  = {};
bool is_calibrated = false; // 'c'로 진입 -- 위 zero_offset_*가 현재 유효한지

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
  if (fabsf(filtered_trq) < SLAVE_TRQ_DEADZONE) {
    filtered_trq = 0.0f;
  } else if (filtered_trq > 0) {
    filtered_trq -= SLAVE_TRQ_DEADZONE;
  } else {
    filtered_trq += SLAVE_TRQ_DEADZONE;
  }

  float target_trq = -1.0f * filtered_trq * SLAVE_TRQ_GAIN;

  float trq_delta = target_trq - state.current_output_trq;

  if (trq_delta > MASTER_TRQ_MAX_STEP) {
    target_trq = state.current_output_trq + MASTER_TRQ_MAX_STEP;
  } else if (trq_delta < -MASTER_TRQ_MAX_STEP) {
    target_trq = state.current_output_trq - MASTER_TRQ_MAX_STEP;
  }

  state.current_output_trq = target_trq;
  return target_trq;
}

// Serial.print()된 것과 완전히 동일한 문자열을 그대로 UDP로도 보낸다 (포맷 문자열을 두 번
// 유지할 필요 없도록 호출부에서 snprintf로 한 번만 렌더링한 버퍼를 넘겨받는다). Fire-and-forget:
// UDP이므로 실패해도 재시도하지 않고, 시리얼 로그 자체에는 영향이 없다.
void sendTelemetryLine(const char* line, size_t len) {
  if (len == 0) return;
  telemetryUdp.beginPacket(telemetryHostIP, TELEMETRY_UDP_PORT);
  telemetryUdp.write((const uint8_t*)line, len);
  telemetryUdp.endPacket();
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
// 9b. 'c' 로깅 전용 영점 캘리브레이션
//
// setupOffset()('e' 전용)과는 완전히 독립적이다. 모터를 켜거나 끄지 않고, pos_offset_can1/2를
// 건드리지 않고, 제어 루프에 아무 영향도 주지 않는다 -- 오직 아래 상태 출력 줄이 무엇을
// 찍을지에만 관여한다. master_valid_can1/2, slave_valid_can1/2는 rxCallbackCan1/2에서 실제
// 피드백을 받을 때마다 계속 true로 유지되므로(마지막 'e' 시도의 setupOffset()이 초기화하지
// 않는 한) 새 상태를 따로 추적할 필요 없이 그대로 재사용한다.
// -------------------------------------------------------------
void calibrateZero() {
  Serial.println("[Teensy] Calibrating zero offset (logging only) from current pose...");

  bool all_ok = true;
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    if (!master_valid_can1[i] || !slave_valid_can1[i]) {
      Serial.printf("[CALIB CAN1] Pair %d: no CAN feedback yet -- FAILED\r\n", i + 1);
      all_ok = false;
      continue;
    }
    zero_offset_master_can1[i] = master_pos_can1[i];
    zero_offset_slave_can1[i] = slave_pos_can1[i];
    Serial.printf("[CALIB CAN1] Pair %d zero set (Master %d raw was %.3f, Slave %d raw was %.3f)\r\n",
                  i + 1, MST_IDS_CAN1[i], zero_offset_master_can1[i],
                  SLV_IDS_CAN1[i], zero_offset_slave_can1[i]);
  }
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    if (!master_valid_can2[i] || !slave_valid_can2[i]) {
      Serial.printf("[CALIB CAN2] Pair %d: no CAN feedback yet -- FAILED\r\n", i + 1 + NUM_MOTORS_CAN1);
      all_ok = false;
      continue;
    }
    zero_offset_master_can2[i] = master_pos_can2[i];
    zero_offset_slave_can2[i] = slave_pos_can2[i];
    Serial.printf("[CALIB CAN2] Pair %d zero set (Master %d raw was %.3f, Slave %d raw was %.3f)\r\n",
                  i + 1 + NUM_MOTORS_CAN1, MST_IDS_CAN2[i], zero_offset_master_can2[i],
                  SLV_IDS_CAN2[i], zero_offset_slave_can2[i]);
  }

  is_calibrated = all_ok;
  if (all_ok) {
    Serial.println("[Teensy] Calibration complete. Status log now reads relative to this pose "
                    "(logging only -- does not affect motor control).");
  } else {
    Serial.println("[Teensy] Calibration INCOMPLETE -- missing feedback for some motor(s). NOT calibrated.");
  }
}

// -------------------------------------------------------------
// 10. 메인 루프 구조
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
  Serial.println("-> Send 'e' to enable motors. Send 'c' to zero the status log (logging only, "
                  "does not affect control).");

  Ethernet.begin(teensyStaticIP, teensySubnetMask, IPAddress(0, 0, 0, 0)); // 직결 케이블, 게이트웨이 없음
  telemetryUdp.begin(TELEMETRY_UDP_PORT);
  Serial.printf("Ethernet up: %d.%d.%d.%d | Telemetry UDP -> %d.%d.%d.%d:%d\r\n",
                teensyStaticIP[0], teensyStaticIP[1], teensyStaticIP[2], teensyStaticIP[3],
                telemetryHostIP[0], telemetryHostIP[1], telemetryHostIP[2], telemetryHostIP[3],
                TELEMETRY_UDP_PORT);

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
        float limit_penetration = 0.0f;

        // 하드웨어 프로토콜 한계(-12.4~12.4 rad) 가상벽
        if (slave_target_raw < RAW_LIMIT_MIN || slave_target_raw > RAW_LIMIT_MAX) {
          limit_penetration += (slave_target_raw > RAW_LIMIT_MAX) ? (slave_target_raw - RAW_LIMIT_MAX) : (slave_target_raw - RAW_LIMIT_MIN);

          static uint32_t lastRawLimitWarnMs[SAFE_BUF_SIZE(NUM_MOTORS_CAN1)] = {0};
          if (millis() - lastRawLimitWarnMs[i] > 200) {
            lastRawLimitWarnMs[i] = millis();
            Serial.printf("[CAN1 RAW LIMIT] Pair %d Slave %d blocked by hardware limit: target %.3f rad clamped to %.3f rad (%.1f~%.1f)\r\n",
                          i + 1, SLV_IDS_CAN1[i], slave_target_raw,
                          slave_target_raw < RAW_LIMIT_MIN ? RAW_LIMIT_MIN : RAW_LIMIT_MAX,
                          RAW_LIMIT_MIN, RAW_LIMIT_MAX);
          }

          if (slave_target_raw < RAW_LIMIT_MIN) slave_target_raw = RAW_LIMIT_MIN;
          if (slave_target_raw > RAW_LIMIT_MAX) slave_target_raw = RAW_LIMIT_MAX;
        }

        operationControlCan1(SLV_IDS_CAN1[i], 0.0f, slave_target_raw, 0.0f, slave_kp, slave_kd);

        // 3. 마스터 피드백 토크 연산 (슬레이브 토크 기반 햅틱 피드백 + 한계 가상벽 반력)
        float master_trq = processSlaveTorqueSafety(slave_trq_can1[i], mst_trq_state_can1[i]);
        if (limit_penetration != 0.0f) {
          master_trq -= K_WALL * limit_penetration;
        }
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
        float limit_penetration = 0.0f;

        // 하드웨어 프로토콜 한계(-12.4~12.4 rad) 가상벽
        if (slave_target_raw < RAW_LIMIT_MIN || slave_target_raw > RAW_LIMIT_MAX) {
          limit_penetration += (slave_target_raw > RAW_LIMIT_MAX) ? (slave_target_raw - RAW_LIMIT_MAX) : (slave_target_raw - RAW_LIMIT_MIN);

          static uint32_t lastRawLimitWarnMs[SAFE_BUF_SIZE(NUM_MOTORS_CAN2)] = {0};
          if (millis() - lastRawLimitWarnMs[i] > 200) {
            lastRawLimitWarnMs[i] = millis();
            Serial.printf("[CAN2 RAW LIMIT] Pair %d Slave %d blocked by hardware limit: target %.3f rad clamped to %.3f rad (%.1f~%.1f)\r\n",
                          i + 1 + NUM_MOTORS_CAN1, SLV_IDS_CAN2[i], slave_target_raw,
                          slave_target_raw < RAW_LIMIT_MIN ? RAW_LIMIT_MIN : RAW_LIMIT_MAX,
                          RAW_LIMIT_MIN, RAW_LIMIT_MAX);
          }

          if (slave_target_raw < RAW_LIMIT_MIN) slave_target_raw = RAW_LIMIT_MIN;
          if (slave_target_raw > RAW_LIMIT_MAX) slave_target_raw = RAW_LIMIT_MAX;
        }

        operationControlCan2(SLV_IDS_CAN2[i], 0.0f, slave_target_raw, 0.0f, slave_kp, slave_kd);

        float master_trq = processSlaveTorqueSafety(slave_trq_can2[i], mst_trq_state_can2[i]);
        if (limit_penetration != 0.0f) {
          master_trq -= K_WALL * limit_penetration;
        }
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
    if (millis() - lastPrint >= LOG_PERIOD) {
      lastPrint = millis();

      // 'c'로 캘리브레이션된 경우 로그에는 영점 기준 상대값을 찍는다 (raw 모터 좌표계
      // 그대로인 pos_offset_can1/2, 실제 제어 루프와는 무관 -- 오직 여기, 사람이/파이썬이
      // 읽는 이 줄에만 영향). 캘리브레이션 전이면 zero_offset_*가 0.0이라 raw 그대로 찍힌다.
      // 포맷 자체(숫자 하나)는 캘리브레이션 이전과 동일하게 유지했다 -- teensy_link.py의
      // 정규식이 바뀔 필요가 없도록.
      //
      // 각 줄을 snprintf로 한 번만 버퍼에 렌더링한 뒤 Serial과 UDP 양쪽에 그대로 내보낸다 --
      // 포맷 문자열을 두 곳에 따로 유지하지 않기 위함(위 주석에서 언급한 '포맷 불변' 요구와
      // 동일한 이유).
      char line_buf[160];
      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        int len = snprintf(line_buf, sizeof(line_buf),
                      "[CAN1] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad | Offset: %.3f (%s) | SLV Trq: %.2f Nm | MST FB: %.2f Nm\r\n",
                      MST_IDS_CAN1[i], master_pos_can1[i] - zero_offset_master_can1[i],
                      SLV_IDS_CAN1[i], slave_pos_can1[i] - zero_offset_slave_can1[i],
                      pos_offset_can1[i],
                      (system_enabled && offset_ready_can1[i]) ? "ENABLED" : "DISABLED",
                      slave_trq_can1[i], mst_trq_state_can1[i].current_output_trq);
        Serial.print(line_buf);
        if (len > 0) sendTelemetryLine(line_buf, (size_t)len);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        int len = snprintf(line_buf, sizeof(line_buf),
                      "[CAN2] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad | Offset: %.3f (%s) | SLV Trq: %.2f Nm | MST FB: %.2f Nm\r\n",
                      MST_IDS_CAN2[i], master_pos_can2[i] - zero_offset_master_can2[i],
                      SLV_IDS_CAN2[i], slave_pos_can2[i] - zero_offset_slave_can2[i],
                      pos_offset_can2[i],
                      (system_enabled && offset_ready_can2[i]) ? "ENABLED" : "DISABLED",
                      slave_trq_can2[i], mst_trq_state_can2[i].current_output_trq);
        Serial.print(line_buf);
        if (len > 0) sendTelemetryLine(line_buf, (size_t)len);
      }

      Serial.println();
    }
  }
}

// -------------------------------------------------------------
// 11. 시리얼 명령 수신 인터럽트
// -------------------------------------------------------------
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == 'c' || ch == 'C') {
      calibrateZero();
    }
    else if (ch == 'e' || ch == 'E') {
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