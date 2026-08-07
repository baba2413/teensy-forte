#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t NUM_MOTORS_CAN1 = 2;
const uint8_t MST_IDS_CAN1[NUM_MOTORS_CAN1] = {1, 2};
const uint8_t SLV_IDS_CAN1[NUM_MOTORS_CAN1] = {11, 12};

const uint8_t NUM_MOTORS_CAN2 = 2;
const uint8_t MST_IDS_CAN2[NUM_MOTORS_CAN2] = {3, 4};
const uint8_t SLV_IDS_CAN2[NUM_MOTORS_CAN2] = {13, 14};

const uint8_t HOST_ID = 253;

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
// 3. 제어 주기 설정 (텔레오퍼레이션용 500 Hz / dt = 0.002초)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000;
elapsedMicros controlTimer;

// -------------------------------------------------------------
// 4. 실시간 상태 및 오프셋 변수
// -------------------------------------------------------------
volatile float master_pos_can1[NUM_MOTORS_CAN1] = {0.0f, 0.0f};
volatile float slave_pos_can1[NUM_MOTORS_CAN1]  = {0.0f, 0.0f};
float pos_offset_can1[NUM_MOTORS_CAN1]          = {0.0f, 0.0f};

volatile float master_pos_can2[NUM_MOTORS_CAN2] = {0.0f, 0.0f};
volatile float slave_pos_can2[NUM_MOTORS_CAN2]  = {0.0f, 0.0f};
float pos_offset_can2[NUM_MOTORS_CAN2]          = {0.0f, 0.0f};

// -------------------------------------------------------------
// 5. 데이터 스케일링 헬퍼 함수
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

// -------------------------------------------------------------
// 6. Robstride CAN1 송신 제어 함수군
// -------------------------------------------------------------
void enableMotorCan1(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  mode_msg.len = 8;

  mode_msg.buf[0] = 0x05;
  mode_msg.buf[1] = 0x70;
  mode_msg.buf[2] = 0x00;
  mode_msg.buf[3] = 0x00;
  mode_msg.buf[4] = 0x00;
  mode_msg.buf[5] = 0x00;
  mode_msg.buf[6] = 0x00;
  mode_msg.buf[7] = 0x00;

  Can1.write(mode_msg);
  delay(20);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  for (int i = 0; i < 8; i++) enable_msg.buf[i] = 0;

  Can1.write(enable_msg);
  Serial.printf("[Teensy CAN1] Motor %d Enabled successfully!\r\n", motor_id);
}

void disableMotorCan1(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;

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
// 7. Robstride CAN2 송신 제어 함수군
// -------------------------------------------------------------
void enableMotorCan2(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  mode_msg.len = 8;

  mode_msg.buf[0] = 0x05;
  mode_msg.buf[1] = 0x70;
  mode_msg.buf[2] = 0x00;
  mode_msg.buf[3] = 0x00;
  mode_msg.buf[4] = 0x00;
  mode_msg.buf[5] = 0x00;
  mode_msg.buf[6] = 0x00;
  mode_msg.buf[7] = 0x00;

  Can2.write(mode_msg);
  delay(20);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  for (int i = 0; i < 8; i++) enable_msg.buf[i] = 0;

  Can2.write(enable_msg);
  Serial.printf("[Teensy CAN2] Motor %d Enabled successfully!\r\n", motor_id);
}

void disableMotorCan2(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4UL << 24) | ((uint32_t)HOST_ID << 8) | motor_id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;

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
    uint16_t p_raw = ((uint16_t)msg.buf[0] << 8) | msg.buf[1];
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);

    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      if (motor_id == MST_IDS_CAN1[i]) {
        master_pos_can1[i] = current_pos;
        break;
      } else if (motor_id == SLV_IDS_CAN1[i]) {
        slave_pos_can1[i] = current_pos;
        break;
      }
    }
  }
}

void rxCallbackCan2(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;

  if (mode == 2) {
    uint8_t motor_id = (msg.id >> 8) & 0xFF;
    uint16_t p_raw = ((uint16_t)msg.buf[0] << 8) | msg.buf[1];
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);

    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      if (motor_id == MST_IDS_CAN2[i]) {
        master_pos_can2[i] = current_pos;
        break;
      } else if (motor_id == SLV_IDS_CAN2[i]) {
        slave_pos_can2[i] = current_pos;
        break;
      }
    }
  }
}

// -------------------------------------------------------------
// 9. 초기 위치 오프셋 측정 헬퍼 함수
// -------------------------------------------------------------
void setupOffset() {
  // 모터 피드백 유도를 위한 Dummy 명령 전송
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    operationControlCan1(MST_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    operationControlCan1(SLV_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    operationControlCan2(MST_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    operationControlCan2(SLV_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  // 피드백 데이터 수신 대기 (100ms)
  uint32_t waitStart = millis();
  while (millis() - waitStart < 100) {
    Can1.events();
    Can2.events();
  }

  // CAN1 각 쌍별 초기 오프셋 계산
  for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
    pos_offset_can1[i] = slave_pos_can1[i] - master_pos_can1[i];
    Serial.printf("[SETUP CAN1] Pair %d Offset Calculated: %.3f rad (Master %d: %.3f, Slave %d: %.3f)\r\n",
                  i + 1, pos_offset_can1[i], MST_IDS_CAN1[i], master_pos_can1[i],
                  SLV_IDS_CAN1[i], slave_pos_can1[i]);
  }

  // CAN2 각 쌍별 초기 오프셋 계산
  for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
    pos_offset_can2[i] = slave_pos_can2[i] - master_pos_can2[i];
    Serial.printf("[SETUP CAN2] Pair %d Offset Calculated: %.3f rad (Master %d: %.3f, Slave %d: %.3f)\r\n",
                  i + 3, pos_offset_can2[i], MST_IDS_CAN2[i], master_pos_can2[i],
                  SLV_IDS_CAN2[i], slave_pos_can2[i]);
  }
}

// -------------------------------------------------------------
// 10. 메인 루프 구조
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== Robstride Unidirectional Teleoperation with Initial Offset ===");
  Serial.println("CAN1: 1,2 -> 11,12");
  Serial.println("CAN2: 3,4 -> 13,14");

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

  // 1초(1000ms)마다 내장 LED 반짝임 (토글)
  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= 1000) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 2ms 주기 제어 루프 (500Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    float slave_kp = 25.0f;
    float slave_kd = 1.0f;

    // CAN1: 마스터 1, 2 / 슬레이브 11, 12
    for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
      operationControlCan1(MST_IDS_CAN1[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      float slave_target_pos = master_pos_can1[i] + pos_offset_can1[i];
      operationControlCan1(SLV_IDS_CAN1[i], 0.0f, slave_target_pos, 0.0f, slave_kp, slave_kd);
    }

    // CAN2: 마스터 3, 4 / 슬레이브 13, 14
    for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
      operationControlCan2(MST_IDS_CAN2[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      float slave_target_pos = master_pos_can2[i] + pos_offset_can2[i];
      operationControlCan2(SLV_IDS_CAN2[i], 0.0f, slave_target_pos, 0.0f, slave_kp, slave_kd);
    }

    // 500ms마다 상태 모니터링 출력
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();

      for (int i = 0; i < NUM_MOTORS_CAN1; i++) {
        float slave_target_pos = master_pos_can1[i] + pos_offset_can1[i];
        Serial.printf("[CAN1] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad (Target: %.3f)\r\n",
                      MST_IDS_CAN1[i], master_pos_can1[i],
                      SLV_IDS_CAN1[i], slave_pos_can1[i], slave_target_pos);
      }

      for (int i = 0; i < NUM_MOTORS_CAN2; i++) {
        float slave_target_pos = master_pos_can2[i] + pos_offset_can2[i];
        Serial.printf("[CAN2] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad (Target: %.3f)\r\n",
                      MST_IDS_CAN2[i], master_pos_can2[i],
                      SLV_IDS_CAN2[i], slave_pos_can2[i], slave_target_pos);
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