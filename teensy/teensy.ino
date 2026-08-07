#include <Arduino.h>
#include <FlexCAN_T4.h>

// -------------------------------------------------------------
// 1. 모터 ID 리스트 및 설정
// -------------------------------------------------------------
const uint8_t MOTOR_IDS[] = {1,2,3,4,11,12,13,14}; 
const size_t NUM_MOTORS = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

const uint32_t PRINT_PERIOD_MS = 2000; // 시리얼 출력 및 LED 반전 주기 (100ms = 0.1초)

// Teensy 4.0/4.1 CAN1 사용 설정
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// -------------------------------------------------------------
// 2. Robstride 프로토콜 물리적 한계값 (스케일 변환용)
// -------------------------------------------------------------
const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;

// -------------------------------------------------------------
// 3. 모터 상태 저장 구조체 및 글로벌 변수
// -------------------------------------------------------------
struct MotorStatus {
  uint8_t id;
  float pos;       // 위치 (rad)
  float vel;       // 속도 (rad/s)
  float torque;    // 토크 (Nm)
  bool updated;    // 피드백 수신 여부
};

MotorStatus motorStates[NUM_MOTORS];
elapsedMillis printTimer;
bool ledState = false; // 내장 LED 토글용 상태 변수

// -------------------------------------------------------------
// 4. 변환 및 헬퍼 함수
// -------------------------------------------------------------
float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

int getMotorIndex(uint8_t id) {
  for (size_t i = 0; i < NUM_MOTORS; i++) {
    if (MOTOR_IDS[i] == id) return i;
  }
  return -1;
}

void requestMotorStatus(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (1 << 24) | motor_id;
  msg.len = 8;
  
  uint16_t p_int = floatToUint(0.0f, P_MIN, P_MAX, 16);
  uint16_t v_int = floatToUint(0.0f, V_MIN, V_MAX, 16);
  
  msg.buf[0] = (p_int >> 8) & 0xFF;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;
  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;

  Can1.write(msg);
}

// -------------------------------------------------------------
// 5. CAN 수신 콜백 함수 (피드백 데이터 파싱)
// -------------------------------------------------------------
void rxCallback(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;

  if (mode == 2) {
    uint8_t motor_id = (msg.id >> 8) & 0xFF;
    int idx = getMotorIndex(motor_id);

    if (idx < 0) {
      motor_id = msg.id & 0xFF;
      idx = getMotorIndex(motor_id);
    }

    if (idx >= 0) {
      uint16_t p_raw = (msg.buf[0] << 8) | msg.buf[1];
      uint16_t v_raw = (msg.buf[2] << 8) | msg.buf[3];
      uint16_t t_raw = (msg.buf[4] << 8) | msg.buf[5];

      motorStates[idx].pos = uintToFloat(p_raw, P_MIN, P_MAX);
      motorStates[idx].vel = uintToFloat(v_raw, V_MIN, V_MAX);
      motorStates[idx].torque = uintToFloat(t_raw, T_MIN, T_MAX);
      motorStates[idx].updated = true;
    }
  }
}

// -------------------------------------------------------------
// 6. 메인 로직
// -------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // 내장 LED 핀 출력 설정

  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("=== Robstride Multi-Motor Status Monitor ===");

  for (size_t i = 0; i < NUM_MOTORS; i++) {
    motorStates[i].id = MOTOR_IDS[i];
    motorStates[i].pos = 0.0f;
    motorStates[i].vel = 0.0f;
    motorStates[i].torque = 0.0f;
    motorStates[i].updated = false;
  }

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(64);
  Can1.setMBFilter(ACCEPT_ALL);
  Can1.distribute();
  Can1.enableMBInterrupts();
  Can1.onReceive(rxCallback);

  Serial.println("CAN bus initialized. Monitoring motor states...");
}

void loop() {
  Can1.events();

  if (printTimer >= PRINT_PERIOD_MS) {
    printTimer -= PRINT_PERIOD_MS;

    // 내장 LED 상태 토글 (켜짐 <-> 꺼짐)
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);

    // 1. 등록된 모든 모터에 상태 피드백 요청 프레임 전송
    for (size_t i = 0; i < NUM_MOTORS; i++) {
      requestMotorStatus(MOTOR_IDS[i]);
    }

    // 2. 수신된 최신 모터 상태 시리얼 출력
    Serial.println("\n--- Motor Status ---");
    for (size_t i = 0; i < NUM_MOTORS; i++) {
      Serial.printf("[ID %2d] Pos: %7.3f rad | Vel: %6.2f rad/s | Trq: %6.2f Nm | State: %s\r\n",
                    motorStates[i].id,
                    motorStates[i].pos,
                    motorStates[i].vel,
                    motorStates[i].torque,
                    motorStates[i].updated ? "OK" : "NO_RESPONSE");
      
      motorStates[i].updated = false;
    }
  }
}