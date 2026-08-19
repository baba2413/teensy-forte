#include <Arduino.h>
#include <FlexCAN_T4.h>

// -------------------------------------------------------------
// 여러 모터 동시 확인용 테스트 코드
// - 'e' : 모든 모터 enable + 각자의 현재 위치 읽기 -> 그 위치부터 천천히
//         음의 방향으로 동일한 명령(작은 회전)을 동시에 송신 시작
// - 'd' : 모든 모터 disable + 회전 정지
// -------------------------------------------------------------

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const uint8_t MOTOR_IDS[] = {11, 13};
const uint8_t NUM_MOTORS = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);
const uint8_t HOST_ID = 253;

// Robstride 프로토콜 물리적 한계값 (MIT 모드 패킹용)
const float P_MIN = -12.5f;   const float P_MAX = 12.5f;
const float V_MIN = -45.0f;   const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;  const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;   const float T_MAX = 18.0f;

// 제어 게인 (확인용이므로 약하게)
const float KP = 10.0f;
const float KD = 1.0f;

// 천천히 음의 방향으로 회전하는 속도 (raw 위치 단위, rad/s)
const float ROTATE_SPEED = 0.05f;

// 시작 위치 기준 안전 이동 범위 (raw 위치 단위, rad). 이 이상은 더 가지 않고 멈춘다.
const float ROTATE_RANGE_LIMIT = 3.0f;

const uint32_t CONTROL_PERIOD_US = 20000; // 50Hz
elapsedMicros controlTimer;

uint32_t LOG_PERIOD_MS = 500;
uint32_t lastLogMs = 0;

struct MotorFeedback {
  float pos;
  float vel;
  float torque;
  bool updated;
};
MotorFeedback fb[NUM_MOTORS];

bool running = false;
float start_pos[NUM_MOTORS];
float target_pos[NUM_MOTORS];

// -------------------------------------------------------------
// 진단용 상태 (회전 안 하는 원인 분석용, 모터별)
// -------------------------------------------------------------
volatile uint32_t rx_total_count = 0;                    // 수신된 모든 CAN 프레임 수 (mode/ID 무관)
volatile uint32_t rx_mode2_count = 0;                     // mode==2(피드백) 프레임 수 (ID 무관)
volatile uint32_t rx_matched_count[NUM_MOTORS] = {0};      // mode==2 이면서 해당 motor_id인 프레임 수
volatile uint32_t last_rx_id = 0;                          // 가장 최근 수신된 프레임의 raw CAN ID (mode/ID 무관)
volatile uint32_t last_rx_ms = 0;                          // 가장 최근 수신 시각

volatile uint8_t fault_bits[NUM_MOTORS] = {0};             // Type2 피드백의 fault 비트 (bits 21-16, 6bit)
volatile uint8_t motor_run_mode[NUM_MOTORS] = {0};         // Type2 피드백의 모터 상태 (bits 23-22): 0=Reset,1=Cali,2=Run

uint32_t tx_control_count = 0;             // operationControl() 호출(=CAN TX 시도) 횟수
uint32_t last_can_id_tx = 0;
uint8_t last_can_buf_tx[8] = {0};

int motorIndex(uint8_t motor_id) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (MOTOR_IDS[i] == motor_id) return i;
  }
  return -1;
}

void printFaultBits(uint8_t motor_id, uint8_t bits) {
  Serial.printf("[FAULT] Motor %d | Raw: 0x%02X", motor_id, bits);
  if (bits == 0) {
    Serial.println(" | CLEARED");
    return;
  }
  if (bits & (1 << 0)) Serial.print(" | UNDERVOLTAGE");
  if (bits & (1 << 1)) Serial.print(" | THREE_PHASE_OVERCURRENT");
  if (bits & (1 << 2)) Serial.print(" | OVERTEMPERATURE");
  if (bits & (1 << 3)) Serial.print(" | MAGNETIC_ENCODER_FAULT");
  if (bits & (1 << 4)) Serial.print(" | STALL_OVERLOAD");
  if (bits & (1 << 5)) Serial.print(" | UNCALIBRATED");
  Serial.println();
}

uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

void rxCallback(const CAN_message_t &msg) {
  // 진단용: mode/ID와 무관하게 CAN 버스에 뭔가 들어오고 있는지부터 확인
  rx_total_count++;
  last_rx_id = msg.id;
  last_rx_ms = millis();

  uint8_t mode = (msg.id >> 24) & 0x1F;
  if (mode != 2) return;
  rx_mode2_count++;

  uint8_t motor_id = (msg.id >> 8) & 0xFF;
  // Robstride Type2 피드백 ID 레이아웃: bits23-22=motor_mode, bits21-16=fault, bits15-8=motor_id
  uint8_t new_fault = (msg.id >> 16) & 0x3F;
  uint8_t new_run_mode = (msg.id >> 22) & 0x03;

  int idx = motorIndex(motor_id);
  if (idx < 0) return;
  rx_matched_count[idx]++;

  fault_bits[idx] = new_fault;
  motor_run_mode[idx] = new_run_mode;

  uint16_t p_raw = (msg.buf[0] << 8) | msg.buf[1];
  uint16_t v_raw = (msg.buf[2] << 8) | msg.buf[3];
  uint16_t t_raw = (msg.buf[4] << 8) | msg.buf[5];

  fb[idx].pos = uintToFloat(p_raw, P_MIN, P_MAX);
  fb[idx].vel = uintToFloat(v_raw, V_MIN, V_MAX);
  fb[idx].torque = uintToFloat(t_raw, T_MIN, T_MAX);
  fb[idx].updated = true;
}

void enableMotor(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12 << 24) | (HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  for (int i = 0; i < 8; i++) mode_msg.buf[i] = 0;
  mode_msg.buf[0] = 0x05; mode_msg.buf[1] = 0x70; // Run Mode 주소 (0x7005)
  mode_msg.buf[4] = 0x00; // MIT 운전 제어 모드 (0)
  Can0.write(mode_msg);
  delay(50);

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3 << 24) | (HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  for (int i = 0; i < 8; i++) enable_msg.buf[i] = 0;
  Can0.write(enable_msg);
  Serial.printf("[Teensy] Motor ID %d Enabled.\r\n", motor_id);
}

void disableMotor(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4 << 24) | (HOST_ID << 8) | motor_id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;
  Can0.write(msg);
  Serial.printf("[Teensy] Motor ID %d Disabled.\r\n", motor_id);
}

void operationControl(uint8_t motor_id, float feed_forward, float pos, float vel, float kp, float kd) {
  uint16_t p_int  = floatToUint(pos,          P_MIN, P_MAX,  16);
  uint16_t v_int  = floatToUint(vel,          V_MIN, V_MAX,  16);
  uint16_t kp_int = floatToUint(kp,           0.0f,  KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd,           0.0f,  KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX,  16);

  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (1 << 24) | (t_int << 8) | motor_id;
  msg.len = 8;
  msg.buf[0] = (p_int >> 8) & 0xFF;  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF; msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF; msg.buf[7] = kd_int & 0xFF;
  Can0.write(msg);

  tx_control_count++;
  last_can_id_tx = msg.id;
  memcpy(last_can_buf_tx, msg.buf, 8);
}

// enable 직후: 모든 모터에 무저항(zero-gain) 프로브 프레임을 보내 현재 위치를 읽는다.
void readAllCurrentPos() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    fb[i].updated = false;
    operationControl(MOTOR_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  uint32_t waitStart = millis();
  while (millis() - waitStart < 100) {
    Can0.events();
  }

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    uint8_t motor_id = MOTOR_IDS[i];
    if (!fb[i].updated) {
      Serial.printf("[WARN] Motor %d 현재 위치 피드백을 받지 못했습니다. 0으로 시작합니다.\r\n", motor_id);
      Serial.printf("       rx_total=%lu rx_mode2=%lu rx_matched=%lu (motor_id=%d 프레임이 안 들어오면 배선/보드레이트/ID 문제)\r\n",
                    (unsigned long)rx_total_count, (unsigned long)rx_mode2_count,
                    (unsigned long)rx_matched_count[i], motor_id);
      start_pos[i] = 0.0f;
    } else {
      Serial.printf("[Teensy] Motor %d 현재 위치 확인됨 = %.4f rad. run_mode=%d fault=0x%02X\r\n",
                    motor_id, fb[i].pos, motor_run_mode[i], fault_bits[i]);
      printFaultBits(motor_id, fault_bits[i]);
      start_pos[i] = fb[i].pos;
    }
    target_pos[i] = start_pos[i];
  }
}

void setup() {
  Serial.begin(115200);

  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps
  Can0.setMaxMB(64);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();
  Can0.enableMBInterrupts();
  Can0.onReceive(rxCallback);

  Serial.println("==================================================");
  Serial.printf("[Teensy] 다중 모터(%d개) 동시 확인용 코드: ", NUM_MOTORS);
  for (uint8_t i = 0; i < NUM_MOTORS; i++) Serial.printf("%d ", MOTOR_IDS[i]);
  Serial.println();
  Serial.println("e = enable (모든 모터가 각자의 현재 위치부터 동일한 명령으로 천천히 음의 방향 회전 시작)");
  Serial.println("d = disable (모든 모터 정지)");
  Serial.println("==================================================");

  controlTimer = 0;
}

void loop() {
  Can0.events();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'e' || ch == 'E') {
      for (uint8_t i = 0; i < NUM_MOTORS; i++) enableMotor(MOTOR_IDS[i]);
      delay(20);
      readAllCurrentPos();
      running = true;
      Serial.println("[Teensy] 모든 모터가 동일한 명령으로 음의 방향 회전을 시작합니다.");
    } else if (ch == 'd' || ch == 'D') {
      running = false;
      for (uint8_t i = 0; i < NUM_MOTORS; i++) disableMotor(MOTOR_IDS[i]);
    }
  }

  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    if (running) {
      float step = ROTATE_SPEED * (CONTROL_PERIOD_US / 1000000.0f);
      for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        float min_target = start_pos[i] - ROTATE_RANGE_LIMIT;
        if (target_pos[i] > min_target) {
          target_pos[i] -= step;
          if (target_pos[i] < min_target) target_pos[i] = min_target;
        }
        operationControl(MOTOR_IDS[i], 0.0f, target_pos[i], 0.0f, KP, KD);
      }
    }

    if (millis() - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = millis();
      Serial.printf("[STATUS] running=%d\r\n", running);
      for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Serial.printf("  Motor %2d: target=%.4f fb_pos=%.4f fb_vel=%.4f fb_trq=%.4f %s\r\n",
                      MOTOR_IDS[i], target_pos[i], fb[i].pos, fb[i].vel, fb[i].torque,
                      fb[i].updated ? "FB_OK" : "FB_NONE");
      }
    }
  }
}
