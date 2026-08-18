#include <Arduino.h>
#include <FlexCAN_T4.h>

// -------------------------------------------------------------
// shoulder_pitch (CAN ID 13) 단독 확인용 테스트 코드
// - 'e' : 모터 enable + 현재 위치 읽기 -> 그 위치부터 천천히 음의 방향으로 회전 시작
// - 'd' : 모터 disable + 회전 정지
// -------------------------------------------------------------

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const uint8_t MOTOR_ID = 14;   // shoulder_pitch
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
MotorFeedback fb = {0, 0, 0, false};

bool running = false;
float start_pos = 0.0f;
float target_pos = 0.0f;

// -------------------------------------------------------------
// 진단용 상태 (회전 안 하는 원인 분석용)
// -------------------------------------------------------------
volatile uint32_t rx_total_count = 0;      // 수신된 모든 CAN 프레임 수 (mode/ID 무관)
volatile uint32_t rx_mode2_count = 0;      // mode==2(피드백) 프레임 수 (ID 무관)
volatile uint32_t rx_matched_count = 0;    // mode==2 이면서 motor_id==MOTOR_ID인 프레임 수
volatile uint32_t last_rx_id = 0;          // 가장 최근 수신된 프레임의 raw CAN ID (mode/ID 무관)
volatile uint32_t last_rx_ms = 0;          // 가장 최근 수신 시각

volatile uint8_t fault_bits = 0;           // Type2 피드백의 fault 비트 (bits 21-16, 6bit)
volatile uint8_t motor_run_mode = 0;       // Type2 피드백의 모터 상태 (bits 23-22): 0=Reset,1=Cali,2=Run
volatile bool fault_changed = false;

uint32_t tx_control_count = 0;             // operationControl() 호출(=CAN TX 시도) 횟수
uint32_t last_can_id_tx = 0;
uint8_t last_can_buf_tx[8] = {0};

void printFaultBits(uint8_t bits) {
  Serial.printf("[FAULT] Motor %d | Raw: 0x%02X", MOTOR_ID, bits);
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

  if (motor_id != MOTOR_ID) return;
  rx_matched_count++;

  if (new_fault != fault_bits) {
    fault_bits = new_fault;
    fault_changed = true;
  }
  motor_run_mode = new_run_mode;

  uint16_t p_raw = (msg.buf[0] << 8) | msg.buf[1];
  uint16_t v_raw = (msg.buf[2] << 8) | msg.buf[3];
  uint16_t t_raw = (msg.buf[4] << 8) | msg.buf[5];

  fb.pos = uintToFloat(p_raw, P_MIN, P_MAX);
  fb.vel = uintToFloat(v_raw, V_MIN, V_MAX);
  fb.torque = uintToFloat(t_raw, T_MIN, T_MAX);
  fb.updated = true;
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

// enable 직후 1회: 무저항(zero-gain) 프로브 프레임으로 현재 위치를 읽는다.
float readCurrentPos() {
  fb.updated = false;
  operationControl(MOTOR_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  uint32_t waitStart = millis();
  while (millis() - waitStart < 100) {
    Can0.events();
  }

  if (!fb.updated) {
    Serial.println("[WARN] 현재 위치 피드백을 받지 못했습니다. 0으로 시작합니다.");
    Serial.printf("       rx_total=%lu rx_mode2=%lu rx_matched=%lu (motor_id=%d 프레임이 안 들어오면 배선/보드레이트/ID 문제)\r\n",
                  (unsigned long)rx_total_count, (unsigned long)rx_mode2_count, (unsigned long)rx_matched_count, MOTOR_ID);
    if (rx_total_count > 0) {
      Serial.printf("       마지막 수신 프레임 id=0x%08lX (%lums 전)\r\n",
                    (unsigned long)last_rx_id, (unsigned long)(millis() - last_rx_ms));
    }
    return 0.0f;
  }
  Serial.printf("[Teensy] 현재 위치 확인됨. run_mode=%d fault=0x%02X\r\n", motor_run_mode, fault_bits);
  printFaultBits(fault_bits);
  return fb.pos;
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
  Serial.println("[Teensy] shoulder_pitch (CAN ID 13) 단독 확인용 코드");
  Serial.println("e = enable (현재 위치부터 천천히 음의 방향으로 회전 시작)");
  Serial.println("d = disable (정지)");
  Serial.println("==================================================");

  controlTimer = 0;
}

void loop() {
  Can0.events();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'e' || ch == 'E') {
      enableMotor(MOTOR_ID);
      delay(20);
      start_pos = readCurrentPos();
      target_pos = start_pos;
      running = true;
      Serial.printf("[Teensy] 시작 위치 = %.4f rad. 음의 방향으로 회전을 시작합니다.\r\n", start_pos);
    } else if (ch == 'd' || ch == 'D') {
      running = false;
      disableMotor(MOTOR_ID);
    }
  }

  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    if (running) {
      float step = ROTATE_SPEED * (CONTROL_PERIOD_US / 1000000.0f);
      float min_target = start_pos - ROTATE_RANGE_LIMIT;
      if (target_pos > min_target) {
        target_pos -= step;
        if (target_pos < min_target) target_pos = min_target;
      }

      operationControl(MOTOR_ID, 0.0f, target_pos, 0.0f, KP, KD);
    }

    if (millis() - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = millis();
      Serial.printf("[STATUS] running=%d target=%.4f fb_pos=%.4f fb_vel=%.4f fb_trq=%.4f %s\r\n",
                    running, target_pos, fb.pos, fb.vel, fb.torque, fb.updated ? "FB_OK" : "FB_NONE");
    }
  }
}
