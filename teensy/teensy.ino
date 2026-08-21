#include <Arduino.h>
#include <FlexCAN_T4.h>

// -------------------------------------------------------------
// Test code for checking multiple motors simultaneously
// - 'e' : enable all motors + read each one's current position -> starting from that position,
//         start sending the same command (a small rotation) slowly in the negative direction to all at once
// - 'd' : disable all motors + stop rotation
// -------------------------------------------------------------

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

const uint8_t MOTOR_IDS[] = {11, 13};
const uint8_t NUM_MOTORS = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);
const uint8_t HOST_ID = 253;

// Robstride protocol physical limit values (for MIT mode packing)
const float P_MIN = -12.5f;   const float P_MAX = 12.5f;
const float V_MIN = -45.0f;   const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;  const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;   const float T_MAX = 18.0f;

// Control gains (kept weak since this is just for checking)
const float KP = 10.0f;
const float KD = 1.0f;

// Speed for slowly rotating in the negative direction (raw position units, rad/s)
const float ROTATE_SPEED = 0.05f;

// Safe travel range relative to the start position (raw position units, rad). Stops rather than going further than this.
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
// Diagnostic state (per-motor, for analyzing why rotation isn't happening)
// -------------------------------------------------------------
volatile uint32_t rx_total_count = 0;                    // Count of all CAN frames received (regardless of mode/ID)
volatile uint32_t rx_mode2_count = 0;                     // Count of mode==2 (feedback) frames (regardless of ID)
volatile uint32_t rx_matched_count[NUM_MOTORS] = {0};      // Count of frames that are mode==2 and match this motor_id
volatile uint32_t last_rx_id = 0;                          // Raw CAN ID of the most recently received frame (regardless of mode/ID)
volatile uint32_t last_rx_ms = 0;                          // Timestamp of the most recent receive

volatile uint8_t fault_bits[NUM_MOTORS] = {0};             // Fault bits from Type2 feedback (bits 21-16, 6 bits)
volatile uint8_t motor_run_mode[NUM_MOTORS] = {0};         // Motor state from Type2 feedback (bits 23-22): 0=Reset,1=Cali,2=Run

uint32_t tx_control_count = 0;             // Number of operationControl() calls (=CAN TX attempts)
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
  // For diagnostics: first confirm whether anything at all is coming in on the CAN bus, regardless of mode/ID
  rx_total_count++;
  last_rx_id = msg.id;
  last_rx_ms = millis();

  uint8_t mode = (msg.id >> 24) & 0x1F;
  if (mode != 2) return;
  rx_mode2_count++;

  uint8_t motor_id = (msg.id >> 8) & 0xFF;
  // Robstride Type2 feedback ID layout: bits23-22=motor_mode, bits21-16=fault, bits15-8=motor_id
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
  mode_msg.buf[0] = 0x05; mode_msg.buf[1] = 0x70; // Run Mode address (0x7005)
  mode_msg.buf[4] = 0x00; // MIT operation control mode (0)
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

// Right after enable: send a zero-resistance (zero-gain) probe frame to all motors and read their current position.
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
      Serial.printf("[WARN] Motor %d did not receive current position feedback. Starting at 0.\r\n", motor_id);
      Serial.printf("       rx_total=%lu rx_mode2=%lu rx_matched=%lu (if motor_id=%d frames aren't coming in, it's a wiring/baud-rate/ID problem)\r\n",
                    (unsigned long)rx_total_count, (unsigned long)rx_mode2_count,
                    (unsigned long)rx_matched_count[i], motor_id);
      start_pos[i] = 0.0f;
    } else {
      Serial.printf("[Teensy] Motor %d current position confirmed = %.4f rad. run_mode=%d fault=0x%02X\r\n",
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
  Serial.printf("[Teensy] Code for checking %d motors simultaneously: ", NUM_MOTORS);
  for (uint8_t i = 0; i < NUM_MOTORS; i++) Serial.printf("%d ", MOTOR_IDS[i]);
  Serial.println();
  Serial.println("e = enable (all motors start rotating slowly in the negative direction with the same command, from each one's current position)");
  Serial.println("d = disable (stop all motors)");
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
      Serial.println("[Teensy] All motors are starting negative-direction rotation with the same command.");
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
