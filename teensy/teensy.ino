#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 1. Motor and communication configuration parameters
// -------------------------------------------------------------
const uint8_t NUM_MOTORS = 3;
const uint8_t MST_IDS[NUM_MOTORS] = {1, 2, 3};     // Master motor ID
const uint8_t SLV_IDS[NUM_MOTORS] = {11, 12, 13};  // Slave motor ID
const uint8_t HOST_ID = 253;                       // Teensy (host) ID

// Uses Teensy 4.0/4.1 CAN1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// -------------------------------------------------------------
// 2. Robstride protocol physical limit values
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
// 3. Control period configuration (500 Hz for teleoperation / dt = 0.002s)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000; // dt = 0.002s (2,000us = 500Hz)
elapsedMicros controlTimer;

// -------------------------------------------------------------
// 4. Real-time state and offset variables
// -------------------------------------------------------------
volatile float master_pos[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
volatile float slave_pos[NUM_MOTORS]  = {0.0f, 0.0f, 0.0f};
float pos_offset[NUM_MOTORS]          = {0.0f, 0.0f, 0.0f}; // Initial angle difference offset

// -------------------------------------------------------------
// 5. Data scaling helper functions
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
// 6. Robstride CAN transmit control functions
// -------------------------------------------------------------
void enableMotor(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12 << 24) | (HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  
  mode_msg.buf[0] = 0x05; 
  mode_msg.buf[1] = 0x70; 
  mode_msg.buf[2] = 0x00;
  mode_msg.buf[3] = 0x00;
  mode_msg.buf[4] = 0x00; 
  mode_msg.buf[5] = 0x00;
  mode_msg.buf[6] = 0x00;
  mode_msg.buf[7] = 0x00;
  
  Can0.write(mode_msg);
  delay(20); 

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3 << 24) | (HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  for (int i = 0; i < 8; i++) enable_msg.buf[i] = 0;
  
  Can0.write(enable_msg);
  Serial.printf("[Teensy] Motor %d Enabled successfully!\r\n", motor_id);
}

void disableMotor(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4 << 24) | (HOST_ID << 8) | motor_id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;
  
  Can0.write(msg);
  Serial.printf("[Teensy] Motor %d Disabled.\r\n", motor_id);
}

CAN_message_t operationControl(uint8_t motor_id, float feed_forward, float pos, float vel, float kp, float kd) {
  uint16_t p_int  = floatToUint(pos,          P_MIN, P_MAX,  16);
  uint16_t v_int  = floatToUint(vel,          V_MIN, V_MAX,  16);
  uint16_t kp_int = floatToUint(kp,           0.0f,  KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd,           0.0f,  KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX,  16);

  CAN_message_t msg;
  msg.flags.extended = 1;
  
  msg.id = (1 << 24) | (t_int << 8) | motor_id;
  msg.len = 8;
  
  msg.buf[0] = (p_int >> 8) & 0xFF;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;
  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF;
  msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF;
  msg.buf[7] = kd_int & 0xFF;

  Can0.write(msg);
  return msg;
}

// -------------------------------------------------------------
// 7. CAN receive interrupt callback
// -------------------------------------------------------------
void rxCallback(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;
  
  if (mode == 2) {
    uint8_t motor_id = (msg.id >> 8) & 0xFF; 
    uint16_t p_raw = (msg.buf[0] << 8) | msg.buf[1];
    float current_pos = uintToFloat(p_raw, P_MIN, P_MAX);

    // Parse whether the received ID belongs to a Master or a Slave
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (motor_id == MST_IDS[i]) {
        master_pos[i] = current_pos;
        break;
      } else if (motor_id == SLV_IDS[i]) {
        slave_pos[i] = current_pos;
        break;
      }
    }
  }
}

// -------------------------------------------------------------
// 8. Initial position offset measurement helper function
// -------------------------------------------------------------
void setupOffset() {
  // Send dummy command to induce motor feedback
  for (int i = 0; i < NUM_MOTORS; i++) {
    operationControl(MST_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    operationControl(SLV_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
  
  // Wait to receive feedback data (100ms)
  uint32_t waitStart = millis();
  while (millis() - waitStart < 100) {
    Can0.events();
  }

  // Calculate initial offset per pair (pos_offset = slave - master)
  for (int i = 0; i < NUM_MOTORS; i++) {
    pos_offset[i] = slave_pos[i] - master_pos[i];
    Serial.printf("[SETUP] Pair %d Offset Calculated: %.3f rad (Master %d: %.3f, Slave %d: %.3f)\r\n", 
                  i + 1, pos_offset[i], MST_IDS[i], master_pos[i], SLV_IDS[i], slave_pos[i]);
  }
}

// -------------------------------------------------------------
// 9. Main loop structure
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); 

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== Robstride Unidirectional Teleoperation with Initial Offset (1,2,3 -> 11,12,13) ===");

  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps
  Can0.setMaxMB(64);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();
  Can0.enableMBInterrupts();
  Can0.onReceive(rxCallback);

  Serial.println("Teensy CAN initialized.");
  delay(1000);


  // Run initial position offset calculation
  setupOffset();

  controlTimer = 0;
}

void loop() {
  Can0.events(); // Process CAN receive events

  // Blink (toggle) the built-in LED every 1 second (1000ms)
  static uint32_t lastLedToggle = 0;
  if (millis() - lastLedToggle >= 1000) {
    lastLedToggle = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 2ms period control loop (500Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    float slave_kp = 25.0f; // Slave position tracking strength
    float slave_kd = 1.0f;

    for (int i = 0; i < NUM_MOTORS; i++) {
      // 1. Master motors (1, 2, 3): for manual teaching (Kp=0, Kd=0)
      operationControl(MST_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      // 2. Slave motors (11, 12, 13): track master position with offset applied
      float slave_target_pos = master_pos[i] + pos_offset[i];
      operationControl(SLV_IDS[i], 0.0f, slave_target_pos, 0.0f, slave_kp, slave_kd);
    }

    // Print status monitoring every 500ms
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      
      for (int i = 0; i < NUM_MOTORS; i++) {
        float slave_target_pos = master_pos[i] + pos_offset[i];
        Serial.printf("[Pair %d] Master %d Pos: %.3f rad | Slave %d Pos: %.3f rad (Target: %.3f)\r\n", 
                      i + 1, MST_IDS[i], master_pos[i], SLV_IDS[i], slave_pos[i], slave_target_pos);
      }
      Serial.println(); // Blank line for separation
    }
  }
}

// -------------------------------------------------------------
// 10. Serial command receive interrupt
// -------------------------------------------------------------
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd' || ch == 'D') {
      for (int i = 0; i < NUM_MOTORS; i++) {
        disableMotor(MST_IDS[i]); delay(20);
        disableMotor(SLV_IDS[i]); delay(20);
      }
      Serial.println("[Teensy] All Motors Disabled.");
    } else if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_MOTORS; i++) {
        enableMotor(MST_IDS[i]); delay(20);
        enableMotor(SLV_IDS[i]); delay(20);
      }
      setupOffset(); // Recalculate offset upon re-enabling
      Serial.println("[Teensy] All Motors Enabled & Offset Reset.");
    }
  }
}