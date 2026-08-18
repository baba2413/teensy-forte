#include <Arduino.h>
#include <QNEthernet.h>
#include <FlexCAN_T4.h>
#include <math.h>

using namespace qindesign::network;

struct Vec3 { float x, y, z; };
struct Mat3 { float m[3][3]; };


// -------------------------------------------------------------
// 1. 관절 구성 (현재 구현된 4개 모터: yaw / pitch / roll / elbow)
// -------------------------------------------------------------
const uint8_t NUM_JOINTS = 4;
const uint8_t HOST_ID = 253; // Teensy 호스트 ID

// index 순서 = Isaac Sim 관절 순서 (shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch)
// CAN ID는 실제 하드웨어 배선 기준. 설계 당시와 달리 shoulder_roll=2, shoulder_pitch=3으로 배선됨.
const uint8_t JOINT_CAN_IDS[NUM_JOINTS] = {11, 13, 12, 14};

// -------------------------------------------------------------
// 2. 모터:링크 회전비 (모터 자체 내부 기어비와는 별개인 외부 감속단)
//    motor_angle = link_angle * GEAR_RATIO
//    JOINT_CAN_IDS와 동일한 index 순서로 매칭됨.
// -------------------------------------------------------------
const float GEAR_RATIO[NUM_JOINTS] = {4.8077f, 3.180f, 1.0f, 1.0f};

// -------------------------------------------------------------
// 3. 캘리브레이션 (링크 좌표계 기준 sign / offset)
//    offset은 매 'e'(enable) 시점에 computeCalibOffsets()가 자동으로 재계산해
//    덮어쓰므로 여기서는 0.0f로만 초기화해두면 된다 (영구 저장 불필요).
//    sign은 자동 계산 대상이 아니므로, 실제 회전 방향이 반대인 조인트가
//    있으면 여기서 직접 -1.0f로 바꿔야 한다.
// -------------------------------------------------------------
struct JointCalibration {
  float sign;
  float offset;
};

JointCalibration calib[NUM_JOINTS] = {
  {1.0f, 0.0f}, // Joint 0 (shoulder_yaw,   CAN 1)
  {1.0f, 0.0f}, // Joint 1 (shoulder_pitch, CAN 3)
  {1.0f, 0.0f}, // Joint 2 (shoulder_roll,  CAN 2)
  {1.0f, 0.0f}, // Joint 3 (elbow_pitch,    CAN 4)
};

const char* JOINT_NAMES[NUM_JOINTS] = {"shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_pitch"};

// multiple_ik_udp.py MyCustomSceneCfg.robot.init_state.joint_pos 값 (링크 좌표계, rad)
// computeCalibOffsets()에서 "실물을 이 자세로 맞춘 뒤 오프셋을 역산"하는 기준값으로 사용됨.
const float SIM_DEFAULT_LINK_POS[NUM_JOINTS] = {0.0f, -1.0472f, 0.0f, 2.0944f};

// -------------------------------------------------------------
// 3b. 실기 최초 동작 확인용 안전 범위 제한 (링크 좌표계, SIM_DEFAULT_LINK_POS 기준 +-)
//     지금은 "작동만 확인"하는 단계이므로 최대한 좁게 잡는다.
//     UDP로 어떤 값이 들어와도 이 범위를 벗어나면 클램프된다.
// -------------------------------------------------------------
const float TEST_LINK_RANGE_LIMIT = 1.0f; // rad (약 8.6deg)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// -------------------------------------------------------------
// 4. Robstride 프로토콜 물리적 제한 한계값 (MIT 모드 패킹용)
// -------------------------------------------------------------
const float P_MIN = -12.5f;   const float P_MAX = 12.5f;
const float V_MIN = -45.0f;   const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;  const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;   const float T_MAX = 18.0f;

// 프로토콜 한계(P_MIN/P_MAX)보다 살짝 좁은 안전 마진. teensy-forte teleop-bi와 동일하게
// 이 범위를 벗어나는 명령은 클램프하지 않고 통째로 버린다(모터에 절대 전송하지 않음).
const float RAW_LIMIT_MIN = -12.4f;
const float RAW_LIMIT_MAX = 12.4f;

// -------------------------------------------------------------
// 5. 네트워크 및 UDP 제어 주기 설정
// -------------------------------------------------------------
const uint16_t UDP_PORT = 5005;
EthernetUDP udp;

IPAddress staticIP(192, 168, 1, 15); // Teensy 고정 IP (Isaac 쪽 TEENSY_IP와 일치해야 함)
IPAddress subnet(255, 255, 255, 0);

const uint32_t CONTROL_PERIOD_US = 20000; // 50Hz (20ms)
elapsedMicros controlTimer;

// -------------------------------------------------------------
// 6. 외부 입력 버퍼 (Isaac Sim으로부터 수신, 링크 좌표계 rad)
// -------------------------------------------------------------
float ext_target_pos[NUM_JOINTS] = {0.0f};
bool ext_control_active = false;
uint32_t last_packet_time = 0;
const uint32_t WATCHDOG_TIMEOUT_MS = 500; // 0.5초간 패킷 미수신 시 안전 정지

// -------------------------------------------------------------
// 6b. 오프셋 1회 자동 계산용 모터 피드백 버퍼
//     사용법: 로봇팔을 손으로 SIM_DEFAULT_LINK_POS 자세와 맞춘 뒤 'e'를 누르면,
//     enable 직후 computeCalibOffsets()가 한 번만 실행되어 각 모터의 raw 위치를
//     읽고 calib[].offset을 즉시 계산/반영한다 (teensy-forte teleop-bi의
//     setupOffset() 패턴과 동일: 지속 폴링이 아니라 enable 시점 1회성 스냅샷).
// -------------------------------------------------------------
struct MotorFeedback {
  float pos;
  float vel;
  float torque;
  bool updated;
};
MotorFeedback fb[NUM_JOINTS];

// -------------------------------------------------------------
// 6c. 상태 로깅 (teensy-forte teleop-bi 스타일)
//     LOG_PERIOD_MS(기본 1초)마다 명령값(링크/모터축)과 실시간 피드백(raw)을 출력한다.
// -------------------------------------------------------------
uint32_t LOG_PERIOD_MS = 1000;
uint32_t lastLogMs = 0;

float last_cmd_link_pos[NUM_JOINTS] = {0.0f};
float last_cmd_motor_pos[NUM_JOINTS] = {0.0f};
float last_cmd_ff_torque[NUM_JOINTS] = {0.0f};

// TEST_LINK_RANGE_LIMIT 클램프 / RAW_LIMIT 드롭 발생 횟수. 매 패킷마다 로그를 찍으면
// Isaac Sim의 큰 동작 폭 때문에 시리얼이 도배되므로, LOG_PERIOD_MS 요약에만 합쳐서 출력한다.
uint32_t range_clamp_count[NUM_JOINTS] = {0};
uint32_t raw_drop_count[NUM_JOINTS] = {0};

// 마지막으로 수신한 원본 UDP 패킷 (strtok()이 원본 버퍼를 파괴하므로 로깅용으로 별도 보관)
char last_udp_raw[64] = "";
uint32_t last_udp_recv_ms = 0;

// 마지막으로 모터에 실제 전송한 CAN 명령 프레임 (operationControl()에서 채움)
uint32_t last_can_id[NUM_JOINTS] = {0};
uint8_t last_can_buf[NUM_JOINTS][8] = {{0}};
bool last_can_sent[NUM_JOINTS] = {false};

// -------------------------------------------------------------
// 7. 데이터 정수화 및 모터 제어 명령 함수군
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

int getJointIndex(uint8_t motor_id) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (JOINT_CAN_IDS[i] == motor_id) return i;
  }
  return -1;
}

// CAN 피드백(Type 2) 수신 콜백: 모터 현재 위치/속도/토크 파싱
void rxCallback(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;
  if (mode != 2) return;

  uint8_t motor_id = (msg.id >> 8) & 0xFF;
  int idx = getJointIndex(motor_id);
  if (idx < 0) return;

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
  int idx = getJointIndex(motor_id);

  if (pos < RAW_LIMIT_MIN || pos > RAW_LIMIT_MAX) {
    // 매 패킷마다 찍으면 시리얼이 도배되므로 카운트만 하고 LOG_PERIOD_MS 요약에서 출력
    if (idx >= 0) {
      raw_drop_count[idx]++;
      last_can_sent[idx] = false; // 이번엔 실제로 전송되지 않았음을 로깅에 반영
    }
    return; // 패킷을 버리고 CAN 전송하지 않음
  }

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

  if (idx >= 0) {
    last_can_id[idx] = msg.id;
    memcpy(last_can_buf[idx], msg.buf, 8);
    last_can_sent[idx] = true;
  }
}

// enable 직후 1회 호출: 현재(=사용자가 손으로 맞춰놓은) 자세를 raw로 읽어
// SIM_DEFAULT_LINK_POS와 비교, calib[].offset을 계산해 즉시 반영한다.
void computeCalibOffsets() {
  for (int i = 0; i < NUM_JOINTS; i++) fb[i].updated = false;

  // 무저항(zero-gain) 프로브 프레임: 모터를 움직이지 않고 피드백만 요청
  for (int i = 0; i < NUM_JOINTS; i++) {
    operationControl(JOINT_CAN_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  // 피드백 수신 대기 (100ms 동안 CAN 이벤트 펌핑)
  uint32_t waitStart = millis();
  while (millis() - waitStart < 100) {
    Can0.events();
  }

  Serial.println("--- [CALIB] Offset snapshot (motor pos vs sim default pose) ---");
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (!fb[i].updated) {
      Serial.printf("  Joint %d %-14s (CAN %2d): FAILED - no feedback received\r\n",
                    i, JOINT_NAMES[i], JOINT_CAN_IDS[i]);
      continue;
    }
    float link_pos_measured = fb[i].pos / GEAR_RATIO[i]; // 모터축 -> 링크 좌표계
    float offset = link_pos_measured - SIM_DEFAULT_LINK_POS[i];
    calib[i].offset = offset; // 즉시 반영 (재부팅마다 이 함수로 다시 계산하는 방식이라 영구 저장은 하지 않음)
    Serial.printf("  Joint %d %-14s (CAN %2d) raw=%7.3f rad -> link=%7.3f rad (sim default=%.3f) => offset=%.4f rad [applied]\r\n",
                  i, JOINT_NAMES[i], JOINT_CAN_IDS[i], fb[i].pos, link_pos_measured, SIM_DEFAULT_LINK_POS[i], offset);
  }
  Serial.println("[CALIB] Applied to calib[] for this session.");
}

// -------------------------------------------------------------
// 8b. 중력 보상 (feed-forward torque)
//     workspace2/urdf/forteme_collision_fixed.urdf의 질량/COM/조인트 원점으로 순기구학을
//     구성하고, 가상일(virtual work) 방법으로 shoulder_pitch/shoulder_roll/elbow_pitch의
//     중력 토크를 계산한다.
//       tau_j = sum_k  m_k * g_vec . (axis_j x (com_k - p_j))   [distal 링크 k에 대해 합산]
//     shoulder_yaw는 회전축(Z)이 중력과 평행해 항상 0이므로 계산 대상에서 제외.
//
//     [근사/한계] lower_arm_roll, wrist_pitch는 모터 미연결로 자유롭게 매달려(백드라이브)
//     있어 실제 각도를 알 수 없다. 여기서는 두 관절이 URDF 영점(0rad) 자세를 유지한다고
//     가정하고 elbow_link에 고정 부착된 것처럼 계산한다. 실제 처짐이 크면 오차가 생길 수
//     있으니, 처음 켤 때는 작은 kp/kd로 방향(부호)과 크기부터 반드시 확인할 것.
// -------------------------------------------------------------
const float GRAVITY = 9.81f;


Vec3 vadd(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 vsub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 vcross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Mat3 matMul(const Mat3 &A, const Mat3 &B) {
  Mat3 r;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      r.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
    }
  }
  return r;
}

Vec3 matVec(const Mat3 &A, Vec3 v) {
  return {
    A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z,
    A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z,
    A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z,
  };
}

Mat3 rotX(float a) { float c = cosf(a), s = sinf(a); Mat3 r = {{{1,0,0},{0,c,-s},{0,s,c}}}; return r; }
Mat3 rotY(float a) { float c = cosf(a), s = sinf(a); Mat3 r = {{{c,0,s},{0,1,0},{-s,0,c}}}; return r; }
Mat3 rotZ(float a) { float c = cosf(a), s = sinf(a); Mat3 r = {{{c,-s,0},{s,c,0},{0,0,1}}}; return r; }

// URDF 조인트 원점 (부모 링크 프레임 기준, 미터)
const Vec3 P_SHOULDER_YAW   = {0.0f,   0.0f,    0.018f};  // base_link -> shoulder_yaw
const Vec3 P_SHOULDER_PITCH = {0.0f,  -0.07f,   0.151f};  // shoulder_yaw_link -> shoulder_pitch
const Vec3 P_SHOULDER_ROLL  = {0.002f, 0.07f,   0.0f};    // shoulder_pitch_link -> shoulder_roll
const Vec3 P_ELBOW          = {0.358f,-0.0345f, 0.0f};    // upper_arm_link -> elbow_pitch
const Vec3 P_LOWER_ARM_ROLL = {0.118f, 0.0335f, 0.02f};   // elbow_link -> lower_arm_roll (미구동, 0rad 고정 취급)
const Vec3 P_WRIST_PITCH    = {0.247f,-0.0285f, 0.0f};    // lower_arm_link -> wrist_pitch (미구동, 0rad 고정 취급)

// URDF 링크 질량/무게중심 (자기 자신의 프레임 기준, 미터/kg)
const float M_SHOULDER_PITCH_LINK  = 0.5612f;
const Vec3  COM_SHOULDER_PITCH_LINK = {0.0082f, 0.0856f, -0.0002f};
const float M_UPPER_ARM_LINK       = 0.8932f;
const Vec3  COM_UPPER_ARM_LINK      = {0.1441f, 0.00004f, 0.00003f};
const float M_ELBOW_LINK           = 0.4441f;
const Vec3  COM_ELBOW_LINK          = {0.0235f, 0.0334f, 0.0173f};
const float M_LOWER_ARM_LINK       = 0.5071f;
const Vec3  COM_LOWER_ARM_LINK      = {0.0889f, -0.000003f, -0.00005f};
const float M_WRIST_LINK           = 0.3111f;
const Vec3  COM_WRIST_LINK          = {0.0466f, 0.0281f, 0.0268f};

const Vec3 GRAVITY_VEC = {0.0f, 0.0f, -GRAVITY};

// 안전 캡: 첫 실기 검증 단계에서 계산이 틀리더라도 과도한 토크가 나가지 않도록 제한.
// 관절 전체 가동범위(+-pi, URDF 리밋)를 전수 탐색해 구한 이론상 최댓값은
// pitch 8.17Nm, roll 2.39Nm, elbow 2.39Nm (yaw는 항상 0이라 대상 아님).
// pitch가 roll/elbow보다 3배 이상 커서 하나의 캡을 공유하면 pitch는 계속 잘리거나
// roll/elbow의 마진이 과도하게 느슨해지므로, 관절별로 따로 잡는다
// (최댓값 대비 10~15% 여유, T_MAX=18Nm 대비 충분히 낮은 수준).
const float GRAVITY_FF_LIMIT_PITCH_NM = 5.0f; // Nm (링크축 기준)
const float GRAVITY_FF_LIMIT_ROLL_NM  = 2.0f; // Nm (링크축 기준)
const float GRAVITY_FF_LIMIT_ELBOW_NM = 2.0f; // Nm (링크축 기준)

float clampTorque(float v, float limit) {
  if (v < -limit) return -limit;
  if (v > limit) return limit;
  return v;
}

float gravityContrib(float m, Vec3 com, Vec3 p_j, Vec3 axis_j) {
  Vec3 r = vsub(com, p_j);
  Vec3 cr = vcross(axis_j, r);
  return m * vdot(GRAVITY_VEC, cr);
}

// q_yaw/q_pitch/q_roll/q_elbow: 링크(sim) 좌표계 라디안 (캘리브레이션 적용 후 값)
// tau_link_out[4]: {yaw, pitch, roll, elbow} 순서, 링크축 기준 Nm (yaw는 항상 0)
void computeGravityTorques(float q_yaw, float q_pitch, float q_roll, float q_elbow, float tau_link_out[4]) {
  Mat3 R_yaw   = rotZ(q_yaw);
  Mat3 R_pitch = matMul(R_yaw,   rotY(q_pitch));
  Mat3 R_roll  = matMul(R_pitch, rotX(q_roll));
  Mat3 R_elbow = matMul(R_roll,  rotY(q_elbow));

  Vec3 p_yaw   = P_SHOULDER_YAW;
  Vec3 p_pitch = vadd(p_yaw,   matVec(R_yaw,   P_SHOULDER_PITCH));
  Vec3 p_roll  = vadd(p_pitch, matVec(R_pitch, P_SHOULDER_ROLL));
  Vec3 p_elbow = vadd(p_roll,  matVec(R_roll,  P_ELBOW));
  Vec3 p_lower_arm_roll = vadd(p_elbow,          matVec(R_elbow, P_LOWER_ARM_ROLL));
  Vec3 p_wrist_pitch    = vadd(p_lower_arm_roll, matVec(R_elbow, P_WRIST_PITCH)); // 미구동=0rad 가정

  Vec3 com_shoulder_pitch = vadd(p_pitch, matVec(R_pitch, COM_SHOULDER_PITCH_LINK));
  Vec3 com_upper_arm      = vadd(p_roll,  matVec(R_roll,  COM_UPPER_ARM_LINK));
  Vec3 com_elbow          = vadd(p_elbow, matVec(R_elbow, COM_ELBOW_LINK));
  Vec3 com_lower_arm      = vadd(p_lower_arm_roll, matVec(R_elbow, COM_LOWER_ARM_LINK));
  Vec3 com_wrist          = vadd(p_wrist_pitch,    matVec(R_elbow, COM_WRIST_LINK));

  Vec3 axis_pitch = matVec(R_yaw,   {0, 1, 0});
  Vec3 axis_roll  = matVec(R_pitch, {1, 0, 0});
  Vec3 axis_elbow = matVec(R_roll,  {0, 1, 0});

  float tau_elbow = gravityContrib(M_ELBOW_LINK,      com_elbow,     p_elbow, axis_elbow)
                   + gravityContrib(M_LOWER_ARM_LINK, com_lower_arm, p_elbow, axis_elbow)
                   + gravityContrib(M_WRIST_LINK,     com_wrist,     p_elbow, axis_elbow);

  float tau_roll = gravityContrib(M_UPPER_ARM_LINK,  com_upper_arm, p_roll, axis_roll)
                  + gravityContrib(M_ELBOW_LINK,      com_elbow,     p_roll, axis_roll)
                  + gravityContrib(M_LOWER_ARM_LINK,  com_lower_arm, p_roll, axis_roll)
                  + gravityContrib(M_WRIST_LINK,      com_wrist,     p_roll, axis_roll);

  float tau_pitch = gravityContrib(M_SHOULDER_PITCH_LINK, com_shoulder_pitch, p_pitch, axis_pitch)
                   + gravityContrib(M_UPPER_ARM_LINK,      com_upper_arm,      p_pitch, axis_pitch)
                   + gravityContrib(M_ELBOW_LINK,          com_elbow,          p_pitch, axis_pitch)
                   + gravityContrib(M_LOWER_ARM_LINK,      com_lower_arm,      p_pitch, axis_pitch)
                   + gravityContrib(M_WRIST_LINK,          com_wrist,          p_pitch, axis_pitch);

  tau_link_out[0] = 0.0f; // shoulder_yaw: 축이 중력과 평행 -> 항상 0
  tau_link_out[1] = clampTorque(tau_pitch, GRAVITY_FF_LIMIT_PITCH_NM);
  tau_link_out[2] = clampTorque(tau_roll,  GRAVITY_FF_LIMIT_ROLL_NM);
  tau_link_out[3] = clampTorque(tau_elbow, GRAVITY_FF_LIMIT_ELBOW_NM);
}

// LOG_PERIOD_MS마다 호출: 조인트별 명령값(링크/모터축, 중력보상 토크), 실시간 raw 피드백,
// 그리고 그 사이 발생한 클램프/드롭 횟수 요약을 출력한다.
void logStatus() {
  Serial.println("--- [STATUS] ---");

  uint32_t udp_age_ms = last_udp_recv_ms == 0 ? 0 : (millis() - last_udp_recv_ms);
  if (last_udp_recv_ms == 0) {
    Serial.println("  [UDP] no packet received yet");
  } else {
    Serial.printf("  [UDP] last=\"%s\" age=%lums\r\n", last_udp_raw, (unsigned long)udp_age_ms);
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.printf("  Joint %d %-14s (CAN %2d) | cmd_link=%7.3f cmd_motor=%7.3f ff_trq=%6.3f | fb_raw=%7.3f fb_vel=%6.2f fb_trq=%6.2f | %s | clamped=%lu dropped=%lu\r\n",
                  i, JOINT_NAMES[i], JOINT_CAN_IDS[i],
                  last_cmd_link_pos[i], last_cmd_motor_pos[i], last_cmd_ff_torque[i],
                  fb[i].pos, fb[i].vel, fb[i].torque,
                  fb[i].updated ? "FB_OK" : "FB_NONE",
                  (unsigned long)range_clamp_count[i], (unsigned long)raw_drop_count[i]);

    if (last_can_sent[i]) {
      Serial.printf("      -> CAN TX id=0x%08lX buf=[%02X %02X %02X %02X %02X %02X %02X %02X] (pos|vel|kp|kd, 2B each)\r\n",
                    (unsigned long)last_can_id[i],
                    last_can_buf[i][0], last_can_buf[i][1], last_can_buf[i][2], last_can_buf[i][3],
                    last_can_buf[i][4], last_can_buf[i][5], last_can_buf[i][6], last_can_buf[i][7]);
    } else {
      Serial.println("      -> CAN TX: none (dropped by RAW_LIMIT or not yet sent)");
    }

    range_clamp_count[i] = 0;
    raw_drop_count[i] = 0;
  }
  Serial.printf("  ext_control_active=%d\r\n\r\n", ext_control_active);
}

// -------------------------------------------------------------
// 8. 메인 루프
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps
  Can0.setMaxMB(64);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();
  Can0.enableMBInterrupts();
  Can0.onReceive(rxCallback);

  Ethernet.begin(staticIP, subnet, IPAddress(0, 0, 0, 0));
  udp.begin(UDP_PORT);

  IPAddress ip = Ethernet.localIP();
  Serial.println("==================================================");
  Serial.println("[Teensy 4.1] 4-Joint Arm Control (Ethernet-CAN)");
  Serial.printf("Joints (idx->CAN): yaw->%d, pitch->%d, roll->%d, elbow->%d\r\n",
                JOINT_CAN_IDS[0], JOINT_CAN_IDS[1], JOINT_CAN_IDS[2], JOINT_CAN_IDS[3]);
  Serial.printf("Static IP       : %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
  Serial.printf("UDP Port        : %d\r\n", UDP_PORT);
  Serial.println("Serial commands : d=disable");
  Serial.println("                  e=pose arm to sim default pose by hand, THEN press e");
  Serial.println("                    -> enables motors + auto-computes calib[] offsets once");
  Serial.println("==================================================");

  controlTimer = 0;
}

void loop() {
  Can0.events();

  // UDP 수신 처리
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[64];
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';

      // 포맷 파싱: P,yaw,pitch,roll,elbow (링크 좌표계 rad)
      if (packetBuffer[0] == 'P') {
        // strtok()이 아래에서 packetBuffer를 파괴하므로, 로깅용 원본은 미리 복사해둔다
        strncpy(last_udp_raw, packetBuffer, sizeof(last_udp_raw) - 1);
        last_udp_raw[sizeof(last_udp_raw) - 1] = '\0';
        last_udp_recv_ms = millis();

        char* token = strtok(packetBuffer, ",");
        int i = 0;
        while (token != NULL && i < NUM_JOINTS) {
          token = strtok(NULL, ",");
          if (token != NULL) {
            float v = atof(token);

            // 실기 최초 확인 단계 안전장치: SIM_DEFAULT_LINK_POS 기준 +-TEST_LINK_RANGE_LIMIT로 클램프
            // (Isaac Sim 쪽 실제 동작 폭이 이 범위보다 훨씬 커서 거의 매 패킷 클램프되는 게 정상.
            //  매번 로그를 찍으면 시리얼이 도배되므로 카운트만 하고 LOG_PERIOD_MS 요약에서 출력)
            float lo = SIM_DEFAULT_LINK_POS[i] - TEST_LINK_RANGE_LIMIT;
            float hi = SIM_DEFAULT_LINK_POS[i] + TEST_LINK_RANGE_LIMIT;
            if (v < lo || v > hi) {
              range_clamp_count[i]++;
              v = (v < lo) ? lo : hi;
            }

            ext_target_pos[i] = v;
            i++;
          }
        }
        ext_control_active = true;
        last_packet_time = millis(); // 왓치독 리셋
      }
    }
  }

  // 20ms 제어 루프 (50Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    if (ext_control_active && (millis() - last_packet_time < WATCHDOG_TIMEOUT_MS)) {

      float target_vel = 0.0f;
      float kp = 15.0f;
      float kd = 1.0f;

      // 1. 링크 좌표계에서 sign/offset 캘리브레이션 적용 (4관절 전체 먼저 계산)
      float link_pos[NUM_JOINTS];
      for (int i = 0; i < NUM_JOINTS; i++) {
        link_pos[i] = (ext_target_pos[i] * calib[i].sign) + calib[i].offset;
      }

      // 2. 현재 목표 자세 기준 중력 보상 토크 계산 (링크축, Nm)
      float tau_gravity_link[NUM_JOINTS];
      computeGravityTorques(link_pos[0], link_pos[1], link_pos[2], link_pos[3], tau_gravity_link);

      for (int i = 0; i < NUM_JOINTS; i++) {
        // 3. 모터:링크 회전비를 곱해 모터축(=CAN 명령) 좌표계로 변환
        float motor_pos = link_pos[i] * GEAR_RATIO[i];
        // 토크는 위치와 반대로 회전비로 나눔 (외부 감속단이 토크를 ratio배 증폭하므로)
        float feed_forward_torque = tau_gravity_link[i] / GEAR_RATIO[i];

        last_cmd_link_pos[i] = link_pos[i];
        last_cmd_motor_pos[i] = motor_pos;
        last_cmd_ff_torque[i] = feed_forward_torque;
        fb[i].updated = false; // 이번 명령에 대한 응답이 왔는지 로그에서 확인하기 위해 리셋

        operationControl(JOINT_CAN_IDS[i], feed_forward_torque, motor_pos, target_vel, kp, kd);
      }

    } else {
      // 왓치독 타임아웃 처리
      if (ext_control_active) {
        Serial.println("[EMERGENCY] UDP Timeout! Disabling all motors.");
        ext_control_active = false;
        for (int i = 0; i < NUM_JOINTS; i++) {
          disableMotor(JOINT_CAN_IDS[i]);
        }
      }
    }

    if (millis() - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = millis();
      logStatus();
    }
  }
}

// 시리얼 명령 수동 제어
// E: 로봇팔을 SIM_DEFAULT_LINK_POS 자세로 손으로 맞춘 뒤 누르면 전체 모터 enable + 오프셋 1회 자동 계산
// D: 전체 끄기
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd' || ch == 'D') {
      for (int i = 0; i < NUM_JOINTS; i++) {
        disableMotor(JOINT_CAN_IDS[i]);
      }
    } else if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_JOINTS; i++) {
        enableMotor(JOINT_CAN_IDS[i]);
        delay(20); // CAN 버스 연속 명령 안정성을 위한 미세 딜레이
      }
      computeCalibOffsets();
    }
  }
}
