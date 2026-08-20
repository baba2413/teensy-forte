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
// 3. 조인트 회전 방향 (링크 좌표계 기준). 실제 회전 방향이 반대인 조인트가
//    있으면 여기서 직접 -1.0f로 바꿔야 한다 (자동 계산 대상 아님).
//
//    절대 위치 오프셋(구 calib[].offset / computeCalibOffsets())은 더 이상
//    쓰지 않는다. 대신 'e'(enable) 시점에 baseline을 스냅샷해서, 그 이후
//    UDP 목표값의 "변화량(delta)"만 현재 모터 위치에 더해 추종한다
//    (captureBaseline() 참고). sim이 enable 순간 절대적으로 어떤 위치에
//    있었든 첫 명령의 delta는 항상 0이라 튀어나갈 여지가 없다.
// -------------------------------------------------------------
const float JOINT_SIGN[NUM_JOINTS] = {
  1.0f, // Joint 0 (shoulder_yaw,   CAN 11)
  1.0f, // Joint 1 (shoulder_pitch, CAN 13)
  1.0f, // Joint 2 (shoulder_roll,  CAN 12)
  1.0f, // Joint 3 (elbow_pitch,    CAN 14)
};

const char* JOINT_NAMES[NUM_JOINTS] = {"shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_pitch"};

// multiple_ik_udp.py MyCustomSceneCfg.robot.init_state.joint_pos 값 (링크 좌표계, rad)
// 지금은 UDP 입력 안전 클램프(TEST_LINK_RANGE_LIMIT)의 중심값으로만 쓰인다.
const float SIM_DEFAULT_LINK_POS[NUM_JOINTS] = {0.0f, -1.0472f, 0.0f, 2.0944f};

// -------------------------------------------------------------
// 3b. 실기 최초 동작 확인용 안전 범위 제한 (링크 좌표계, SIM_DEFAULT_LINK_POS 기준 +-)
//     지금은 "작동만 확인"하는 단계이므로 최대한 좁게 잡는다.
//     UDP로 어떤 값이 들어와도 이 범위를 벗어나면 클램프된다.
// -------------------------------------------------------------
const float TEST_LINK_RANGE_LIMIT = 0.5f; // rad (약 8.6deg)

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
// 6a. Enable 시점 baseline (delta 추종 방식의 핵심)
//     'e' 누르는 순간의 실측 모터 위치 + 그 순간의 UDP 목표값을 스냅샷해둔다.
//     이후 매 tick마다 "UDP 목표값이 baseline 대비 얼마나 변했는가(delta)"만
//     구해서 baseline 모터 위치에 더한다. sim이 enable 순간 어떤 절대 위치에
//     있었든 첫 명령의 delta는 항상 0이므로, enable 순간 절대 튀어나가지 않는다.
// -------------------------------------------------------------
float baseline_motor_pos[NUM_JOINTS] = {0.0f};   // 'e' 시점 fb[i].pos 스냅샷
float baseline_link_target[NUM_JOINTS] = {0.0f}; // 'e' 시점 ext_target_pos 스냅샷
bool baseline_ready = false;

// -------------------------------------------------------------
// 6b. 모터 피드백 버퍼 (항상 최신 유지, idle 중에도 무저항 프로브로 계속 갱신됨)
// -------------------------------------------------------------
struct MotorFeedback {
  float pos;
  float vel;
  float torque;
  bool updated;
  uint8_t fault_bits;   // Type2 피드백 ID의 bit16~21 (6bit), teleop-bi a483a4c와 동일 레이아웃
  uint8_t run_mode;     // Type2 피드백 ID의 bit22~23 (2bit): 0 reset, 1 cali, 2 motor run
  bool fault_changed;   // 직전 값과 달라졌을 때만 loop()에서 1회 로깅하기 위한 플래그
};
MotorFeedback fb[NUM_JOINTS];

// CAN 수신 진단용 카운터/원본값. rxCallback()이 애초에 호출되고 있는지, mode==2 프레임이
// 오고 있는지, motor_id 매칭이 실패하는지를 로그로 직접 확인하기 위함 (getpos.txt는 되는데
// 이 코드는 안 되는 원인을 추측이 아니라 실측으로 좁히기 위해 추가).
volatile uint32_t can_rx_total = 0;      // rxCallback 진입 총 횟수 (mode 무관)
volatile uint32_t can_rx_mode2 = 0;      // mode==2 (피드백) 프레임 총 횟수
volatile uint32_t can_rx_unmatched = 0;  // mode==2인데 motor_id가 JOINT_CAN_IDS와 매칭 안 된 횟수
volatile uint32_t last_raw_can_id = 0;
volatile uint8_t  last_raw_can_len = 0;
volatile uint32_t last_raw_can_ms = 0;

// Type2 피드백 CAN ID에 실린 fault 비트 의미 (teensy-forte teleop-bi a483a4c 기준, 검증됨)
void printFaultBits(int idx, uint8_t fault_bits) {
  Serial.printf("[FAULT] Joint %d %-14s (CAN %2d) | Raw: 0x%02X",
                idx, JOINT_NAMES[idx], JOINT_CAN_IDS[idx], fault_bits);

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
// 6c. 상태 로깅 (teensy-forte teleop-bi 스타일)
//     LOG_PERIOD_MS마다 명령값(링크/모터축)과 실시간 피드백(raw)을 출력한다.
// -------------------------------------------------------------
const uint32_t NORMAL_LOG_PERIOD_MS = 1000;
// enable_debug=true일 때 'e'로 enable하면 이 주기로 낮춰서 촘촘하게(제어 루프와 동일 주기,
// 즉 매 tick) 로그를 남긴다. 로그가 빠르게 쌓이므로 'p'로 일시정지시켜 원하는 시점을 확인한다.
const uint32_t DEBUG_LOG_PERIOD_MS = 700;

// 디버그 모드 수동 스위치. true/false는 코드에서 직접 바꿔서 재업로드한다.
// true면 'e' enable 시 LOG_PERIOD_MS가 DEBUG_LOG_PERIOD_MS로 낮아져 로그가 훨씬 자주 찍힌다.
bool enable_debug = true;

uint32_t LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS;
uint32_t lastLogMs = 0;
bool serialLogPaused = false; // 'p' 명령으로 토글 (시리얼 도배 방지용 일시정지)

// 'g' 명령으로 토글. 켜면 매 제어 루프(50Hz)마다 plot_positions.py가 파싱할 수 있는
// 한 줄짜리 CSV(PLOT,...)를 출력한다. logStatus()의 사람이 읽기용 요약과는 별개.
bool plotStreamEnabled = false;

float last_cmd_link_pos[NUM_JOINTS] = {0.0f};
float last_cmd_motor_pos[NUM_JOINTS] = {0.0f};
float last_cmd_ff_torque[NUM_JOINTS] = {0.0f};
float last_cmd_delta_link[NUM_JOINTS] = {0.0f};

// TEST_LINK_RANGE_LIMIT 클램프 / RAW_LIMIT 드롭 발생 횟수. 매 패킷마다 로그를 찍으면
// Isaac Sim의 큰 동작 폭 때문에 시리얼이 도배되므로, LOG_PERIOD_MS 요약에만 합쳐서 출력한다.
uint32_t range_clamp_count[NUM_JOINTS] = {0};
uint32_t raw_drop_count[NUM_JOINTS] = {0};

// 마지막으로 수신한 원본 UDP 패킷 (strtok()이 원본 버퍼를 파괴하므로 로깅용으로 별도 보관)
char last_udp_raw[64] = "";
uint32_t last_udp_recv_ms = 0;

// 안전 클램프(TEST_LINK_RANGE_LIMIT) 적용 "이전"의 UDP 원본 요청값. 클램프 후 값(ext_target_pos)과
// 나란히 로깅해 "UDP가 실제로 뭘 요청했는지" vs "클램프 후 뭘 썼는지"를 구분할 수 있게 한다.
float last_udp_target_raw[NUM_JOINTS] = {0.0f};

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
  can_rx_total++;
  last_raw_can_id = msg.id;
  last_raw_can_len = msg.len;
  last_raw_can_ms = millis();

  uint8_t mode = (msg.id >> 24) & 0x1F;
  if (mode != 2) return;
  can_rx_mode2++;

  uint8_t motor_id = (msg.id >> 8) & 0xFF;
  int idx = getJointIndex(motor_id);
  if (idx < 0) {
    // getpos.txt에서 검증된 fallback: 일부 피드백 프레임은 모터 ID가 bit8~15가 아니라
    // 하위 바이트(bit0~7)에 실려 온다. 여기서도 없으면 진짜로 매칭 실패.
    motor_id = msg.id & 0xFF;
    idx = getJointIndex(motor_id);
  }
  if (idx < 0) { can_rx_unmatched++; return; }

  uint8_t new_fault_bits = (msg.id >> 16) & 0x3F;
  uint8_t new_run_mode = (msg.id >> 22) & 0x03;
  if (new_fault_bits != fb[idx].fault_bits) {
    fb[idx].fault_bits = new_fault_bits;
    fb[idx].fault_changed = true; // 실제 출력은 loop()에서 (콜백 안에서 Serial.printf 남발 방지)
  }
  fb[idx].run_mode = new_run_mode;

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

// 과전류/과열/스톨 등으로 fault 상태에 빠져 위치 명령이 먹히지 않을 때 호출.
// Type4 프레임에 fault-clear용 데이터 바이트가 있다는 설(buf[0]=1)은 이 레포의 어떤 브랜치
// 기록에도 사용된 전례가 없어 검증되지 않은 추측이었다(펌웨어가 무시할 가능성이 높음).
// 대신 'e'가 매번 수행하는, 실제로 검증된 재기동 시퀀스(정지 -> Run Mode 재기록 -> enable)를
// 그대로 재사용한다: 컨트롤러가 Run Mode 재진입 시 fault를 재평가/해제하는 것이 표준 동작이며,
// 위치 명령은 실리지 않으므로 팔이 갑자기 움직이지는 않는다.
void clearFault(uint8_t motor_id) {
  disableMotor(motor_id);
  delay(20);
  enableMotor(motor_id);
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

// 'e'(enable) 처리 중 호출: 그 순간의 실측 모터 위치와 그 순간의 UDP 목표값을
// baseline으로 스냅샷한다. fb[]/ext_target_pos는 idle 중에도 항상 최신으로 유지되고
// 있으므로(무저항 프로브 / UDP 파싱이 계속 돌아감) 별도로 기다릴 필요가 없다.
// 이후 제어는 baseline 대비 UDP 목표값의 "변화량(delta)"만 현재 모터 위치에 더하므로,
// 이 시점에 sim이 어떤 절대 자세에 있었든 최초 명령은 항상 delta=0 -> 절대 안 튄다.
void captureBaseline() {
  Serial.println("--- [BASELINE] Captured at enable ---");
  for (int i = 0; i < NUM_JOINTS; i++) {
    baseline_motor_pos[i] = fb[i].pos;
    baseline_link_target[i] = ext_target_pos[i];
    if (!fb[i].updated) {
      Serial.printf("  Joint %d %-14s (CAN %2d): WARNING - no motor feedback received yet, baseline motor pos=0\r\n",
                    i, JOINT_NAMES[i], JOINT_CAN_IDS[i]);
    } else {
      Serial.printf("  Joint %d %-14s (CAN %2d) motor_baseline=%7.4f rad  link_baseline=%7.4f rad\r\n",
                    i, JOINT_NAMES[i], JOINT_CAN_IDS[i], baseline_motor_pos[i], baseline_link_target[i]);
    }
  }
  baseline_ready = true;
  Serial.println("[BASELINE] motor_pos = motor_baseline + (udp_target - link_baseline) * sign * gear_ratio");
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

// 중력 보상 사용 여부 (필요시 false로 바꿔 feed-forward 토크를 끈다)
bool GRAVITY_COMPENSATION_ENABLED = false;

// 안전 캡: 첫 실기 검증 단계에서 계산이 틀리더라도 과도한 토크가 나가지 않도록 제한.
// 관절 전체 가동범위(+-pi, URDF 리밋)를 전수 탐색해 구한 이론상 최댓값은
// pitch 8.17Nm, roll 2.39Nm, elbow 2.39Nm (yaw는 항상 0이라 대상 아님).
// pitch가 roll/elbow보다 3배 이상 커서 하나의 캡을 공유하면 pitch는 계속 잘리거나
// roll/elbow의 마진이 과도하게 느슨해지므로, 관절별로 따로 잡는다
// (최댓값 대비 10~15% 여유, T_MAX=18Nm 대비 충분히 낮은 수준).
const float GRAVITY_FF_LIMIT_PITCH_NM = 3.0f; // Nm (링크축 기준)
const float GRAVITY_FF_LIMIT_ROLL_NM  = 1.0f; // Nm (링크축 기준)
const float GRAVITY_FF_LIMIT_ELBOW_NM = 1.0f; // Nm (링크축 기준)

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

const char* runModeStr(uint8_t mode) {
  switch (mode) {
    case 0: return "RESET (idle)";
    case 1: return "CALIBRATING";
    case 2: return "RUNNING";
    default: return "UNKNOWN";
  }
}

// fault_bits를 사람이 읽을 수 있는 이름 목록으로 변환 (printFaultBits()와 동일한 비트 의미 사용)
void faultBitsToStr(uint8_t bits, char* out, size_t outSize) {
  if (bits == 0) { snprintf(out, outSize, "none"); return; }
  out[0] = '\0';
  const char* names[6] = {
    "UNDERVOLTAGE", "THREE_PHASE_OVERCURRENT", "OVERTEMPERATURE",
    "MAGNETIC_ENCODER_FAULT", "STALL_OVERLOAD", "UNCALIBRATED"
  };
  for (int b = 0; b < 6; b++) {
    if (!(bits & (1 << b))) continue;
    if (out[0] != '\0') strncat(out, "+", outSize - strlen(out) - 1);
    strncat(out, names[b], outSize - strlen(out) - 1);
  }
}

// LOG_PERIOD_MS마다 호출: 조인트별로
//   1. 현재 모터 포지션(실측) -> 2. UDP 패킷에 의한 목표 포지션 -> 3. 이 모터 오프셋 ->
//   4. 기어비 -> 5. 오프셋+기어비 적용한 최종 포지션 -> 6. CAN에 포장된 최종 포지션(5와 비교)
// 순서로 출력한다. 그 사이 발생한 클램프/드롭 횟수도 함께 요약한다.
void logStatus() {
  uint32_t now = millis();
  Serial.println("=======================================================================");
  Serial.printf("STATUS  t=%lums  control_active=%s\r\n",
                (unsigned long)now, ext_control_active ? "YES" : "NO (watchdog timeout / not enabled)");

  if (last_udp_recv_ms == 0) {
    Serial.println("UDP packet : none received yet");
  } else {
    Serial.printf("UDP packet : \"%s\"  (received %lums ago, watchdog timeout=%lums)\r\n",
                  last_udp_raw, (unsigned long)(now - last_udp_recv_ms), (unsigned long)WATCHDOG_TIMEOUT_MS);
  }

  // CAN 수신 진단: rxCallback이 실제로 호출되고 있는지(any mode), 그 중 mode==2(피드백)가
  // 몇 개인지, 그중 motor_id 매칭에 실패한 게 몇 개인지, 마지막으로 본 raw CAN ID/길이.
  // 이 값들이 전부 0이면 인터럽트 자체가 안 도는 것(배선/버스 문제), can_rx_total>0인데
  // can_rx_mode2=0이면 mode 필드 해석이 잘못된 것, can_rx_unmatched>0이면 motor_id 위치가
  // 잘못된 것 - 원인을 추측이 아니라 이 숫자로 좁힐 수 있다.
  Serial.printf("CAN RX diag : total=%lu  mode2(feedback)=%lu  unmatched_id=%lu  last_raw_id=0x%08lX len=%u (%lums ago)\r\n",
                (unsigned long)can_rx_total, (unsigned long)can_rx_mode2, (unsigned long)can_rx_unmatched,
                (unsigned long)last_raw_can_id, (unsigned)last_raw_can_len,
                (unsigned long)(last_raw_can_ms == 0 ? 0 : now - last_raw_can_ms));

  for (int i = 0; i < NUM_JOINTS; i++) {
    char faultStr[96];
    faultBitsToStr(fb[i].fault_bits, faultStr, sizeof(faultStr));

    Serial.printf("\r\n[%s] (CAN ID %d)  motor state=%s  fault=%s\r\n",
                  JOINT_NAMES[i], JOINT_CAN_IDS[i], runModeStr(fb[i].run_mode), faultStr);

    if (fb[i].updated) {
      Serial.printf("  1. Current motor position (measured)     : %8.4f rad\r\n", fb[i].pos);
      Serial.printf("     current velocity / torque             : %6.2f rad/s  /  %6.2f Nm\r\n", fb[i].vel, fb[i].torque);
    } else {
      Serial.println("  1. Current motor position (measured)     : no feedback received yet");
    }

    Serial.printf("  2. Target position from UDP (link axis)  : %8.4f rad  (clamped %lu time(s) this period, raw request=%.4f)\r\n",
                  ext_target_pos[i], (unsigned long)range_clamp_count[i], last_udp_target_raw[i]);
    Serial.printf("  3. Baseline @ last enable  motor=%8.4f rad  link=%8.4f rad  (sign=%+.1f)\r\n",
                  baseline_motor_pos[i], baseline_link_target[i], JOINT_SIGN[i]);
    Serial.printf("  4. Delta from baseline (link axis, gear=x%.4f) : %8.4f rad\r\n",
                  GEAR_RATIO[i], last_cmd_delta_link[i]);
    Serial.printf("  5. Final position (baseline+delta*gear)  : %8.4f rad (motor axis)   <- computed, full precision\r\n",
                  last_cmd_motor_pos[i]);

    if (last_can_sent[i]) {
      uint16_t p_raw = (last_can_buf[i][0] << 8) | last_can_buf[i][1];
      float encoded_pos = uintToFloat(p_raw, P_MIN, P_MAX);
      Serial.printf("  6. Final position as packed into CAN     : %8.4f rad (motor axis)   <- compare with #5 (quantization)  [SENT]\r\n",
                    encoded_pos);
    } else {
      Serial.printf("  6. Final position as packed into CAN     : DROPPED, not sent  (outside +-%.1f rad raw limit, %lu drop(s) this period)\r\n",
                    RAW_LIMIT_MAX, (unsigned long)raw_drop_count[i]);
    }

    range_clamp_count[i] = 0;
    raw_drop_count[i] = 0;
  }
  Serial.println("=======================================================================\r\n");
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
  Serial.printf("Debug mode      : %s (LOG_PERIOD after 'e' enable: %lums)\r\n",
                enable_debug ? "ON" : "OFF",
                (unsigned long)(enable_debug ? DEBUG_LOG_PERIOD_MS : NORMAL_LOG_PERIOD_MS));
  Serial.println("Serial commands : d=disable");
  Serial.println("                  e=enable all motors + capture baseline (current motor pos + current");
  Serial.println("                    UDP target). From then on, motor follows ONLY the delta of the UDP");
  Serial.println("                    target from that baseline -> no jump at enable, regardless of sim pose.");
  Serial.println("                  p=pause/resume periodic status log");
  Serial.println("                  g=toggle real-time PLOT,... CSV stream (for plot_positions.py)");
  Serial.println("                  f=attempt fault reset on all motors (stop->re-enable->stop)");
  Serial.println("==================================================");

  controlTimer = 0;
}

void loop() {
  Can0.events();

  // fault 상태 변화 시 즉시 출력 (LOG_PERIOD_MS 대기 없이, 값이 바뀔 때만 1회)
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (fb[i].fault_changed) {
      fb[i].fault_changed = false;
      printFaultBits(i, fb[i].fault_bits);
    }
  }

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
            last_udp_target_raw[i] = v; // 클램프 적용 전 원본값 (로깅용)

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

  // 20ms 제어 루프 (50Hz). teensy-forte teleop-bi의 메인 루프와 동일한 구조:
  // 매 tick마다 조인트당 정확히 한 번, "활성 제어중이면 실제 명령 / 아니면 무저항(kp=kd=0)
  // 프로브"를 무조건 보낸다 (enable 여부와 무관). 이 한 번의 Type1 프레임이 명령이자 동시에
  // 피드백 요청이므로 fb[]는 항상 최신으로 유지된다.
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // fb[i].updated는 여기서 리셋하지 않는다. 이 tick에서 방금 보낸 명령의 응답은 다음 tick
    // 이후에나 rxCallback()으로 들어오므로, 매 tick 시작 시점에 false로 지워버리면 바로 아래
    // logStatus()가 그 값을 읽을 기회가 전혀 없다 (실측: mode2 피드백은 계속 들어오는데도
    // 로그에는 항상 "no feedback received yet"으로 찍히는 버그였음). rxCallback()이 실제
    // 응답을 받을 때만 true로 세팅하고, 그 이후 계속 true로 유지되는 것으로 충분하다.
    if (ext_control_active && (millis() - last_packet_time < WATCHDOG_TIMEOUT_MS)) {

      if (!baseline_ready) {
        // 'e'를 아직 안 눌러 baseline이 없는 상태 -> 무저항 프로브만 유지 (움직이지 않음)
        for (int i = 0; i < NUM_JOINTS; i++) {
          operationControl(JOINT_CAN_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
      } else {
        float target_vel = 0.0f;
        float kp = 15.0f;
        float kd = 1.0f;

        // 1. baseline 대비 UDP 목표값의 변화량(delta, 링크축)만 계산 - 절대 위치는 안 씀
        float delta_link[NUM_JOINTS];
        float link_pos_est[NUM_JOINTS]; // sim 자체 관점의 절대 각도 추정치 (중력보상 참고용)
        for (int i = 0; i < NUM_JOINTS; i++) {
          delta_link[i] = (ext_target_pos[i] - baseline_link_target[i]) * JOINT_SIGN[i];
          link_pos_est[i] = ext_target_pos[i] * JOINT_SIGN[i];
        }

        // 2. 현재 목표 자세 기준 중력 보상 토크 계산 (링크축, Nm)
        float tau_gravity_link[NUM_JOINTS];
        computeGravityTorques(link_pos_est[0], link_pos_est[1], link_pos_est[2], link_pos_est[3], tau_gravity_link);

        for (int i = 0; i < NUM_JOINTS; i++) {
          // 3. delta에 모터:링크 회전비를 곱해 모터축(=CAN 명령) 좌표계로 변환한 뒤 baseline에 더함
          float motor_pos = baseline_motor_pos[i] + delta_link[i] * GEAR_RATIO[i];
          // 토크는 위치와 반대로 회전비로 나눔 (외부 감속단이 토크를 ratio배 증폭하므로)
          float feed_forward_torque = GRAVITY_COMPENSATION_ENABLED ? (tau_gravity_link[i] / GEAR_RATIO[i]) : 0.0f;

          last_cmd_delta_link[i] = delta_link[i];
          last_cmd_link_pos[i] = link_pos_est[i];
          last_cmd_motor_pos[i] = motor_pos;
          last_cmd_ff_torque[i] = feed_forward_torque;

          operationControl(JOINT_CAN_IDS[i], feed_forward_torque, motor_pos, target_vel, kp, kd);
        }

        if (plotStreamEnabled) {
          // PLOT,millis,link_yaw,link_pitch,link_roll,link_elbow,motor_yaw,motor_pitch,motor_roll,motor_elbow
          Serial.printf("PLOT,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                        (unsigned long)millis(),
                        last_cmd_link_pos[0], last_cmd_link_pos[1], last_cmd_link_pos[2], last_cmd_link_pos[3],
                        last_cmd_motor_pos[0], last_cmd_motor_pos[1], last_cmd_motor_pos[2], last_cmd_motor_pos[3]);
        }
      }

    } else {
      // 왓치독 타임아웃 처리
      if (ext_control_active) {
        Serial.println("[EMERGENCY] UDP Timeout! Disabling all motors.");
        ext_control_active = false;
        baseline_ready = false; // 다음 활성 제어는 반드시 새 'e'로 baseline을 다시 잡아야 함
        for (int i = 0; i < NUM_JOINTS; i++) {
          disableMotor(JOINT_CAN_IDS[i]);
        }
        LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS; // disable 시 debug 촘촘 로그 주기 해제, 평소 주기로 복귀
      }

      // 활성 제어중이 아닐 때도 무저항 프로브로 피드백은 계속 요청한다 (teleop-bi와 동일)
      for (int i = 0; i < NUM_JOINTS; i++) {
        operationControl(JOINT_CAN_IDS[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
    }

    if (!serialLogPaused && millis() - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = millis();
      logStatus();
    }
  }
}

// 시리얼 명령 수동 제어
// E: 전체 모터 enable (MIT run mode) + baseline 캡처(현재 모터 위치 + 현재 UDP 목표값).
//    이후 제어는 이 baseline 대비 UDP 목표값의 변화량(delta)만 추종한다.
// D: 전체 끄기 (다음 enable 전까지 baseline 무효화)
// P: 주기 상태 로그(logStatus) 출력 일시정지/재개 토글 (모터 제어/UDP 수신에는 영향 없음)
// G: 실시간 위치 CSV(PLOT,...) 스트림 토글 (plot_positions.py가 이 출력을 파싱함)
// F: 전체 모터 fault 초기화 시도 (정지->Run Mode 재기록->enable->정지, 이후 다시 e로 enable 필요)
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd' || ch == 'D') {
      baseline_ready = false;
      for (int i = 0; i < NUM_JOINTS; i++) {
        disableMotor(JOINT_CAN_IDS[i]);
      }
      LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS; // disable 시 debug 촘촘 로그 주기 해제, 평소 주기로 복귀
    } else if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_JOINTS; i++) {
        enableMotor(JOINT_CAN_IDS[i]);
        delay(20); // CAN 버스 연속 명령 안정성을 위한 미세 딜레이
      }
      captureBaseline(); // 이 순간의 모터 위치 + UDP 목표값을 baseline으로 스냅샷
      LOG_PERIOD_MS = enable_debug ? DEBUG_LOG_PERIOD_MS : NORMAL_LOG_PERIOD_MS;
      if (enable_debug) {
        Serial.printf("[Teensy] enable_debug=true -> LOG_PERIOD_MS=%lums (dense logging). Press 'p' to pause and inspect.\r\n",
                      (unsigned long)LOG_PERIOD_MS);
      }
    } else if (ch == 'p' || ch == 'P') {
      serialLogPaused = !serialLogPaused;
      Serial.printf("[Teensy] Status log %s.\r\n", serialLogPaused ? "PAUSED" : "RESUMED");
    } else if (ch == 'g' || ch == 'G') {
      plotStreamEnabled = !plotStreamEnabled;
      Serial.printf("[Teensy] Plot CSV stream %s.\r\n", plotStreamEnabled ? "ON" : "OFF");
    } else if (ch == 'f' || ch == 'F') {
      ext_control_active = false; // fault 초기화 후에는 재enable('e') 전까지 UDP 제어 재개하지 않음
      baseline_ready = false; // 재enable 전까지 baseline도 무효화
      for (int i = 0; i < NUM_JOINTS; i++) {
        clearFault(JOINT_CAN_IDS[i]); // 정지 -> Run Mode 재기록 -> enable (재기동으로 fault 재평가 유도)
        delay(20);
        // enable 직후 위치 명령(Type1)을 아직 보내지 않은 상태에서 모터가 어떻게 반응하는지는
        // 검증되지 않았으므로, 안전하게 다시 정지 상태로 확실히 되돌려놓는다.
        disableMotor(JOINT_CAN_IDS[i]);
        delay(20);
      }
      LOG_PERIOD_MS = NORMAL_LOG_PERIOD_MS; // disable 시 debug 촘촘 로그 주기 해제, 평소 주기로 복귀
    }
  }
}
