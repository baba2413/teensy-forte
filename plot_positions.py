"""
Teensy가 모터로 실제 전송하는 위치 커맨드(link/motor axis)를 실시간으로 그래프로 그려주는 도구.

teensy.ino의 제어 루프(50Hz)는 매 tick마다 각 조인트에 보낼 위치를 계산해서 CAN으로
전송하는데, 이 값(last_cmd_link_pos / last_cmd_motor_pos)을 시리얼로도 CSV 한 줄씩
내보내도록 firmware에 'g' 명령(plotStreamEnabled 토글)을 추가해두었다. 이 스크립트는
"PLOT,..." 로 시작하는 줄만 파싱해 실시간 그래프로 그린다.

데이터 소스는 두 가지 모드가 있다:
  1. --port  : 이 스크립트가 시리얼 포트를 직접 연다 (PuTTY 등 다른 프로그램이 그
               포트를 이미 열어두면 Windows에서는 동시에 열 수 없어 실패한다).
  2. --logfile : PuTTY가 세션 로깅으로 기록 중인 텍스트 파일을 tail -f 처럼 읽는다.
               PuTTY를 계속 시리얼 콘솔로 써야 할 때 이 모드를 쓴다. PuTTY 설정:
               Session > Logging > Log file to write to = 이 파일 경로,
               Logging 방식은 "Printable output" 또는 "All session output" 아무거나 OK.
               이 모드에서는 'g' 명령을 스크립트가 대신 보낼 수 없으므로, PLOT 데이터가
               안 보이면 PuTTY 창에 직접 g를 입력해 스트림을 켜야 한다.

사용법은 파일 하단 또는 `python plot_positions.py --help` 참고.
"""

import argparse
import os
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

JOINT_NAMES = ["shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_pitch"]
JOINT_CAN_IDS = [11, 13, 12, 14]  # teensy.ino의 JOINT_CAN_IDS와 동일 순서/값을 유지할 것
BAUD_DEFAULT = 115200

# 조인트별로 그래프에 그릴지 여부를 CAN ID 기준 true/false로 토글.
# 여기 없는 CAN ID는 기본 True로 취급.
ENABLED_MOTORS = {
    11: True,   # shoulder_yaw
    13: True,   # shoulder_pitch
    12: False,  # shoulder_roll
    14: False,  # elbow_pitch
}


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(f"  {p.device}  ({p.description})")
    return [p.device for p in ports]


def pick_port(explicit_port):
    if explicit_port:
        return explicit_port
    ports = list(serial.tools.list_ports.comports())
    if len(ports) == 1:
        print(f"[plot_positions] 포트 자동 선택: {ports[0].device}")
        return ports[0].device
    print("[plot_positions] --port 로 시리얼 포트를 지정하세요. 사용 가능한 포트:")
    list_ports()
    sys.exit(1)


def ensure_stream_enabled(ser, timeout_s=2.0):
    """이미 PLOT 스트림이 켜져 있는지 확인하고, 꺼져 있으면 'g'를 보내 켠다."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("PLOT,"):
            print("[plot_positions] 기존에 스트림이 켜져 있음. 그대로 사용합니다.")
            return
    print("[plot_positions] 스트림이 꺼져 있어 'g' 명령을 전송해 켭니다.")
    ser.write(b"g")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("PLOT,"):
            return
    print("[plot_positions] 경고: 'g' 전송 후에도 PLOT 데이터가 보이지 않습니다. "
          "Teensy가 UDP로 위치 명령을 받고 있는지 확인하세요 (ext_control_active).")


class SerialSource:
    """이 스크립트가 시리얼 포트를 직접 열어 읽는 모드."""

    def __init__(self, port, baud):
        print(f"[plot_positions] {port} @ {baud}bps 연결 중...")
        self.ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(2.0)  # Teensy USB 시리얼 재연결 시 리셋되는 경우 대비
        self.ser.reset_input_buffer()
        ensure_stream_enabled(self.ser)

    def get_new_lines(self):
        lines_read = []
        while self.ser.in_waiting:
            raw = self.ser.readline().decode(errors="ignore").strip()
            if raw:
                lines_read.append(raw)
        return lines_read

    def close(self):
        self.ser.close()


class LogFileSource:
    """PuTTY가 세션 로깅으로 기록 중인 파일을 tail -f 방식으로 읽는 모드.
    PuTTY가 포트를 독점하고 있어 이 스크립트가 직접 포트를 열 수 없을 때 사용."""

    def __init__(self, path):
        self.path = path
        print(f"[plot_positions] 로그 파일 tail 모드: {path}")
        deadline = time.time() + 10.0
        while not os.path.exists(path):
            if time.time() > deadline:
                raise FileNotFoundError(
                    f"{path} 가 없습니다. PuTTY의 Session > Logging 에서 이 경로로 로깅을 켰는지 확인하세요."
                )
            time.sleep(0.2)
        self._fh = open(path, "r", encoding="utf-8", errors="ignore")
        self._fh.seek(0, os.SEEK_END)  # 새로 추가되는 내용부터만 읽음
        self._pos = self._fh.tell()
        self._start_time = time.time()
        self._warned = False

    def get_new_lines(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return []
        if size < self._pos:  # PuTTY가 로그 파일을 새로 만든 경우 (재시작 등)
            self._fh.seek(0)
            self._pos = 0
        lines_read = [raw.rstrip("\r\n") for raw in self._fh if raw.strip()]
        self._pos = self._fh.tell()
        if not lines_read and not self._warned and time.time() - self._start_time > 5.0:
            print("[plot_positions] 로그 파일에 PLOT 데이터가 없습니다. "
                  "PuTTY 창에 직접 'g'를 입력해 스트림을 켜세요.")
            self._warned = True
        return lines_read

    def close(self):
        self._fh.close()


def parse_plot_line(line):
    # PLOT,millis,link0,link1,link2,link3,motor0,motor1,motor2,motor3
    parts = line.split(",")
    if len(parts) != 10 or parts[0] != "PLOT":
        return None
    try:
        vals = [float(x) for x in parts[1:]]
    except ValueError:
        return None
    t_ms = vals[0]
    link_pos = vals[1:5]
    motor_pos = vals[5:9]
    return t_ms, link_pos, motor_pos


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="시리얼 포트 (예: COM5). 생략 시 자동 감지 시도")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--logfile",
                     help="시리얼 포트를 직접 여는 대신, PuTTY 세션 로깅으로 기록 중인 파일을 tail 방식으로 읽음 "
                          "(PuTTY가 포트를 이미 점유하고 있어 --port 를 못 쓸 때 사용)")
    ap.add_argument("--window", type=float, default=10.0, help="화면에 표시할 최근 구간(초), 기본 10초")
    ap.add_argument("--motor", action="store_true",
                     help="link axis(rad) 대신 motor axis(기어비 적용 후) 위치를 표시")
    ap.add_argument("--list-ports", action="store_true", help="사용 가능한 시리얼 포트만 출력하고 종료")
    args = ap.parse_args()

    if args.list_ports:
        list_ports()
        return

    if args.logfile and args.port:
        print("[plot_positions] --port 와 --logfile 은 동시에 쓸 수 없습니다.")
        sys.exit(1)

    if args.logfile:
        source = LogFileSource(args.logfile)
    else:
        source = SerialSource(pick_port(args.port), args.baud)

    enabled_idx = [i for i in range(4) if ENABLED_MOTORS.get(JOINT_CAN_IDS[i], True)]
    if not enabled_idx:
        print("[plot_positions] ENABLED_MOTORS 에서 모든 조인트가 False 입니다. 최소 하나는 켜세요.")
        sys.exit(1)

    buf_t = [deque() for _ in range(4)]
    buf_pos = [deque() for _ in range(4)]
    t0 = None

    axis_label = "motor axis position (rad)" if args.motor else "link axis position (rad)"
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))
    fig.suptitle("Teensy -> Motor commanded position (real-time)")

    lines = [None] * 4
    for i in range(4):
        ax = axes[i]
        label = f"{JOINT_NAMES[i]} (CAN {JOINT_CAN_IDS[i]})"
        if i in enabled_idx:
            (line,) = ax.plot([], [], color=f"C{i}")
            lines[i] = line
            ax.set_ylabel(axis_label, fontsize=8)
            ax.set_title(label, loc="left", fontsize=9)
            ax.grid(True, alpha=0.3)
        else:
            ax.set_title(f"{label} - disabled", loc="left", fontsize=9, color="gray")
            ax.set_yticks([])
            ax.text(0.5, 0.5, "disabled (ENABLED_MOTORS)", ha="center", va="center",
                     color="gray", transform=ax.transAxes)
    axes[-1].set_xlabel("time (s)")

    def update(_frame):
        nonlocal t0
        for raw in source.get_new_lines():
            parsed = parse_plot_line(raw)
            if parsed is None:
                continue
            t_ms, link_pos, motor_pos = parsed
            if t0 is None:
                t0 = t_ms
            t_s = (t_ms - t0) / 1000.0
            pos = motor_pos if args.motor else link_pos
            for i in enabled_idx:
                buf_t[i].append(t_s)
                buf_pos[i].append(pos[i])
                while buf_t[i] and (t_s - buf_t[i][0]) > args.window:
                    buf_t[i].popleft()
                    buf_pos[i].popleft()

        updated_artists = []
        for i in enabled_idx:
            lines[i].set_data(buf_t[i], buf_pos[i])
            updated_artists.append(lines[i])
            if buf_t[i]:
                t_max = buf_t[i][-1]
                axes[i].set_xlim(max(0, t_max - args.window), max(args.window, t_max))
                lo, hi = min(buf_pos[i]), max(buf_pos[i])
                pad = max(0.02, (hi - lo) * 0.1)
                axes[i].set_ylim(lo - pad, hi + pad)
        return updated_artists

    ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)

    try:
        plt.show()
    finally:
        source.close()


if __name__ == "__main__":
    main()
