"""
A tool that plots, in real time, the position commands (link/motor axis) that Teensy actually
sends to the motors.

teensy.ino's control loop (50Hz) computes the position to send to each joint every tick and
transmits it over CAN; a 'g' command (toggling plotStreamEnabled) was added to the firmware so
that this same value (last_cmd_link_pos / last_cmd_motor_pos) is also emitted over serial as one
CSV line. This script only parses lines starting with "PLOT,..." and plots them in real time.

There are two data source modes:
  1. --port  : this script opens the serial port directly (if another program such as PuTTY
               already has the port open, this fails on Windows since it can't be opened at the
               same time).
  2. --logfile : reads, like tail -f, a text file that PuTTY is writing to via session logging.
               Use this mode when PuTTY needs to keep being used as the serial console. PuTTY
               settings:
               Session > Logging > Log file to write to = this file's path,
               the logging mode can be either "Printable output" or "All session output".
               In this mode the script can't send the 'g' command on your behalf, so if PLOT data
               isn't showing up, type g directly into the PuTTY window to turn the stream on.

See the bottom of the file, or `python plot_positions.py --help`, for usage.
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
JOINT_CAN_IDS = [11, 13, 12, 14]  # Keep the same order/values as teensy.ino's JOINT_CAN_IDS
BAUD_DEFAULT = 115200

# Toggle per-joint whether to plot it, keyed by CAN ID as true/false.
# Any CAN ID not listed here defaults to True.
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
        print(f"[plot_positions] Auto-selected port: {ports[0].device}")
        return ports[0].device
    print("[plot_positions] Specify the serial port with --port. Available ports:")
    list_ports()
    sys.exit(1)


def ensure_stream_enabled(ser, timeout_s=2.0):
    """Checks whether the PLOT stream is already on, and sends 'g' to turn it on if it's off."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("PLOT,"):
            print("[plot_positions] Stream is already on. Using it as-is.")
            return
    print("[plot_positions] Stream is off, sending the 'g' command to turn it on.")
    ser.write(b"g")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("PLOT,"):
            return
    print("[plot_positions] WARNING: PLOT data still not visible after sending 'g'. "
          "Check whether Teensy is receiving position commands over UDP (ext_control_active).")


class SerialSource:
    """Mode where this script opens the serial port directly and reads it."""

    def __init__(self, port, baud):
        print(f"[plot_positions] Connecting to {port} @ {baud}bps...")
        self.ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(2.0)  # In case Teensy resets on USB serial reconnect
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
    """Mode that reads, like tail -f, a file PuTTY is writing to via session logging.
    Used when PuTTY has exclusive hold of the port and this script can't open it directly."""

    def __init__(self, path):
        self.path = path
        print(f"[plot_positions] Log file tail mode: {path}")
        deadline = time.time() + 10.0
        while not os.path.exists(path):
            if time.time() > deadline:
                raise FileNotFoundError(
                    f"{path} does not exist. Check whether PuTTY's Session > Logging has logging turned on to this path."
                )
            time.sleep(0.2)
        self._fh = open(path, "r", encoding="utf-8", errors="ignore")
        self._fh.seek(0, os.SEEK_END)  # Only read content added from here on
        self._pos = self._fh.tell()
        self._start_time = time.time()
        self._warned = False

    def get_new_lines(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return []
        if size < self._pos:  # In case PuTTY created a new log file (e.g. on restart)
            self._fh.seek(0)
            self._pos = 0
        lines_read = [raw.rstrip("\r\n") for raw in self._fh if raw.strip()]
        self._pos = self._fh.tell()
        if not lines_read and not self._warned and time.time() - self._start_time > 5.0:
            print("[plot_positions] No PLOT data in the log file. "
                  "Type 'g' directly into the PuTTY window to turn on the stream.")
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
    ap.add_argument("--port", help="Serial port (e.g. COM5). If omitted, attempts auto-detection")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--logfile",
                     help="Instead of opening the serial port directly, tail a file PuTTY is writing to via session logging "
                          "(use when PuTTY already occupies the port and --port can't be used)")
    ap.add_argument("--window", type=float, default=10.0, help="Recent time span to display (seconds), default 10s")
    ap.add_argument("--motor", action="store_true",
                     help="Show motor axis (after gear ratio applied) position instead of link axis (rad)")
    ap.add_argument("--list-ports", action="store_true", help="Print available serial ports only, then exit")
    args = ap.parse_args()

    if args.list_ports:
        list_ports()
        return

    if args.logfile and args.port:
        print("[plot_positions] --port and --logfile cannot be used together.")
        sys.exit(1)

    if args.logfile:
        source = LogFileSource(args.logfile)
    else:
        source = SerialSource(pick_port(args.port), args.baud)

    enabled_idx = [i for i in range(4) if ENABLED_MOTORS.get(JOINT_CAN_IDS[i], True)]
    if not enabled_idx:
        print("[plot_positions] All joints are False in ENABLED_MOTORS. Enable at least one.")
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
