# Branches in this repo

**Every branch here is a standalone firmware build for one specific job — not a feature branch
meant to be merged, rebased onto another, or combined.** There is no shared trunk that
accumulates features across branches: `teensy/teensy.ino` on each branch is a complete,
self-contained snapshot, and two branches can (and do) speak entirely different wire protocols,
wire the CAN bus differently, or control a different number of arms. Flashing "the wrong" branch
for what you're trying to do isn't a merge conflict, it's a firmware that doesn't understand the
commands you're sending it (or worse, understands *different* commands that happen to share a
letter).

**Workflow: pick the one branch that matches your current job, flash only that one, don't try to
layer another branch's features onto it.** If you need a feature from two different branches at
once, that's a new branch (a fresh copy of the closer one, with the other feature ported in
deliberately) — not a merge.

## Quick reference — which branch for which job

| I want to... | Flash this branch | Arms | Transport |
| --- | --- | --- | --- |
| Record a teleoperated dataset (bilateral, human on the master arm) | **`teleop-bi-c`** | master + slave | serial (control) + serial & UDP (telemetry) |
| Run a trained policy on the real arm, no human in the loop (eval) | **`goal`** | slave only | serial (`'c'`/`'d'`) + UDP (goal stream) |
| Drive the real arm live from an Isaac Sim simulation | **`isaacsim-udp`** | slave only | serial (`'e'`/`'d'`/`'p'`/`'g'`/`'f'`) + UDP (sim targets) |
| Sanity-check CAN wiring / read raw motor feedback, no control | **`getpos`** | any motors on the bus | serial (read-only) |
| Bench-test one or two motors in isolation before wiring a full build | **`checkmotor`** | 1-2 motors | serial (`'e'`/`'d'`) |
| Reproduce/compare against older bilateral prototypes | `main`, `teleop`, `teleop-bi` | see below | serial |

If your task isn't in this table, it's probably not supported by any current branch yet — see the
per-branch notes below for what each one actually does before assuming.

---

## `teleop-bi-c` — current bilateral teleoperation (data recording)

**Use this for:** recording teleoperated datasets with `lerobot_robot_forte_arm`
(`lerobot-record`/`lerobot-teleoperate`, `forte_arm`/`forte_arm_master`).

Bilateral (master + slave), dual CAN bus, 4 motors total:
- CAN1: master `1`→slave `11` (shoulder_yaw), master `2`→slave `12` (shoulder_roll)
- CAN2: master `3`→slave `13` (shoulder_pitch), master `4`→slave `14` (elbow_pitch)

Control scheme is **position-torque**: the slave is position-controlled to mirror the master
(`SLV_KP`/`SLV_KD`), while the master receives real force feedback computed from the slave's
*actual sensed contact torque* (not just a position spring) plus a virtual-wall reaction torque
when the slave hits the ±12.4 rad hardware limit (`K_WALL`), clamped to `MAX_SAFE_TORQUE` for
operator safety. This is `teleop-bi-p-t`'s exact control loop, untouched.

What's added on top of `teleop-bi-p-t`:
- **`'c'` (serial) — logging-only zero-calibration.** Captures each motor's current raw position
  as a per-motor offset; from then on the periodic status line prints `raw - offset` instead of
  raw absolute. Does **not** touch the bilateral position offset (`'e'`'s job), the control loop,
  or move anything — it only changes what the status line *prints*. This is what makes
  `lerobot_robot_forte_arm`'s recorded `.pos` values zero-relative without any host-side
  subtraction.
- **UDP telemetry**, sent in parallel with (not replacing) the existing serial status line —
  identical text on both channels (`sendTelemetryLine()`), unicast to a hardcoded host IP:port
  (`192.168.1.10:5006` by default). This means the operator's serial console (minicom) never has
  to be closed for `lerobot_robot_forte_arm`'s Python side to read positions — it reads UDP only,
  serial is free for the human the entire session, including every episode's reset window.

Serial commands: `'e'` (enable both arms, compute master↔slave position offset from current pose),
`'c'` (logging-only zero, see above), `'d'` (disable). No goal-position command — the slave can
only mirror the master, never move on its own.

**Network setup required** (new versus `teleop-bi-p-t`): direct Ethernet cable between host and
Teensy, host static IP `192.168.1.10/24`, Teensy static IP `192.168.1.15/24`, no gateway.

## `teleop-bi-p-t` — bilateral teleoperation, position-torque (pre-UDP)

Identical control loop and wiring to `teleop-bi-c` (see above — that branch is this one plus a
purely-additive `'c'` + UDP telemetry). Use this instead of `teleop-bi-c` only if you specifically
need the older serial-only telemetry behavior, or as the base to port a new addition from rather
than building on top of `teleop-bi-c`'s.

Serial commands: `'e'` (enable + compute offset), `'d'` (disable). No `'c'`, no UDP, no Ethernet
hardware needed at all.

## `teleop-bi` — bilateral teleoperation, position-position (older)

Dual CAN, 4 motors, but wired **differently** from `teleop-bi-p-t`/`teleop-bi-c` — CAN1 carries
master `1`→slave `11` and master `3`→slave `13`, CAN2 carries master `2`→slave `12` and master
`4`→slave `14` (interleaved, not grouped by CAN bus the way the newer branches are).

Control scheme is **position-position**, not position-torque: both the master and the slave are
position-controlled toward each other (`KP`/`KD` on both sides) — a symmetric position spring, not
real force/contact-torque feedback. The code comments call this "haptic feedback," but there's no
torque sensing driving it the way `teleop-bi-p-t`/`teleop-bi-c` do. Historical — superseded by
`teleop-bi-p-t`'s real torque feedback for anything where haptic quality matters.

Serial commands: `'e'`, `'d'`.

## `teleop` — bilateral teleoperation, single pair, position-only (earliest)

Single CAN bus (CAN1) with only 2 motors active (master `1`→slave `11`), CAN2 declared but empty
(`NUM_MOTORS_CAN2 = 0`). Position-only mirroring (`KP`/`KD`, no torque feedback at all), plus a
`POSITION_JUMP_MARGIN_RAD` safety check against implausible position jumps between reads. Earliest
bilateral prototype in this repo, predates the 4-motor dual-CAN builds. No practical reason to
flash this over `teleop-bi-p-t`/`teleop-bi-c` today.

Serial commands: `'e'`, `'d'`.

## `main` — earliest prototype, single CAN, 3 motors

Single CAN bus, 3 motors (master `1,2,3` → slave `11,12,13`), plain position mirroring, no torque
feedback, no safety margin checks beyond the basic ones common to every branch. The oldest/simplest
build in this repo — kept for history, not a realistic choice for current work.

Serial commands: `'e'`, `'d'`.

---

## `goal` — standalone single-arm eval (trained policy control)

**Use this for:** evaluating a trained policy on the real arm with
`lerobot_robot_forte_arm`'s `ForteArmGoal` (`lerobot-rollout` / `lerobot-record
--policy.path=...`, `forte_arm_goal`).

Single arm (slave only), no master, no bilateral logic, no haptic feedback at all — this build only
exists to let a host stream goal joint positions to the slave. 4 motors: CAN1 `11` (shoulder_yaw),
`12` (shoulder_roll); CAN2 `13` (shoulder_pitch), `14` (elbow_pitch).

Hybrid transport, deliberately split by job (referenced from `isaacsim-udp`, which solved the same
problem first):
- **USB serial**: `'c'` (zero-calibration + arms the per-joint safety clamp) and `'d'` (disable) —
  single-char, human-supervised, no line-parsing state machine to get stuck in.
- **Ethernet UDP** (`192.168.1.15:5005`, direct cable, no gateway): the continuous goal-position
  stream, plain CSV `"<yaw>,<pitch>,<roll>,<elbow>"` (raw motor radians, kinematic order — motor
  ids `11,13,12,14`, *not* CAN-wiring order). Every packet both sets the 4 targets and
  enters/refreshes GOAL mode; a 500ms watchdog (`GOAL_TIMEOUT_MS`) auto-disables if packets stop
  arriving.

Targets are **absolute**, not baseline+delta — unlike `isaacsim-udp`, this pipeline's dataset and
this arm already share the same raw-motor-radian units (no simulation coordinate frame to
reconcile), so there's nothing to reconcile a delta scheme would be solving.

`'c'` also arms a **per-joint safety clamp**: once calibrated, `JOINT_LIMIT_MIN/MAX_CAN1/CAN2`
(hand-tuned ranges, relative to the `'c'`-time zero) clamp every incoming UDP target before it's
sent to the motor, logging `[CANx JOINT LIMIT] ... clamped to [...]` whenever a target gets
clamped. Uncalibrated, only the wide ±12.4 rad hardware protocol limit applies.

## `isaacsim-udp` — real arm driven live from Isaac Sim

**Use this for:** streaming simulated joint targets from Isaac Sim to the physical slave arm in
real time. Not used by `lerobot_robot_forte_arm` — this is a separate integration with its own
host-side scripts (`isaacsim_script/arm-ik/{test_udp,multiple_ik_udp}.py`, not in this repo).

Single arm (slave only), 4 motors (CAN ids `11,13,12,14`, matching Isaac Sim's joint order:
shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch). Unlike every other branch here, this one
**does** apply the motor:link `GEAR_RATIO` and a per-joint `JOINT_SIGN` correction — because it has
to reconcile Isaac Sim's simulated link-space convention with the real motor's raw shaft position,
a problem none of the other branches have (they all work in the same raw-motor units end to end).

UDP payload carries a leading type marker (unlike `goal`'s bare CSV, because this port has to
disambiguate more than one kind of packet): `"P,<yaw>,<pitch>,<roll>,<elbow>"` in **link-space**
radians. Targets are **baseline + delta**, not absolute: `'e'` snapshots the arm's current position
as a baseline the instant it's sent, and every subsequent UDP target is applied as a *change* from
whatever Isaac Sim's target was at that same instant — so however Isaac Sim's simulated arm happens
to be posed when you hit enable, the real arm's very first commanded delta is always zero, and it
can never jump.

Serial commands: `'e'` (enable + capture baseline), `'d'` (disable, invalidates baseline), `'p'`
(pause/resume the periodic status log — doesn't affect control), `'g'` (toggle a real-time
`PLOT,...` CSV stream consumed by the companion `plot_positions.py` script), `'f'` (attempt to
clear a motor fault — stops control, cycles the motor through Run Mode again, then leaves it
disabled; requires `'e'` again afterward, doesn't resume control automatically).

---

## `getpos` — passive CAN feedback dump (diagnostic)

**Use this for:** confirming CAN wiring is correct and motors are responding, without powering,
enabling, or moving anything.

Reads and periodically prints position/velocity/torque feedback for a fixed list of 8 motor ids
(`1,2,3,4,11,12,13,14` — every master and slave id used across the bilateral branches), whether or
not they're wired as master/slave pairs. No `'e'`, no control loop, no way to move a motor from
this firmware at all — purely observational. Useful as a first check after wiring a new motor
before trusting it to any of the teleop/goal branches.

## `checkmotor` — single/pair motor exercise tool (bench test)

**Use this for:** verifying one or two motors' wiring, direction, and response in isolation before
integrating them into a full bilateral or goal build.

Single CAN bus, a small hardcoded motor id list (`{11, 13}` as checked in). `'e'` enables the listed
motors and slowly rotates them in the negative direction at a fixed rate (`ROTATE_SPEED`), stopping
automatically at `ROTATE_RANGE_LIMIT` from the start position; `'d'` disables and stops. Deliberately
weak control gains (`KP`/`KD`) since this is a confirmation test, not production control — edit
`MOTOR_IDS` for the motor(s) you're actually testing before flashing.
