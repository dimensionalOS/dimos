# M20 rl_deploy v1.5.8: controller specification

What the vendor walking controller (`/opt/robot/share/rl_deploy`, binary md5 b16070a4bf2abb52a0ce33e863a4df2e) does,
decoded from its disassembly and confirmed in MuJoCo. Both re-implementations follow this file and read the same
config, `dimos/robot/deeprobotics/m20/controller/rust/m20_controller.toml`:

- the Python reference sim (this folder): `python -m dimos.simulation.m20_policy_sim.run_all`
- the Rust controller: `dimos/robot/deeprobotics/m20/controller/rust` (tested against the sim, never run on the robot)

Tags: [C] read from code or constants, [I] inferred or tuned in the sim, [U] unknown.

## Getting the policies

The 15 ONNX policies are embedded in rl_deploy as `__policy_<name>_` byte arrays (vendor IP, not committed). Copy the
binary off the AOS board (read-only), then:

    python -m dimos.simulation.m20_policy_sim.extract_policies rl_deploy ~/.cache/dimos/m20_policies

`M20_POLICY_DIR` points both implementations elsewhere. 13 are referenced by the gait table; `policy_m20_38` and
`policy_m20_rough_6` are embedded but unused.

## Conventions

- Joint order everywhere ("robot order"): FL, FR, HL, HR x (hipx, hipy, knee, wheel); index = 4 leg + joint. Joint
  angles are the vendor MJCF joint frame: the hardware offset / direction tables cancel out [C].
- Pose convention for the stand-up / lie-down IK: hind legs negate hipy and knee, FR / HR negate hipx [C].
- Leg IK for a hip-to-wheel height h (L1 = L2 = 0.25 m): hipy = -acos((h^2 + L1^2 - L2^2) / (2 h L1)),
  knee = pi - acos((L1^2 + L2^2 - h^2) / (2 L1 L2)), each clamped to the front-leg range [C].
- Gravity obs = R^T (0, 0, -1) with R from the IMU attitude; gyro is body frame [C].
- Motor command per joint (16 x 5 matrix, columns kp, q, kd, dq, tau) [C]: the driver applies
  tau = kp (q - q_meas) + kd (dq - dq_meas) + tau_ff. rl_deploy sends position = (q - offset) dir,
  velocity = dq dir, torque = tau dir, kp, kd unchanged, control word 4. No clamps on gains or torque at this point.
  Wheels always get kp 0 and a velocity target.

## Rates and real-time behavior [C]

- Main loop: 1 kHz timerfd + epoll; each tick refreshes robot data, runs the state, checks the transition, publishes
  state feedback, then sleeps 0.5 ms (retried on EINTR). A failed timer read only prints.
- Policy: own thread `PolicyRunner` pinned to cores 4-7, 50 Hz (decimation 20). The 1 kHz loop sends the latest targets.
- Watchdog: `epoll_wait == -1` exits the process with no EINTR check (the known exit-on-EINTR bug; do not ptrace it).
- Command timeout: `/STEER` or `/REAL_STEER` silent for 150 ms zeroes vx, vy, wz; the state is kept.
- The Rust controller keeps the 1 kHz / 50 Hz split, sleeps on absolute deadlines and retries every syscall on EINTR,
  never exits on a signal (SIGINT / SIGTERM send joint damping first), and drops a policy request when the worker is
  busy instead of queueing it (no timers for the policy).

## State machine [C]

State codes: Idle 0, StandUp 1, JointDamping 2, StartDamping 3, LieDown 4, SetZero 5, CarMove 0x10, RLControl 0x11,
JointTest 0x12, SdkControl 0x64. UserCommand: +0x08 fault mask, +0x0c target state (`/MOTION_STATE`), +0x10 gait
(`/GAIT`), +0x20/+0x24/+0x28 normalized vx, vy, wz (`/STEER`, or `/REAL_STEER` m/s divided by the per-gait maxima).

| state | command each tick | gains (hipx, hipy, knee; wheel) |
|---|---|---|
| Idle (passive) | all zero | 0 / 0 |
| StandUp | 0-1.5 s: per-joint cubic Hermite from the entry pose (q0, dq0) to IK(0.12) = (0, -1.3284, 2.6568); 1.5-3 s: body height cubic 0.12 -> 0.38 m through the IK, dq by 1 ms finite difference; then holds IK(0.38) = (0, -0.7075, 1.415) | kp 200, kd 4; wheel kp 0, kd 2 |
| LieDown | 0-2 s: cubic to IK(0.03) = (0, -1.5108, 2.793); 2-4 s: damping only (kp 0, kd kept); then all zero | kp 300, kd 4.5; wheel kd 2 |
| JointDamping | kp 0, q = dq = 0 | kd 4; wheel 1 |
| RLControl | policy targets (50 Hz), runner by gait | per runner (below) |
| StartDamping, SetZero, CarMove, JointTest, SdkControl | not re-implemented ([U] internals; StartDamping kd 0.5 x damping [I]) | |

Transitions (any state: fault mask != 0 -> JointDamping):

| from | to | condition |
|---|---|---|
| Idle | StandUp | target StandUp, >= 2 s since the robot interface came up |
| Idle | RLControl | target RLControl with gait 0xf003 (not re-implemented) |
| StandUp | RLControl / LieDown | target 0x11 / 4, only after 3 s |
| StandUp, LieDown, RLControl | JointDamping | target 2 |
| LieDown | StandUp | target 1, only after 4 s |
| RLControl | LieDown | target 4, not while a stand-in policy runs |
| JointDamping | Idle | after 2 s, or at once if target != 2; under a fault it stays (and resets joint errors every 3 s) |

Faults (SafeController::Run, 1 kHz, from its first tick; the fault byte it writes into the UserCommand sends
StandUp, LieDown and RLControl to JointDamping and keeps Idle in Idle). Its fault byte gets:
- Tilt (0x183980, no debounce): |roll| or |pitch| of the IMU's own angles (f32 rad) strictly above a limit set by the
  UserCommand's target state and gait: Idle (0, 3) 0.349066 rad (20 deg), StandUp (1) 0.7854, RLControl (0x11) roll
  0.7854 and pitch 1.134464 (65 deg; 1.5708 on 0x1002), none on 0x2xxx (Extre) and 0xf003, none for any other target
  (constants 0x1e1bf8 / 0x1e1c00 / 0x1e1c08 / 0x1e1c10).
- Joint data (IsJointDataNormal 0x1773a8), per joint: 31 readings in a row with a NaN in position, velocity or
  torque; 51 in a row identical to the one before (a frozen motor, all 16 joints); 31 in a row of a hip / knee
  position more than 0.1 + 0.14 rad past ControlParameters' front-leg range, mirrored per leg.
- Drivers: every tick, any status word other than 1; every 1000 ticks (after the first 1000), the temperatures,
  latched per joint: a motor at 120 deg C or more until below 100, a driver at 100 or more until below 80.
Not faults: the IMU's age (a new sample more than 20 ms after the last, or none for 20 ms) and a NaN in its readings
set status bits SafeController publishes (0x158 bits 1 and 0; PubMotionStatus logs "IMU数据更新超时"), and leave the
fault byte alone; so does a missing joint reply.
The re-implementations raise exactly these faults (the tilt's target: the operator's, joint damping while in it,
rl_control's own gait; the drivers' status words and temperatures from the motor bus's replies) and, like rl_deploy,
keep Idle in Idle under a fault. Their one extra: an inference error (a model that fails to run) is a fault into
JointDamping, where rl_deploy's Ort::ThrowStatus would end the process. After a fault they ignore the operator's
standing request until a different one arrives (rl_deploy resets the target to damping). The IMU sample each step
uses is the latest that arrived, however old, as rl_deploy's. examples/fault_replay.rs runs them over a session's every reply:
none on 11-20 14:16 (218687 replies) and 16:36 (142448), nor on 11-24 (118525; no tilt there: its IMU is not in the
capture).

### RLControl entry and gait handling [C]

- Accepted entry gaits: 0x1001, 0x3001, 0x3004, 0xf003; any other becomes 0x1001 ("protect to basic flat gait").
  Later gait changes (the gait-change marker UC+0x14) are followed as sent.
- Entering on 0x1001 first runs a stand-in: `m20_finetune_stand_15kg` (gait 0xf001) for 3 s on the first entry,
  1.5 s later, with stand mode 0, then 0.5 s with mode -1, then BasicFlat. RLControl -> LieDown is refused meanwhile.
- 0x3001 (NaviCharge) runs the same stand policy for 1.5 s every time the gait becomes 0x3001.
- NaviCharge multiplies kp / kd by 1.3 for its first 29 policy steps. A x0.5 for 49 steps when UC+0x18 == 0x1004 [U].
- Overheat fallback: motor >= 100 C or driver >= 90 C, zero command, body speed < 0.08 and > 70 C for 1 s drops to
  the 0xf001 stand policy (not re-implemented).

### Gait code -> runner -> policy [C]

| gait | runner | policy |
|---|---|---|
| 0x1001 (and unknown codes) | BasicFlat | policy_m20_50 |
| 0x1002 | BasicPlat | policy_ca9bplatform |
| 0x1003, 0x3003 | BasicStair, NaviStair | policy_m20_5 |
| 0x1004 | BasicBackknee | policy_m20_backknee6 |
| 0x2001 Move, 0x2002 Jump, 0x2003 Bipedal, 0x2004 Inv_Bipedal | Extre | m20_forward_policy_2 |
| 0x3001 | NaviCharge | m20_navi_charge_v8_0_isaaclab |
| 0x3002 | NaviFlat | m20_navi_flat_v4_2 |
| 0x3004 | FinetuneStand | m20_sixaxis_stand |
| 0x3005 | NaviOrienteer | m20_navi_stair_v7_0_himloco |
| 0x5001 | HeightGap | m20_hmap_motion_v0_2 |
| 0xf001 | SixAxisStand | m20_finetune_stand_15kg |
| 0xf002 | StepMove | m20_finetune_forward |
| none | StandPolicy (constructed, never selected) | m20_flat_v1_4 |

`/REAL_STEER` (m/s, rad/s) is normalized per gait by: 0x1001 (2, 0.8, 1.5), 0x1002 (1.5, 0.5, 1.5), 0x1003 / 0x3003
(2, 1, 2; |v| < 0.2 rescaled up to 0.2), 0x1004 (2, 0.8, 1.5), 0x2001 (3, 1, 2), 0x2002-4 (2, 1, 2), 0x3001
(0.5, 0.3, 0.5), 0x3002 (2, 1, 1.5), 0x3005 (1.5, 1, 1.5), 0xf002 (3.9, 0.9, 3), others (2, 0.4, 0.6) [C].
For 0xf001 the axes become pose inputs (yaw, -x, -y, roll, pitch, height = max(-z, 0)); for 0x3004
(7 yaw, -24 x, -18 y, height) [C].

## Shared policy pipeline [C]

1. Normalized axes in [-1, 1] x the runner's command maxima (optionally slew-limited per policy step) = the command.
2. Build the frame (tables below), update the history, run the network.
3. Clip the output (+-100, HeightGap +-50); the clipped output is next step's last action.
4. Map to joints: leg target = default + 0.25 a (or the absolute output for the 36-frame runners), wheel velocity =
   scale x a, in f32; then PolicyRunnerBase::limit_goal_pos_vel (every runner, after every step) clamps each target
   to its compiled-in f32 range: hipy / knee to the joint range, hipx to the range widened by a quarter of its width
   at each end (fl hipx -0.698 .. 0.873: rl_deploy sent 0.628 there on 2026-11-20, past the joint range), wheel
   velocities to +-79.3 rad/s.
5. `vel` (six-frame runners) is the body-frame base linear velocity estimate (vx, vy, vz).

Joint limits (rad, also the MJCF ranges): hipx FL/HL [-0.436, 0.611], FR/HR [-0.611, 0.436]; hipy front
[-2.286, 2.583], hind [-2.583, 2.286]; knee front [-2.809, 2.792], hind [-2.792, 2.809]. Torque limits 76.4 N m legs,
21.6 N m wheels (MJCF).

### Gait clock (FinetuneStand, SixAxisStand, StepMove) [C]

Counter i per policy step, dt 0.02, t = i dt, frequency f (starts 1 Hz), half period P = 0.5 / f; sign
s = sign(sin(2 pi f t)) (+1 at 0); rem = P - fmod(t, P) (0 when fmod <= 1e-6). At each half-period boundary
f = (1 + |wz| / 12) max(1 + vx / 15, 1 + |vy| / 3.5), clamped to [1, 1.5], and i is rescaled to keep the phase
([I] which filtered command it reads).

### HeightGap height map [C, partly U]

`/HEIGHT_IMAGE` (32FC1, from height_map_nav), 41 x 11. Per cell h: NaN, Inf or |h| > 1.2 rejected; v = -(h + off);
kept if v > 0.18 and row >= 20, else 0.6; obs = 5 v. [U]: `off`, the grid spacing and orientation. The sim uses
v = base height above the cell - 0.1 on a 0.05 m grid, under which the policy stands but walks and climbs wrongly.

## Per-runner observation, command and action

Generated from m20_controller.toml by `python -m dimos.simulation.m20_policy_sim.spec_tables`.

<!-- GENERATED: spec_tables.py -->

### basic_flat: `policy_m20_50.onnx` (gaits 0x1001)

- network input `state` [1, 318] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (-0.1, -0.8, 1.6), HL = (-0.1, 0.8, -1.6); FR / HR mirror hipx
- kp (80, 80, 80, 0), kd (6, 6, 6, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (2, 1, 2), slew per step (0.025, 0.025, 0.04), |wz| <= 0.8 while vx > 1
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.05 |
| 37-52 | previous clipped network output (network output order) | - | 1 |

### basic_plat: `policy_ca9bplatform.onnx` (gaits 0x1002)

- network input `state` [1, 318] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (0.1, -0.6, 1.05), HL = (0.1, 0.6, -1.05); FR / HR mirror hipx
- kp (80, 100, 100, 0), kd (2.5, 2.5, 2.5, 2) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (1.5, 0.5, 1.5), no slew
- action: 16 interleaved per leg (hipx, hipy, knee, wheel) x 4; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.01 |
| 37-52 | previous clipped network output (network output order) | - | 1 |

### basic_stair: `policy_m20_5.onnx` (gaits 0x1003, 0x3003)

- network input `state` [1, 318] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (-0.1, -0.8, 1.6), HL = (-0.1, 0.8, -1.6); FR / HR mirror hipx
- kp (80, 80, 80, 0), kd (3, 3, 3, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (2, 1, 2), slew per step (0.04, 0.04, 0.08)
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.05 |
| 37-52 | previous clipped network output (network output order) | - | 1 |

### basic_backknee: `policy_m20_backknee6.onnx` (gaits 0x1004)

- network input `state` [1, 318] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (-0.1, -0.8, 1.6), HL = (-0.1, -0.8, 1.6); FR / HR mirror hipx
- kp (80, 80, 80, 0), kd (3, 3, 3, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (2, 0.8, 1.5), no slew
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.05 |
| 37-52 | previous clipped network output (network output order) | - | 1 |

### navi_charge: `m20_navi_charge_v8_0_isaaclab.onnx` (gaits 0x3001)

- network input `state` [1, 360] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (0, -0.8, 1.6), HL = (0, 0.8, -1.6); FR / HR mirror hipx
- kp (60, 60, 60, 0), kd (1.5, 1.5, 1.5, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0); kp / kd x1.3 for the first 29 steps
- command maxima (vx, vy, wz) = (0.5, 0.3, 0.5), no slew
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (60 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-10 | gait clock sin(2 pi phi), cos(2 pi phi), phi += 1.5 x 0.02 per step | - | 1 |
| 11-15 | constant 1.5, 0.5, 0, 0, 0.5 | - | 1 |
| 16-27 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 28-39 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 40-43 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.01 |
| 44-59 | previous clipped network output (network output order) | - | 1 |

### navi_flat: `m20_navi_flat_v4_2.onnx` (gaits 0x3002)

- network input `state` [1, 324] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (0, -0.8, 1.6), HL = (0, 0.8, -1.6); FR / HR mirror hipx
- kp (60, 60, 60, 0), kd (1.5, 1.5, 1.5, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (2, 1, 1.5), no slew
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (54 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-9 | constant 0.52 (param `body_height`) | - | 1 |
| 10-21 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 22-33 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 34-37 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.01 |
| 38-53 | previous clipped network output (network output order) | - | 1 |

### navi_orienteer: `m20_navi_stair_v7_0_himloco.onnx` (gaits 0x3005)

- network input `state` [1, 318] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (0, -0.8, 1.6), HL = (0, 0.8, -1.6); FR / HR mirror hipx
- kp (60, 60, 60, 0), kd (1.5, 1.5, 1.5, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (1.5, 1, 1.5), no slew
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.01 |
| 37-52 | previous clipped network output (network output order) | - | 1 |

### stand_policy: `m20_flat_v1_4.onnx` (gaits none)

- network input `state` [1, 366] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (0.1, -0.75, 1.5), HL = (0.1, 0.75, -1.5); FR / HR mirror hipx
- kp (80, 100, 100, 0), kd (2.5, 2.5, 2.5, 2) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (1, 1, 1), no slew
- action: 16 interleaved per leg (hipx, hipy, knee, wheel) x 4; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (61 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-10 | gait clock sin(2 pi phi), cos(2 pi phi), phi += 2 x 0.02 per step | - | 1 |
| 11-15 | constant 2, 0.5, 0, 0, 0.5 | - | 1 |
| 16-27 | joint position - default: fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad | 1 |
| 28-39 | joint velocity: fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad/s | 0.05 |
| 40-43 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.01 |
| 44-59 | previous clipped network output (network output order) | - | 1 |
| 60-60 | constant 1 | - | 1 |

### height_gap: `m20_hmap_motion_v0_2.onnx` (gaits 0x5001)

- network input `state` [1, 3024] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (0, -0.8, 1.6), HL = (0, 0.8, -1.6); FR / HR mirror hipx
- kp (80, 80, 120, 0), kd (3, 3, 4, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (2, 0.5, 1.5), no slew
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-50; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (504 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.01 |
| 37-52 | previous clipped network output (network output order) | - | 1 |
| 53-503 | height map 41 x 11, row-major (see HeightGap notes) | m | 5 |

### extre: `m20_forward_policy_2.onnx` (gaits 0x2001, 0x2002, 0x2003, 0x2004)

- network inputs `state` [1, 36, 53] (oldest first, current last), `command` [1, 5], `uncertain` [1, 1] = 0; outputs `output` [1, 16]
- default pose FL (hipx, hipy, knee) = (0, -0.8, 1.6), HL = (0, 0.8, -1.6); FR / HR mirror hipx
- kp (60, 60, 60, 0), kd (1.5, 1.5, 1.5, 1) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (3, 1, 2), no slew
- action: 16 interleaved per leg (hipx, hipy, knee, wheel) x 4; no clip; leg target = output (the ONNX applies 0.25 and the joint clip itself); wheel velocity target = 20 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | accelerometer specific force x, y, z (+1 g on z at rest) | m/s^2 | 0.101937 |
| 3-5 | body angular velocity x, y, z | rad/s | 0.25 |
| 6-8 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 9-24 | previous clipped network output (network output order) | - | 1 |
| 25-36 | joint position (absolute): fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad | 1 |
| 37-52 | joint velocity: fl_hipx, fl_hipy, fl_knee, fl_wheel, fr_hipx, fr_hipy, fr_knee, fr_wheel, hl_hipx, hl_hipy, hl_knee, hl_wheel, hr_hipx, hr_hipy, hr_knee, hr_wheel | rad/s | 0.1, 0.1, 0.1, 0.0333333 |

`command` vector:

| index | value |
|---|---|
| 0 | command vx |
| 1 | command vy |
| 2 | command wz |
| 3 | constant 0 |
| 4 | param `mode` x 1 |

### finetune_stand: `m20_sixaxis_stand.onnx` (gaits 0x3004)

- network inputs `state` [1, 36, 53] (oldest first, current last), `command` [1, 21], `uncertain` [1, 1] = 0; outputs `output` [1, 16]
- default pose FL (hipx, hipy, knee) = (0, -0.65, 1.3), HL = (0, 0.65, -1.3); FR / HR mirror hipx
- kp (60, 60, 60, 0), kd (1.5, 1.5, 1.5, 1) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (0, 0, 0), no slew
- action: 16 interleaved per leg (hipx, hipy, knee, wheel) x 4; no clip; leg target = output (the ONNX applies 0.25 and the joint clip itself); wheel velocity target = 20 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | accelerometer specific force x, y, z (+1 g on z at rest) | m/s^2 | 0.101937 |
| 3-5 | body angular velocity x, y, z | rad/s | 0.25 |
| 6-8 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 9-24 | previous clipped network output (network output order) | - | 1 |
| 25-36 | joint position (absolute): fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad | 1 |
| 37-52 | joint velocity: fl_hipx, fl_hipy, fl_knee, fl_wheel, fr_hipx, fr_hipy, fr_knee, fr_wheel, hl_hipx, hl_hipy, hl_knee, hl_wheel, hr_hipx, hr_hipy, hr_knee, hr_wheel | rad/s | 0.1, 0.1, 0.1, 0.0333333 |

`command` vector:

| index | value |
|---|---|
| 0 | constant 0 |
| 1 | constant 0 |
| 2 | constant 0 |
| 3 | param `height` x -0.25 + 0.51 |
| 4 | param `roll` x 0.253073 (max 14.5 deg) |
| 5 | param `pitch` x 0.427606 (max 24.5 deg) |
| 6 | param `mode` x 1 |
| 7 | gait clock half_period x 5 |
| 8 | gait clock sign x 1 |
| 9 | gait clock sign x -1 |
| 10 | gait clock sign x -1 |
| 11 | gait clock sign x 1 |
| 12 | gait clock remaining x 5 |
| 13 | foot FL x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 14 | foot FL y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 15 | foot FR x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 16 | foot FR y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 17 | foot HL x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 18 | foot HL y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 19 | foot HR x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 20 | foot HR y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |

Feet: nominal (x, y) FL, FR, HL, HR = [[0.3141, 0.2], [0.3141, -0.2], [-0.3141, 0.2], [-0.3141, -0.2]] m, yaw_scale 0.427606 rad (24.5 deg), dx_scale 0.145 m, dy_scale 0.1 m. Params (operator inputs): height = 0, roll = 0, pitch = 0, yaw = 0, dx = 0, dy = 0, mode = -1.

### six_axis_stand: `m20_finetune_stand_15kg.onnx` (gaits 0xf001)

- network inputs `state` [1, 36, 53] (oldest first, current last), `command` [1, 21], `uncertain` [1, 1] = 0; outputs `output` [1, 16]
- default pose FL (hipx, hipy, knee) = (0, -0.7, 1.4), HL = (0, 0.7, -1.4); FR / HR mirror hipx
- kp (60, 60, 60, 0), kd (1.5, 1.5, 1.5, 1) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (0, 0, 0), no slew
- action: 16 interleaved per leg (hipx, hipy, knee, wheel) x 4; no clip; leg target = output (the ONNX applies 0.25 and the joint clip itself); wheel velocity target = 20 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | accelerometer specific force x, y, z (+1 g on z at rest) | m/s^2 | 0.101937 |
| 3-5 | body angular velocity x, y, z | rad/s | 0.25 |
| 6-8 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 9-24 | previous clipped network output (network output order) | - | 1 |
| 25-36 | joint position (absolute): fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad | 1 |
| 37-52 | joint velocity: fl_hipx, fl_hipy, fl_knee, fl_wheel, fr_hipx, fr_hipy, fr_knee, fr_wheel, hl_hipx, hl_hipy, hl_knee, hl_wheel, hr_hipx, hr_hipy, hr_knee, hr_wheel | rad/s | 0.1, 0.1, 0.1, 0.0333333 |

`command` vector:

| index | value |
|---|---|
| 0 | constant 0 |
| 1 | constant 0 |
| 2 | constant 0 |
| 3 | param `height` x -0.25 + 0.51 |
| 4 | param `roll` x 0.253073 (max 14.5 deg) |
| 5 | param `pitch` x 0.427606 (max 24.5 deg) |
| 6 | param `mode` x 1 |
| 7 | gait clock half_period x 5 |
| 8 | gait clock sign x 1 |
| 9 | gait clock sign x -1 |
| 10 | gait clock sign x -1 |
| 11 | gait clock sign x 1 |
| 12 | gait clock remaining x 5 |
| 13 | foot FL x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 14 | foot FL y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 15 | foot FR x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 16 | foot FR y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 17 | foot HL x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 18 | foot HL y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 19 | foot HR x: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |
| 20 | foot HR y: Rz(yaw_scale x yaw) nominal + (dx_scale dx, dy_scale dy) |

Feet: nominal (x, y) FL, FR, HL, HR = [[0.3141, 0.2], [0.3141, -0.2], [-0.3141, 0.2], [-0.3141, -0.2]] m, yaw_scale 0.427606 rad (24.5 deg), dx_scale 0.145 m, dy_scale 0.1 m. Params (operator inputs): height = 0, roll = 0, pitch = 0, yaw = 0, dx = 0, dy = 0, mode = -1.

### step_move: `m20_finetune_forward.onnx` (gaits 0xf002)

- network inputs `state` [1, 36, 53] (oldest first, current last), `command` [1, 13], `uncertain` [1, 1] = 0; outputs `output` [1, 16]
- default pose FL (hipx, hipy, knee) = (0, -1, 1.6), HL = (0, 1, -1.6); FR / HR mirror hipx
- kp (60, 60, 60, 0), kd (1.5, 1.5, 1.5, 1) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (3.9, 0.9, 1), no slew, axis deadband [0.0, 0.0, 0.1], wz max from the speed (see wz_speed_limit)
- action: 16 interleaved per leg (hipx, hipy, knee, wheel) x 4; no clip; leg target = output (the ONNX applies 0.25 and the joint clip itself); wheel velocity target = 20 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | accelerometer specific force x, y, z (+1 g on z at rest) | m/s^2 | 0.101937 |
| 3-5 | body angular velocity x, y, z | rad/s | 0.25 |
| 6-8 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 9-24 | previous clipped network output (network output order) | - | 1 |
| 25-36 | joint position (absolute): fl_hipx, fl_hipy, fl_knee, fr_hipx, fr_hipy, fr_knee, hl_hipx, hl_hipy, hl_knee, hr_hipx, hr_hipy, hr_knee | rad | 1 |
| 37-52 | joint velocity: fl_hipx, fl_hipy, fl_knee, fl_wheel, fr_hipx, fr_hipy, fr_knee, fr_wheel, hl_hipx, hl_hipy, hl_knee, hl_wheel, hr_hipx, hr_hipy, hr_knee, hr_wheel | rad/s | 0.1, 0.1, 0.1, 0.0333333 |

`command` vector:

| index | value |
|---|---|
| 0 | command vx |
| 1 | command vy |
| 2 | command wz |
| 3 | param `height` x -0.26 + 0.51 |
| 4 | param `roll` x 0.2618 (max 15.0 deg) |
| 5 | param `pitch` x 0.4363 (max 25.0 deg) |
| 6 | constant 0 |
| 7 | gait clock half_period x 5 |
| 8 | gait clock sign x 1 |
| 9 | gait clock sign x -1 |
| 10 | gait clock sign x -1 |
| 11 | gait clock sign x 1 |
| 12 | gait clock remaining x 5 |

wz max = clamp(min(10.8 / (vx + 2.4) - 0.15, 2.85 / (1 + 0.5 |vy|)), 1, 3); wz = axis x wz max.

### unreferenced_m20_38: `policy_m20_38.onnx` (gaits none)

- network input `state` [1, 318] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (-0.1, -0.8, 1.6), HL = (-0.1, 0.8, -1.6); FR / HR mirror hipx
- kp (80, 80, 80, 0), kd (6, 6, 6, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (2, 1, 2), no slew
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.05 |
| 37-52 | previous clipped network output (network output order) | - | 1 |

### unreferenced_rough_6: `policy_m20_rough_6.onnx` (gaits none)

- network input `state` [1, 318] = [current frame, 5 past frames oldest -> newest] (the history is updated first, so the current frame appears twice); outputs `output` [1, 16], `vel` [1, 3] (base velocity estimate)
- default pose FL (hipx, hipy, knee) = (-0.1, -0.8, 1.6), HL = (-0.1, 0.8, -1.6); FR / HR mirror hipx
- kp (80, 80, 80, 0), kd (3, 3, 3, 0.6) per (hipx, hipy, knee, wheel); wheels are velocity-controlled (kp 0)
- command maxima (vx, vy, wz) = (2, 1, 2), no slew
- action: 12 legs by type (hipx x4, hipy x4, knee x4; FL, FR, HL, HR) then 4 wheels; clip +-100; leg target = default + 0.25 a; wheel velocity target = 10 a (rad/s); then clamp to the joint range (hipx mid +- 0.75 half range), wheels +-79.3

Frame (53 values):

| index | content | unit | scale |
|---|---|---|---|
| 0-2 | body angular velocity x, y, z | rad/s | 0.25 |
| 3-5 | projected gravity R^T (0, 0, -1) x, y, z | unit | 1 |
| 6-8 | velocity command vx, vy, wz (after max / slew) | m/s, m/s, rad/s | 1, 1, 0.25 |
| 9-20 | joint position - default: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad | 1 |
| 21-32 | joint velocity: fl_hipx, fr_hipx, hl_hipx, hr_hipx, fl_hipy, fr_hipy, hl_hipy, hr_hipy, fl_knee, fr_knee, hl_knee, hr_knee | rad/s | 0.05 |
| 33-36 | joint velocity: fl_wheel, fr_wheel, hl_wheel, hr_wheel | rad/s | 0.05 |
| 37-52 | previous clipped network output (network output order) | - | 1 |

<!-- END GENERATED -->

## Sim results (`run_all`, 1 ms physics, PD at 1 kHz, torques clipped to the MJCF limits)

Every walk sequence: lying -> stand-up -> rl_control entry (stand-in) -> the runner's gait -> stand 4 s -> vx 0.5,
vy 0.3, wz 0.5 (6 s each) -> stop -> lie down. Values are body-frame means over the second half of each segment;
"pass" = no fall, every segment reached, base above 0.3 m while standing.

| sequence | result | stand z (m) | vx 0.5 | vy 0.3 | wz 0.5 | other checks |
|---|---|---|---|---|---|---|
| walk: basic_flat | pass | 0.59 | 0.52 | 0.00 | 0.20 | vy / small wz dead zone (policy's own) |
| walk: basic_plat | pass | 0.48 | 0.65 | 0.26 | 0.61 | |
| walk: basic_stair | pass | 0.51 | 0.53 | 0.07 | 0.47 | |
| walk: basic_backknee | pass | 0.51 | 0.59 | 0.07 | 0.43 | |
| walk: navi_charge | pass | 0.46 | 0.53 | 0.30 | 0.59 | |
| walk: navi_flat | pass | 0.47 | 0.51 | 0.17 | 0.45 | |
| walk: navi_orienteer | pass | 0.47 | 0.58 | 0.30 | 0.48 | |
| walk: stand_policy | pass | 0.47 | 0.54 | 0.29 | 0.64 | |
| walk: extre (mode 0) | pass | 0.46 | 0.48 | 0.26 | 0.54 | |
| walk: step_move | pass | 0.47 | 0.46 | 0.24 | 1.33 | wz axis 0.5 x speed-dependent max 2.85 = 1.43 commanded |
| walk: unreferenced m20_38 / rough_6 | pass | 0.60 / 0.57 | 0.44 / 0.58 | 0.06 / 0.00 | 0.13 / 0.42 | |
| walk: height_gap | known fail | 0.40 | 1.70 | - | - | falls at 20 s; height-map convention [U] |
| stand: finetune_stand (sixaxis_stand.onnx) | pass | 0.51 | - | - | - | height 0.8 lowers 0.18 m, pitch 0.8 -> 15.5 deg, roll 0.8 -> 10.6 deg |
| stand: six_axis_stand (finetune_stand_15kg.onnx) | pass | 0.49 | - | - | - | lowers 0.18 m, pitch 16.0 deg, roll 10.2 deg |
| stairs (3 x 8 cm): basic_stair, navi_orienteer, navi_flat | pass | | | | | climbed 0.24 m |
| stairs: height_gap | known fail | | | | | falls at 14 s |
| dock / undock: navi_charge | pass | | | | | dock x error 0.005 m, y 0.002 m, back to start 0.002 m |
| gait_switch (12 gait changes while walking 0.2 m/s) | pass | | | | | |
| switch: basic_flat -> navi_flat | known fail | | | | | BasicFlat's 0.59 m sim stance is outside NaviFlat's distribution: after 0.02-4 s of BasicFlat, NaviFlat drives hr_hipy to -2.64..-2.71 rad (limit -2.583, the robot faulted at -2.688) and falls; NaviCharge / BasicStair -> NaviFlat work |
| standup_rpc: 0x1001 entry, gait 0x3003 at once (stand-in -> basic_stair, no BasicFlat) | pass | 0.51 | 0.47 | | | M20Controller.standup(); no fault, 2.4 m in 5 s at 0.5 m/s |
| standup_rpc: 0x1001 entry, gait 0x3002 at once (stand-in -> navi_flat, no BasicFlat) | pass | 0.47 | 0.49 | | | the same with 0.5 s of BasicFlat before 0x3002 (the vendor captures' dwell) faults at hr_hipy |
| switch: basic_plat -> basic_backknee (walking) | known fail | | | | | standing switch works |

The Rust controller passes the same lie -> stand-up -> walk 0.5 m/s -> lie-down run in real time against the sim
(`cargo test --test sim_sequence`), and its 15 runners match the Python ones step for step (`--test golden`).

## Checked against the robot (replay.py, 2026-11-20 14:16 moving session)

`python -m dimos.simulation.m20_policy_sim.replay SESSION --out DIR [--rust .../examples/replay]` feeds the Python
pipeline and the Rust controller what rl_deploy saw (1 kHz joint frames, body IMU from DDS, cmd_vel, journal) and
compares their commands with rl_deploy's. The deterministic states, gains and enable frames match on every tick; the
policies match to float precision once their state lines up. Rules this settled [C, robot]:
- On entering stand_up, lie_down and rl_control rl_deploy sends 3 enable frames (raw zeros, control word 2); the
  state's clock starts at the first of them. Stand-up / lie-down also spline the wheels' position targets to 0
  (velocity 0); the IK clamps to the front leg's ControlParameters range (knee 2.793) before mirroring.
- rl_deploy times its splines on a wall clock that slips by a tick now and then (up to 0.001 rad here). That clock is
  f32(seconds since rl_deploy started), each reply's arrival (EcanHardwareInterface::Handler), now minus entry in f32
  (StandUp / LieDownState): after 1.5 days up its stand-up and lie-down targets step every 1/64 s (robot, 2026-11-24),
  and its 3 s / 4 s limits fall on that grid. The Rust controller keeps it ([timing] f32_state_clock, the stamp being
  seconds since the bus opened) with rl_deploy's f32 arithmetic and FMAs (aarch64 disassembly): the cubic's
  coefficients, the IK, the 1 ms difference of phase 2, the joint / motor angle conversions
  (tests/ecan_dock_rec_20261124.rs).
- A gait switch (update_policy_ptr, only while |vx| + |vy| + |wz| < 0.01) runs the old runner's OnExit and the new
  one's OnEnter; rl_control's entry runs OnEnter(false) on the runner OnExit left current (the fallback, BasicFlat).
  OnExit writes the last action to two globals (last_leg_action_mid: leg positions, default + 0.25 a with an FMA, the
  absolute runners their output as is; last_wheel_action: 10 or 20 x a); OnEnter turns them back into the new
  runner's last action ((q - default) / 0.25, wheels / scale; laid out per leg for per_leg runners and on entry, by
  joint type otherwise) and zeroes its history, command slew and step counter. The phase and a gait clock's frequency
  carry on (the clock's counter restarts). Start-up gains (NaviCharge x1.3, 29 steps, legs only) restart with it.
- The history starts as zeros (constructor and OnEnter), not as copies of the first frame. The gait phase advances
  before it is observed.
- The 36-frame runners hold the default pose (as their output and last action) for their first 37 steps after
  OnEnter while still running the network; SixAxisStand only while UserCommand byte 0x1c is set (rl_control's entry
  stand-in, first phase), so the 0x3001 stand-in infers from its first step. Their command (cal_command, f32): height
  0.51 - 0.25 h (fused), roll / pitch x f32(f64(f32(deg / 180)) x pi) with 14.5 / 24.5 deg, the gait clock in f32
  (t = counter x 0.02, sign of sinf(t x 2 pi f), fmodf(t, 1 / 2f) above 1e-6, the frequency re-picked when a full
  period wraps), the feet Rz(yaw x 24.5 deg) nominal + (0.145 dx, 0.1 dy) with its FMAs, or nominal x and y = +-0.1
  while UserCommand byte 0x6e is set (update_policy_ptr sets it on a switch to 0xf001 during rl_control's entry
  stand-in). SixAxisStand's mode is -(UserCommand 0x6c), but 0 while its flag is armed and the body upright
  (gravity z < -0.9), the flag counting 100 upright steps and re-arming on gait 0xf003 (so on 0xf001 the mode is
  -(0x6c)); FinetuneStand's is -1. Their
  accelerometer input is accel / 9.81 in f32 (a division). update_policy_ptr sets 0x6c to 0 on the entry stand-in's
  first phase and the 0x3001 stand-in, and Run sets it to 1 when the entry stand-in's second phase starts.
- The accelerometer the runners see is the mean of the last 20 rl_control ticks' samples (UpdateRobotObservation's
  ring, f32, kept across entries).
- Entering rl_control on 0x3001 runs no stand-in; switching to 0x3001 does (1.5 s). The 0x1001 stand-in ran 6.14 s:
  mode 0 for 3 s, then mode -1 (what makes it 6.14 s rather than 3.5 s is unknown).
- /NAV_CMD is metric (m/s, rad/s) and divided per gait (BasicFlat (2, 1, 2), not the decoded (2, 0.8, 1.5)); the
  charge manager's steer is normalized and, on 0x3004 / 0xf001, also drives the pose inputs (7 yaw, -24 x, -18 y).
- The body IMU is a 56-byte DDS sample on UDP 7441 (200 Hz): stamp, roll / pitch / yaw in degrees, gyro rad/s,
  accelerometer m/s^2; rotation from ZYX roll-pitch-yaw.

## Docking (charge_manager's sequence in the Rust controller)

The sequence is charge_manager v4.0.30's, decompiled (`rust/src/dock/vendor.rs`; Ghidra of NOS
/opt/robot/share/charge_manager/bin), on its own charge_manager.toml read at startup [C]:

- Steer per dock pose (10 Hz), f32 with the binary's fused multiply-adds: incremental PID (x, y: 1.2 / 0 / 0.2; yaw
  1.4 / 0 / 0.1, yaw negated) through limitSpeed (clamp to +-max, anything nearer 0 than min out to +-min). Staging
  point (x - pre_point_x): max (0.8, 1.0, 0.8), min (0.35, 0.45, min_speed_yaw), reached inside delta + (0.04, 0.08,
  0.03); fine point: max (0.7, 0.8, 0.4), min (min_speed_*), reached inside delta + (0.003, 0.003, 0.015). delta_yaw
  is /100 (rad). No stop-and-settle: it steers every pose and goes on the pose it is inside.
- The first 6 poses with x > 0 are not steered; the least-|y| one is the start point (mInit), the undock's goal with
  back_way = auto.
- FinetuneStand (0x3004): steer = the raw (x, y, yaw) error and z = -0.05 k (k 2 -> 20); rl_deploy feeds it as
  pose inputs dx -24 x, dy -18 y, yaw 7 yaw, height max(-z, 0). Done inside delta after 3 s, or at 4 s; then (0, 0,
  0, -1.1) held 1 s, lie down, 6 s of charge-enable, then up to first_detection_current_timeout for every pack's charge
  MOS. No charge: stand on NaviCharge, 2 s, back to the staging point (adjust_posture_times), then goBack (undock).
- A pose missing 200 ms stops the robot (rl_deploy zeroes the steer at 150 ms); 400 in a row restart the detector,
  and a failed FindTarget lies the robot down. The approach and the undock each fail after 60 s (BasicFlat).
- The obstacle check (ObstacleDetection, grid): base_link points of both Airys, z in [charge_pile_h - body height,
  0.3], x in [0, 0.9] ahead (approach) or [-0.9, 0] behind (undock, retry), |y| <= 0.3, outside the body box (0.45 x
  0.25); 0.3 m cells, 20 points occupy one; the nearest occupied centre from (+-0.45, 0). Under ObstacleAvoidDis
  (0.3: only the cell 0.3-0.6 m out, at 0.15) the robot stops; 10 s stopped fails on BasicFlat. The dock's posts sit in
  the next cell (0.335, logged through every vendor dock). Not in FinetuneStand.
- Undock (only from charging or after a failure): the detector restarts and must see the dock (lying), stand, back off
  to the exit staging point (pre_exitPile_point_x - 0.1) and on to the start point (ext_delta), 2 s, then the gait
  docking started from.
- reflective_column_node's output stage: positions outside x [-4, -0.05], |y| 1.5 dropped; after a restart 15 sweeps
  held back, the 16th publishes the last within 3 sigma (FindTarget's ~1.5 s); a position 0.8 m from the one before
  dropped. Neither binary reads the IMU or tf (no such DDS topic in either); the cloud is base_link as mounted.

`cargo test --release --test vendor_replay` (and `examples/charge_replay.rs` on any charge_manager log) replays
charge_manager's own logs through the core: all of NOS's (2026-11-04 to 11-23: 100 docks, 94 undocks, 24580 poses)
give every steer to the log's 6 decimals (max 2.7e-6) and all 1559 events (staging, fine point, FinetuneStand done,
974 obstacle stops, retries, undock staging / end) at the same pose; one 4 s limit decision falls within 1 ms of its
threshold (the log's wall clock cannot place it) and one line was cut off by a restart.

`cargo test --release --test sim_sequence dock` docks the Rust controller in the sim: world.py's `charge_dock` (two
retro-reflective posts, intensity 250 against 30 for everything else) seen by ray-cast Airys (96 beams over 0-90 deg,
0.4 deg azimuth, 1 cm range noise) through the post detector at 10 Hz, and battery packs whose charge MOS comes on
when the robot lies with base_link within 3 cm (along the dock's axis) and 2 cm (across) of the docked point (an
assumption about the pads). From ~2 m out with a lateral offset and a turned dock it reaches the fine point, lies down,
confirms charging and undocks to where it started; with no charge it retries twice from the staging point and backs
off. With FinetuneStand as the vendor drives it (raw-error pose inputs, the body lowered to height 1.0 then 1.1, 1 s
held, then lie down) the sim lies 2.0-3.1 cm short of the docked point and 3.4-3.8 deg turned (fine point within 1 cm),
so the 3 cm contact window charges one scene on the first try and the other on its last retry. The vendor charged on
the first try in 94 of its 100 logged docks (one more after a retry): the suspect is the sim's lie-down (base_link settling back 1.4-2 cm, foot slip
on the lowered stance) and the assumed pad window, not the sequence.

Checked against a vendor dock (NOS dock_rec 2026-11-21: tcpdump of both Airys' multicast, charge_manager's log),
`examples/dock_pcap.rs` + `dock_compare.py`. The posts return intensity 200-255 (the floor 100-150), stand ~0.6 m,
0.374 m apart; docked, base_link is 0.735 m out from them (sd 0.2 mm lying). Saturated, they bloom to 0.25 m across
and trail ghosts up to 0.36 m behind themselves at 2 m (hence the depth band and the median-then-circle centre).
charge_manager's logged poses are the robot relative to the docked point (x 0 docked; staging "reached" at x 0.99).
Ours minus charge_manager's (median / MAD; x, y mm, yaw mrad), robot this far out from the docked point:

| run | 0-0.12 m | 0.25 m | 0.5 m | 0.75 m | 1.0 m |
|---|---|---|---|---|---|
| dock 13:27:49 (598/599 sweeps found) | +1.0/0.5, -0.3/1.4, -0.4/2.3 | -0.1/0.5, +5.3/4.1, +4.3/4.0 | -2.4/0.9, +23/8, +19/5 | -2.6/1.8, +19/12, +11/8 | -4.0/1.1, +17/5, +11/4 |
| undock 13:22:06 (179/180 after standing) | +3.0/0.7, -0.8/1.6, +3.1/0.9 | +2.7/0.8, -5.8/2.7, -5.1/2.9 | +2.8/0.9, -4.4/5.7, -4.3/4.6 | +1.3/1.2, -3.9/8.4, -4.4/4.7 | -2.4/1.4, +2.0/9.5, -0.3/5.4 |

At the fine point the two agree to about a millimetre. The approach's +2 cm across, +1-2 deg, while the robot walked
in from 0.2 m to the side, is not explained by a frame convention; the likeliest cause is the body's roll while
stepping sideways (the detector takes base_link as level; the sim shows the same few-mm effect standing).
charge_manager's own detector glitched on 5 of 67 undock poses (0.45-0.5 m across for one sweep).

## Checked against a fresh vendor dock (2026-11-24 recording, `tests/dock_rec_20261124.rs`)

Recorded with the vendor stack running (NOS: both Airys' raw multicast, every UDP 7400-7700 DDS packet, charge_manager
and reflective_column_node logs; AOS: rl_deploy's ECAN both ways, DDS, logs): an undock from charging (05:50:23),
the operator driving, a dock (05:51:07) and the charge confirmed (05:51:31). NOS-local DDS (/LIDAR/POINTS, the
detector's pose) is shared memory and in no capture, and so is the body IMU on AOS that day.

- Detector: every sweep reflective_column_node logged (390) decoded from the raw capture and run through ours: every
  logged value (point counts, cluster ratios, post counts, centres, position) as it prints it (`reflective_column_replay`
  with the capture; `dock_rec_20261124` checks its committed per-sweep poses against the capture bit for bit).
- Sequence: [`Docking`] fed what charge_manager was fed (rl_deploy's /MOTION_INFO and /BATTERY_DATA as they reached
  NOS, the ChargeEnable calls and obstacle readings from its log, our detector's poses when reflective_column_node
  published each) writes what charge_manager wrote to rl_deploy, decoded from the RTPS in the capture (dr_msgs
  Steer / MotionState / Gait / StdMsgInt32): all 197 the same and in the same order (4 MotionState: 1, 17, 17 restored, 4; 4 Gait: 0x3001,
  0x3003 restored, 0x3001, 0x3004; 179 Steer, every float bit for bit; 10 charge-enables: 1 every 5 s charging, 0 at
  the undock's start and end, 1 x 3 at the power-on), every one within 0.79 ms of the wire (charge_manager's 1 ms
  tick). Two vendor waits are modelled: a missed pose wait's next 200 ms wait starts 216 us after the last ended (its
  warning and stopMove; 18 in a row logged 200.17-200.29 ms apart), and charge_manager waits for its detector
  process's replies to the close / open service calls (53-83 ms each, all in the request's delivery: the detector
  logs each call's handling in 0.05-4 ms, the reply reaches charge_manager 0.35 ms later). The replay feeds those
  replies from reflective_column_node's log; ours has no such process, so on the robot ([dock.sequence]
  detector_service = false) the undock's last charge-enable 0, the charging loop and FindTarget come 55-130 ms
  earlier than the vendor's. The charge is confirmed, and charging begins, on charge_manager's tick. The undock's goal comes from the dock
  before the recording and is in the log to 6 decimals only: its x is taken as the f32 one ULP off the printed value
  that the undock's steers imply.
- New from this recording (charge_manager's binary, now in [`Docking`]): StandUp2WideWalk stands (1), waits 2 s, then
  rl_control (17) with NaviCharge; ObstacleDetection::Open sleeps 10 ms; the back-off kick goes out 5 times; WideWalk
  writes the gait only off NaviCharge; the charging loop writes a charge-enable every 5 s (3 s wait, write, 2 s
  sleep) and an undock asked for during the sleep starts at its end (1.35 s here); the undock writes charge-enable 0
  at its start and end and restores the motion state, then 2 s later the gait. rl_deploy turns each charge-enable 1
  into 3 board frames on its next ticks and has no charge rule of its own, so the controller sends the dock
  sequence's and m20-battery's own rule is off in the policy loop.

## The body IMU in recordings

rl_deploy reads the body IMU from yesense_node on AOS (DDS participant "yesense"): rt/IMU_DATA, drdds
ImuData_ (header, roll / pitch / yaw deg, gyro rad/s, accelerometer m/s^2), 200 Hz. Fast-DDS hands it over shared
memory when every reader is on AOS and speaks it (2026-11-24: in no capture); on 2026-11-20 it went over UDP (7441)
and the replay read it from net.pcap. `m20_imu_record` (cmd_vel/rust, a best-effort reader: no writes, no
acknowledgements) beside rl_deploy is a reader without shared memory, so yesense also sends it every sample over UDP:
record.sh starts it when `$dir/m20_imu_record` exists, writing session/imu.txt, and net.pcap has the same samples
(what replay.py reads).

### Policy numerics, step for step (`tests/policy_20261120.rs`) [C, robot]

rl_deploy computes each step in f32 (RLControl / PolicyRunnerBase, aarch64 disassembly):
- Command: PolicyRunnerBase::limit_cmd_vel moves the normalized axes toward the input by at most the runner's
  per-step limits (BasicStair 0.04, 0.04, 0.08; BasicFlat 0.025, 0.025, 0.04), then multiplies by command_max;
  NaviCharge calls it without the slew.
- Phase: phase = f32(f64(phase) + 0.02 x f64(frequency)) - f32(trunc of that sum); observed as f32(sin / cos of
  f64(phase) x 2 pi).
- Joint terms: f32(q) - f32(default), dq x scale, gyro x 0.25, each in f32.
- Gravity: from yesense's own roll / pitch / yaw (degrees, f32; rad = f32(f64(f32(deg / 180)) x pi)), Eigen's
  AngleAxis(yaw, Z) AngleAxis(pitch, Y) AngleAxis(roll, X) (sincosf of the half angles, its NEON quaternion product
  and toRotationMatrix with FMAs), gravity = -(R20, R21, R22) (types.rs `vendor_projected_gravity`).
- Inference: ONNX Runtime 1.20.0 (lib/onnxruntime in the deb), 1 intra-op thread, all graph optimizations. Other
  builds differ in the last bits: macOS's 1.20 by up to 4e-6, tract by up to 5e-5. The controller infers through it
  when `[inference] onnxruntime` or $M20_ONNXRUNTIME names the library (src/ort.rs), otherwise with tract.

With the joint state as the ECAN reply before each step carried it, the last IMU sample before that reply, and
rl_deploy's previous action as the last action, the 11-20 14:16 session's basic_stair, navi_charge and basic_flat
steps match rl_deploy's leg targets within 1e-6 on 95%, 97% and 87% of steps (median 1.2e-7, Linux ONNX Runtime).
Still differing [U]: the first step after each rl_control entry and gait switch (up to 0.6 rad: some state the runner
keeps or rebuilds there is not modelled), basic_flat's command ramp after the six_axis stand-in, the sequence runners
(six_axis_stand, finetune_stand, 10-13% of steps), and the accelerometer's 20-sample moving average.

### Whole sessions, closed loop (`examples/policy_session.rs`) [C, robot]

Every policy step of the 11-20 14:16 and 16:36 sessions, each runner fed its own previous output, rl_control entries
and gait switches as above, the operator's commands with rl_deploy's steer timeout (MonitorSteer zeroes the velocity
when neither steer channel updated within 150 ms, polling every 10 ms: 155 ms fits both sessions), late steps reading
their on-cadence tick's reply (rl_deploy's policy thread keeps a 20-tick cadence), and FinetuneStand's height from the
steer's z (HandlerSteer on 0x3004: height = -z, 0 for z > 0). Leg targets within 1e-6 or 4 ULPs (macOS ONNX Runtime
1.20): 14:16 78% of 4206 steps (basic_stair 91%, navi_charge 91%, basic_flat 78%, finetune_stand 36%, six_axis_stand
10%); 16:36 71% of 4756 (basic_stair 74%, navi_charge 93%). With rl_deploy's previous action as the last action, 84%
and 81%. Where they part [U]:
- The IMU sample: rl_deploy reads the latest sample DDS had delivered, and delivery jitters by ~2 ms, which the
  capture does not show; at most of 16:36's basic_stair onsets the sample before the latest one (arrived 1.4-1.9 ms
  before the reply) reproduces rl_deploy's step (e.g. rows 73521, 78221, 84521: 1e-2 .. 4e-3 -> 6e-4 .. 7e-5), and no
  one offset between the captures fits every step.
- SixAxisStand in rl_control's entry stand-in (the robot standing up) is off by ~1e-3 from its first inferred step;
  the runner switched to after it inherits that through the handover (e.g. basic_flat at 14:16 87942).
- Wheel velocity commands match far less often than leg targets. rl_deploy's wheel command is exactly ours (fmul of
  the clipped output by 10 or 20 in f32, then limit_goal_pos_vel's +-79.3 clamp; no filter: SixAxisStand's f64 output
  low-pass is computed and then overwritten in every branch), and its leg target is fmadd(a, 0.25, default) (0.25
  makes it the same as ours). So a wheel miss is the network output itself off by tens of ULPs, which the leg
  target's rounding hides: an input a few ULPs off, or (closed loop) one that was: last action and history feed every
  error back, so a session never becomes exact after a start it cannot know (rl_deploy's globals and the first runner's
  history from before the recording). On rl_deploy's library (Linux) 22% / 8% of steps match on the wheels, against
  10% / 6% on macOS's. The inference recording (ort_tap) gives each step's real inputs to settle this.
- FinetuneStand on rl_deploy's library is 1-2e-6 (10-16 ULPs) off on most 14:16 steps. Not its feet: their
  rotation's op order is the disassembly's (fmul, fmla, fmla with the zero z term, fmadd with 0.145 dx / 0.1 dy), and
  dock::libm::sincosf is glibc 2.31's to the bit below |x| = 120 (tests/libm_glibc.rs).

## Vendor functions, one by one

Every function of the vendor binaries, with each branch, topic, service and timer: ported (our file:line) or n/a
with the reason, and the differences still to fix ordered by risk:
[charge_manager](SPEC_charge_manager.md), [reflective_column_node](SPEC_reflective_column.md).

## Building the controller for the robot

    cd dimos/robot/deeprobotics/m20/controller/rust
    cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.31   # ~19 MB, glibc 2.31 (Ubuntu 20.04 boards)
    cargo test                                                         # golden + real-time sim tests (need policies)

Policy inference is `tract` (pure Rust) unless `[inference] onnxruntime` names rl_deploy's ONNX Runtime (its
numerics, above). tract needs no shared library on the board, runs the same graph on macOS and aarch64, and supports
every op these 15 models use; the transformer runners skip tract's graph optimizer (it mis-shapes their command Tile)
and run the typed graph, at about 0.2-2 ms per step on an M-series Mac.

## The controller as a board service (`rust/service`, `controller/service.py`)

rl_deploy is a system service and basic_server / the blueprint are its clients, so a run that stops leaves the robot
as it is. Ours does the same as `m20-controller.service` on AOS (the module binary, its stdin line from launch.json,
streams under `dimos/m20_controller/`); a run reaches it through `M20ControllerClient` (`--choice M20Walker=service`,
`M20Joints=service`), whose stop only zeroes the velocity. Stopping the unit lies a standing robot down first (its
SIGTERM path, at most `safety.shutdown_lie_down_max_s`), as does stopping the in-run module (`M20Walker=controller`).
Hazard, as with rl_deploy: a run restart no longer resets the controller, so a fault's blocked standing request, a
usage mode and a gait persist into the next run until a different motion command or mode arrives.

## basic_server in front of the controller (`src/basic_server.rs`)

basic_server (md5 538e34f2) sits between the app / navigation and rl_deploy; ours is the same layer in the controller:
- usage mode (setControlUsageMode 0x261ba8, on a change only): 0 regular, 1 navigation, 2 assist; 0 and 2 also
  SwitchGait(0x1001). /NAV_CMD, /REAL_STEER and charge_manager's steer reach rl_deploy only in modes 1 and 2.
- charge_manager's status (ChannelChargerStatusCallback 0x220c60): docking or undocking (1, 3) -> mode 1, idle (0) ->
  mode 0 (so an undock ends on 0x1001 in regular mode), charging (2) changes nothing.
- a stand (SetMotionState(1) 0x2622d0): stand, wait up to 5 s for it, then RL control, SendGait(0x1001), mode 0.
- SwitchGait (0x261640): a code outside 0x1001-5, 0x2001-4, 0x3001-5, 0x5001-3, 0xf001-3 is dropped. Not ours: its
  3 x 3 s wait for /MOTION_INFO (the layer applies the gait at once), Extre's authorization code, sensor power.
rl_deploy's own gait handling (RLControlState::Run / update_policy_ptr 0x179960): Run copies the UserCommand in and
back every tick; the 0x1001 stand-in writes 0xf001 over its gait slot, then 0x1001 at its end, so a gait sent during
the stand-in is lost (M20Connection resends it); a gait switch waits until |vx| + |vy| + |wz| < 0.01.

## Still unknown [U]

- UserCommand +0x14 / +0x18 semantics beyond the gait-change marker, and which DDS input sets the stand mode bytes.
- Extre gait -> `command[4]` mode mapping (sim: 0 walks, -2 tall stance, +-1 fall over).
- CarMove, JointTest, SetZero, SdkControl internals; StartDamping gains.
- StepMove's vx rate limit (`limit_target_v`); the exact command fields `update_gait_freq` reads.
- HeightGap map offset, grid spacing and orientation.
- The charge manager's compiled-in numbers: its docked target distance, its staging tolerance and steer gains, and
  reflective_column_node's intensity threshold, post spacing and radius. The Rust controller's docking
  (`rust/src/dock`, `[dock]` in m20_controller.toml) copies every number charge_manager.toml has and sets these for
  the sim's reflector-post dock (world.py `charge_dock`); they need the real dock measured and vendor dock recordings.
- On the motor bus: per-joint position offset and direction (the TX/RX positions differ while damping), kp / kd
  scaling (x10 assumed from ECAN2DDS), leg order of `can_ch`, and what the board does when commands stop.
