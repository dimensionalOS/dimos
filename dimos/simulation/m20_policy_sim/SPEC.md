# M20 rl_deploy v1.5.8: controller specification

What the vendor walking controller (`/opt/robot/share/rl_deploy`, binary md5 b16070a4bf2abb52a0ce33e863a4df2e) does,
decoded from its disassembly and confirmed in MuJoCo. Both re-implementations follow this file and read the same
config, `dimos/robot/deeprobotics/m20/controller/m20_controller.toml`:

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

Faults (SafeController, 1 kHz, 1 s start-up grace): joint position beyond its range by > 0.1 rad; tilt |roll| or
|pitch| > 20 deg in Idle, > 45 deg in StandUp, in RLControl roll > 45 deg or pitch > 65 deg (BasicPlat pitch 90 deg;
Extre and 0xf003 skip it); IMU stale > 20 ms; battery and motor temperature flags. The re-implementations recompute
faults every tick like SafeController and, after a fault, ignore the operator's standing request until a different one
arrives (rl_deploy resets the target to damping).

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
   scale x a; clamp legs to the joint range with hipx narrowed to mid +- 0.75 half range, wheels to +-79.3 rad/s.
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
| switch: basic_flat -> navi_flat | known fail | | | | | BasicFlat's 0.59 m sim stance is outside NaviFlat's distribution; NaviCharge / BasicStair -> NaviFlat work |
| switch: basic_plat -> basic_backknee (walking) | known fail | | | | | standing switch works |

The Rust controller passes the same lie -> stand-up -> walk 0.5 m/s -> lie-down run in real time against the sim
(`cargo test --test sim_sequence`), and its 15 runners match the Python ones step for step (`--test golden`).

## Building the controller for the robot

    cd dimos/robot/deeprobotics/m20/controller/rust
    cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.31   # ~19 MB, glibc 2.31 (Ubuntu 20.04 boards)
    cargo test                                                         # golden + real-time sim tests (need policies)

Policy inference is `tract` (pure Rust): no ONNX Runtime shared library to ship to the board, the same graph on
macOS and aarch64, and every op these 15 models use is supported; the transformer runners skip tract's graph
optimizer (it mis-shapes their command Tile) and run the typed graph, at about 0.2-2 ms per step on an M-series Mac.

## Still unknown [U]

- UserCommand +0x14 / +0x18 semantics beyond the gait-change marker, and which DDS input sets the stand mode bytes.
- Extre gait -> `command[4]` mode mapping (sim: 0 walks, -2 tall stance, +-1 fall over).
- CarMove, JointTest, SetZero, SdkControl internals; StartDamping gains.
- StepMove's vx rate limit (`limit_target_v`); the exact command fields `update_gait_freq` reads.
- HeightGap map offset, grid spacing and orientation.
- The charge manager's dock / undock velocities (outside rl_deploy; NaviCharge is a plain velocity policy). The sim
  docks with a P controller capped at 0.3 m/s, backing onto a wall 1 m behind the start.
- On the motor bus: per-joint position offset and direction (the TX/RX positions differ while damping), kp / kd
  scaling (x10 assumed from ECAN2DDS), leg order of `can_ch`, and what the board does when commands stop.
