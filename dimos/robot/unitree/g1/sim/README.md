# g1sim

The G1 plant for MuJoCo, on the shared identification pipeline
(`dimos/simulation/sysid`, the method go2sim was built with): record the real
robot, replay the recorded joint targets open loop through the plant, fit the
knobs the data resolves, keep the rest declared.

Mode A only for now: the GR00T policy is not in the loop, so there is no
referee (go2sim's loop 2). `groot_mujoco.py` is the closed-loop viewer that a
future Mode B grows from.

## 1. Watch it

```bash
# what the recording contains: rates, epoch, stand/walk segments
python -m dimos.robot.unitree.g1.sim.sysid.ingest data/g1_groot_characterization_2026-08-27.db

# open-loop replay of a walk segment, the recorded pelvis pose as a ghost
python -m dimos.robot.unitree.g1.sim.sysid.replay data/g1_groot_characterization_2026-08-27.db --list
python -m dimos.robot.unitree.g1.sim.sysid.replay data/g1_groot_characterization_2026-08-27.db \
    --preset stock --segment 3 --view --speed 0.5

# the GR00T policy closed loop, with the ghost (the recording's twists drive it)
python -m dimos.robot.unitree.g1.sim.groot_mujoco data/g1_groot_characterization_2026-08-27.db
```

`--preset` takes a built-in name or a JSON a fit wrote (`presets/`). The
viewer and the headless run are the same function; `--no-reinit` shows free
open-loop divergence, the only way to SEE what open loop costs.

## 2. The plant

- **Base model.** The in-repo `assets/g1_29dof.xml` (NVIDIA
  GR00T-WholeBodyControl's 29-DOF G1) attached to the blueprint's
  `scene_empty.xml`. Every fitted knob is a delta on those bytes, so
  `model.py` pins the XML and the mesh tree by hash (`test_model`). The eight
  unnamed 5 mm foot spheres are named and raised to contact priority 1 at
  load: at stock a no-op (held by test), and what makes a foot override bind
  instead of averaging with the floor.
- **`stock`** is the bare model: the experimental control every claim is
  comparative against. The scene compiles to Newton 100/50 and a PYRAMIDAL
  cone (the Go2 scene is elliptic); the solver is recorded on every preset.
- **Knobs** (`ranges.py`): the Go2 set, applied to the 12 leg joints, the
  torso (`torso_link`: where the mass, the IMU, the lidar and any unweighed
  payload are), the 12 leg links and the 8 foot spheres. Every range carries
  its `why`.
- **Declared, not measured** (`sysid/fit.py` DEFAULT_PINS): nothing on the G1
  has been weighed, so trunk mass/inertia/CoM, leg mass, the floor's friction
  and the foot's torsional friction are pinned at stock with that said;
  `damping` is pinned because only a hanging recording resolves it.

## 3. The recording

`data/g1_groot_characterization_2026-08-27.db` (LFS): 517 s, `motor_states`
and `imu` at 495 Hz, GR00T targets at 50 Hz (99 Hz on the wire), Point-LIO
odometry at ~30 Hz, operator twists at 20 Hz over a 317 s window. Epoch is
the first velocity command; walking starts 22 s later.

Three rig facts live in `sysid/ingest.py` as constants. The IMU site is in
`torso_link`, so the recorded attitude is moved to the pelvis through the
measured waist angles before it seeds the free joint. The Mid-360 is
mounted upside down on the torso; Point-LIO is moved to the pelvis the same
way, and stands in for the tracker (position only; it is odometry, ~30 Hz,
and drifts). `CMD_TIME_OFFSET_S` is the command-clock question left open:
the payload's own timestamp runs ~50 ms behind the store clock.

Policy-mode segments are derived from GR00T's own balance/walk threshold
(|cmd| > 0.05), so `stand`/`walk` in `--list` are the modes the policy was
actually in.

## 4. What the data resolves

`identify --channel joint` over the first 120 s: **0 of 14 knobs** at one
residual RMS. The walking is gentle (joint speed p90 0.5 rad/s) and two of
eight segments are standing. Closest: `foot_solref_time` 1.9,
`leg_mass_scale` 2.7, `foot_solref_damp` 4.3, `armature` 4.5;
`frictionloss` 10.4, `damping` 23.5. No degenerate pairs. The fix is
capture, not fitting: harder manoeuvres, and a hanging recording for the
joint-level knobs (go2sim §5, step 2).

## 5. Results

**First fit, 2026-08-28** (`presets/measured.*`): stock as the incumbent,
searched `armature, frictionloss, actuator_tau, foot_solref_time,
foot_solref_damp`, everything else pinned as declared; 12 studies x 30
trials on t=20..150 s, 8 segments, multiple shooting U(0.05, 0.8) s,
weights joint .30 / dq .30 / tau .15. The shipped point is the median of the
14 pooled near-optimal trials, never the best draw:

| knob | stock | measured | p10 .. p90 | of range |
|---|---:|---:|---:|---:|
| armature (kg m^2) | 0.010 | 0.0176 | 0.0063 .. 0.0288 | 47% |
| frictionloss (N m) | 0.10 | 6.58 | 5.02 .. 7.74 | 8.5% |
| actuator_tau (s) | 0 | 0.00236 | 0.0018 .. 0.0054 | 30% |
| foot_solref_time (s) | 0.020 | 0.0067 | 0.0040 .. 0.0121 | 41% |
| foot_solref_damp | 1.0 | 1.25 | 1.07 .. 1.59 | 35% |

The fit hit the study cap without the leave-one-out drift stabilising
(0.20 at k=12): the region is wider than this data pins, which is the
result, not a failure; the spread ships with the point. The frictionloss
region sits near the range's upper bound (8.0): widen it before refitting.

| window | stock | measured | change |
|---|---:|---:|---:|
| fit set, t=20..150 s | 0.816 | 0.671 | -17.7% |
| held out, t=200..260 s (never fitted) | 0.797 | 0.630 | -21.0% |

Every channel improved on both windows, scored or not (held out: joint
0.046 -> 0.037 rad, dq 0.50 -> 0.37 rad/s, tau 4.22 -> 3.72 N m, accel
2.13 -> 1.96 m/s^2, pos 0.071 -> 0.063 m, rot 0.157 -> 0.131 rad).

For comparison only: the fork `aaryan/g1-groot-characterization` fitted
damping 5.6e-4 / armature 0.0138 / frictionloss 3.25 on this same
recording by a different method. Not used here.

**Loop 2, 2026-08-28** (`sysid/ground.py`, `presets/loop2.plant.json`): by
decision the fit moved to the closed loop. GR00T drives the plant from the
measured mid-walk state (its six-frame history teacher-forced from the
recording) on the recorded velocity commands; the loss is the tracking
area against Point-LIO (along/cross in the real heading frame, yaw),
normalised by stock, 8 windows x 10 s, 3 studies x 40 trials. Point =
median of the top 12 trials: armature 0.0049, frictionloss 5.38,
actuator_tau 1.8 ms, foot_solref_time 7.4 ms, foot_solref_damp 1.52.
In sample: stock 1.00 -> 0.57.

Graded on FRESH windows (seed 1, 3 perturbed replicates; the chaos floor is
~0.002 in every term, so the ordering is real):

| plant | along (m) | cross (m) | yaw (rad) |
|---|---:|---:|---:|
| stock | 0.120 | 0.104 | 0.214 |
| measured (loop 1) | 0.118 | 0.117 | 0.163 |
| loop2 | 0.117 | 0.104 | **0.114** |

The in-sample position gain (-39% along) did not transfer: it was the fit
windows'. The yaw gain (-47%) did. Position tracking is not reachable with
these five knobs, on either loop; what closes it is a capture or a
contact-model question, not a search. This is the anti-transfer pattern
go2sim measured, one storey up: fitting through the controller finds what
the controller lets it find.

What this does NOT say: one recording on one undeclared floor with nothing
weighed anchors none of the pinned values, and the replicate floor is the
sim against itself, not the robot against itself.

## 6. Owed

- The floor material of the recording (declare it in a
  `data/<recording>.meta.json` sidecar: `{"suspended": false, "floor": "..."}`).
- A scale reading; the model says 35.112 kg.
- Whether the real IMU is torso- or pelvis-mounted (the MJCF says torso).
- A hanging recording with leg motion, and a harder walking one.
- Mode B: GR00T in the loop, grounded against Point-LIO, and draw selection.
