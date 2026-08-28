# g1sim

The G1 plant for MuJoCo, on the shared identification pipeline
(`dimos/simulation/sysid`, the method go2sim was built with). Unlike the Go2,
the G1 is FITTED on loop 2 (`sysid/ground.py`): the GR00T policy drives the
plant on the recorded commands and Point-LIO is the yardstick. Loop 1
(`sysid/replay`, `identify`, `fit`) is kept as the open-loop sensitivity
diagnostic. The tune is `measured` (section 5).

## 1. Watch it

```bash
# what the recording contains: rates, epoch, stand/walk segments
python -m dimos.robot.unitree.g1.sim.sysid.ingest data/g1_groot_characterization_2026-08-27.db

# open-loop replay of a walk segment, the recorded pelvis pose as a ghost
python -m dimos.robot.unitree.g1.sim.sysid.replay data/g1_groot_characterization_2026-08-27.db --list
python -m dimos.robot.unitree.g1.sim.sysid.replay data/g1_groot_characterization_2026-08-27.db \
    --preset stock --segment 3 --view --speed 0.5

# loop 2: the GR00T policy driving the tuned plant on the recorded commands,
# from the measured mid-walk state, the Point-LIO pelvis as a ghost
python -m dimos.robot.unitree.g1.sim.sysid.ground data/g1_groot_characterization_2026-08-27.db \
    --preset measured --view            # --start S --seconds N --speed 0.5 to narrow
# grade any preset held out, or refit (writes <out>.plant.json + .loop.json)
python -m dimos.robot.unitree.g1.sim.sysid.ground data/g1_groot_characterization_2026-08-27.db \
    --preset measured --windows 8 --seconds 10 --seed 1 --replicates 3
python -m dimos.robot.unitree.g1.sim.sysid.ground data/g1_groot_characterization_2026-08-27.db \
    --preset measured --fit 60 --studies 3 --workers 8 --out dimos/robot/unitree/g1/sim/presets/candidate
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

## 5. The tune

`measured` = `presets/measured.plant.json` (the plant half, restated as
`ranges.MEASURED`) + `presets/measured.loop.json` (the loop half). Fitted on
loop 2, 2026-08-28: the GR00T policy drives the plant from the robot's
measured mid-walk state on the recorded commands, and the loss is the
tracking area against Point-LIO (along/cross in the real heading frame,
yaw) plus the sway cadence error, each divided by stock; 8 windows x 10 s,
3 studies x 60 CMA-ES trials over 18 knobs, shipped as the median of the
top tenth of trials.

| knob | stock | measured |
|---|---:|---:|
| armature (kg m^2) | 0.010 | 0.0135 |
| frictionloss (N m) | 0.10 | 3.49 |
| actuator_tau (s) | 0 | 0.0031 |
| foot_solref_time / damp | 0.020 / 1.0 | 0.0074 / 0.91 |
| foot_solimp_dmin / width | 0.90 / 0.001 | 0.69 / 0.0036 |
| foot_friction | 1.0 | 0.92 |
| trunk_mass_scale, leg_mass_scale | 1.0, 1.0 | 1.08, 0.96 |
| envelope: gain beyond speed | none | 0.63 beyond 7.5 rad/s |
| action_delay (physics steps) | 0 | ~2 (10 ms) |
| cmd_dt / rig_dt (s) | 0 / 0 | -0.017 / +0.010 (rig_dt measured, pinned) |

Held out (8 fresh windows, 3 perturbed replicates, chaos floor ~0.003):

| | stock | measured |
|---|---:|---:|
| along (m) | 0.128 | 0.113 |
| cross (m) | 0.105 | 0.115 |
| yaw (rad) | 0.207 | 0.077 |
| cadence | 10.9% | 7.9% |

What was learned on the way, so nobody re-opens it: five plant knobs alone
move only yaw; a wider search without a cadence term bought along-track
area by striding slower (the tracking area cannot see it over 10 s); the
rig lever-arm knobs come back at ~1 cm with wide spread, so the Point-LIO
mount stands; Point-LIO trails the IMU by 10 ms on every axis (measured by
cross-correlation, pinned as `rig_dt`); and every fit kept a speed-torque
envelope and ~10 ms of action delay, the two things to MEASURE on the
hardware next (delivered-vs-demanded torque at speed; a command-echo
timestamp). Loop 1 (`replay`, `identify`, `fit`) remains as the
sensitivity diagnostic; walking this gentle resolves no knob there.

Not said by any of this: the replicate floor is the sim against itself,
one recording on one undeclared floor with nothing weighed anchors none of
the declared values, and cross-track never moved.

## 6. Owed

- The floor material of the recording (declare it in a
  `data/<recording>.meta.json` sidecar: `{"suspended": false, "floor": "..."}`).
- A scale reading; the model says 35.112 kg.
- Whether the real IMU is torso- or pelvis-mounted (the MJCF says torso).
- A hanging recording with leg motion, and a harder walking one.
- Measure the envelope and the action delay on the hardware instead of fitting them.
- A robot-repeat floor (two recordings of the same walk) in place of the sim-perturb one.
