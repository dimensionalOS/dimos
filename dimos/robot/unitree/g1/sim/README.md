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

(filled in by the first fit)

## 6. Owed

- The floor material of the recording (declare it in a
  `data/<recording>.meta.json` sidecar: `{"suspended": false, "floor": "..."}`).
- A scale reading; the model says 35.112 kg.
- Whether the real IMU is torso- or pelvis-mounted (the MJCF says torso).
- A hanging recording with leg motion, and a harder walking one.
- Mode B: GR00T in the loop, grounded against Point-LIO, and draw selection.
