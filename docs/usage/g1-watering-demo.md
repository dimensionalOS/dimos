# G1 Watering Demo

This note records the operational context for the G1 mobile-manipulation
watering demo on branch `refactor/g1-watering-task-architecture`.

## Current status

- The hardware approach has reached the generated stance reliably enough for
  testing using Point-LIO pose feedback and the holonomic path follower.
- A complete hardware pour has succeeded.
- Approach and pour remain separate operator-approved RPCs.
- The latest return change has unit coverage but has not been hardware-tested:
  after holding the pour, the task now plans back to the upright Cartesian pose
  before planning the inward return to the captured init joints. If untipping
  fails, it does not attempt the potentially unsafe inward return.
- The reach artifact is generated for the configured pour height and spout TCP.
  Regenerate it whenever either value changes.

## Runtime architecture

```text
RealSense -> marker detection -> marker latch -> target observation
Mid-360 -> Point-LIO -> G1LioBasePose -> world/pelvis pose

target + base pose -> WateringTaskModule -> approach Path
approach Path + base pose -> HolonomicPathFollower -> autonomy Twist
operator Twist + autonomy Twist -> ControlCoordinator -> one GR00T policy

WateringTaskModule -> ManipulationModule -> arm trajectory task
```

The control coordinator owns operator/autonomy Twist arbitration. There is no
navigation stack in this demo. The watering task owns orchestration and safety
state; the path follower owns the approach control law; the manipulation module
owns planning and trajectory execution.

## Hardware startup

Install the manipulation dependencies without pruning unrelated packages:

```bash
uv sync --extra manipulation --inexact
```

Start on the G1 (add a calibrated gravity scale only when required):

```bash
LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 \
dimos --rerun-open none --rerun-host 0.0.0.0 \
  run unitree-g1-water-demo --daemon
```

Inspect and control:

```bash
dimos status
dimos log -f
dimos shell
```

In the RPC shell:

```python
app.ControlCoordinator.set_dry_run(False)
app.WateringTaskModule.preview_watering()
app.WateringTaskModule.start_approach()
app.WateringTaskModule.get_status()
app.WateringTaskModule.start_pour()
app.WateringTaskModule.get_status()
app.WateringTaskModule.cancel_watering()
```

Clear the latched marker and wait for three consistent sightings:

```python
app.MarkerLatchModule.reset()
app.MarkerLatchModule.get_latched_marker_id()
app.MarkerLatchModule.get_latched_pose()
```

Capture an explicit safe arm-return posture before pouring if startup posture
was not the desired init posture:

```python
app.ManipulationModule.set_init_joints_to_current("g1")
app.ManipulationModule.get_init_joints("g1")
```

## Tuning points

- Marker physical black-border side length: `_MARKER_LENGTH_M` in
  `unitree_g1_water_demo.py`. It must equal the measured printed marker size.
- Spout TCP: `DEFAULT_SPOUT_OFFSET_IN_PALM` in `manip_stance.py`.
- Pour height and roll: `POUR_Z` and `TIP_RADIANS` in `manip_stance.py`.
- Filled can mass: `_WATERING_CAN_MASS_KG` in `unitree_g1_water_demo.py`.
- Gravity calibration: launch with `--gravity-ff-scale VALUE`. Hardware defaults
  to zero. The scale multiplies both modeled arm and configured payload torque;
  tune upward cautiously while physically supporting the loaded arm.
- Path-following speeds and tolerances: `HolonomicPathFollower.blueprint(...)`
  in `unitree_g1_water_demo.py`.
- Approach and final-correction budgets: `approach_timeout` and
  `settle_timeout` in the watering blueprint.

## Reach artifact

Regenerate after changing `POUR_Z`, `TIP_RADIANS`, or the spout TCP:

```bash
uv run --no-sync python -m dimos.robot.unitree.g1.tool_pour_reach_map \
  --spout-offset 0.0 0.10 0.0 \
  --out dimos/robot/unitree/g1/artifacts/right_arm_pour_reach.json \
  --plot /tmp/g1-right-pour-reach.png
```

Use the actual configured offset in the command. Artifact metadata is checked at
runtime, and stale artifacts are rejected.

## Validation

The focused local gate is:

```bash
UV_CACHE_DIR=/tmp/dimos-uv-cache uv run --no-sync pytest \
  -p no:rerunfailures \
  dimos/robot/unitree/g1/test_manip_stance.py \
  dimos/robot/unitree/g1/test_watering_task.py \
  dimos/hardware/whole_body/test_gravity.py -q
```

The last run passed 83 tests. The untested hardware delta is the explicit
untip-before-return sequence and loaded-arm gravity/payload tuning.

## Known operational issues

- The marker latch deliberately persists after the tag leaves view; reset it
  before selecting a different tag.
- A blueprint restart is required after changing marker size, payload mass, or
  gravity configuration.
- `go_init()` targets the first joint state captured by `ManipulationModule`,
  unless `set_init_joints_to_current()` is called explicitly.
- Arm trajectory completion is time-based. Payload sag can leave the physical
  joints short of the planned init pose, after which the lower-priority servo
  holds the measured handoff position. Gravity/payload compensation therefore
  matters for the loaded return.
- The watering can is represented by a TCP and gravity point mass, not full
  collision geometry. Keep hardware tests guarded even though the untip step
  removes the known hip-sweep failure mode.
