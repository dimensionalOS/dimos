# R1Pro ACT simulation check

This first integration connects LeRobot ACT inference to an actual MuJoCo R1Pro
through the DimOS policy runtime, coordinator, and shared-memory motor adapter.
It supports a native MuJoCo window and composing the cached HSSD house package.

This is **deployment validation**, not a trained household grasping policy.
The supplied diagnostic checkpoint runs an ACT network with a constant output
head: left wrist +0.05 rad, right wrist -0.05 rad. No training or grasp-success
claim is attached to it.

## Current simulation contract

- 18 position actions in radians: torso 1–4, left arm 1–7, right arm 1–7.
- One external overview RGB image, 320 by 240; 15 Hz policy, 30 Hz capture.
- Fixed base, wheels, and grippers. The pinned vendor URDF gives the gripper
  joints zero travel, effort, and velocity limits. This check preserves that
  constraint. A future grasping profile must add validated gripper coordinates.
- Gravity-compensated position servos, not calibrated hardware dynamics.
- Mesh collisions enabled, with the planning model's structural exclusions and
  the overlapping base/first-torso convex hull pair excluded.
- House fixtures and props keep their dynamics. The builder does not weld props
  to make them stable or attach objects to grippers.
- Explicit robot joint/actuator mappings prevent house free joints from becoming
  policy actions. The 14-dimensional OpenYAM checkpoints are incompatible.

## Run the diagnostic with a native window

Use this branch's checkout as the working directory. Keep the host environment
already used for DimOS; the commands below install LeRobot in its separate locked
Python project. No OpenAI API key is needed for this check.

```bash
source .venv/bin/activate
export PYTHONPATH="$PWD"
export MUJOCO_GL=glfw

uv run --frozen --project dimos/imitation/policy/lerobot/python \
  --with-editable . python -m dimos_lerobot.demo_r1pro_checkpoint \
  "$PWD/recordings/r1pro-act-check/checkpoint"

python -m dimos.robot.galaxea.r1pro.demo_act_sim \
  --artifact "$PWD/recordings/r1pro-act-check/checkpoint" \
  --output "$PWD/recordings/r1pro-act-check/native-studio" \
  --zenoh-scout-addr 224.0.0.224:19467
```

Create the checkpoint only once; creation refuses to overwrite an existing path.
The rollout lasts six seconds after preflight, then stops and closes its window.
Use `--seconds 30` for more viewing time. The result is saved to `result.json` in
the output directory. Give concurrent instances different output paths and
messaging addresses.

For the local HSSD house, add:

```bash
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115
```

Use a separate output directory such as `native-house`. The default camera is
external to the robot and may be occluded by room walls; move the native viewer's
free camera to inspect the robot. A future learned grasping policy needs a
purposeful robot-camera setup matching its demonstrations.

`build_r1pro_act_sim()` accepts an explicit compatible checkpoint for further
experiments. `demo_act_sim` intentionally accepts only the labelled diagnostic
checkpoint because it checks those exact wrist targets. Neither command launches
a language-model agent or real robot connection.

## Verified on 2026-09-10

Native GLFW display and CUDA ACT inference passed in both the simple scene and
HSSD house. The house accepted 21 chunks in six seconds, reached the diagnostic
wrist targets, and stopped in about 6 ms with no subsequent action publication.
The simulator's fixed-step clock now catches up after rendering, and viewer sync
runs at a display cadence instead of every motor tick. This corrected the first
house run's slow motion without loosening the acceptance tolerance.

The code has focused physics, runtime contract, reset, camera, and scheduler
checks. This remains a deployment diagnostic with an untrained output head.

## Next work toward mobile grasping

1. Confirm the intended locomanipulation branch, then integrate its base command
   and planning contracts into physics without replacing the ACT runtime.
2. Resolve gripper travel, axes, coupling, contact shapes, and force limits from
   the hardware/model source. Validate an actual contact-supported grasp first.
3. Define a new camera/state/action profile including grippers; collect R1Pro
   demonstrations and train/evaluate a matching ACT checkpoint. OpenYAM weights
   cannot be reused as if they described R1Pro joints.
4. Combine navigation to a reachable station, parked-base ACT manipulation, and
   transport. Add whole-body ACT only if the data and task require it.

The older planar R1Pro preview uses mock state and is not contact physics. This
integration branches from the completed OpenYAM policy work to reuse the actual
isolated inference stack; it does not merge the ongoing locomanipulation work.
