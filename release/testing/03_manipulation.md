# 3. Manipulation

These checks need the `manipulation` extra. Most run on mock arms or in MuJoCo without hardware. The first run of an arm blueprint clones the arm's description from GitHub. The planning viewer is Viser at http://127.0.0.1:8095. See the [manipulation docs](../../docs/capabilities/manipulation/index.md).

## 3.1 Motion planning in Viser

```bash
uv run dimos run xarm7-planner-coordinator --visualization.backend viser
# In another terminal:
uv run dimos shell
```

* In Viser drag the target gizmo a few centimetres, wait for "Feasibility: feasible", then click Plan, Preview and Execute.
* Set Planning mode to "Cartesian space" and repeat.

...in the shell:

```python
from dimos.manipulation.sdk import Arm
arm = Arm.from_app(app)
q = arm.joints()
q[0] += 0.3
arm.move_joints(q, speed_scale=0.5)
```

Expected result: the mock arm reaches the target in both planning modes, and `move_joints` turns the first joint in Viser.

## 3.2 Arms on mock hardware

Without an arm address these blueprints drive a mock arm.

```bash
uv run dimos run keyboard-teleop-xarm7
uv run dimos run lite6-planner-coordinator
```

* Keyboard teleop: in the "Keyboard Teleop" window hold W and the arm moves in +X in Viser. Press [ and ] to open and close the gripper.
* Planner: drag the target gizmo in Viser, click Plan, then Execute.
* Repeat the keyboard check with `keyboard-teleop-xarm6`, `keyboard-teleop-lite6`, `keyboard-teleop-piper`, `keyboard-teleop-a750` and `keyboard-teleop-a1z`. Repeat the planner check with `a1z-planner-coordinator`, `openarm-planner-coordinator` and `dual-openyam-planner-coordinator`, adding `--visualization.backend viser`.
* The single OpenYAM blueprints are mocks only under `--simulation`. Run them as `uv run dimos --simulation run <blueprint>`. Do the keyboard check with `keyboard-teleop-openyam` and `keyboard-teleop-openyam-planner`, and the planner check with `openyam-planner-coordinator`. Add `--visualization.backend viser` to the last two.

Expected result: each blueprint starts without an arm and the model moves in Viser.

## 3.3 xArm pick and place in MuJoCo

The first run pulls the scene and the detector weights. A scan on CPU takes a minute or two. See [xArm grasping](../../docs/capabilities/manipulation/xarm-grasp.md).

```bash
uv run dimos --simulation run xarm-grasp
# In another terminal:
uv run dimos shell
```

...in the shell:

```python
from dimos.robot.manipulators.xarm.blueprints.grasp import XARM_GRASP_PROMPTS
scan = app.PickAndPlaceModule.scan_objects(XARM_GRASP_PROMPTS)
for o in scan.metadata["objects"]:   # labels are unreliable, so pick by position
    print(o["object_id"], app.ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id(o["object_id"]).points_f32().mean(axis=0))
app.PickAndPlaceModule.pick_object("<object_id>")   # the can, near x=0.57 y=0.00
app.PickAndPlaceModule.place_at(0.45, -0.25, 0.25)
```

Expected result: the scan finds six objects, the pick returns "Pick complete", the place returns "Place complete", and Viser shows the arm lifting the can and setting it down. Restart the blueprint before another pick. `xarm-grasp-graspgenx` is the same check with learned grasps, and needs a CUDA 12.8 GPU.

## 3.4 Agentic xArm in MuJoCo

Needs the agent credentials and a CUDA or MPS GPU, see [Agentic xArm simulation](../../docs/capabilities/manipulation/agentic.md).

```bash
uv run dimos run xarm-perception-sim-agent
# In another terminal:
uv run dimos mcp call get_robot_state
uv run dimos mcp call scan_objects --json-args '{"prompts": ["cup"]}'
uv run humancli
```

* In `humancli` send "Scan for objects." and then "Move 10 cm to the left."

Expected result: MuJoCo shows the arm, a table, an apple, an orange and a cup. `get_robot_state` prints the joints, end-effector pose and gripper state, `scan_objects` reports the cup with an object ID, the agent lists the objects it found, and the arm moves 10 cm sideways.

## 3.5 WebXR headset teleop

Needs a Meta Quest or PICO headset on the same network. The xArm7 runs in MuJoCo.

```bash
uv run dimos --simulation run teleop-webxr-xarm7
```

* In the headset browser open https://<host-ip>:8443/teleop, accept the self-signed certificate and tap Connect.
* Hold the right grip and move the controller. Squeeze the trigger while holding the grip, then release the grip.

Expected result: the end effector follows the controller while the grip is held, the gripper closes while the trigger is squeezed, and the arm stops when the grip is released.

## 3.6 Imitation learning data collection

Needs the `learning` extra and a WebXR headset on the same network. The arm and its camera run in MuJoCo.

```bash
uv run dimos --simulation run learning-collect-webxr-xarm7 --db-path recordings/collect.db --mujocosimmodule.fps 30
uv run dimos dataprep inspect recordings/collect.db
uv run dimos dataprep build --source recordings/collect.db --config dimos/imitation/dataprep/example_config.json
uv run dimos dataprep inspect data/datasets/session
```

* Open https://<host-ip>:8443/teleop in the headset, accept the certificate, tap Connect and hold the right grip to move the arm.
* Press B, move the arm for a few seconds and press B again. The HUD shows SAVED 001. Then press Ctrl+C.

Expected result: the recording holds one saved episode with the `color_image`, `coordinator_joint_state` and `status` streams, the build prints "wrote dataset to data/datasets/session", and the dataset has one episode.

## 3.7 R1 Pro planar-base planning

No robot is needed. The first run fetches the R1 Pro URDF from GitHub.

```bash
uv run dimos run r1pro-planar-preview
```

* Open the Viser URL from the log. Under Planning Groups select only `moving_base`.
* In Joint Control set `moving_base/r1pro/base_x` to `6.0`, then click Plan, Preview and Execute.
* Set `moving_base/r1pro/base_yaw` to `3.0`, Plan and Execute. Then set it to `-3.0`, Plan and Preview.

Expected result: each plan succeeds and the mock robot ends at each executed target. The yaw preview from 3.0 to -3.0 turns the short way through pi.
