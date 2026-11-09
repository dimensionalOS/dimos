# LeRobot Policy Module

`LeRobotPolicyModule` runs trained LeRobot policies in a managed Python-native
subprocess. Its LeRobot, Transformers, Torch, and NumPy versions live in the
`native/python/lerobot` project and do not change the main DimOS environment.

The host contract subscribes to:

- `color_image: Image`
- `coordinator_joint_state: JointState`
- `button_pressed: UInt32`
- `teleop_buttons: UInt32`

These are generated CDR messages from `dimos.messages.sensor_msgs.msg` and
`dimos.messages.std_msgs.msg`. Images must use `rgb8` at the configured size;
row padding is respected. Joint positions are selected by the configured
names. The packed button bit layout stays the same as WebXR.

It submits complete, timestamped action chunks to the existing
`joint_trajectory` task through the control coordinator. The state and action
vectors use `joint_names` order, including any gripper joint. The policy output
is already postprocessed into each joint's native absolute coordinate; the
runtime does not reinterpret gripper values.

```python
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule

policy = LeRobotPolicyModule.blueprint(
    policy_path="outputs/pick/checkpoints/last/pretrained_model",
    task="pick up the object",
    joint_names=["arm/joint1", "arm/joint2", "arm/gripper"],
    fps=30.0,
    robot_type="my_robot",
    image_width=640,
    image_height=480,
)
```

The module exposes `preflight_rollout`, `start_rollout`, `stop_rollout`, and
`rollout_status` RPCs. Preflight loads the checkpoint and processors, validates
the control task and fresh live observations, and sends no trajectory.
`start_rollout` refuses to run until preflight passes and rechecks observations
before starting.
The runtime rejects missing or stale observations, missing joints, non-finite
values, incompatible checkpoint features, and malformed action chunks. Pressing the
configured Quest button (A by default) toggles a preflighted rollout. Pressing
either middle-finger grip stops rollout. Release both grips before explicitly
restarting; releasing a grip alone never resumes the policy.

The runtime calls LeRobot's `predict_action_chunk()`, postprocesses the entire
chunk, clips every action dimension to the checkpoint's recorded data range,
and executes its first `n_action_steps` at the configured `fps`. Trajectory execution uses the coordinator's existing start-position and velocity handling. Configure `fps` to
match the action frequency used by the training dataset.

Current limitation: this contract assumes every postprocessed action is an
absolute target in the connected hardware joint's native coordinate. A generic
contract for checkpoints that encode grippers in normalized or device-specific
coordinates remains future work; this runtime does not special-case those
grippers.

Run isolated runtime checks with:

```bash
cd native/python/lerobot
uv run --isolated --locked --group tests --with-editable ../../.. python -m pytest
uv run --isolated --locked --group tests --with-editable ../../.. python -m mypy
```

The checks use a fake policy and coordinator, require no checkpoint or robot,
and exercise real CDR memq/loopback Zenoh inputs. Policy images and state are
copied at the input boundary; source Header nanoseconds remain in cached state.
Trajectory RPCs retain their existing interface.

Runtime projects still require the shared checkout described in
[the isolated Python guide](../../../experimental/isolated_python/README.md).
Installing a DimOS wheel does not ship this optional runtime project or pin
that shared checkout to the host revision. Use a matching checkout for a CDR
host and child; the package/checkout alignment gate remains separate from
these encoding and policy-lifecycle checks.
