# LeRobot Policy Module

`LeRobotPolicyModule` runs trained LeRobot policies in a managed Python-native
subprocess. Its LeRobot, Transformers, Torch, and NumPy versions live in the
sibling `python/` project and do not change the main DimOS environment.

The host contract subscribes to:

- `color_image: Image`
- `coordinator_joint_state: JointState`
- `button_pressed: Buttons`

It submits complete, timestamped action chunks to one named
`JointTrajectoryTask` through the control coordinator. The state and action
vectors use `joint_names` order, including any gripper joint. The policy output
is already postprocessed into each joint's native absolute coordinate; the
runtime does not reinterpret gripper values.

```python
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule

policy = LeRobotPolicyModule.blueprint(
    policy_path="outputs/pick/checkpoints/last/pretrained_model",
    task="pick up the object",
    joint_names=["arm/joint1", "arm/joint2", "arm/gripper"],
    trajectory_task_name="policy_rollout",
    fps=30.0,
    robot_type="my_robot",
)
```

The module exposes `start_rollout`, `stop_rollout`, and `rollout_status` RPCs.
It owns one configured checkpoint and loads it lazily on the first rollout.
The runtime rejects missing or stale observations, missing joints, non-finite
values, incompatible checkpoint features, malformed action chunks, and
trajectories outside the hardware's declared position limits. Pressing the
configured Quest button (A by default) toggles rollout.

The runtime calls LeRobot's `predict_action_chunk()`, postprocesses the entire
chunk, and executes its first `n_action_steps` at the configured `fps`. Each
trajectory starts with the joint-state observation used for inference, so the
coordinator rejects a stale start if the robot moved in the meantime. Configure
`fps` to match the action frequency used by the training dataset.

Current limitation: this contract assumes every postprocessed action is an
absolute target in the connected hardware joint's native coordinate. A generic
contract for checkpoints that encode grippers in normalized or device-specific
coordinates remains future work; this runtime does not special-case those
grippers.

Run isolated runtime checks with:

```bash
cd dimos/imitation/policy/lerobot/python
uv run --isolated --locked --group tests --with-editable ../../../../../ python -m pytest
uv run --isolated --locked --group tests --with-editable ../../../../../ python -m mypy
```
