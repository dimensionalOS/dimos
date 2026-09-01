# LeRobot Policy Module

`LeRobotPolicyModule` runs trained LeRobot policies in a managed Python-native
subprocess. Its LeRobot, Transformers, Torch, and NumPy versions live in the
sibling `python/` project and do not change the main DimOS environment.

The host contract subscribes to:

- `color_image: Image`
- `coordinator_joint_state: JointState`

It publishes arm targets through `joint_command: JointState`. When
`gripper_joint_name` is set, it removes that element from `joint_command` and
publishes it through `gripper_command: Float32`. The checkpoint still receives
and returns the complete vector in `joint_names` order.

```python
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule

policy = LeRobotPolicyModule.blueprint(
    policy_path="outputs/pick/checkpoints/last/pretrained_model",
    task="pick up the object",
    joint_names=["arm/joint1", "arm/joint2", "arm/gripper"],
    gripper_joint_name="arm/gripper",
    fps=30.0,
    robot_type="my_robot",
)
```

The module exposes `start_rollout`, `stop_rollout`, and `rollout_status` RPCs.
It owns one configured checkpoint and loads it lazily on the first rollout.
The runtime rejects missing or stale observations, missing joints, non-finite
values, incompatible checkpoint features, and actions with the wrong dimension.

The runtime calls LeRobot's `select_action()` at the configured rate. ACT keeps
its action chunk internally and returns one queued action per call. Configure
the runtime rate to match the action frequency used by the training dataset.

Run isolated runtime checks with:

```bash
cd dimos/imitation/policy/lerobot/python
uv run --isolated --locked --group tests --with-editable ../../../../../ python -m pytest
uv run --isolated --locked --group tests --with-editable ../../../../../ python -m mypy
```
