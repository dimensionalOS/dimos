# LeRobot Policy Module

`LeRobotPolicyModule` runs trained LeRobot policies in a managed Python-native
subprocess. Its LeRobot, Transformers, Torch, and NumPy versions live in the
sibling `python/` project and do not change the main DimOS environment.

The host contract subscribes to:

- `color_image: Image`
- `coordinator_joint_state: JointState`

It publishes `joint_command: JointState` in the configured `joint_names` order.
The receiving coordinator and hardware stack must enforce joint limits and
other actuation safety constraints.

```python
from dimos.imitation.policy.lerobot.module import (
    LeRobotPolicyConfig,
    LeRobotPolicyModule,
)

policy = LeRobotPolicyModule.blueprint(
    policies={
        "pick": LeRobotPolicyConfig(
            policy_path="outputs/pick/checkpoints/last/pretrained_model",
            task="pick up the object",
        )
    },
    joint_names=["arm/joint1", "arm/joint2", "arm/gripper"],
    fps=15.0,
    robot_type="my_robot",
)
```

The module exposes three RPCs: `execute_learned_policy`,
`stop_learned_policy`, and `policy_status`. Checkpoints are loaded lazily on the
first execution. The runtime rejects missing or stale observations, missing
joints, non-finite values, incompatible checkpoint features, and actions with
the wrong dimension.

Run the hardware-independent process smoke example from the repository root:

```bash
uv run python examples/native-modules/python_lerobot.py
```

The example starts the real isolated runtime and calls `policy_status`, but it
does not load the placeholder checkpoint or publish a command.

Run isolated runtime checks with:

```bash
cd dimos/imitation/policy/lerobot/python
uv sync --locked --group tests
uv run --locked --group tests pytest
uv run --locked --group tests mypy
```
