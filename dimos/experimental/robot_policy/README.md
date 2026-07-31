# Experimental Robot Policies

This package contains live robot-policy runtimes whose APIs and checkpoint compatibility are
still experimental. Recording and dataset preparation remain in `dimos.imitation`.

Install the optional LeRobot runtime with:

```bash
uv sync --extra lerobot --no-default-groups
```

LeRobot 0.6 requires Python 3.12 or newer and Transformers 5. Its environment is intentionally
separate from the `perception` extra and development groups, which currently use Transformers 4.

`LeRobotPolicyModule` loads named checkpoints lazily and converts live observations into joint
targets. Compose it with modules that publish:

- `color_image: Image`
- `coordinator_joint_state: JointState`

Configure the module inside a robot-specific blueprint:

```python
from dimos.experimental.robot_policy.lerobot import LeRobotPolicyConfig, LeRobotPolicyModule

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

The module publishes `joint_command: JointState` in its configured `joint_names` order. It exposes
`execute_learned_policy`, `stop_learned_policy`, and `policy_status`; subclasses can add named
background skills using `start_configured_policy`.

The runtime rejects missing or stale observations, missing joints, non-finite values, and policy
outputs with the wrong size. It does not clip commands or enforce robot-specific joint limits.
Compose `joint_command` through a coordinator and hardware stack that enforce the target robot's
actuation and safety constraints.
