# LeRobot Policy Backend

`OpenYamLeRobotPolicy` is generated from `OPENYAM_QUEST_IO`. Its host process
contains no LeRobot imports; the sibling locked project implements the common
policy backend and runs prediction in the same isolated process as the shared
rollout loop.

```python
from dimos.imitation.policy.lerobot.module import OpenYamLeRobotPolicy

policy = OpenYamLeRobotPolicy.blueprint(
    instance_name="PolicyRolloutModule",
    artifact="outputs/pick/checkpoints/last/pretrained_model",
    task="pick up the object",
)
```

The profile supplies `wrist_image`, `coordinator_joint_state`, feature keys,
image shape, joint order, and 30 Hz rate. The LeRobot adapter validates those
keys and dimensions, loads pre/postprocessors, returns a 2-D action chunk, and
reports checkpoint action bounds to the common safety loop.

Run isolated checks with:

```bash
cd dimos/imitation/policy/lerobot/python
uv run --isolated --locked --group tests --with-editable ../../../../../ python -m pytest
uv run --isolated --locked --group tests --with-editable ../../../../../ python -m mypy
```
