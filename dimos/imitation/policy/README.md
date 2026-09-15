# Policy rollout

`PolicyModule` runs ABC or LeRobot inference in an independently locked Python
subprocess. Both backends share observation handling, preflight, start/stop,
status, and trajectory execution. Select the backend with module configuration:
`--policy.backend abc` or `--policy.backend lerobot`.

```python
from dimos.imitation.policy.module import PolicyModule

policy = PolicyModule.blueprint(
    backend="lerobot",
    policy_path="outputs/pick/checkpoints/last/pretrained_model",
    task="pick up the object",
    joint_names=["arm/joint1", "arm/joint2", "arm/gripper"],
    image_mapping={"wrist_image": "observation.images.wrist"},
    fps=30.0,
)
```

`image_mapping` maps typed image input ports to checkpoint feature names.
The factory declares these ports before `autoconnect`; normal blueprint
remappings and transports connect cameras. CLI overrides may change feature
names on those declared ports. Changing camera topology requires changing the
blueprint. Each camera must provide fresh HWC uint8 RGB images; state arrives
on `coordinator_joint_state`. `joint_names` defines the hardware vector order.
An explicit `policy_joint_names` maps a different checkpoint order in both
directions; LeRobot defaults to `joint_names`, and ABC requires it explicitly.

The module exposes `preflight_rollout`, `start_rollout`, `stop_rollout`, and
`rollout_status`. Preflight loads the model, validates inputs, and performs a
motion-free inference warmup. It never submits a trajectory. Start requires a
successful preflight and fresh observations. Existing attached controls
(`dimos imitation rollout`) preflight on an explicit start request. In a Quest
blueprint, A toggles a preflighted rollout. Backend changes take effect at launch.

After start, inference and execution repeat automatically. Each submitted
trajectory begins at the observed joint state and includes the configured
execution horizon. The coordinator rejects a moved starting state; the runtime
waits for a newer observation before trying again. Invalid outputs, stale inputs,
and other trajectory errors stop rollout and cancel its trajectory. Stop clears
backend action state. Chunk acceptance logs include backend, inference time,
executed steps, and action frequency.

## Backend contracts

| Backend | Inputs and preprocessing | Execution defaults |
|---|---|---|
| LeRobot | Checkpoint image features, state, task; official pre/postprocessors | Checkpoint `n_action_steps`, 30 Hz unless configured |
| ABC-DiT | `top`, `left`, `right`, 14-joint state, task; official resize/pad and normalization | Predict 30 actions, execute 15 at a 0.034-second period |

Both adapters return absolute joint targets in hardware order, including
grippers. LeRobot clips to its checkpoint action statistics. The coordinator
continues to enforce hardware limits. Cartesian or delta actions require a
different adapter contract and are unsupported here.

ABC uses the released default sequential inference scheme. RTC is not enabled.
`fast_inference` enables the official BF16, compile, and CUDA-graph path; warmup
can take substantially longer than subsequent predictions. See
[ABC setup](abc/README.md) for the checkpoint, assets, and three-camera rig.

LeRobot image dimensions must match checkpoint features. Its pre/postprocessors
and `predict_action_chunk()` remain responsible for model-specific inference.
Temporal ensembling is unsupported. Configure `fps` to match the training data.
The existing dataset conversion and training commands continue to use the
LeRobot environment.

## Validation

Host tests live beside the shared module and robot blueprints. Each isolated
project has its own backend tests and type checks:

```bash
cd dimos/imitation/policy/abc/python
uv run --locked --group tests --with-editable ../../../../../ python -m pytest
uv run --locked --group tests --with-editable ../../../../../ python -m mypy
```

Run the same commands in `lerobot/python` for LeRobot. The ABC unit test compares
the adapter with the actual upstream sampler using a small model; it needs no
GPU, network, or downloaded checkpoint.
