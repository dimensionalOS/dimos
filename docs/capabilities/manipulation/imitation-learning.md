# Imitation Learning for Manipulation

Use `dimos run` to launch and configure a collection blueprint. The imitation
TUI attaches to its episode-control interface; it does not own the robot.

## Collect demonstrations

| Blueprint | Cameras | State and action |
| --- | --- | --- |
| `openyam-teach-collection` | Wrist RGB | Measured 7-D joints for both |
| `openyam-quest-collection` | Wrist RGB | Measured state, accepted commands |
| `dual-openyam-quest-collection` | Two wrist RGB cameras | Measured state, accepted commands; 14-D |

```bash
dimos run dual-openyam-quest-collection --daemon \
  --recorder.recording recordings/fold-001 \
  --recorder.format mcap \
  --episodes.task "fold the towel" \
  --controlcoordinator.left-can-port follower_l \
  --controlcoordinator.right-can-port follower_r \
  --left-wrist.hardware.camera-index /dev/video0 \
  --right-wrist.hardware.camera-index /dev/video2

dimos imitation collect
```

These are ordinary module-config flags. Use `dimos run BLUEPRINT --help` to see
all options, including camera hardware settings. JSON config and environment
overrides use the same matching rules as other DimOS blueprints.

Space starts or saves an episode; D discards it. Q detaches. During an active
episode, Q asks for confirmation: **recording and the robot continue after the
TUI exits**. Use `dimos stop` separately to stop the stack; stopping real
hardware may de-torque the arms, so support them first.

The dual profile uses 640×480 RGB images at 30 Hz with a 20 ms alignment
tolerance anchored on the left wrist. Joint order is left arm joints 1–6,
right arm joints 1–6, left gripper, right gripper.

## Recording directories

```text
recordings/fold-001/
├── schema.json
└── recording.mcap
```

Choose `--recorder.format sqlite` for `recording.db` instead. A new directory
is required; existing directories are never overwritten or resumed. Copy or move
the whole directory.

The collection layer writes the schema before capture. It contains the relative
payload filename, profile identity, dataset features, joint ordering, episode
extraction, synchronization, and quality settings. No Python classes or absolute
dataset paths are serialized. An interrupted session remains available for
inspection; incomplete and discarded episodes are not exported.

## Profiles and external robot packages

One `CollectionProfile` declares the typed source streams and their dataset
interpretation. A `CollectionFeature` adds a Python `message_type` to the
dataprep feature fields: `stream`, `field`, `dtype`, `shape`, and `names`.
Several features can project different fields or joint subsets from one source;
the recorder captures that source once.

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import collection_recorder

# MY_PROFILE, my_robot, and my_cameras are defined in your robot package.
collect = autoconnect(
    my_robot,
    *my_cameras,
    collection_recorder(profile=MY_PROFILE, instance_name="recorder"),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
)
```

The factory returns an ordinary blueprint with typed inputs before autoconnect.
It accepts optional `recording=Path(...)` and `format="mcap" | "sqlite"`
blueprint defaults. The output directory can instead be supplied through run
configuration. No recorder subclass is needed.

The Python graph chooses camera producers, devices, and transports. An image
feature does **not** construct a webcam. Add any number of camera features and
matching producers, using normal blueprint remappings when names differ.
Source names must be nonreserved Python identifiers, and message classes must
be importable and support native LCM encoding. The recorder also requires the
reserved `status: In[EpisodeStatus]` input.

Export the blueprint using an installed package entry point:

```toml
# pyproject.toml, for a distribution named vendor-robot
[project.entry-points."dimos.blueprints"]
collect = "vendor_robot.collection:collect"
```

```bash
dimos run vendor-robot.collect --daemon \
  --recorder.recording recordings/session-001 \
  --episodes.task "pick up the cup"
dimos imitation collect
```

No imitation workflow registration is needed. The TUI uses
`Dimos.connect().find_module_by_spec(EpisodeControlSpec)`. External controllers
can implement the same typed RPCs instead of subclassing our episode monitor.
Exactly one implementation must match; missing or ambiguous matches are errors.
The controller class must be importable in the client.

## Prepare and train

```bash
dimos imitation inspect recordings/fold-001
dimos imitation prepare recordings/fold-001 --output datasets/fold
dimos imitation train \
  --dataset.repo_id=local/dual-openyam-quest \
  --dataset.root=datasets/fold \
  --policy.type=act \
  --output_dir=outputs/dual-openyam-act
```

Preparation reads the saved schema, not the current robot blueprint. Python
callers can use `RecordingSchema.read(directory).dataprep_config(directory, output)`
from `dimos.imitation.collection.recording`, then call
`run_lerobot_dataprep(config)` or `run_dataprep(config)` for HDF5 output.

Native recordings describe their message types and codecs. Preparation imports
those types: only prepare trusted recordings, and install custom message
packages in the conversion environment.

## Existing policy rollout

```bash
dimos --can-port follower_l run openyam-lerobot-rollout --daemon \
  --policy.policy-path CHECKPOINT_DIR \
  --policy.task "pick up the red block" \
  --policy.device cuda \
  --wristcamera.hardware.camera-index 0
dimos imitation rollout
```

Use `openyam-lerobot-quest-rollout` for the graph with Quest takeover.
The optional rollout panel discovers `RolloutControlSpec`; Space explicitly
starts/stops policy execution. A start request checks preflight readiness.
Quitting only detaches, even while the policy is active. Neither UI is a
deadman switch: lost connectivity does not guarantee stopping motion.

See the [LeRobot module contract](/dimos/imitation/policy/lerobot/README.md)
for checkpoint and control requirements. This refactor retains its existing
single-camera contract. ABC integration, dual-arm policy rollout, and policy
backend generalization are deferred.
