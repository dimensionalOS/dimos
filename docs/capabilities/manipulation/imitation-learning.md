# Imitation Learning for Manipulation

Use `dimos run` to launch and configure a collection blueprint. The imitation
TUI attaches to its episode-control interface; it does not own the robot.

## Learning workflow

Collect demonstrations, prepare a dataset, train a policy, then run a compatible
checkpoint on the robot. The examples below use one OpenYAM arm and a wrist RGB
camera. Direct hand teaching does not require Quest.

The teaching stack starts gravity compensation as soon as the robot starts,
including between episodes. Its hand-guiding task sends zero-stiffness motor
commands on every control tick; the hardware adapter adds gravity torque and
the configured damping. Episode start/save/discard only controls dataset
boundaries. Support the arm before stopping the runtime.

| Step | Command | Result |
| --- | --- | --- |
| Launch collection | `dimos run openyam-teach-collection --daemon` with module options | Running robot, camera, recorder, and episode controller |
| Record takes | `dimos imitation collect` | Saved or discarded episodes in a recording directory |
| Inspect recording | `dimos imitation inspect recordings/session-001` | Readable recording and quality summary |
| Prepare data | `dimos imitation prepare recordings/session-001 --output datasets/session-001` | LeRobot dataset containing saved episodes |
| Visualize dataset | `dimos imitation visualize datasets/session-001` | Camera playback, joint states, and actions in Rerun |
| Train | `dimos imitation train` with LeRobot arguments | Training outputs and checkpoints |
| Launch rollout | `dimos run openyam-lerobot-rollout --daemon` with module options | Policy stack ready for operator controls |
| Execute policy | `dimos imitation rollout` | Explicit policy start/stop controls |

Launch commands configure the hardware and task. The attached panels control
episodes or policy execution. Use `dimos stop` to shut down a running stack
before switching from collection to rollout.

## Collect demonstrations

| Blueprint | Cameras | State and action |
| --- | --- | --- |
| `openyam-teach-collection` | Wrist RGB | Measured 7-D joints for both |
| `openyam-quest-collection` | Wrist RGB | Measured state, accepted commands |
| `dual-openyam-quest-collection` | Two wrist RGB cameras | Measured state, accepted commands; 14-D |

```bash
dimos --can-port follower_l run openyam-teach-collection --daemon \
  --recorder.recording recordings/session-001 \
  --recorder.format mcap \
  --episodes.task "pick up the cube" \
  --wrist.hardware.camera-index /dev/video0

dimos imitation collect
```

These are ordinary module-config flags. Use `dimos run BLUEPRINT --help` to see
all options, including camera hardware settings. JSON config and environment
overrides use the same matching rules as other DimOS blueprints.

The panel shows the task, recording state, elapsed time, and saved/discarded
episode counts. Reset the scene before each take, then guide the arm through
the demonstration.

| Key | Action |
| --- | --- |
| Space, while ready | Start an episode |
| Space, while recording | Save the episode and return to ready |
| D, while recording | Discard the episode and return to ready |
| Q | Detach from the running collection |

During an active episode, Q asks for confirmation; press Q again to detach.
**Recording and the robot continue after the TUI exits.** Run
`dimos imitation collect` again to reattach. A connection error disables panel
controls but does not stop the running stack.

When collection is finished, save or discard the final take and detach. Support
the arm before stopping the stack, since shutdown may de-torque it:

```bash skip
dimos stop
```

### Dual-arm collection with Quest

For two arms with Quest, launch the dual collection blueprint and attach the
same episode controls:

```bash skip
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

The dual profile uses 640×480 RGB images at 30 Hz with a 20 ms alignment
tolerance anchored on the left wrist. Joint order is left arm joints 1–6,
right arm joints 1–6, left gripper, right gripper. Prepare and train using the
same commands below with the dual recording and dataset paths. Dual-arm
collection and training do not imply support for dual-arm policy rollout.

## Recording directories

```text
recordings/session-001/
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

After stopping collection, inspect the recording and export its saved episodes:

```bash
dimos imitation inspect recordings/session-001
dimos imitation prepare recordings/session-001 --output datasets/session-001
dimos imitation inspect datasets/session-001
```

`inspect` prints a human-readable summary for either a recording or a prepared
dataset. Recordings show episode totals, stream message counts, and quality
metrics with units. Failed and incomplete episodes are always listed. Quality
checks describe the data, not whether the robot completed the physical task.

Use `--verbose` to see every assessed episode, or `--json` for the complete
machine-readable result with unrounded values:

```bash
dimos imitation inspect recordings/session-001 --verbose
dimos imitation inspect recordings/session-001 --json
```

Dataset summaries show frame counts, rates, episode lengths, feature shapes and
types, and statistics availability. Redirecting output keeps the readable
format; use `--json` explicitly for scripts. `--json` takes precedence over
`--verbose` when both are supplied.

Preparation requires a new output directory. Without `--output`, it writes to
`~/.local/state/dimos/datasets/<recording-directory-name>` by default.

Before training, view an episode in Rerun:

```bash
dimos imitation visualize datasets/session-001 --episode 0
```

The viewer shows camera images, joint states, and actions on a shared timeline.
Use its playback controls to play, pause, and scrub through the episode. Episode
indices start at zero; omitting `--episode` selects the first episode.

Visualization requires a local graphical display and a prepared LeRobot dataset,
not a raw recording. It runs LeRobot's existing viewer in the isolated Python
environment and streams loading progress to the terminal. Unlike inspection,
visualization decodes the episode's frames, so loading can take longer. Dataset
loading is local-only; missing files are not downloaded from Hugging Face.

Start ACT training with the prepared dataset:

```bash
dimos imitation train \
  --dataset.repo_id=local/openyam-teach \
  --dataset.root=datasets/session-001 \
  --policy.type=act \
  --output_dir=outputs/openyam-act
```

Training forwards all arguments to `lerobot-train` in its isolated Python
environment, including `--help`. Its output streams to the terminal and a failed
training process returns its exit status. Use `dimos imitation train --help` to
see the available training options.

Preparation reads the saved schema, not the current robot blueprint. Python
callers can use `RecordingSchema.read(directory).dataprep_config(directory, output)`
from `dimos.imitation.collection.recording`, then call
`run_lerobot_dataprep(config)` or `run_dataprep(config)` for HDF5 output.

Native recordings describe their message types and codecs. Preparation imports
those types: only prepare trusted recordings, and install custom message
packages in the conversion environment.

## Existing policy rollout

Stop the collection stack before launching rollout. Replace `CHECKPOINT_DIR`
with a compatible pretrained-model directory, such as
`outputs/openyam-act/checkpoints/last/pretrained_model`.

```bash
dimos --can-port follower_l run openyam-lerobot-rollout --daemon \
  --policy.policy-path CHECKPOINT_DIR \
  --policy.task "pick up the red block" \
  --policy.device cuda \
  --wristcamera.hardware.camera-index 0
dimos imitation rollout
```

Use `openyam-lerobot-quest-rollout` for the graph with Quest takeover.
The optional rollout panel discovers `RolloutControlSpec` and shows policy
state and errors. Space explicitly starts/stops policy execution. Before
starting, preflight loads the checkpoint and checks the control task and fresh
observations without sending a trajectory. A failed check leaves the policy
stopped and reports the error.

Q only detaches, even while the policy is active. Run `dimos imitation rollout`
again to reattach. To finish, stop the policy with Space, detach, support the
arm, and use `dimos stop` to shut down the runtime. Neither UI is a deadman
switch: lost connectivity does not guarantee stopping motion.

See the [LeRobot module contract](/dimos/imitation/policy/lerobot/README.md)
for checkpoint and control requirements. Rollout uses the existing single-arm,
single-camera contract with absolute joint targets in the hardware's native
coordinates. A prepared dataset does not establish checkpoint compatibility
with another robot. ABC integration, dual-arm policy rollout, and policy
backend generalization are deferred.
