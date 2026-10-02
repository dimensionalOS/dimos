# Offline dataset preparation

DataPrep reads a completed DimOS SQLite (`.db`) or MCAP (`.mcap`) recording,
selects saved episodes, aligns the configured features, and writes a dataset.
Inspection and export use the same feature validation and alignment plan.
The source recording remains unchanged.

## Configure and inspect the source

Start from [example_config.json](/dimos/imitation/dataprep/example_config.json). Set `source` to your
recording, adapt the feature shapes and names to your robot, and set
`output.format` and `output.path`. The example maps its action to measured
`coordinator_joint_state` snapshots; a feature named `joint_target` does not
make that data an accepted command.

`inspect_recording` accepts a `DataPrepConfig` to report saved-episode quality
before export. The CLI command `dimos dataprep inspect data/recordings/session.db`
reports stream counts, saved/discarded episodes, and unfinished episodes. Use
your actual path; `.mcap` recordings are supported too. The CLI's recording
inspection does not accept a feature config, so use the Python API for quality
inspection with the same config you will export.

For MCAP, sample alignment uses source/publish timestamps, rather than reception
timestamps. Recorded message-type metadata supplies codecs for supported typed
channels; the reader environment must contain their Python message classes.

## Explicit features

`observation` and `action` each map a distinct output feature name to a
`FeatureSpec` in [schema.py](/dimos/imitation/dataprep/schema.py). Each feature declares:

| Field | Meaning |
| --- | --- |
| `stream` | Recorded source stream name. |
| `field` | Optional message attribute or dictionary key, such as `position`. |
| `dtype` | NumPy dtype for numerical data, or `video` for uint8 image arrays. |
| `shape` | Exact per-frame dimensions, excluding the time axis. |
| `names` | Ordered vector element names, or image axis names. |
| `source_kind` | `snapshot` (default) or `joint_position_updates`. |
| `message_type` | Optional live-capture Python message class; omitted from JSON. |

The same `FeatureSpec` is used for live collection and offline preparation.
Offline configurations need no `message_type`: recorded channel metadata owns
decoding. Collection requires a message class for every captured stream and
checks its native codec before creating the recorder. Saved feature schemas and
the isolated-runtime JSON protocol contain only the portable projection fields.

Both source kinds use the same alignment planner. They describe different
recorded meanings, not separate observation/action pipelines: a measured
snapshot may align to a nearby future sample, while an accepted target must use
only prior commands and retain joints omitted by sparse updates. Both can use
`JointState` messages, so the message class cannot choose this meaning for you.

JointState vectors are projected by joint name into the configured order.
Missing joints, duplicate source joint names, shape mismatches and non-finite
numeric values are validation errors. Numerical features are converted to the
declared dtype. Video features must match their declared shape and contain
uint8 image data. Features sharing a recorded stream must agree on its
`source_kind`.

This executable example defines measured observations and accepted-command
actions separately. It only validates a config; building requires the named
recorded streams to exist.

```python no-result
from dimos.imitation.dataprep.schema import (
    DataPrepConfig, FeatureSpec, OutputConfig, SyncConfig,
)

joints = ["arm/joint1", "arm/gripper"]
config = DataPrepConfig(
    source="data/recordings/session.db",
    observation={
        "observation.state": FeatureSpec(
            stream="coordinator_joint_state", field="position",
            dtype="float32", shape=(2,), names=joints,
        ),
    },
    action={
        "action": FeatureSpec(
            stream="applied_joint_position_command", field="position",
            dtype="float32", shape=(2,), names=joints,
            source_kind="joint_position_updates",
        ),
    },
    sync=SyncConfig(anchor="observation.state", rate_hz=30, tolerance_ms=20),
    output=OutputConfig(format="hdf5", path="data/datasets/session.hdf5"),
)
assert config.action["action"].names == joints
assert config.action["action"].source_kind == "joint_position_updates"
```

## Timestamps and accepted targets

`sync.anchor` names an output feature, not a raw stream. The fixed-rate timeline
starts at that feature's first timestamp inside the episode and stops at its
last timestamp. `sync.rate_hz` must be positive.

Snapshot features choose the nearest recorded sample. The effective tolerance
is the smaller of `sync.tolerance_ms` and `quality.max_alignment_error_ms`.
There is no implicit next-frame action shift: observation and action are both
evaluated at the same dataset timestamp, according to their source semantics.

`joint_position_updates` requires JointState position updates with unique
configured joint names. DataPrep reads accepted command history before episode
start, retains each joint's last accepted target when a sparse update omits it,
and applies only updates at or before each dataset timestamp. Future commands
and measured joint state do not initialize missing accepted targets. Every
configured joint needs an accepted target before a complete frame can be built.
Holding an accepted target between commands is normal command semantics and
does not mark a frame as filled.

## Episode selection and quality

The default `episodes.extractor` is `episode_status`, using the `status` stream.
Its versioned JSON events provide episode boundaries from their `ts` field and
carry task labels. Saved episodes are exported; discarded episodes are excluded.
An episode still open at end of recording is reported as incomplete and excluded.
A later `start` event closes a prior pending episode as successful at that time.
For recordings without status events, `extractor="ranges"` accepts explicit
`[start, end]` timestamp pairs; these ranges do not supply task labels.

`quality.mode` defaults to `strict`. A saved episode is rejected if a required
fixed-rate target cannot be aligned, a selected feature fails validation, or a
camera fails the configured source-rate/gap checks. Defaults are
`min_source_rate_ratio=0.95`, `max_camera_gap_ms=100`, and
`max_alignment_error_ms=20`. Camera checks use the requested dataset rate.

In `fill` mode, an out-of-tolerance snapshot can reuse its latest sample at or
before the target. Targets without an eligible source candidate are omitted
rather than fabricated. Schema validation still applies: a selected target
vector with missing joints rejects the episode. A frame using this fallback
has `complementary_info.is_filled=True`; other frames have `False`. There is
currently no age limit on this snapshot fallback or on held accepted targets.
The camera-gap setting does not impose a fill-age limit.

Quality reports include expected/emitted/filled frame counts, source camera
rates and maximum gaps, alignment errors, and rejection reasons. Export
excludes invalid saved episodes and raises an error if none remain.

## Export and provenance

Use `dimos dataprep build --source data/recordings/session.db --config
dimos/imitation/dataprep/example_config.json --format hdf5 --output
data/datasets/session.hdf5` with your recording and adapted config. The source,
format and output flags override the corresponding config values. The Python
API is `run_dataprep(config)` in [build.py](/dimos/imitation/dataprep/build.py).

HDF5 export requires `h5py` and a nonempty task label on every exported episode.
It stores each episode under `/episodes/episode_NNNNNN`, with `observation/`,
`action/`, and `complementary_info/is_filled` datasets. `timestamp` contains
float32 seconds relative to the episode's first emitted sample; the group's
`start_ts` retains that sample's absolute timestamp. Task indices and labels
are stored separately in the episode attributes and `/tasks`.

The sidecar is `<dataset-stem>.dimos_meta.json` beside an HDF5 file, or
`dimos_meta.json` inside a dataset directory. It records the source, explicit
feature schemas, synchronization and quality settings, quality reports,
exported episode boundaries/task labels, output format and metadata.
`fps` defaults to `sync.rate_hz` unless explicitly supplied in output metadata.

`dimos dataprep inspect data/datasets/session.hdf5` inspects the built dataset.
LeRobot output uses the [isolated runtime](/dimos/imitation/dataprep/lerobot.md), with
`run_lerobot_dataprep(config)` as its host entry point. Choosing an exporter
does not change the source feature or alignment contract.

Unlabeled explicit ranges use `output.metadata.default_task_label` (default
`"task"`) for dataset task metadata. Saved episode labels take precedence.
Fill-mode exports retain `complementary_info.is_filled` as a boolean feature.
SQLite recording inspection and HDF5 builds do not require the optional MCAP
package; MCAP inputs require the `learning` extra.

Alignment keeps timestamps, validation results and numeric command history in
memory. It decodes video values as selected frames are emitted instead of
retaining every decoded image in the episode. Format writers have their own
buffering behavior; the HDF5 writer currently buffers an episode before writing
its datasets.
