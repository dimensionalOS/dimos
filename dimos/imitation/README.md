# Imitation learning

Collection uses ordinary DimOS Blueprints. The graph owns robot hardware,
cameras, transports, and runtime lifecycle. A `CollectionProfile` declares
typed inputs and dataset projections; `collection_recorder(profile=...)`
creates matching recorder ports before autoconnect.

Profiles have no separate registry. `dimos run` discovers Blueprints through the
built-in registry or installed `dimos.blueprints` entry points. The Blueprint
passes a profile to its recorder; the profile name is recording metadata, not
a Blueprint lookup key. Profile validation checks declarations and shared-source
consistency. Recorder wiring checks required inputs; preparation validates the
actual recorded values.

## OpenYAM Quest collection

```bash
dimos run openyam-quest-collection \
  --recorder.recording recordings/session-001 \
  --episodes.task "pick up the cube"
```

Configure camera and hardware options through `dimos run BLUEPRINT --help`.
Quest B starts/saves an episode; Y discards it. Python clients can use
`Dimos.connect().find_module_by_spec(EpisodeControlSpec)` and its
`get_status()` and `command(event)` RPCs instead.

The recording is a new directory containing `schema.json` and
`recording.mcap`, or `recording.db` with `--recorder.format sqlite`.
Existing directories are rejected. Copy or move the complete directory.
Stopping the runtime leaves an active episode incomplete; export excludes
incomplete and discarded episodes. Support the arms before shutdown.

## Prepare a recording

```python
from pathlib import Path

from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.core import OutputConfig
from dimos.imitation.dataprep.lerobot import run_lerobot_dataprep

directory = Path("recordings/session-001")
config = RecordingSchema.read(directory).dataprep_config(
    directory, OutputConfig(format="lerobot", path=Path("datasets/session-001"))
)
run_lerobot_dataprep(config)
```

Preparation uses the saved schema, not a current robot profile. Only prepare
trusted recordings; custom message classes must be installed in the reader
environment. Generic `run_dataprep(config)` supports HDF5 output.

Each feature declares its recorded source's meaning with `source_kind`:

- `"snapshot"` (default): align to the nearest observation within the configured
  tolerance. This also applies when measured state supplies a teaching action.
- `"joint_position_updates"`: reconstruct persistent `JointState.position`
  targets by joint name, using only updates at or before each dataset timestamp.
  Omitted joints retain their targets, including across episode boundaries.
  Missing initial joints and malformed updates fail validation.

Features sharing a recorded stream must declare the same source kind. Command
history is reconstructed once before projecting individual features. Inspection
and preparation share alignment and value checks; MCAP and SQLite capture remain
unaligned, native-rate streams. Start a new recording after an unrecorded target
reset or control-mode change.

## Policy execution

The [LeRobot module](policy/lerobot/README.md) provides isolated checkpoint
loading, preflight, and controlled trajectory execution. Collection profiles
do not define arbitrary policy-backend compatibility.
