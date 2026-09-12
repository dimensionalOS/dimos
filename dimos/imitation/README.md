# Imitation learning

Collection uses ordinary DimOS Blueprints. The graph owns robot hardware,
cameras, transports, and runtime lifecycle. A `CollectionProfile` declares
typed inputs and dataset projections; `collection_recorder(profile=...)`
creates matching recorder ports before autoconnect.

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

## Policy execution

The [LeRobot module](policy/lerobot/README.md) provides isolated checkpoint
loading, preflight, and controlled trajectory execution. Collection profiles
do not define arbitrary policy-backend compatibility.
