# Imitation Learning

See [Collection profiles](/dimos/imitation/collection/README.md) for the recording contract,
shared feature class and executable custom-robot examples.

Collect demonstrations, build training datasets, and run trained policies in
DimOS. Teleoperation records episodes, and DataPrep converts SQLite or MCAP
recordings into a LeRobot or HDF5 dataset for imitation learning.

```
teleop (WebXR) -> CollectionRecorder -> recording directory -> DataPrep -> dataset
```

After training, use the production
[`LeRobotPolicyModule`](policy/lerobot/README.md) to run a checkpoint against
live camera and joint-state observations.

---

## 1. Record a session

Run a collection blueprint. Add `--simulation` to drive MuJoCo; omit it for real
hardware (a RealSense + the arm).

```bash
# XArm7 in sim
dimos --simulation run learning-collect-webxr-xarm7

# Piper on real hardware
dimos run learning-collect-webxr-piper
```

This brings up teleop, a RealSense (real only), the episode monitor, and the
recorder, all wired together.

### Controls (WebXR)

| Button | Action |
| --- | --- |
| **A** (right) / **X** (left) | **Hold to engage** — the arm tracks the controller only while held |
| **B** | **Toggle record** — press to start an episode, press again to save it |
| **Y** | **Discard** the in-progress episode |

So a take is: hold **A** to move the arm into place → press **B** to start →
perform the task → press **B** to save (or **Y** to throw it away). The terminal
prints one line per transition:

```
[collect] ▶ RECORDING episode  (state=recording  saved=0  discarded=0)
[collect] ✓ SAVED episode      (state=idle       saved=1  discarded=0)
```

> End each good take with **B** before quitting — an episode still recording at
> shutdown is dropped.

### Where the recording goes

The recorder creates a new session directory under
`~/.local/state/dimos/recordings/`. It contains `schema.json` and one raw
payload: `recording.mcap` by default, or `recording.db` for SQLite.

The saved schema describes feature projections, episode markers, alignment and
quality rules. Its stream names are the actual recorded topics. No Python
message classes or absolute recording paths are stored in it. The profile
supplies the typed inputs; multiple features can project different fields from
one recorded stream.

The exact directory is printed when the recorder starts. Existing directories
are errors, so a new capture cannot silently replace a previous session.

---

## 2. Build a dataset

Use the schema saved beside the payload to prepare a collection. This preserves
its topic names and joint ordering rather than guessing them from the robot name.
The input directory in this example must contain a completed recording:

```python skip
from pathlib import Path

from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.lerobot import run_lerobot_dataprep
from dimos.imitation.dataprep.schema import OutputConfig

recording = Path("data/recordings/session")
schema = RecordingSchema.read(recording)
config = schema.dataprep_config(recording, OutputConfig(path=Path("data/datasets/session")))
run_lerobot_dataprep(config)
```

For HDF5, select `OutputConfig(format="hdf5", path=...)` and call the host
`run_dataprep(config)`. LeRobot build and inspection use the isolated runtime.

The low-level `dimos dataprep` command remains available for explicit SQLite or
MCAP configs. Generate a config from the saved schema before passing it to
`--config`; `schema.json` itself is a recording contract, not a `DataPrepConfig`.
The old CLI is removed only when its `dimos imitation` replacement is registered.

See the [offline dataset guide](/dimos/imitation/dataprep/README.md) and
[isolated LeRobot exporter](/dimos/imitation/dataprep/lerobot.md) for validation,
inspection and output provenance. Discarded and unfinished episodes are excluded.

---

## 3. Config reference

- **`source`**: the recorded `.db` or `.mcap` file.
- **`observation` / `action`**: distinct feature names mapped to explicit
  `stream`, `field`, `dtype`, `shape`, `names`, and `source_kind` schemas.
  JointState vectors follow the configured joint-name order.
- **`sync`**: output-feature `anchor`, fixed `rate_hz`, and nearest-snapshot
  `tolerance_ms`. There is no implicit next-frame action shift.
- **`quality`**: `strict` validation by default, or `fill` with per-frame fill
  provenance. Existing fill behavior has no snapshot-age limit.
- **`output`**: `format` (`lerobot` or `hdf5`), `path`, and `metadata`.
  A sidecar records feature schemas, alignment settings and quality reports.

---

## Notes

- **Sim vs real camera** — under `--simulation` the MuJoCo camera supplies
  `color_image`; on real hardware a RealSense does. The blueprint picks the
  right one automatically.
- **Action semantics are explicit**: the example config uses measured joint-state
  snapshots. To train on accepted targets, record their command stream and use
  `source_kind="joint_position_updates"`; this reconstructs sparse targets
  causally, including accepted history before episode start.
- **Old vs new sessions** — recordings made before the `coordinator_joint_state`
  rename use the old stream name; point a matching config at them, or re-record.
