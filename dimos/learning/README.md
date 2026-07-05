# Teleop Data Collection → Dataset

End-to-end: teleoperate an arm, record episodes to a session DB, then convert
that DB into a LeRobot or HDF5 dataset for imitation learning.

```
teleop (Quest) ─▶ CollectionRecorder ─▶ session_<robot>_<ts>.db ─▶ dimos dataprep ─▶ dataset
```

---

## 1. Record a session

Run a collection blueprint. Add `--simulation` to drive MuJoCo; omit it for real
hardware (a RealSense + the arm).

```bash
# XArm7 in sim
dimos --simulation run learning-collect-quest-xarm7

# Piper on real hardware
dimos run learning-collect-quest-piper
```

This brings up teleop, a RealSense (real only), the episode monitor, and the
recorder, all wired together.

### Controls (Quest)

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

```
~/.local/state/dimos/recordings/session_<robot>_<YYYYMMDD_HHMMSS>.db
```

A new timestamped file per run (nothing is overwritten). It records three
streams: `color_image`, `coordinator_joint_state`, and `status` (the episode
start/save/discard markers).

The exact path is printed when the recorder starts — note it for the next step.

---

## 2. Build a dataset

DataPrep is an offline batch step that reads a session DB and writes a dataset.
The obs/action stream mapping is nested, so it comes from a JSON config — start
from [`dataprep/example_config.json`](dataprep/example_config.json) and edit the
`source`/`output` to taste.

```bash
# LeRobot v3.0 (default)
dimos dataprep build \
  --source ~/.local/state/dimos/recordings/session_xarm7_20260622_120000.db \
  --config dimos/learning/dataprep/example_config.json

# HDF5 instead
dimos dataprep build -s <session.db> -c <config.json> -f hdf5
```

`--source` / `--output` / `--format` override whatever the config specifies, so
you can reuse one config across runs and just swap `--source`. The dataset is
written to the config's `output.path` (the example uses `data/datasets/session`)
unless you pass `--output`.

Inspect the result (features, shapes, dtypes, episode/frame counts):

```bash
dimos dataprep inspect data/datasets/session       # LeRobot dir
dimos dataprep inspect data/datasets/session.hdf5  # HDF5 file
```

Each dataset gets a `dimos_meta.json` sidecar recording exactly how it was built
(source, sync, episodes).

---

## 3. Config reference

See [`dataprep/example_config.json`](dataprep/example_config.json) for a full,
working example. The fields that matter:

- **`source`** — the session `.db`.
- **`observation` / `action`** — map a dataset feature name to a recorded
  `{stream, field}`. Action defaults to the *next* frame's joint state (see
  `action_shift`), giving a next-state behavioral-cloning target.
- **`sync`** — resample everything onto one timeline: `anchor` stream,
  `rate_hz`, nearest-match `tolerance_ms`, and `action_shift` (1 = next-state BC,
  0 = action == state). `fps` is derived from `rate_hz` unless set explicitly.
- **`output`** — `format` (`lerobot` | `hdf5`), `path`, and `metadata`
  (`robot`, `default_task_label`, …).

---

## Notes

- **Sim vs real camera** — under `--simulation` the MuJoCo camera supplies
  `color_image`; on real hardware a RealSense does. The blueprint picks the
  right one automatically.
- **"action" is the measured next joint state**, not a recorded command. For
  true commanded actions you'd record `joint_command` and map `action` to it.
- **Old vs new sessions** — recordings made before the `coordinator_joint_state`
  rename use the old stream name; point a matching config at them, or re-record.
