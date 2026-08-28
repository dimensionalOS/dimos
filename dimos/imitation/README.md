# Imitation Learning

Collect demonstrations, build training datasets, and run trained policies in
DimOS. Teleoperation records episodes to a SQLite or MCAP artifact, and DataPrep
converts that recording into a LeRobot or HDF5 dataset for imitation learning.

```
teleop (Quest) ─▶ CollectionRecorder ─▶ session_<robot>_<ts>.db/.mcap ─▶ dimos dataprep ─▶ dataset
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

A new timestamped SQLite file per run (nothing is overwritten). The native
OpenYAM profile can instead write MCAP when selected through its module config.
Both formats record `color_image`, `coordinator_joint_state`,
`applied_joint_position_command`, and `status` (the episode
start/save/discard markers).

The exact path is printed when the recorder starts — note it for the next step.

---

## 2. Build a dataset

DataPrep is an offline batch step that reads a session `.db` or `.mcap` and writes a dataset.
The obs/action stream mapping is nested, so it comes from a JSON config — start
from [`dataprep/example_config.json`](dataprep/example_config.json) and edit the
`source`/`output` to taste.

```bash
# LeRobot v3.0 (default)
dimos dataprep build \
  --source ~/.local/state/dimos/recordings/session_xarm7_20260622_120000.db \
  --config dimos/imitation/dataprep/example_config.json

# HDF5 instead
dimos dataprep build -s <session.db> -c <config.json> -f hdf5

# Physical OpenYam + Quest + /dev/video0 wrist camera
dimos run learning-collect-quest-openyam --can-port can0
# Experimental native recorder over reliable Zenoh
dimos --transport zenoh --can-port can0 \
  run learning-collect-quest-openyam-native \
  --task "pick up the red block" \
  --WristCamera.webcam.camera-index /dev/video0
dimos dataprep build \
  --source <session_openyam.db> \
  --config dimos/imitation/dataprep/openyam_lerobot.json

# Select MCAP for one native collection run
dimos --transport zenoh --can-port can0 \
  run learning-collect-quest-openyam-native \
  --task "pick up the red block" \
  --WristCamera.webcam.camera-index /dev/video0 \
  --nativecollectionrecorder.store.kind mcap \
  --nativecollectionrecorder.store.path data/recordings/session_openyam.mcap
dimos dataprep build \
  --source data/recordings/session_openyam.mcap \
  --config dimos/imitation/dataprep/openyam_lerobot.json \
  --output data/datasets/openyam-mcap
```

The native OpenYAM blueprint is an explicit experiment; the command without
`-native` keeps using the Python recorder and shared-memory camera transport.
The native graph uses reliable Zenoh for all recorded streams. Override its
session backend and path with both
`--nativecollectionrecorder.store.kind mcap` and
`--nativecollectionrecorder.store.path /path/to/session_openyam.mcap`.
SQLite remains the default. Stop collection gracefully so MCAP can finalize its
summary and indexes; MCAP append mode is unsupported.

`--source` / `--output` / `--format` override whatever the config specifies, so
you can reuse one config across runs and just swap `--source`. The dataset is
written to the config's `output.path` (the example uses `data/datasets/session`)
unless you pass `--output`.

Inspect the result (features, shapes, dtypes, episode/frame counts):

```bash
dimos dataprep inspect data/datasets/session       # LeRobot dir
dimos dataprep inspect data/datasets/session.hdf5  # HDF5 file

# Validate saved recording episodes before conversion
dimos dataprep inspect session_openyam.db \
  --config dimos/imitation/dataprep/openyam_lerobot.json
dimos dataprep inspect session_openyam.mcap \
  --config dimos/imitation/dataprep/openyam_lerobot.json
```

Each dataset gets a `dimos_meta.json` sidecar recording exactly how it was built
(source, schema, sync, quality settings, included episodes, and rejection or
fill reports).

---

## 3. Config reference

See [`dataprep/example_config.json`](dataprep/example_config.json) for a full,
working example. The fields that matter:

- **`source`** — the session `.db` or `.mcap`.
- **`observation` / `action`** — map each final dataset feature name to an
  explicit `{stream, field, dtype, shape, names}` schema. OpenYAM actions come
  from the accepted `applied_joint_position_command`, not future feedback.
- **`sync`** — resample everything onto one timeline: `anchor` stream,
  `rate_hz`, and nearest-match `tolerance_ms`. `fps` is derived from `rate_hz`
  unless set explicitly.
- **`quality`** — `strict` excludes only invalid episodes; `fill` preserves the
  fixed-rate grid with causal holds and marks filled frames in
  `complementary_info.is_filled`. The build fails when no valid episode remains.
- **`output`** — `format` (`lerobot` | `hdf5`), `path`, and `metadata`
  (`repo_id`, `robot_type`, …).

---

## Notes

- **Sim vs real camera** — under `--simulation` the MuJoCo camera supplies
  `color_image`; on real hardware a RealSense does. The blueprint picks the
  right one automatically.
- **"action" is an applied command** — it is published only after arbitration
  and hardware acceptance. Rejected and non-position commands are not emitted.
