# Imitation Learning

Collect demonstrations, build training datasets, and run trained policies in
DimOS. Teleoperation records episodes, and DataPrep converts SQLite or MCAP
recordings into a LeRobot or HDF5 dataset for imitation learning.

```
teleop (WebXR) ─▶ CollectionRecorder ─▶ session_<robot>_<ts>.db ─▶ dimos dataprep ─▶ dataset
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

```
~/.local/state/dimos/recordings/session_<robot>_<YYYYMMDD_HHMMSS>.db
```

A new timestamped file per run (nothing is overwritten). It records three
streams: `color_image`, `coordinator_joint_state`, and `status` (the episode
start/save/discard markers).

The exact path is printed when the recorder starts — note it for the next step.

---

## 2. Build a dataset

DataPrep is an offline batch step over SQLite (`.db`) or MCAP (`.mcap`)
recordings. Start from [example_config.json](/dimos/imitation/dataprep/example_config.json),
adapt the feature schemas to your robot, and set the source and output paths.

Run `dimos dataprep build --source data/recordings/session.db --config
dimos/imitation/dataprep/example_config.json --format hdf5 --output
data/datasets/session.hdf5` with your actual recording and config. The
`--source`, `--output`, and `--format` flags override config values. LeRobot
output uses `--format lerobot`.

Use `dimos dataprep inspect data/recordings/session.db` to inspect a recording,
or `dimos dataprep inspect data/datasets/session.hdf5` to inspect the output.
Saved episodes are validated before export; discarded and unfinished episodes
are excluded. HDF5 export requires an episode task label.

See the [offline dataset guide](/dimos/imitation/dataprep/README.md) for an executable config
example, Python quality-inspection API, alignment rules, and output provenance.

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
