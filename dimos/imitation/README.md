# Imitation Learning

Collect demonstrations, build training datasets, and run trained policies in
DimOS. Quest teleoperation or direct arm teaching records episodes to a SQLite
or MCAP artifact. DataPrep converts that recording into a LeRobot or HDF5
dataset for imitation learning.

```
Quest teleop ─┐
              ├─▶ recorder ─▶ session_<robot>_<ts>.db/.mcap ─▶ dimos dataprep ─▶ dataset
direct teach ─┘
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
dimos --simulation run learning-collect-quest-xarm7 --task "pick up the red block"

# Piper on real hardware
dimos run learning-collect-quest-piper --task "pick up the red block"

# OpenYAM + Quest + wrist camera; native MCAP
dimos --can-port follower_l run learning-collect-quest-openyam \
  --task "pick up the red block" \
  --WristCamera.hardware.camera-index 0 \
  --nativecollectionrecorder.store.path data/recordings/session_openyam.mcap
```

This brings up teleop, a RealSense (real only), the episode monitor, and the
recorder, all wired together.

### Controls (Quest)

| Button | Action |
| --- | --- |
| Controller grip | **Hold to engage** — the arm tracks the controller only while held |
| **B** | **Toggle record** — press to start an episode, press again to save it |
| **Y** | **Discard** the in-progress episode |

So a take is: hold the controller grip to move the arm into place → press **B** to start →
perform the task → press **B** to save (or **Y** to throw it away). The terminal
prints one line per transition:

```
[collect] ▶ RECORDING episode  (state=recording  saved=0  discarded=0)
[collect] ✓ SAVED episode      (state=idle       saved=1  discarded=0)
```

> End each good take with **B** before quitting — an episode still recording at
> shutdown is dropped.

### OpenYAM direct teaching

Direct teaching removes the Quest teleoperator. The arm runs with gravity
compensation, zero position stiffness, and joint damping. Move it by hand while
the existing OpenYAM observation and action streams are recorded.

Start the hardware stack in one terminal. Be ready to support the arm as it
activates, and keep people and obstacles outside its workspace.

```bash
dimos --can-port follower_l run learning-collect-teach-openyam --daemon \
  --task "pick up the red block" \
  --WristCamera.hardware.camera-index 0 \
  --nativecollectionrecorder.store.path data/recordings/openyam-teach.mcap
```

Attach the collection panel from another terminal:

```bash
dimos collect
```

```text
┌────────────────────── OpenYAM teach collection ──────────────────────┐
│ Task       pick up the red block                                    │
│ State      RECORDING                                                │
│ Episodes   2 saved, 0 discarded                                     │
│ Gripper    passive — move by hand                                   │
└─────────────────────────────────────────────────────────────────────┘
```

| Key | Action |
| --- | --- |
| **Space** | Start an episode; press again to save it |
| **D** | Discard the in-progress episode |
| **Q** or **Ctrl-C** | Detach the panel while idle |

The panel refuses to detach while recording. Save or discard the take first.
Detaching closes only the panel's RPC connection; the arm remains active in
gravity-compensation mode until you run `dimos stop`.

A complete session is:

```text
start daemon ─▶ attach panel ─▶ start/save takes ─▶ detach panel ─▶ stop daemon
```

After collection, stop the stack cleanly and build the dataset with the
direct-teach profile:

```bash
dimos stop
dimos dataprep build \
  --source data/recordings/openyam-teach.mcap \
  --profile dimos.robot.manipulators.openyam.learning:OPENYAM_TEACH_LEARNING_PROFILE \
  --output data/datasets/openyam-teach
```

The action row contains the measured arm and gripper positions at that instant.
The direct-teach profile therefore reads both state and action from the continuous
coordinator joint-state stream; Quest collection continues to use accepted commands.

Before a production collection, run one hardware smoke test:

1. Support the arm, start the daemon, and confirm that the arm and gripper can
   be moved by hand without position-hold resistance. The arm should retain
   light joint damping while the enabled gripper runs with zero stiffness,
   zero damping, and zero feed-forward torque.
2. Attach `dimos collect` and verify that the panel reports the gripper as
   passive.
3. Record and save a short take, stop the daemon, then inspect the MCAP with
   `dimos dataprep inspect` and `OPENYAM_TEACH_LEARNING_PROFILE`.

### Where the recording goes

```
~/.local/state/dimos/recordings/session_<robot>_<YYYYMMDD_HHMMSS>.db
```

A new timestamped SQLite file per XArm/Piper run, or a timestamped MCAP file
for OpenYAM (nothing is overwritten). Both formats record `color_image`,
`coordinator_joint_state`,
`applied_joint_position_command`, and `status` (the episode
start/save/discard markers).

The exact path is printed when the recorder starts — note it for the next step.

---

## 2. Build a dataset

DataPrep is an offline batch step that reads a session `.db` or `.mcap` and
writes a dataset. A typed Python profile owns the observation/action schema and
output format; each invocation supplies only its source and optional output.

```bash
# OpenYAM's typed profile selects LeRobot and its 30 Hz wrist/joint/action schema
dimos dataprep build \
  --source data/recordings/session_openyam.mcap \
  --profile dimos.robot.manipulators.openyam.learning:OPENYAM_LEARNING_PROFILE \
  --output data/datasets/openyam-mcap
```

The OpenYAM graph uses the globally selected transport (Zenoh by default) and
MCAP. Override the path with
`--nativecollectionrecorder.store.path /path/to/session_openyam.mcap`. Stop
collection gracefully so MCAP can finalize its summary and indexes; MCAP
append mode is unsupported.

Reuse the profile across runs and swap `--source`. `--output` overrides the
profile's output path, and `--quality-mode strict|fill` overrides its validation
mode. A profile can select HDF5 instead of LeRobot in its `DataPrepConfig`.

Inspect the result (features, shapes, dtypes, episode/frame counts):

```bash
dimos dataprep inspect data/datasets/session       # LeRobot dir
dimos dataprep inspect data/datasets/session.hdf5  # HDF5 file

# Validate saved recording episodes before conversion
dimos dataprep inspect session_openyam.mcap \
  --profile dimos.robot.manipulators.openyam.learning:OPENYAM_LEARNING_PROFILE
```

Each dataset gets a `dimos_meta.json` sidecar recording exactly how it was built
(source, schema, sync, quality settings, included episodes, and rejection or
fill reports).

---

## 3. Profile reference

A profile is a Python object with `dataprep_config() -> DataPrepConfig`. Pass it
as `module:attribute`; profiles are imported and executed, so use trusted local
code. The returned config is a reusable template whose fields mean:

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
- **`output`** — `format` (`lerobot` | `hdf5`), default `path`, and `metadata`
  (`repo_id`, `robot_type`, …).

---

## Notes

- **Sim vs real camera** — under `--simulation` the MuJoCo camera supplies
  `color_image`; on real hardware a RealSense does. The blueprint picks the
  right one automatically.
- **"action" is an applied command** — it is published only after arbitration
  and hardware acceptance. Rejected and non-position commands are not emitted.

## 4. Roll out an OpenYAM checkpoint

```bash
uv run dimos \
  --can-port follower_l \
  run learning-rollout-quest-openyam \
  --LeRobotPolicyModule.policy-path \
    "$PWD/outputs/train/openyam-act/checkpoints/last/pretrained_model" \
  --WristCamera.hardware.camera-index 0
```

Quest **A** toggles policy rollout. Hold the right controller grip to take over
with teleoperation; its higher-priority tasks preempt the current policy chunk.
Release the grip to let the next policy chunk resume control. Press **A** again
to stop rollout and cancel the active chunk. **B** and **Y** remain reserved for
collection save/discard controls.
