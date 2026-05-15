# Recording demonstrations to `.rrd` for robot learning

The `teleop_quest_piper_data_collection` blueprint pairs the live Rerun viewer
with a standalone **`RerunDataRecorder`** that writes one `.rrd` per
demonstration *episode*. The viewer and the recorder are independent DimOS
modules — either can be omitted without affecting the other — and they share a
single `piper_data_collection_rerun_config()` so the operator sees exactly what
gets recorded. The bridge module is unchanged; the recorder owns its own
`rr.RecordingStream` and subscribes to the same LCM topics in parallel.

## Layout

```
{data_dir}/piper_data_collection/<session_ts>/episode_001.rrd
{data_dir}/piper_data_collection/<session_ts>/episode_002.rrd
…
```

`<session_ts>` is a UTC `YYYYMMDDTHHMMSSZ` stamp. The episode counter is
1-indexed and zero-padded to 3 digits.

## Per-episode entities

Each `.rrd` carries:

- `/observation/camera/usb` — the USB camera frames (`rr.Image`).
- `/observation/state/<joint>` — per-joint measured scalars including the gripper.
- `/action/<joint>` — per-joint commanded scalars including the gripper.
- `/meta/episode_id`, `/meta/episode_index`, `/meta/session_id`, and
  (when `DIMOS_OPERATOR` is set) `/meta/operator` — static text documents.

## Recording on/off toggle

Press the **right-controller B** button (`right_secondary`) to **toggle**
recording. The recorder has two states:

- **RECORDING** — `.rrd` is open, messages flow in.
- **IDLE** — no `.rrd` is open; messages on the bus continue to drive the
  live viewer but are silently discarded by the recorder.

The recorder **starts in IDLE** — nothing lands on disk until you press the
toggle button. This keeps warmup motion, scene setup, and gripper calibration
out of every recording. Each press flips the state:

1. Blueprint starts → recorder is IDLE; no `.rrd` exists.
2. Get the robot in position and ready to demo → press B → recorder opens
   `episode_001.rrd`, enters RECORDING.
3. Demo finishes → press B → recorder flushes and closes `episode_001.rrd`,
   enters IDLE.
4. Take your time resetting the scene — the robot may still be commanded,
   the camera may still stream, none of it lands on disk.
5. Press B again → recorder opens `episode_002.rrd`, enters RECORDING.

Press feedback appears in the log:

```
EpisodeBoundary: recording stopped — waiting for next press to resume
EpisodeBoundary: recording started → /…/episode_002.rrd
```

The toggle is debounced (500 ms by default); bouncing the button twice in
quick succession only counts once. If you toggle off before any payload has
been logged, the empty file is removed; the next toggle-on opens the next
slot number (so filenames may have honest holes if you press twice with no
demo in between).

> **Why B, not A?** A (`right_primary`) is the press-and-hold engage button
> used by the teleop tasks (`TeleopIKTask` / `PinkTeleopTask`) — reusing it
> would toggle the recording on every engage. B is on the same hand but a
> deliberate reach away.

## Reading recordings for training

For ad-hoc reads, post-session, GoP-split and keyframe-tag a captured episode:

```bash
rerun rrd optimize /path/to/episode_007.rrd
```

Then read it from a PyTorch training loop with the experimental dataloader:

```python skip
import rerun.experimental as rrx

dataset = rrx.dataloader(
    rrds=["/path/to/episode_007.rrd"],
    window=(0.0, 1.0),  # 1-second observation/action chunks
)
sample = next(iter(dataset))
# sample contains /observation/camera/usb, /observation/state/*, /action/*
```

> **Note:** `rerun.experimental.dataloader` is experimental. The on-disk `.rrd`
> format is backward-compatible; only the read-side API may shift.

## Browsing collected episodes

Point a Rerun catalog server at the data-collection directory to inspect /
flag episodes in the Dataset Review UI (rerun-sdk 0.32+):

```python skip
import rerun as rr

rr.server.Server(datasets={"piper": "/path/to/piper_data_collection"})
```

Each `.rrd` becomes one addressable row in the resulting dataset; the static
`/meta/*` entities surface as per-row columns.
