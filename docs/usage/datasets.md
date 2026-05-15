# Datasets

DimOS records demonstrations as per-episode `.rrd` files via
[`RerunDataRecorder`](visualization.md#data-collection-recordings). To train a
[LeRobot](https://github.com/huggingface/lerobot) policy on those recordings,
convert them into a `LeRobotDataset` v2 directory using the in-tree converter.

## Install

The converter depends on the LeRobot SDK (≥ 0.5.1) and `rerun.experimental.RrdReader`
(rerun-sdk ≥ 0.32). LeRobot ships with the `manipulation` extra:

```bash
pip install dimos[manipulation]
```

If LeRobot's pins drag rerun-sdk below 0.32, upgrade it explicitly:

```bash
pip install --upgrade 'rerun-sdk>=0.32'
```

The contract package itself (`dimos.manipulation.policy.contract`,
`.contracts.piper`, `.contracts.registry`) imports neither LeRobot nor the
rerun dataframe API and can be used in environments without the extra.

## Convert one session

Most teleop sessions contain many demonstrations of the same task. Point the
converter at the session directory and pass the task description:

```bash
python scripts/datasets/rrd_to_lerobot.py \
    --input  data/piper_data_collection/20260514T210114Z/ \
    --output data/lerobot/20260514T210114Z/ \
    --task   "pick the red block"
```

One `.rrd` file becomes exactly one LeRobot episode. Frames are gated on
camera arrival (~30 Hz); joint state and action scalars use Rerun's latest-at
semantics on the same query, so per-frame rate matches the camera rate
without an explicit resampler.

The default `--output` is `data/lerobot/<session_dir_name>/` and the default
`--repo-id` is `<contract>/<session_dir_name>` (e.g. `piper/20260514T210114Z`).

## Convert one episode

`--input` also accepts a single file:

```bash
python scripts/datasets/rrd_to_lerobot.py \
    --input data/piper_data_collection/20260514T210114Z/episode_001.rrd \
    --task  "pick the red block"
```

## Per-episode tasks

When one session contains heterogeneous demonstrations, pass a JSONL file
mapping episode stem → task description:

```bash
cat > tasks.jsonl <<'EOF'
{"episode": "episode_001", "task": "pick"}
{"episode": "episode_002", "task": "place"}
{"episode": "episode_003", "task": "pick"}
EOF

python scripts/datasets/rrd_to_lerobot.py \
    --input data/piper_data_collection/20260514T210114Z/ \
    --task-per-episode tasks.jsonl
```

Every rrd file in the input must have an entry; the converter fails fast and
writes nothing if any episode is unlabeled.

## Gripper binarization

LeRobot policies typically expect a binary gripper command (open/close), but
the recorder writes a continuous gripper position (e.g. `0.0` closed → `0.85`
open for Piper). The converter binarizes the `action` slot by default:

- Normalize: `norm = (raw − closed_pos) / (open_pos − closed_pos)`.
- Threshold: `1.0` if `norm > 0.7`, else `0.0`.

The threshold is applied **only to the action** — the `observation.state`
gripper slot is rescaled to fraction-open in `[0, 1]` for unit consistency
but is **not** thresholded, so the model still observes the actual continuous
position.

Toggle / override:

```bash
# Use a different threshold (must be in [0.0, 1.0])
python scripts/datasets/rrd_to_lerobot.py ... --gripper-threshold 0.5

# Skip binarization entirely — preserve raw gripper position in BOTH state
# and action (no rescaling, no thresholding).
python scripts/datasets/rrd_to_lerobot.py ... --no-binarize-gripper
```

The default threshold and the `open_pos` / `closed_pos` values come from the
selected `RobotContract` (Piper: `0.85` / `0.0`).

## Recommended pre-processing

For long recordings, run [`rerun rrd optimize`](https://rerun.io/docs)
before conversion to GoP-split and keyframe-tag the recording:

```bash
rerun rrd optimize data/piper_data_collection/<session>/episode_007.rrd
```

This keeps reads fast on multi-minute episodes. Short demonstrations don't
need it.

## Output layout

The converter produces a standard LeRobotDataset v3.0 directory:

```
data/lerobot/<session>/
├── meta/info.json                              ← codebase_version="v3.0", schema, fps, episode count
├── meta/stats.json                             ← per-feature stats across the whole dataset
├── meta/tasks.parquet                          ← unique task strings (replaces v2 tasks.jsonl)
├── meta/episodes/chunk-000/file-000.parquet    ← per-episode metadata (replaces v2 episodes.jsonl)
├── data/chunk-000/file-000.parquet             ← all frames concatenated; episode_index column distinguishes them
└── videos/observation.images.usb/chunk-000/file-000.mp4   ← all episode video concatenated, per camera
```

v3.0 swaps the v2 per-episode file pattern for concatenated parquet/mp4 files
that distinguish episodes via the `episode_index` column. Reads via
`LeRobotDataset(repo_id=..., root=...)` return the same per-frame view either
way; the on-disk layout is the only difference.

Frame schema (per `PiperRobotContract`):

| Key                          | Dtype     | Shape          |
|------------------------------|-----------|----------------|
| `observation.images.usb`     | `video`   | `(480, 640, 3)`|
| `observation.state`          | `float32` | `(7,)`         |
| `action`                     | `float32` | `(7,)`         |

`observation.state` and `action` are packed in `joint1, …, joint6, gripper`
order.

## Selecting a different robot

The CLI resolves robot-specific schema knowledge through a registry. List
available contracts and pick one:

```bash
python scripts/datasets/rrd_to_lerobot.py ... --contract piper
```

To add a new robot, implement `RobotContract` in
`dimos/manipulation/policy/contracts/<robot>.py` and register it in
`dimos/manipulation/policy/contracts/registry.py`.
