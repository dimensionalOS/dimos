# Stage 1 — Data (v2)

Two phases:

1. **Recording** — live; teleop + camera + `EpisodeMonitorModule` produce
   LCM streams. `RecordReplay` (CLI flag) captures every active topic
   into `session.db` (memory2 `SqliteStore` format).
2. **DataPrep** — offline; `DataPrepModule` reads `session.db` and writes
   a training-ready dataset on disk in one of two formats (LeRobot v2,
   HDF5).

Same code paths drive both phases as DimOS blueprints. Format dispatch is
config-only; the same module backs every output format.

---

## Phase A — Recording

### Blueprint

```python
# dimos/learning/collection/blueprint.py
_DEFAULT_BUTTON_MAP = {"start": "A", "save": "B", "discard": "X"}
_TRANSPORTS = {
    ("buttons",     Buttons):       LCMTransport("/teleop/buttons",          Buttons),
    ("color_image", Image):         LCMTransport("/camera/color_image",      Image),
    ("status",      EpisodeStatus): LCMTransport("/learning/episode_status", EpisodeStatus),
}

learning_collect_quest_xarm7 = autoconnect(
    teleop_quest_xarm7,
    RealSenseCamera.blueprint(enable_pointcloud=False),
    EpisodeMonitorModule.blueprint(button_map=_DEFAULT_BUTTON_MAP),
).transports(_TRANSPORTS)

# Variants (one per arm config) follow the same pattern:
#   learning_collect_quest_xarm6
#   learning_collect_quest_piper
#   learning_collect_quest_dual
```

`RecordReplay` (`--record-path`) captures every transport above into
`session.db`. Recording is a transport-layer hook, not a Module — every
LCM stream is recorded uniformly.

### EpisodeMonitorModule

Translates teleop input (Quest buttons, optional keyboard) into the
canonical `EpisodeStatus` stream. DataPrep reads only this stream, never
raw button presses.

```python
# dimos/learning/collection/episode_monitor.py

class EpisodeStatus(BaseModel):
    state:                     Literal["idle", "recording"]
    episodes_saved:            int
    episodes_discarded:        int
    current_episode_start_ts:  float | None
    last_event:                Literal["start", "save", "discard", "init"] = "init"
    task_label:                str | None = None


class KeyPress(BaseModel):
    key: str
    ts:  float


class EpisodeMonitorModuleConfig(ModuleConfig):
    button_map:         dict[Literal["start", "save", "discard"], str] = {"start": "A", "save": "B", "discard": "X"}
    keyboard_map:       dict[Literal["start", "save", "discard"], str] = {}
    default_task_label: str | None = None


class EpisodeMonitorModule(Module):
    config:   EpisodeMonitorModuleConfig

    buttons:  In[Buttons]
    keyboard: In[KeyPress]
    status:   Out[EpisodeStatus]

    @rpc
    def reset_counters(self) -> EpisodeStatus: ...
    @rpc
    def get_status(self)     -> EpisodeStatus: ...
```

State machine (mirrored offline by `DataPrep.extract_episodes`):

```
IDLE      --start-->     RECORDING
RECORDING --save-->      IDLE        (commit, saved += 1)
RECORDING --discard-->   IDLE        (drop,   discarded += 1)
RECORDING --start-->     RECORDING   (auto-commit prev, begin new)
session end mid-episode: always discarded
```

Friendly button names (`A`/`B`/`X`/...) resolve to `Buttons` attributes
via `BUTTON_ALIASES` (e.g. `"A"` → `right_primary`). Override with the
attribute name directly in `button_map`.

### Run

```bash
dimos run learning-collect-quest-xarm7 --record-path data/recordings/pick_red.db
```

---

## Phase B — DataPrep

### Blueprints

```python
# dimos/learning/dataprep_blueprint.py

learning_dataprep = autoconnect(
    DataPrepModule.blueprint(
        source="data/recordings/pickplace_001.db",
        episodes=EpisodeExtractor(extractor="ranges", ranges=[(t0, t1)]),
        observation={
            "image":       StreamField(stream="color_image", field="data"),
            "joint_state": StreamField(stream="joint_state", field="position"),
        },
        action={
            "joint_target": StreamField(stream="joint_state", field="position"),
        },
        sync=SyncConfig(anchor="image", rate_hz=14.0, tolerance_ms=80.0),
        output=OutputConfig(
            format="lerobot",
            path="data/datasets/pickplace_001",
            metadata={"fps": 14, "robot": "xarm7", "default_task_label": "pick_and_place"},
        ),
        auto_run=True,
    ),
).transports({})


# Variant for one-demo-per-file recordings (no episode_status stream).
learning_dataprep_whole_session = autoconnect(
    DataPrepModule.blueprint(
        source="data/session.db",
        episodes=EpisodeExtractor(extractor="whole_session"),
        observation={...},
        action={...},
        sync=SyncConfig(anchor="image", rate_hz=30.0, tolerance_ms=50.0),
        output=OutputConfig(format="lerobot", path="data/datasets/default",
                            metadata={"fps": 30, "robot": "xarm7"}),
        auto_run=True,
    ),
).transports({})
```

All `DataPrepModuleConfig` fields are defaulted — the DimOS CLI's
per-module override path validates user kwargs in isolation, so
required-without-default fields would reject partial `-o ...` overrides.
Real values come from the blueprint atom; CLI flags overlay on top.

### DataPrepModule

```python
# dimos/learning/dataprep.py
from dimos.protocol.service.spec import BaseConfig


class EpisodeExtractor(BaseConfig):
    extractor:     Literal["episode_status", "ranges", "whole_session"] = "episode_status"
    status_stream: str = "episode_status"
    ranges:        list[tuple[float, float]] | None = None


class StreamField(BaseConfig):
    stream: str
    field:  str | None = None


class SyncConfig(BaseConfig):
    anchor:       str
    rate_hz:      float
    tolerance_ms: float
    strategy:     Literal["nearest", "interp"] = "nearest"


class OutputConfig(BaseConfig):
    format:   Literal["lerobot", "hdf5"] = "lerobot"
    path:     Path
    metadata: dict[str, Any] = {}
```

```python
# dimos/learning/dataprep_module.py

class DataPrepModuleConfig(ModuleConfig):
    source:      str = ""
    episodes:    EpisodeExtractor = EpisodeExtractor()
    observation: dict[str, StreamField] = {}
    action:      dict[str, StreamField] = {}
    sync:        SyncConfig = SyncConfig(anchor="image", rate_hz=30.0, tolerance_ms=50.0)
    output:      OutputConfig = OutputConfig(format="lerobot", path="data/datasets/default")
    auto_run:    bool = False


class DataPrepModule(Module):
    config: DataPrepModuleConfig

    @rpc
    def build(self)      -> None:           ...   # spawns build thread; returns immediately
    @rpc
    def get_status(self) -> dict[str, Any]: ...   # state, current_phase, progress_pct, samples_seen, error
    @rpc
    def inspect(self)    -> dict[str, Any]: ...   # streams, episode counts, duration distribution
```

`build()` opens the `SqliteStore`, walks samples episode-by-episode,
hands them to the configured format writer, and snapshots the spec
(`config.model_dump()`) into `<output.path>/dimos_meta.json`. The build
thread is a daemon — there is no mid-iteration cancel.

### Pure helpers (in `dataprep.py`)

Stateless, importable without booting a Module. Reused by every format
writer (single source of truth for obs construction).

```python
def resolve_field(msg: Any, ref: StreamField) -> np.ndarray: ...

def extract_episodes(store: SqliteStore, cfg: EpisodeExtractor) -> list[Episode]:
    """
    episode_status: replay EpisodeMonitorModule's state machine over
                    the recorded EpisodeStatus events.
    ranges:         emit one Episode per (start, end) tuple.
    whole_session:  one Episode covering every stream's combined time range.
    """

def iter_episode_samples(
    store:       SqliteStore,
    episode:     Episode,
    streams:     dict[str, StreamField],     # observation ∪ action
    sync:        SyncConfig,
    obs_keys:    set[str] | None = None,
    action_keys: set[str] | None = None,
) -> Iterator[Sample]:
    """
    Anchor-rate timestep walker. Caches each stream once per episode,
    bisect-nearest within tolerance_ms; skips frames where any required
    stream lacks a nearby sample.
    """

def compute_stats(
    samples:            Iterator[Sample],
    image_subsample:    int = 10,
    quantile_reservoir: int = 10_000,
    seed:               int = 0,
) -> dict[str, Any]:
    """Welford mean/std + reservoir quantiles. Image features (≥3D)
    subsampled and reduced to per-channel summaries."""
```

### Format writers (`dimos/learning/formats/`)

Both writers consume `Iterator[Sample] + OutputConfig` and accumulate
stats via the shared `StreamingStats` (`formats/_stats.py`) so format-
agnostic stats logic exists in exactly one place.

```python
# dimos/learning/formats/_stats.py
class StreamingStats:
    def __init__(self, image_subsample=10, quantile_reservoir=10_000, seed=0): ...
    def update(self, name: str, value: np.ndarray) -> None: ...
    def finalize(self) -> dict[str, dict[str, Any]]: ...
```

| Format | Layout | Heavy dep |
|---|---|---|
| `lerobot` | `meta/{info,episodes,tasks,stats}.json` + `data/chunk-000/episode_NNNNNN.parquet` + `videos/chunk-000/observation.images.<k>/episode_NNNNNN.mp4` | `pyarrow`, `opencv-python` |
| `hdf5` | single `.hdf5` with `/episodes/episode_NNNNNN/{timestamp, observation/<k>, action/<k>}` + `/stats/<feat>` + `/tasks` + root attrs | `h5py` |

#### LeRobot v2 specifics

- **Image columns are NOT in the parquet** — lerobot's
  `get_hf_features_from_features` skips dtype="video" and reads frames
  from MP4 at `__getitem__` time.
- **Timestamps are episode-relative** (subtract `episode.start_ts`)
  because lerobot stores `timestamp` as float32 and validates frame-to-
  frame deltas against `1/fps`. Absolute Unix epoch values would collide
  in float32.
- **Feature naming follows lerobot convention** — single low-dim obs ⇒
  `observation.state`, single action key ⇒ `action`, image keys ⇒
  `observation.images.<key>`.
- **`info.json` features include per-dim `names` lists** (required by
  lerobot 0.3+).

### dimos_meta.json sidecar

Written into every dataset directory; describes how it was built and
records the obs/action schema alongside the data.

```json
{
  "source":      "data/recordings/pickplace_001.db",
  "observation": {"image": {...}, "joint_state": {...}},
  "action":      {"joint_target": {...}},
  "sync":        {"anchor": "image", "rate_hz": 14.0, ...},
  "episodes":    [{"id": "ep_000000", "start_ts": ..., "end_ts": ..., "task_label": ...}],
  "format":      "lerobot",
  "metadata":    {"fps": 14, "robot": "xarm7", ...}
}
```

### Run

```bash
dimos run learning-dataprep
```

Override per run:

```bash
dimos run learning-dataprep \
  -o dataprepmodule.source=data/recordings/foo.db \
  -o dataprepmodule.output.path=data/datasets/foo \
  -o dataprepmodule.output.format=hdf5
```

For complex nested overrides (observation/action stream maps), use a JSON
config:

```bash
dimos run learning-dataprep -c data/foo_dataset.json
```

---

## End-to-end

```bash
dimos run learning-collect-quest-xarm7 --record-path data/recordings/pick_red.db
dimos run learning-dataprep \
  -o dataprepmodule.source=data/recordings/pick_red.db \
  -o dataprepmodule.output.path=data/datasets/pick_red
```

```
data/recordings/pick_red.db ─► data/datasets/pick_red/
                                    ├── data/    (parquet)            ─┐
                                    ├── videos/  (MP4)                  ├─ format=lerobot
                                    └── meta/    (info, episodes,      ─┘
                                                  tasks, stats)
                                    └── dimos_meta.json   (always)
```

---

## Compatibility note — JpegCodec

Recordings made before commit `<this branch>` ship Image blobs with a
1-byte format tag (`b'J'`) preceding the LCM envelope. `JpegCodec.decode`
strips it transparently so old + new sessions both read cleanly. Affects
any consumer of recorded JPEG-encoded `Image` streams, not just learning.

---

## Module / non-Module split for Stage 1

| Component | Type | Why |
|---|---|---|
| `EpisodeMonitorModule` | `Module` | Long-lived; subscribes to teleop input; publishes status |
| `DataPrepModule`       | `Module` | Long-running build job; thread + `get_status` RPC |
| `RecordReplay`         | transport hook | Captures every stream uniformly; not a per-Module concern |
| `StreamingStats`       | helper class | No lifecycle, no I/O — pure accumulator |
| `extract_episodes` / `iter_episode_samples` / `resolve_field` / `compute_stats` | functions | Pure helpers; reused by every format writer |
