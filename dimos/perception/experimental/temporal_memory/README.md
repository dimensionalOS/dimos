# Temporal Memory

Video RAG pipeline that builds entity-centric memory over live or replayed
video streams. A VLM extracts evidence in sliding windows, tracks entities
across time, maintains a rolling summary, and persists relations in a SQLite
graph for query-time context.

## Architecture

```
color_image (In[Image])
    │
    ▼
FrameWindowAccumulator   ← fps, window_s, stride_s, max_frames_per_window
    │
    ▼
WindowAnalyzer           ← VLM calls: caption + entities + relations
    │
    ├──▶ TemporalState   ← rolling summary, entity roster
    ├──▶ EntityGraphDB   ← persistent SQLite graph (memory/temporal/)
    └──▶ JSONL log       ← per-run raw VLM output (logs/<run>/temporal_memory/)
    └──▶ JSONL dump      ← persistent raw dump (memory/temporal_memory/)
```

Five components, each independently testable:

| Component | Responsibility |
|---|---|
| `TemporalMemory(Module)` | Thin orchestrator, RxPY pipeline, lifecycle |
| `FrameWindowAccumulator` | Bounded frame buffer, sliding window extraction |
| `WindowAnalyzer` | Stateless VLM calls (window analysis, summaries, distances) |
| `TemporalState` | Thread-safe state: entity roster, rolling summary, counters |
| `EntityGraphDB` | SQLite persistence: entities, relations, distances |

## Quickstart

```bash
# With a real robot or simulation
export OPENAI_API_KEY=...
dimos --simulation run unitree-go2-temporal-memory

# With replay data
dimos --replay run unitree-go2-temporal-memory

# Chat with the agent (queries temporal memory)
humancli
```

The standalone `temporal-memory` component is registered in `all_blueprints.py`
and can be composed with any camera source:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.experimental.temporal_memory import temporal_memory

bp = autoconnect(your_camera_blueprint, temporal_memory())
```

## Configuration

All VLM frequency knobs are exposed via `TemporalMemoryConfig` so you can
tune cost / latency / accuracy without touching code:

```python
from dimos.perception.experimental.temporal_memory import TemporalMemory, TemporalMemoryConfig

config = TemporalMemoryConfig(
    # Frame processing
    fps=1.0,                    # Target frame sampling rate (Hz)
    window_s=5.0,               # Window duration (seconds)
    stride_s=5.0,               # Stride between windows (seconds)
    max_frames_per_window=3,    # Max frames sent to VLM per window
    max_buffer_frames=100,      # Ring buffer capacity

    # VLM call frequencies
    summary_interval_s=30.0,    # Rolling summary update interval
    enable_distance_estimation=True,  # Background distance VLM calls
    max_distance_pairs=5,       # Max entity pairs per distance call
    stale_scene_threshold=5.0,  # Seconds before scene considered stale

    # VLM parameters
    max_tokens=900,             # Max tokens per VLM response
    temperature=0.2,            # VLM temperature

    # Storage
    db_dir=None,                # Persistent DB dir (default: ~/.local/state/dimos/temporal_memory/)
    new_memory=False,           # Clear persistent DB on start

    # Visualization
    visualize=True,             # Rerun entity graph (GraphNodes + GraphEdges)

    # CLIP filtering
    use_clip_filtering=True,    # Filter duplicate/static frames via CLIP
    clip_model="ViT-B/32",     # CLIP model name

    # Graph context
    max_relations_per_entity=10,  # Max relations returned per entity query
    nearby_distance_meters=5.0,   # Threshold for "nearby" in distance queries
)

bp = TemporalMemory.blueprint(config=config)
```

If no VLM is passed, one is created automatically from `OPENAI_API_KEY`.

## Storage

Two outputs, no overlap:

| Output | Location | Lifetime | Contents |
|---|---|---|---|
| JSONL log | `logs/<run>/temporal_memory/temporal_memory.jsonl` | Per-run | Raw VLM text + parsed JSON (greppable) |
| JSONL dump | `~/.local/state/dimos/temporal_memory/temporal_memory.jsonl` | Persistent | Accumulated raw VLM output across all runs |
| SQLite DB | `~/.local/state/dimos/temporal_memory/entity_graph.db` | Persistent | Entities, relations, distances |

- **JSONL** contains every VLM response verbatim (`raw_response` field) plus
  parsed structured data. Agents can grep natural language directly.
- **SQLite DB** survives across runs. Pass `new_memory=True` to clear on start.
- Set `db_dir=` to override the persistent DB location.
- Both paths are logged at startup so you can find them in the logs.

## VLM Call Budget

At default settings (`window_s=5, stride_s=5, summary_interval_s=30`):

- **Window analysis:** 1 call per 5s = 12/min
- **Rolling summary:** 1 call per 30s = 2/min
- **Distance estimation:** ~1 call per window (if enabled) = 12/min
- **Steady state:** ~26 VLM calls/min

Reduce cost by increasing `stride_s` and `summary_interval_s`, or by
disabling distance estimation (`enable_distance_estimation=False`).

## Testing

```bash
# Unit tests (29 tests, mocked VLMs, no API key needed)
DISPLAY=:99 python -m pytest dimos/perception/experimental/temporal_memory/test_temporal_memory_module.py -v -c /dev/null

# Integration test with real VLM
export OPENAI_API_KEY=...
DISPLAY=:99 python -m pytest dimos/perception/experimental/temporal_memory/test_temporal_memory_module.py -v -c /dev/null -k "integration" --slow
```
