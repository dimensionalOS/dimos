# Experimental Native Memory

The Rust recorder is an experimental high-throughput alternative to the Python
Memory2 recorder. It remains compatible with the existing Python readers while
its API and operational behavior are evaluated. Experimental imports may change
without compatibility aliases.

## Build and runtime packaging

The recorder is built as a locked Nix package. Nix supplies Rust, CMake, NASM,
SQLite, and the native libraries used by TurboJPEG, so none of those tools or
development packages need to be installed on the host.

The Python module builds the package automatically on first use. To build it
ahead of time, run:

```bash
cd native/rust
nix --extra-experimental-features 'nix-command flakes' \
  build -L .#dimos-memory-recorder
```

The resulting executable is available at
`native/rust/result/bin/dimos-memory-recorder`. The global `--build-native`
flag forces a rebuild through Nix.

## SQLite

Declare inputs as on the Python recorder. `encoding_threads` sizes the native
encoder pool, while one writer preserves arrival order and writes batches.

```python
from dimos.core.stream import In
from dimos.experimental.memory.rust_recorder import (
    RustRecorder,
    RustSqliteStoreConfig,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class SensorRecorder(RustRecorder):
    color_image: In[Image]
    lidar: In[PointCloud2]


sensor_recorder = SensorRecorder.blueprint(
    store=RustSqliteStoreConfig(path="session.db"),
    encoding_threads=4,
    stream_codecs={"lidar": "lz4+lcm"},
)
```

SQLite supports LCM-backed messages and the `lcm`, `jpeg`, and `lz4+lcm`
storage codecs. Images default to JPEG quality 50. Configure depth or other
lossless streams explicitly with `lz4+lcm`. The resulting artifact opens and
replays through the stable Python `SqliteStore` API.

## MCAP

Select MCAP to write the same Memory2 storage encodings into an indexed,
portable container:

```python
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig

mcap_recorder = SensorRecorder.blueprint(
    store=RustMcapStoreConfig(path="session.mcap"),
    encoding_threads=4,
    stream_codecs={"lidar": "lz4+lcm"},
)
```

MCAP stores each stream's selected `lcm`, `jpeg`, or `lz4+lcm` representation
in indexed Zstd chunks. Source time is the MCAP publish time and recorder
reception time is the log time. JPEG channels decode automatically. Supply
trusted codecs explicitly for LCM channels instead of trusting artifact
metadata:

```python
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.mcap import McapStore
from dimos.msgs.sensor_msgs.Imu import Imu

store = McapStore(
    path="session.mcap",
    codecs={"imu": Lz4Codec(LcmCodec(Imu))},
)
```

Append mode remains unsupported for MCAP.

Both backends preserve source timestamps for common stamped messages. Arbitrary
pickle payloads, Python `pose_setter_for` hooks, and spatial pose attachment
remain Python-recorder features; unsupported combinations fail during startup.

## Rust replay

`RustReplayer` incrementally publishes selected SQLite streams directly onto
the active DimOS transport. It is an opt-in graph source; the Python Memory2
query and replay APIs remain the default. Native MCAP replay is not supported.

```python
from dimos.core.stream import Out
from dimos.experimental.memory.rust_replayer import (
    RustReplayer,
    RustSqliteReplayStoreConfig,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image


class SensorReplay(RustReplayer):
    color_image: Out[Image]
    odom: Out[PoseStamped]


sensor_replay = SensorReplay.blueprint(
    store=RustSqliteReplayStoreConfig(path="recording.db"),
    stream_remapping={"odom": "go2_odom"},
    speed=1.0,
    loop=False,
)
```

Output declarations provide the expected payload types. Startup rejects
missing streams, type mismatches, and codecs other than `lcm`, `lz4+lcm`, and
`jpeg`. `seek` is relative to the first selected observation;
`from_timestamp` is absolute; `duration` bounds either start. All selected
streams share one replay clock.

The module exposes `pause()`, `resume()`, and `set_speed(speed)` RPCs. Late
observations publish immediately in order until replay catches up; replay does
not drop observations. Runtime seek and arbitrary Memory2 queries remain on the
Python path.

The engine resolves `seek` and `duration` before opening a bounded, globally
ordered SQLite cursor. A blocking reader decodes observations incrementally
into a one-item queue while the async replay clock schedules and publishes them.
Startup validates stream metadata, payload types, codecs, and timestamp indexes.
A corrupt payload discovered later stops replay after any earlier observations
have already been published.

Each selected observation table must have an index whose leading column is
`ts`. New Python and Rust SQLite recordings create a `(ts, id)` index. Older
unindexed artifacts must be indexed explicitly before native replay.

In a source checkout, the module builds the live Cargo workspace automatically
on first use. This includes new files that have not been committed yet. Build
the locked Nix package explicitly with:

```bash
cd native/rust
nix --extra-experimental-features 'nix-command flakes' \
  build -L .#dimos-memory-replayer
```

For the full Go2 navigation and perception graph, use the named experimental
blueprint:

```bash
dimos --replay-db go2_short run unitree-go2-rust-replay
```

The existing `dimos --replay run unitree-go2` command continues to use Python.

### Replay benchmark

Compare the Python and Rust sources with three fresh-process repeats across an
isolated source at 1× and 4× and the full Go2 graph at 1×:

```bash
uv run python -m dimos.experimental.memory.tool_replay_benchmark \
  --dataset go2_short --seek 5 --duration 20 --repeats 3 \
  --transport zenoh --out results/replay-benchmark
```

The Zenoh benchmark channels use reliable, blocking QoS so congestion cannot
masquerade as replay loss. The full profile excludes visualization. Each trial
prebuilds Rust outside the timed interval, starts in a fresh process, and
alternates engine order between repeats. Before the requested target window,
the harness replays an unmeasured one-second prefix to declare lazy publishers
and complete transport discovery for both engines. Every trial also uses a
process-unique topic namespace so concurrent or stale publishers cannot enter
the sample set.

Results appear in `samples.jsonl`, `runs.json`, `summary.json`, and `report.md`.
Timing uses the Memory2 observation timestamp that drives each replay clock,
not the sensor timestamp encoded in the payload. The harness subtracts the
median delivery phase from every sample in a run, then reports absolute schedule
jitter. This removes only the run's constant clock offset while retaining early
and late deviations and cross-stream offsets. The primary p99 is the slowest
stream's p99 in each run; the summary reports the median primary p99 across
repeats. Startup is reported separately and is not an acceptance gate.

Add `--check` to fail when a profile violates any acceptance gate:

- either engine loses, duplicates, publishes an unexpected payload, or reorders
  an observation;
- either engine's median p99 jitter exceeds 50 ms; or
- the Python-to-Rust per-stream drift delta exceeds 10 ms.

The Python-to-Rust p99 improvement remains in the report as a descriptive
comparison, not a pass condition.

Repeat `--profile` to run a smaller matrix, for example
`--profile isolated:4`. Timing gates belong in controlled performance runs;
the self-hosted smoke test checks execution and artifact shape only.
