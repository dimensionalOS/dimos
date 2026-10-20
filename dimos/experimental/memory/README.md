# Experimental Native Memory Recorder

The Rust recorder is an experimental high-throughput alternative to the Python
Memory2 recorder. It remains compatible with the existing Python readers while
its API and operational behavior are evaluated. Experimental imports may change
without compatibility aliases.

## Build and runtime packaging

The recorder is built as a locked Nix package. Nix supplies Rust, CMake, NASM,
SQLite, and the native libraries used by TurboJPEG, so none of those tools or
development packages need to be installed on the host.

The Python module resolves the package through Nix before each launch, so its
wire protocol always matches the Python checkout. Nix reuses the cached package
when the native sources have not changed. To build it ahead of time, run:

```bash
cd dimos/experimental/memory/rust
nix --extra-experimental-features 'nix-command flakes' \
  build -L .#dimos-memory-recorder
```

The resulting executable is available at
`dimos/experimental/memory/rust/result/bin/dimos-memory-recorder`. Both the
blueprint module and experimental `dimos --record-engine rust` integration use
this executable. The global `--build-native` flag forces a rebuild through Nix.

The CLI integration is deliberately staged: Python remains the default recorder,
and Rust is selected only with `--record-engine rust`.

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

SQLite supports LCM-backed messages and the `lcm`, `jpeg`, `lz4+lcm`, and `json`
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

MCAP stores each stream's selected `lcm`, `jpeg`, `lz4+lcm`, or `json` representation
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

MCAP times must be finite, nonnegative seconds that fit an unsigned 64-bit
nanosecond timestamp. Recording fails on an unrepresentable source or reception
time instead of clamping it. Zero is valid. SQLite can retain negative source
times, such as event-relative timestamps.

Both backends preserve source timestamps for common stamped messages. Arbitrary
pickle payloads, Python `pose_setter_for` hooks, and spatial pose attachment
remain Python-recorder features; unsupported combinations fail during startup.


## JSON documents

The `json` codec accepts the existing `std_msgs.String` transport and stores its
JSON text as UTF-8 without an LCM envelope. Ordinary String streams retain their
selected LCM codec and reception timestamp. Configure an event source timestamp
explicitly with `stream_timestamp_fields={"events": "ts"}` and select the codec
with `stream_codecs={"events": "json"}`. Missing, nonnumeric or nonfinite selected
timestamps fail recording. Without a selected field, JSON uses reception time.

Unconfigured JSON options are omitted from the native launch configuration, so
existing non-JSON streams remain compatible with older recorder binaries. JSON
streams require a binary built with JSON codec support.

`stream_json_schemas={"events": schema}` embeds a JSON Schema in MCAP, whose
channel uses `message_encoding="json"`. SQLite retains the same UTF-8 document
in its existing blob table, the event time in `ts`, and reception time in tags.
Native JSON/String MCAP channels decode and replay as String through the existing
memory APIs without importing an application-specific payload class.
