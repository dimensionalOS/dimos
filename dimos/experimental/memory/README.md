# Experimental Native Memory Recorder

The Rust recorder is an experimental high-throughput alternative to the Python
Memory2 recorder, with SQLite support and schema-aware MCAP recording.
Its API and operational behavior are under evaluation. Experimental imports may change
without compatibility aliases.

## Build and runtime packaging

The recorder is built as a locked Nix package. Nix supplies Rust, CMake, NASM,
SQLite, and the native libraries used by TurboJPEG, so none of those tools or
development packages need to be installed on the host.

The Python module builds the package automatically on first use. To build it
ahead of time, run:

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

SQLite supports LCM-backed messages and the `lcm`, `jpeg`, and `lz4+lcm`
storage codecs. Images default to JPEG quality 50. Configure depth or other
lossless streams explicitly with `lz4+lcm`. The resulting artifact opens and
replays through the stable Python `SqliteStore` API.

## MCAP

Select MCAP to write CDR with embedded ROS 2 message definitions, or JSON with
JSON Schema for LineSegments3D, into indexed Zstd chunks:

```python
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig

mcap_recorder = SensorRecorder.blueprint(
    store=RustMcapStoreConfig(path="session.mcap"),
    encoding_threads=4,
)
```

Images default to lossless `sensor_msgs/msg/Image`, including depth data.
Use `stream_codecs={"color_image": "jpeg"}` to opt into lossy 8-bit JPEG as
`sensor_msgs/msg/CompressedImage`; depth-to-JPEG is rejected. Serialization and
compression happen in Rust. LCM is only the input transport representation.
Source time is the MCAP publish time, and recorder reception time is the log time.

Install `dimos[recording]` and open the result with the new schema-aware store:

```python
from dimos.memory.store.mcap_recording import McapRecordingStore

with McapRecordingStore(path="session.mcap") as store:
    print(store.summary())
    image = store.stream("color_image").first().data
```

`dimos mem summary session.mcap` and `dimos mem rerun session.mcap` select this
store automatically for the `dimos` profile. No LCM codec registry is needed.
See [recording usage](../../../docs/usage/recording.md) for the supported types
and [the design decision](../../../docs/architecture/mcap-recording.md) for the
conversion boundaries. Unknown selected types fail before modifying artifacts.

Append mode remains unsupported for MCAP.

Both backends preserve source timestamps for common stamped messages. Arbitrary
pickle payloads, Python `pose_setter_for` hooks, and spatial pose attachment
remain Python-recorder features; unsupported combinations fail during startup.
