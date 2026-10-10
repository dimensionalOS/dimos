# Experimental Native Memory Recorder

The Rust recorder is an experimental high-throughput alternative to the Python
Memory recorder. Python supplies each stream's generated type name and complete
ROS message definition. The native process records custom messages without a
message-specific decoder or rebuild.

## Build and runtime packaging

The recorder is built as a locked Nix package. Nix supplies Rust, Python for build-time message generation,
CMake, and SQLite, so none of those tools or
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
from dimos_generated.sensor_msgs.msg import Image, PointCloud2


class SensorRecorder(RustRecorder):
    color_image: In[Image]
    lidar: In[PointCloud2]


sensor_recorder = SensorRecorder.blueprint(
    store=RustSqliteStoreConfig(path="session.db"),
    encoding_threads=4,
    stream_codecs={"lidar": "lz4+cdr"},
)
```

SQLite stores generated CDR bytes by default. `lz4+cdr` adds lossless blob
compression. Raw images retain their encoding and pixels, including depth data.
For lossy compression, publish an explicit generated CompressedImage value.
The resulting artifact opens and replays through Python's SqliteStore.

## MCAP

Select MCAP for ROS2-profile recordings with CDR channels and embedded, complete
`ros2msg` schemas:

```python
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig

mcap_recorder = SensorRecorder.blueprint(
    store=RustMcapStoreConfig(path="session.mcap"),
    encoding_threads=4,
)
```

The Python reader now expects standard ROS2-profile CDR channels with embedded
`ros2msg` schemas. Installed generated message packages supply the decoders:

```python
from dimos.memory.store.mcap import McapStore

store = McapStore(path="session.mcap")
```

MCAP applies Zstd chunk compression. It rejects per-payload wrappers such as
`lz4+cdr`. Standard compressed-image messages use their own declared schema.
See `examples/message-codegen/demo_native_recording.py` for a hardware-free
external-message recording demo on both LCM and Zenoh.

Append mode remains unsupported for MCAP.

Both backends preserve source timestamps for supported stamped messages. MCAP
keeps source and reception time as integer nanoseconds; SQLite exposes floating
seconds for its existing query API while message payload stamps remain exact.
Unknown timestamp layouts use reception time. Negative source times work in
SQLite; MCAP rejects them because its time fields are unsigned. Arbitrary
pickle payloads, Python `pose_setter_for` hooks, and spatial pose attachment
remain Python-recorder features; unsupported combinations fail during startup.


## JSON documents

The `json` codec accepts the existing `std_msgs.String` transport and stores its
JSON text as UTF-8 without a CDR envelope. Ordinary String streams retain their
selected CDR codec and reception timestamp. Configure an event source timestamp
explicitly with `stream_timestamp_fields={"events": "ts"}` and select the codec
with `stream_codecs={"events": "json"}`. Missing, nonnumeric or nonfinite selected
timestamps fail recording. Without a selected field, JSON uses reception time.

JSON storage is supported by SQLite. The ROS2-profile MCAP backend accepts only
CDR channels with complete message schemas; record a typed custom event message
there instead of selecting the JSON storage codec. JSON source times are converted
to signed nanoseconds with checked range and retained in SQLite.
