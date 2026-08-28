# Experimental Native Memory Recorder

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

## OpenYAM collection profile

The experimental imitation-learning profile uses this recorder without adding
Python-side recording behavior:

```bash
dimos --transport zenoh --can-port can0 \
  run learning-collect-quest-openyam-native \
  --task "pick up the red block"
```

It records the USB wrist camera, aggregate coordinator joint state,
hardware-accepted joint position command, and typed episode status over
reliable Zenoh. SQLite is the default; select MCAP for a run with:

```bash
  --nativecollectionrecorder.store.kind mcap \
  --nativecollectionrecorder.store.path /path/to/session_openyam.mcap
```

Both artifacts feed `dimos dataprep` directly. The stable
`learning-collect-quest-openyam` command continues to use the Python recorder
and pSHM.
