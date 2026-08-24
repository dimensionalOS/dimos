# Experimental Native Memory Recorder

The Rust recorder is an experimental high-throughput alternative to the Python
Memory2 recorder. It remains compatible with the existing Python readers while
its API and operational behavior are evaluated. Experimental imports may change
without compatibility aliases.

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

Select MCAP to preserve original transport packets instead of applying Memory2
storage codecs:

```python
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig

mcap_recorder = SensorRecorder.blueprint(
    store=RustMcapStoreConfig(path="session.mcap"),
    encoding_threads=4,
)
```

MCAP stores original LCM packets in indexed Zstd chunks. Source time is the
MCAP publish time and recorder reception time is the log time. The stable
`McapStore(path="session.mcap")` reader discovers and decodes these channels
without a caller-supplied codec registry. Storage-codec overrides and append
mode are intentionally unsupported for MCAP.

Both backends preserve source timestamps for common stamped messages. Arbitrary
pickle payloads, Python `pose_setter_for` hooks, and spatial pose attachment
remain Python-recorder features; unsupported combinations fail during startup.
