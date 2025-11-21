# Transport

## What is Transport?

Transport is the *abstraction layer for message passing* between modules in DimOS. It sits between the high-level stream interface (`In[T]` and `Out[T]`) and the low-level protocols (LCM, shared memory, Dask RPC), providing a uniform broadcast/subscribe interface regardless of the underlying protocol.

<!-- Citation: stream.py:76-84 - Transport abstract base class with broadcast/subscribe methods -->
<!-- Citation: transport.py:40-232 - All transport implementations (PubSubTransport base and 6 concrete types) -->

```python
from dimos.core import Module, In, Out
from dimos.msgs.sensor_msgs import Image

class SpatialMemory(Module):
    color_image: In[Image] = None  # Stream declaration

    def start(self):
        # Subscribe to stream - transport handles delivery
        self.color_image.subscribe(self.process_image)

    def process_image(self, img: Image):
        # This code is identical whether using:
        # - pSHMTransport (local shared memory)
        # - LCMTransport (network)
        # - JpegLcmTransport (compressed network)
        # - DaskTransport (distributed cluster)
        self.add_to_memory(img)
```

<!-- Citation: perception/spatial_perception.py:64 - Real SpatialMemory module with color_image: In[Image] declaration -->

> [!NOTE]
> The core takeaway: modules can operate on any underlying transport. The same module code deploys across local shared memory, network protocols, or distributed clusters without changes.

## Purpose

The Transport abstraction solves three challenges:

- **Protocol Lock-in**: Traditional systems tie modules to specific middleware. Transport decouples modules from protocols--the same code runs on any transport.
- **Performance Optimization**: Different data needs different protocols. Camera streams need zero-copy locally, JPEG over network. Transport lets you optimize per-stream without changing modules.
- **Deployment Flexibility**: Same stack runs on single machine (dev), robot+laptop (test), or cluster (prod).

<!-- Evidence: unitree_go2_blueprints.py:76-108 shows same modules using different transport types -->
<!-- Citation: transport.py:185-229 - DaskTransport for distributed deployment -->

## Where does Transport sit in DimOS's communication stack?

```ascii
Module (business logic)
    ↓ declares
Stream (In[T]/Out[T] typed endpoints)
    ↓ connected via
Transport (broadcast/subscribe abstraction)
    ↓ implemented by
Protocol (LCM, SharedMemory, Dask RPC)
```

Each layer only knows about its immediate neighbors. Modules declare Streams. Streams use their assigned Transport. Transports handle actual message delivery through Protocols. This separation lets each layer evolve independently.

<!-- Citation: stream.py:87-104 - Stream class holds Transport reference -->
<!-- Citation: stream.py:134-141 - Out[T] has transport property setter/getter -->

### Relationship to Other Concepts

*Modules* never interact with Transport directly. They declare `In[T]` and `Out[T]` streams, and those streams internally use their assigned Transport. This indirection enables modularity.

*Streams* are the direct consumers of Transport. Each stream holds a reference to its Transport instance. When you publish to a stream, it delegates to `transport.broadcast()`. When you subscribe, it delegates to `transport.subscribe()`.

*Blueprints* are where Transport selection happens. During `autoconnect()`, the system examines all stream connections, selects appropriate transports (or uses overrides), and assigns them to streams.

<!-- Citation: blueprints.py:262-302 - _connect_transports method groups by (remapped_name, type) -->
<!-- Citation: blueprints.py:213-236 - _get_transport_for selection logic: checks lcm_encode, creates topic -->

## Transport Types

DimOS provides a rich taxonomy of transports optimized for different scenarios:

<!-- Citation: transport.py:40-232 - All transport implementations -->

| Transport | Use Case | Network? | Performance | Trade-off |
|-----------|----------|----------|-------------|-----------|
| **LCMTransport** | Default for typed messages | ✓ | <1ms local, 1-10ms network | Requires LCM message definitions |
| **pLCMTransport** | Fallback for Python objects | ✓ | Medium | Pickle overhead, Python-only |
| **JpegLcmTransport** | Images over network | ✓ | 5-20x bandwidth reduction | Lossy compression, CPU overhead |
| **pSHMTransport** | High-frequency local data (>30Hz cameras, point clouds) | ✗ | Near-zero-copy, fastest | Local only |
| **JpegShmTransport** | Local images, multiple consumers | ✗ | Reduced memory | Compression CPU cost |
| **DaskTransport** | Distributed cluster | ✓ | RPC overhead | Requires manual configuration |

<!-- Citation: transport.py:54-75,78-100,103-109,112-133,136-157,160-182 - All PubSubTransport implementations -->
<!-- Citation: blueprints.py:232 - Checks for lcm_encode method -->
<!-- Citation: blueprints.py:234 - Selected when use_pickled is True -->
<!-- Example: unitree_go2_blueprints.py:79-81 - Used with DEFAULT_CAPACITY_COLOR_IMAGE -->
<!-- Example: unitree_go2_blueprints.py:91-94 - Used for network image transmission -->

### Class Hierarchy

The transports form this inheritance structure:

```
Transport
├── PubSubTransport (topic-based)
│   ├── LCMTransport → JpegLcmTransport
│   ├── pLCMTransport, SHMTransport, pSHMTransport
│   └── JpegShmTransport, ZenohTransport
└── DaskTransport (RPC-based)
```

> [!NOTE]
> JpegShmTransport extends PubSubTransport directly (not SHMTransport), so it doesn't share SHMTransport's buffer management. This is intentional--it uses its own JPEG-specific memory handling.

**Network-capable:** LCMTransport, pLCMTransport, JpegLcmTransport, ZenohTransport, DaskTransport
**Local-only:** SHMTransport, pSHMTransport, JpegShmTransport

## Using Transports

### Default Behavior

There are reasonable defaults; you don't need to think about transports if you don't want to:

```python
from dimos.core.blueprints import autoconnect
from dimos.robot.unitree_webrtc.unitree_go2 import connection
from dimos.perception.spatial_perception import spatial_memory
from dimos.navigation.bt_navigator.navigator import behavior_tree_navigator

blueprint = autoconnect(
    connection(),
    spatial_memory(),
    behavior_tree_navigator()
)
```

<!-- Example: unitree_go2_blueprints.py:46-56 - Basic blueprint with autoconnect -->

`autoconnect()` matches streams by name and type, then selects an appropriate transport automatically.
<!-- TODO: Link to the API reference / docstring for autoconnect for more details -->

<!-- Citation: blueprints.py:295 - Groups connections by (remapped_name, conn.type) key -->
<!-- Citation: blueprints.py:232-234 - Transport selection: checks lcm_encode, creates topic /{name} -->
<!-- Citation: blueprints.py:299-302 - Loop assigns same transport to all connections with matching key -->

### Custom Configuration

That said, you can override specific transports when you need to:

```python
from dimos.core.transport import pSHMTransport, JpegLcmTransport
from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.msgs.sensor_msgs import Image

# Optimize high-frequency camera stream
blueprint = autoconnect(
    connection(),
    spatial_memory(),
).transports({
    ("color_image", Image): pSHMTransport(
        "/go2/color_image",
        default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
})
```

<!-- Real example: unitree_go2_blueprints.py:76-83 - Standard with SHM variant -->

**Key observations:**

- Override uses `(stream_name, type)` as key
- Must specify exact topic name
- Can mix different transport types
- Last override wins if multiple applied

<!-- Citation: blueprints.py:228 - transport_map.get((name, type), None) -->
<!-- Citation: blueprints.py:88-113 - .transports() merges into transport_map, last wins -->

### Common Patterns

```python
# 1. Basic - LCM for external tool compatibility (Foxglove)
basic = autoconnect(...).transports({
    ("color_image", Image): LCMTransport("/go2/color_image", Image),
})

# 2. SHM - Shared memory for local performance
standard_with_shm = standard.transports({
    ("color_image", Image): pSHMTransport("/go2/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE),
})

# 3. JPEG-LCM - Compressed images over network
standard_with_jpeglcm = standard.transports({
    ("color_image", Image): JpegLcmTransport("/go2/color_image", Image),
})

# 4. JPEG-SHM - Compressed shared memory for multiple consumers
standard_with_jpegshm = standard.transports({
    ("color_image", Image): JpegShmTransport("/go2/color_image", quality=75),
})
```

<!-- Citation: unitree_go2_blueprints.py:58-66 - basic variant for Foxglove -->
<!-- Citation: unitree_go2_blueprints.py:76-89 - standard_with_shm variant -->
<!-- Citation: unitree_go2_blueprints.py:91-95 - standard_with_jpeglcm variant -->
<!-- Citation: unitree_go2_blueprints.py:97-108 - standard_with_jpegshm variant -->

## Design Principles

**Minimal Interface** - Just two methods: `broadcast()` and `subscribe()`.

<!-- Citation: stream.py:78,84 - Only two abstract methods in Transport -->

**Lazy Initialization** - All transports allocate resources only on first use:

<!-- Citation: transport.py:64-69 - pLCMTransport lazy start in broadcast -->
<!-- Same pattern: transport.py:89-94 (LCMTransport), 122-127 (pSHMTransport), 146-151 (SHMTransport), 171-176 (JpegShmTransport) -->

```python
# From transport.py:64-69
def broadcast(self, _, msg) -> None:
    if not self._started:
        self.lcm.start()
        self._started = True
    self.lcm.publish(self.topic, msg)
```

<!-- Citation: transport.py:64-67,89-92,122-125 - Lazy initialization pattern in all PubSubTransport implementations -->

**Shared Transport Instances** - Connections with same `(name, type)` share transport objects, reducing memory and connection overhead.

<!-- Citation: blueprints.py:298-302 - Loop assigns same transport instance to all connections in group -->

**Observable Integration** - Transport extends `ObservableMixin[T]`, providing reactive stream processing:

<!-- Citation: stream.py:40-66 - ObservableMixin methods -->
<!-- Citation: stream.py:65-66 - observable() applies backpressure by default -->

```python
# Throttle high-frequency sensor data
stream.observable().pipe(
    ops.sample(0.1),  # Sample every 0.1s (10Hz)
    ops.filter(lambda img: img.width > 640),  # Only HD images
    ops.buffer_with_time(1.0)  # Batch per second
).subscribe(process_batch)
```

<!-- Citation: stream.py:56-66 - ObservableMixin provides observable(), pure_observable(), get_next(), hot_latest() -->

## Performance Optimization

**Tuning options**: Buffer sizes (`pSHMTransport(topic, default_capacity=1000)`), compression quality (`JpegShmTransport(topic, quality=75)`), selective overrides (`.transports({("color_image", Image): fast_transport})`).

**When to customize**: For large data (>1MB), use pSHMTransport if local (near-zero-copy reads), JpegLcmTransport if network images, pLCMTransport otherwise. Small data uses default LCMTransport.

---

## Related Concepts

- **Modules** - See [Modules concept doc](./modules.md) for how modules use streams
- **Blueprints** - See [Blueprints concept doc](./blueprints.md) for autoconnect() and configuration
