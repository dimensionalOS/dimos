# Transport

## Motivation

Your robot modules might run on a laptop during development, then be split across a GPU server and edge device in production.
Transport abstracts this away: your module code stays the same, only the transport configuration changes.

<!-- Citation: stream.py:76-84 - Transport abstract base class with broadcast/subscribe methods -->
<!-- Citation: transport.py:40-232 - All transport implementations (PubSubTransport base and 6 concrete types) -->

```python
from dimos.core import Module, In, Out
from dimos.msgs.sensor_msgs import Image

class SpatialMemory(Module):
    color_image: In[Image] = None

    def start(self):
        self.color_image.subscribe(self.process_image)

    def process_image(self, img: Image):
        # This code is identical whether using:
        # - pSHMTransport (local shared memory)
        # - LCMTransport (network)
        # - JpegLcmTransport (compressed network)
        self.add_to_memory(img)
```

<!-- Citation: perception/spatial_perception.py:64 - Real SpatialMemory module with color_image: In[Image] declaration -->

> [!NOTE]
> The core takeaway: modules can operate on any underlying transport. The same module code deploys across local shared memory, network protocols, or distributed clusters by changing just the transport configuration.

## How does the Transport abstraction relate to other DimOS concepts?

We've seen, at a high level, how Transport is the abstraction layer for message passing between modules in DimOS. But let's make this more concrete. Recall that when you define a Module, you can declare what sorts of data it consumes and what sorts of data it produces. In particular, you can declare what *streams* the Module has: `In[T]` for input and `Out[T]` for output, where `T` is the type variable for the data carried by the stream.

```python
class NavigationSkillContainer(SkillModule):
    color_image: In[Image] = None
    odom: In[PoseStamped] = None
    # ...
```

This, however, only specifies the informational needs and outputs of the module; it doesn't yet specify *how* such data is to be transported between modules. To put it another way, what concrete transport backends should be used for these streams?

It is here that Transport enters the picture. There is, in the library, a fundamental `Transport` abstraction with methods like `publish`, `subscribe`, and `broadcast`, along with concrete Transport classes that realize that abstraction (more on this shortly). And as we'll see shortly, it's easy to change the choice of concrete Transports in the configuration.
 <!--TODO: Add link to API reference for Transport class  -->

## The default transports

When you call `autoconnect()`, it automatically selects sensible transports for each stream.
<!-- Citation: stream.py:87-104 - Stream class holds Transport reference -->

- If the stream's type can serialize to an LCM (Lightweight Communications and Marshalling) type (if it has `lcm_encode` support), then **`LCMTransport`**, a publish-subscribe messaging system that uses UDP multicast, is selected.
- Otherwise, it's **`pLCMTransport`**: this is basically LCMTransport, but with *pickle-based* serialization.

```python
from dimos.core.blueprints import autoconnect

blueprint = autoconnect(
    connection(),
    spatial_memory(),
    behavior_tree_navigator()
)
# Transport selection happens automatically
```

<!-- Example: unitree_go2_blueprints.py:46-56 - Basic blueprint with autoconnect -->

## When to customize

When should you override the Transport defaults?

Here's a quick decision guide:

```ascii
Is performance acceptable?
├── Yes → Keep defaults, you're done
└── No → Can processes share memory? (same host, not containerized separately)
    ├── Yes → What kind of data?
    │   ├── Camera frames, point clouds → pSHMTransport (memory-mapped, zero-copy)
    │   ├── Images + memory constrained → JpegShmTransport
    │   └── Other data → pSHMTransport
    └── No (network or isolated containers) → What kind of data?
        ├── Images → JpegLcmTransport
        ├── Messages that can serialize to an LCM type → LCMTransport
        └── Python objects without lcm_encode → pLCMTransport
```

<!-- Evidence for this decision tree:
- pSHMTransport for "large payloads like camera frames or point clouds" (transport.py:179-190 docstring)
- JpegShmTransport to "reduce memory footprint" (transport.py:266-277 docstring)
- JpegLcmTransport "reduces bandwidth when transmitting images across network boundaries" (transport.py:156-168)
- Default selection: lcm_encode → LCMTransport, else pLCMTransport (blueprints.py:232-234)
-->

## How to choose a specific transport

Override specific transports using the `.transports()` method:

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

**Key points:**

- The override key is the tuple `(stream_name, type)`
- The topic name must be specified (e.g., `"/go2/color_image"`)
- Can mix different transport types in one blueprint
- Last override wins if multiple applied

<!-- Citation: blueprints.py:228 - transport_map.get((name, type), None) -->
<!-- Citation: blueprints.py:88-113 - .transports() merges into transport_map, last wins -->

### Common patterns

```python
# 1. Shared memory for local performance
standard_with_shm = standard.transports({
    ("color_image", Image): pSHMTransport(
        "/go2/color_image",
        default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
})

# 2. Compressed images over network
standard_with_jpeglcm = standard.transports({
    ("color_image", Image): JpegLcmTransport("/go2/color_image", Image),
})

# 3. LCM for external tool compatibility (Foxglove)
basic = autoconnect(...).transports({
    ("color_image", Image): LCMTransport("/go2/color_image", Image),
})

# 4. Compressed shared memory for multiple local consumers
standard_with_jpegshm = standard.transports({
    ("color_image", Image): JpegShmTransport("/go2/color_image", quality=75),
})
```

<!-- Citation: unitree_go2_blueprints.py:58-66 - basic variant for Foxglove -->
<!-- Citation: unitree_go2_blueprints.py:76-89 - standard_with_shm variant -->
<!-- Citation: unitree_go2_blueprints.py:91-95 - standard_with_jpeglcm variant -->
<!-- Citation: unitree_go2_blueprints.py:97-108 - standard_with_jpegshm variant -->

### Class hierarchy

For reference, the transports form this inheritance structure:

```ascii
Transport
└── PubSubTransport (topic-based)
    ├── LCMTransport → JpegLcmTransport
    ├── pLCMTransport, SHMTransport, pSHMTransport
    └── JpegShmTransport, ZenohTransport
```

**Network-capable:** LCMTransport, pLCMTransport, JpegLcmTransport, ZenohTransport
**Local-only:** SHMTransport, pSHMTransport, JpegShmTransport

> [!NOTE]
> JpegShmTransport extends PubSubTransport directly (not SHMTransport), so it doesn't share SHMTransport's buffer management. This is intentional—it uses its own JPEG-specific memory handling.

---

## For advanced users

### Design principles

**Minimal Interface** — The base Transport class just has three mthods: `broadcast()`, `publish()`, `subscribe()`.
<!-- TODO: Link to the Transport class API doc -->

**Lazy Initialization** — All transports allocate resources only on first use:

```python
# From transport.py:64-69
def broadcast(self, _, msg) -> None:
    if not self._started:
        self.lcm.start()
        self._started = True
    self.lcm.publish(self.topic, msg)
```

<!-- Citation: transport.py:64-67,89-92,122-125 - Lazy initialization pattern in all PubSubTransport implementations -->

**Shared Transport Instances** — Connections with the same `(name, type)` share transport objects, reducing memory and connection overhead.

<!-- Citation: blueprints.py:298-302 - Loop assigns same transport instance to all connections in group -->

### Backpressure handling

Transport provides automatic backpressure through its Observable integration. When subscribers can't keep up with producers (e.g., slow image processing), intermediate values are dropped to prevent unbounded memory growth while ensuring subscribers always see the latest data.

<!-- Citation: stream.py:65-66 - observable() applies backpressure by default -->

You can also use RxPY operators for fine-grained control:

```python
# Throttle high-frequency sensor data
stream.observable().pipe(
    ops.sample(0.1),  # Sample every 0.1s (10Hz)
    ops.filter(lambda img: img.width > 640),  # Only HD images
    ops.buffer_with_time(1.0)  # Batch per second
).subscribe(process_batch)
```

<!-- Citation: stream.py:56-66 - ObservableMixin provides observable(), pure_observable(), get_next(), hot_latest() -->

### Performance tuning

Tuning options:

- **Buffer size** — `pSHMTransport("/cam", default_capacity=6_220_800)`
  Max payload in bytes. Use `DEFAULT_CAPACITY_COLOR_IMAGE` constant for 1080p RGB.

- **JPEG quality** — `JpegShmTransport("/cam", quality=75)`
  Range 1-100. Lower = smaller size, more artifacts.

---

## See also

- [Modules](./modules.md) — How modules declare and use streams
- [Blueprints](./blueprints.md) — How `autoconnect()` wires streams and selects transports
