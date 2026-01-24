
# Dimos Transports

Transports enable communication between [modules](modules.md) across process boundaries and networks. When modules run in different processes or on different machines, they need a transport layer to exchange messages.

While the interface is called "PubSub", transports aren't limited to traditional pub-sub services. A topic can be anything that identifies a communication channel: an IP address and port, a shared memory segment name, a file path, or a Redis channel. The abstraction is flexible enough to support any communication pattern that can publish and subscribe to named channels.

## The PubSub Interface

At the core of all transports is the `PubSub` abstract class. Any transport implementation must provide two methods:

```python session=pubsub_demo ansi=false
from dimos.protocol.pubsub.spec import PubSub

# The interface every transport must implement:
import inspect
print(inspect.getsource(PubSub.publish))
print(inspect.getsource(PubSub.subscribe))
```

```
@abstractmethod
def publish(self, topic: TopicT, message: MsgT) -> None:
    """Publish a message to a topic."""
    ...
@abstractmethod
def subscribe(
    self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
) -> Callable[[], None]:
    """Subscribe to a topic with a callback. returns unsubscribe function"""
    ...
```

Key points:
- `publish(topic, message)` - Send a message to all subscribers on a topic
- `subscribe(topic, callback)` - Register a callback, returns an unsubscribe function

## Implementing a Simple Transport

The simplest transport is `Memory`, which works within a single process:

```python session=memory_demo ansi=false
from dimos.protocol.pubsub.memory import Memory

# Create a memory transport
bus = Memory()

# Track received messages
received = []

# Subscribe to a topic
unsubscribe = bus.subscribe("sensor/data", lambda msg, topic: received.append(msg))

# Publish messages
bus.publish("sensor/data", {"temperature": 22.5})
bus.publish("sensor/data", {"temperature": 23.0})

print(f"Received {len(received)} messages:")
for msg in received:
    print(f"  {msg}")

# Unsubscribe when done
unsubscribe()
```

<!--Result:-->
```
Received 2 messages:
  {'temperature': 22.5}
  {'temperature': 23.0}
```

The full implementation is minimal. See [`memory.py`](/dimos/protocol/pubsub/memory.py) for the complete source.

## Available Transports

Dimos includes several transport implementations:

| Transport | Use Case | Process Boundary | Network |
|-----------|----------|------------------|---------|
| `Memory` | Testing, single process | No | No |
| `SharedMemory` | Multi-process on same machine | Yes | No |
| `LCM` | Network communication (UDP multicast) | Yes | Yes |
| `Redis` | Network communication via Redis server | Yes | Yes |

### SharedMemory Transport

For inter-process communication on the same machine, `SharedMemory` provides high-performance message passing:

```python session=shm_demo ansi=false
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory

shm = PickleSharedMemory(prefer="cpu")
shm.start()

received = []
shm.subscribe("test/topic", lambda msg, topic: received.append(msg))
shm.publish("test/topic", {"data": [1, 2, 3]})

import time
time.sleep(0.1)  # Allow message to propagate

print(f"Received: {received}")
shm.stop()
```

<!--Result:-->
```
Received: [{'data': [1, 2, 3]}]
```

### LCM Transport

For network communication, LCM uses UDP multicast and supports typed messages:

```python session=lcm_demo ansi=false
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.msgs.geometry_msgs import Vector3

lcm = LCM(autoconf=True)
lcm.start()

received = []
topic = Topic(topic="/robot/velocity", lcm_type=Vector3)

lcm.subscribe(topic, lambda msg, t: received.append(msg))
lcm.publish(topic, Vector3(1.0, 0.0, 0.5))

import time
time.sleep(0.1)

print(f"Received velocity: x={received[0].x}, y={received[0].y}, z={received[0].z}")
lcm.stop()
```

<!--Result:-->
```
Received velocity: x=1.0, y=0.0, z=0.5
```

### Inspecting LCM traffic (CLI)

- `dimos lcmspy` shows topic frequency/bandwidth stats.
- `dimos topic echo /topic` listens on typed channels like `/topic#pkg.Msg` and decodes automatically.
- `dimos topic echo /topic TypeName` is the explicit legacy form.

## Encoder Mixins

Transports can use encoder mixins to serialize messages. The `PubSubEncoderMixin` pattern wraps publish/subscribe to encode/decode automatically:

```python session=encoder_demo ansi=false
from dimos.protocol.pubsub.spec import PubSubEncoderMixin, PickleEncoderMixin

# PickleEncoderMixin provides:
# - encode(msg, topic) -> bytes  (uses pickle.dumps)
# - decode(bytes, topic) -> msg  (uses pickle.loads)

# Create a transport with pickle encoding by mixing in:
from dimos.protocol.pubsub.memory import Memory

class PickleMemory(PickleEncoderMixin, Memory):
    pass

bus = PickleMemory()
received = []
bus.subscribe("data", lambda msg, t: received.append(msg))
bus.publish("data", {"complex": [1, 2, 3], "nested": {"key": "value"}})

print(f"Received: {received[0]}")
```

<!--Result:-->
```
Received: {'complex': [1, 2, 3], 'nested': {'key': 'value'}}
```

## Using Transports with Modules

Modules use the `Transport` wrapper class which adapts `PubSub` to the stream interface. You can set a transport on any module stream:

```python session=module_transport ansi=false
from dimos.core.transport import pLCMTransport, pSHMTransport

# Transport wrappers for module streams:
# - pLCMTransport: Pickle-encoded LCM
# - LCMTransport: Native LCM encoding
# - pSHMTransport: Pickle-encoded SharedMemory
# - SHMTransport: Native SharedMemory
# - JpegShmTransport: JPEG-compressed images via SharedMemory
# - JpegLcmTransport: JPEG-compressed images via LCM

# Example: Set a transport on a module output
# camera.set_transport("color_image", pSHMTransport("camera/color"))
print("Available transport wrappers in dimos.core.transport:")
from dimos.core import transport
print([name for name in dir(transport) if "Transport" in name])
```

<!--Result:-->
```
Available transport wrappers in dimos.core.transport:
['JpegLcmTransport', 'JpegShmTransport', 'LCMTransport', 'PubSubTransport', 'SHMTransport', 'ZenohTransport', 'pLCMTransport', 'pSHMTransport']
```

## Testing Custom Transports

The test suite in [`pubsub/test_spec.py`](/dimos/protocol/pubsub/test_spec.py) uses pytest parametrization to run the same tests against all transport implementations. To add your custom transport to the test grid:

```python session=test_grid ansi=false
# The test grid pattern from test_spec.py:
test_pattern = """
from contextlib import contextmanager

@contextmanager
def my_transport_context():
    transport = MyCustomTransport()
    transport.start()
    yield transport
    transport.stop()

# Add to testdata list:
testdata.append(
    (my_transport_context, "my_topic", ["value1", "value2", "value3"])
)
"""
print(test_pattern)
```

<!--Result:-->
```

from contextlib import contextmanager

@contextmanager
def my_transport_context():
    transport = MyCustomTransport()
    transport.start()
    yield transport
    transport.stop()

# Add to testdata list:
testdata.append(
    (my_transport_context, "my_topic", ["value1", "value2", "value3"])
)

```

The test suite validates:
- Basic publish/subscribe
- Multiple subscribers receiving the same message
- Unsubscribe functionality
- Multiple messages in order
- Async iteration
- High-volume message handling (10,000 messages)

Run the tests with:
```bash
pytest dimos/protocol/pubsub/test_spec.py -v
```

## Creating a Custom Transport

To implement a new transport:

1. **Subclass `PubSub`** and implement `publish()` and `subscribe()`
2. **Add encoding** if needed via `PubSubEncoderMixin`
3. **Create a `Transport` wrapper** by subclassing `PubSubTransport`
4. **Add to the test grid** in `test_spec.py`

Here's a minimal template:

```python session=custom_transport ansi=false
template = '''
from dimos.protocol.pubsub.spec import PubSub, PickleEncoderMixin
from dimos.core.transport import PubSubTransport

class MyPubSub(PubSub[str, bytes]):
    """Custom pub/sub implementation."""

    def __init__(self):
        self._subscribers = {}

    def start(self):
        # Initialize connection/resources
        pass

    def stop(self):
        # Cleanup
        pass

    def publish(self, topic: str, message: bytes) -> None:
        # Send message to all subscribers on topic
        for cb in self._subscribers.get(topic, []):
            cb(message, topic)

    def subscribe(self, topic, callback):
        # Register callback, return unsubscribe function
        if topic not in self._subscribers:
            self._subscribers[topic] = []
        self._subscribers[topic].append(callback)

        def unsubscribe():
            self._subscribers[topic].remove(callback)
        return unsubscribe


# With pickle encoding
class MyPicklePubSub(PickleEncoderMixin, MyPubSub):
    pass


# Transport wrapper for use with modules
class MyTransport(PubSubTransport):
    def __init__(self, topic: str):
        super().__init__(topic)
        self.pubsub = MyPicklePubSub()

    def broadcast(self, _, msg):
        self.pubsub.publish(self.topic, msg)

    def subscribe(self, callback, selfstream=None):
        return self.pubsub.subscribe(self.topic, lambda msg, t: callback(msg))
'''
print(template)
```

<!--Result:-->
```

from dimos.protocol.pubsub.spec import PubSub, PickleEncoderMixin
from dimos.core.transport import PubSubTransport

class MyPubSub(PubSub[str, bytes]):
    """Custom pub/sub implementation."""

    def __init__(self):
        self._subscribers = {}

    def start(self):
        # Initialize connection/resources
        pass

    def stop(self):
        # Cleanup
        pass

    def publish(self, topic: str, message: bytes) -> None:
        # Send message to all subscribers on topic
        for cb in self._subscribers.get(topic, []):
            cb(message, topic)

    def subscribe(self, topic, callback):
        # Register callback, return unsubscribe function
        if topic not in self._subscribers:
            self._subscribers[topic] = []
        self._subscribers[topic].append(callback)

        def unsubscribe():
            self._subscribers[topic].remove(callback)
        return unsubscribe


# With pickle encoding
class MyPicklePubSub(PickleEncoderMixin, MyPubSub):
    pass


# Transport wrapper for use with modules
class MyTransport(PubSubTransport):
    def __init__(self, topic: str):
        super().__init__(topic)
        self.pubsub = MyPicklePubSub()

    def broadcast(self, _, msg):
        self.pubsub.publish(self.topic, msg)

    def subscribe(self, callback, selfstream=None):
        return self.pubsub.subscribe(self.topic, lambda msg, t: callback(msg))

```
