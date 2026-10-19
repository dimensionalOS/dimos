# Record and replay generated messages with mem2

Generated values use the CDR codec in the existing SQLite memory store. Legacy
codecs remain available for their existing types. This layer does not change the
default transport or convert historical recordings.

```python
from dimos.memory.store.sqlite import SqliteStore
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegments3D
from dimos.messages.std_msgs.msg import Header

message = LineSegments3D(Header(Time(17, 250_000_000), "map"), [])
with SqliteStore(path="memory.db") as store:
    store.stream("segments", LineSegments3D).append(message, ts=17.25)

with SqliteStore(path="memory.db", must_exist=True) as store:
    assert list(store.replay().streams.segments.iterate_ts()) == [(17.25, message)]
```

The existing recording tap accepts generated types and reads their
`header.stamp`, including zero. Unstamped values use reception time.

MCAP support additionally requires `pip install "mcap>=1.2"`. Write CDR bytes
with the complete installed schema:

```python
from dimos_cdrgen.registry import encode, schema
from dimos.protocol.cdr_mcap import CdrMcapWriter

with CdrMcapWriter("recording.mcap") as writer:
    writer.write(
        "segments", encode(message), schema_name=message.__msgtype__,
        schema=schema(message.__msgtype__),
        log_time_ns=20_000_000_000, publish_time_ns=17_250_000_000,
    )
```

`CdrMcapWriter` writes the requested file and replaces an existing file at that
path. Choose the path deliberately. The runnable demo below instead requires a
new directory and refuses to overwrite earlier output.

MCAP stores use log time by default. To use source publish time as the mem2
observation time, register the stream before writing with
`metadata={"dimos.observation_time": "publish_time"}`. The file always retains
both timestamps. Schemas use `ros2msg`, including imported definitions; payloads
use `cdr`. `McapStore` resolves matching installed types by schema name and
rejects known schema mismatches or conflicting channel contracts. Unknown types
remain raw bytes. An explicit injected codec takes precedence. This is not an
implicit legacy-recording decoder or an arbitrary-schema class generator.

## Run the complete example

From this PR's checkout after installing it in an active Python environment:

```sh
python examples/message-recording/demo_cdr_mem2.py /tmp/my-cdr-recording
```

The example records a nonempty `LineSegments3D` in SQLite, closes and reopens
it, replays it through mem2, exports MCAP with a complete schema closure, and
replays that MCAP through `McapStore`. It checks the generated type, nested
values and source timestamp. No robot, broker or viewer is needed. The two
files remain under the new directory for inspection.

The main test suite runs the exact example. Focused verification is:

```sh
python -m pytest --import-mode=importlib dimos/memory/store/test_cdr_recording.py
```
