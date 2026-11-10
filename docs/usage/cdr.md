# CDR messages

dimOS carries typed messages as [CDR](https://www.omg.org/spec/DDS-XTypes/)
payloads. A message type is a ROS 2 `.msg` definition compiled into a generated
Python class with CDR encode/decode support; the same definition is what the
recorder writes into MCAP and what Rerun, Foxglove and the browser decoders
read. This page is the support-level reference: what the API is, how to select
and use it, and what stays compatible with the pre-CDR workflows.

The generated classes and codecs are support. Application modules that still
use the retired `dimos.msgs` wrappers migrate separately, capability by
capability, on top of this support.

## Message packages and imports

Built-in definitions are committed under `packages/dimos-messages/interfaces`
and built into the `dimos_messages` package by the main `dimos` distribution.
Import message classes from there:

```python
from dimos_messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_messages.std_msgs.msg import Header
```

External projects can own their own definitions and register a generated
provider through the `dimos_messages` entry-point group; see
[External message projects](/docs/development/messages-external-project.md).
Generation, schema provenance and packaging are described in
[CDR generation in dimOS](/docs/development/message-source-packages.md).

## Encode and decode

`dimos_cdrgen.registry` is the process-wide registry. It resolves message
classes, encodes values as XCDR1 (little-endian by default) and returns the
ROS 2 definition for a type:

```python
from dimos_cdrgen.registry import decode, encode, schema
from dimos_messages.geometry_msgs.msg import Vector3

value = Vector3(x=1.0, y=2.0, z=3.0)
payload = encode(value)                 # XCDR1, little-endian
restored = decode(payload, Vector3)     # same generated class
assert restored == value

print(schema("geometry_msgs/msg/Vector3"))   # ROS 2 .msg definition
```

`decode` also accepts the type name as a string, which is how the transport,
recording and viewer layers decode without importing every message module.
`schema()` returns the definition including nested types, which is what makes
a recorded MCAP self-describing.

## Selecting a transport

Typed streams encode registered generated messages as CDR on every carrier.
The carrier is independent of the payload encoding:

```bash
dimos --transport=zenoh run <blueprint>   # default; CDR payloads
dimos --transport=lcm run <blueprint>     # opt-in UDP multicast carrier
```

`ZenohTransport` and `LCMTransport` both carry generated messages as CDR.
`pZenohTransport` / `pLCMTransport` are the explicit CBOR carriers for Python
values that have no generated form. The old statement "LCM means the old LCM
message format" no longer holds: choosing the carrier does not choose the
message format. See [Transports](/docs/usage/transports/index.md) and
[LCM transport and generated messages](/docs/usage/lcm.md).

Inside a module, declare the stream type and let the coordinator pick the
transport from the blueprint/defaults:

```python
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos_messages.geometry_msgs.msg import PoseStamped

class Publisher(Module):
    pose: Out[PoseStamped]
```

The explicit binding API is `Topic(name, message_type=...)`; it validates the
type eagerly. The historical positional `LCMTransport(topic, SomeType)` form
still binds lazily, exactly as before, so blueprints that pass a plain Python
class keep working until they migrate.

## Record

The normal CLI records selected typed streams through the Rust recorder. The
recorder is opaque to message contents: it stores the original CDR bytes plus
the ROS 2 schema in MCAP.

```bash
cargo build --locked \
  --manifest-path dimos/experimental/memory/rust/Cargo.toml \
  --target-dir build/cdr-recorder

dimos --transport zenoh --viewer none --n-workers 1 \
  --record mcap --record-engine rust --record-topics pose \
  --record-executable "$PWD/build/cdr-recorder/debug/dimos-memory-recorder" \
  run <blueprint>
```

Output lands under `recordings/<timestamp>-<run-id>-<blueprint>/memory.mcap`
and the shutdown log prints the full `artifact_path`. In the MCAP:

- channel encoding is `cdr`;
- the schema is the ROS 2 definition (for example
  `geometry_msgs/msg/PoseStamped`, `ros2msg`), including nested types;
- each payload is an XCDR1 encapsulation (`00 01 00 00` for little-endian);
- observation timestamps use reception time; the original `header.stamp`
  stays inside the payload.

Read it back with Python:

```python
from dimos.memory.store.cdr_mcap import CdrMcapStore

with CdrMcapStore(path="recordings/.../memory.mcap") as store:
    print(store.list_streams())
    for observation in store.stream("pose"):
        print(observation.ts, observation.data)
```

Legacy recordings written with the old LCM codecs are still readable through
the legacy `McapStore` path; the CDR store is additive and selected by the
recording file's channel encoding, not by a global switch.

The runnable example is
[`examples/cdr-recording`](/examples/cdr-recording/README.md): a 10 Hz
`PoseStamped` circle over Zenoh, the CLI Rust recorder, Foxglove inspection
and a Python reopen.

## Replay

Replay republishes every recorded stream on the bus using the recorded schema
and type names:

```bash
dimos --replay-db recordings/.../memory.mcap run replay
```

A subscriber on the same transport receives the original generated values
(header stamps preserved), and the viewer path renders them like a live run.
Use an isolated Zenoh domain when replaying next to other processes:

```bash
dimos --zenoh-domain cdr-replay --replay-db <memory.mcap> run replay
```

## View

Two support entrypoints render generated messages:

- **Rerun bridge** — `dimos/visualization/rerun` keeps all legacy renderers and
  adds typed adapters for generated messages. The blueprint opts in explicitly
  with `Config.cdr_transports` / `Config.message_types`; the old `LCM()`
  default still resolves to the transport default, so defaults do not change.
- **Cockpit / web SDK** — the bridge publishes `cdr.v1` channels whose manifest
  embeds the ROS 2 schema, and `web/sdk` decodes CDR in the browser (both byte
  orders). No extra message package is needed in the page.

Runnable, no-hardware examples:

```bash
# live Rerun bridge + native CDR MCAP, exit with Ctrl-C
python -I examples/message-viewer/demo_cdr_rerun.py /tmp/cdr-demo.rrd

# cockpit relay + browser page
python -I examples/message-viewer/demo_cdr_browser.py --port 8899
cd web/sdk && deno task fixture --host 127.0.0.1
# then open the printed http://127.0.0.1:5174/demo_cdr.html?live=1&relay=...
```

See [Bridge](/docs/web/bridge.md), [Cockpit](/docs/web/cockpit.md) and
[Web SDK](/docs/web/web_sdk.md).

## Compatibility and defaults

Support is additive. Landing it does not change existing runtime behaviour:

- existing blueprints keep their current transport default and their current
  message types; nothing is rewritten to generated values at this layer;
- the positional `LCMTransport(topic, legacy_type)` binding keeps its lazy
  semantics (resolution happens on first encode/decode, not at construction);
- legacy LCM-encoded recordings remain readable;
- the viewer bridge keeps every legacy renderer; the typed path is explicit
  opt-in until the viewer consumers migrate.

## Migration recipe

Migrating a consumer means, in its own change:

1. Replace `dimos.msgs` imports with the corresponding `dimos_messages`
   classes and use generated values end to end (do not alias or re-export the
   retired namespace).
2. Pass generated values through `Out[T]` / `In[T]`; encode/decode through
   `dimos_cdrgen.registry` only where a boundary needs raw bytes.
3. For a plain Python value at a transport boundary, use
   `pZenohTransport` / `pLCMTransport` explicitly.
4. Keep the old recording read path if the capability still needs to replay
   historical LCM recordings.
5. Prove the migrated behaviour with the capability's own tests and the
   applicable transport/recording check; a static import grep is not enough.

Capability migrations are independent of each other once this support layer is
in place; they depend on support, not on one another.
