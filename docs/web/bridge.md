# Bridge

The bridge is `RelayBridgeModule`, a dimOS module that runs inside the robot process like any other module. It connects to the relay, tells it which channels the robot offers, encodes each stream into browser-ready frames while at least one viewer is subscribed, and turns browser input (teleop commands, published values) back into stream publishes. A channel is encoded only while somebody watches it. `cockpit()` is how you configure it: panels and `Channel` declarations become the manifest and the module's typed ports.

## Adding it to a blueprint

There are two ways. A flag on the command line appends a bridge with the default cockpit to any blueprint:

```bash
uv run dimos --replay run unitree-go2 --local-relay
uv run dimos --replay run unitree-go2 --relay-url http://localhost:7780
```

Or the blueprint composes `cockpit(...)` itself, as the Go2 cockpit blueprints do. Then the bridge is part of the blueprint with exactly the panels and channels declared, and the flags only choose the relay. Without `--relay-url` such a blueprint starts a local relay by itself.

The default cockpit is built from the streams the blueprint has: a video panel when a module publishes `color_image`, a map when one publishes `global_costmap` (with the pose marker when `odom` exists too), and teleop when a module consumes `tele_cmd_vel`.

Both ways need the `web` extra (`uv sync --extra web --inexact` in a checkout, `dimos[web]` from pip).

## Robot-side options

The relay flags are `dimos run` options. Most of the rest are fields of the bridge module's config, so they are set like any blueprint config field, after the blueprint name: `--serve-dir DIR`. The environment form is `<INSTANCE>__SERVE_DIR`, where the instance name is `relaybridgemodule` for the default bridge but a generated name for a blueprint with `cockpit(...)`, so use the flag.

| Option | Default | Meaning |
|---|---|---|
| `--local-relay` | off | Spawn a relay on this machine and connect to it. |
| `--relay-url URL` | none | Connect to a relay started elsewhere, given its HTTP address (`http://localhost:7780`, `https://relay.example.com`). The bridge fetches `/api/info` there on every connect, so a relay restart (new port, new certificate) is transparent. |
| `--relay-ca PEM` | none | CA bundle that signed the relay's certificate (mkcert, a private CA). Replaces the default trust store for that relay. |
| `RELAY_KEY` | none | The robot's key for a relay with an auth file, bound to the robot id there. Set it in the environment or in `.env`. The `--relay-key` flag exists too, but it shows in the process list. |
| `--robot-id ID` | the hostname | How the relay and the cockpit name this robot. Two robots on one relay need different ids. A global option: `dimos --robot-id go2-lab run ...`. |
| `--robot-model NAME` | none | Shown next to the name in the cockpit. Also a global option. |
| `--robot-name NAME` | the id | Display name in the cockpit. |
| `--serve-dir DIR` | none | Serve this directory at `/` instead of the cockpit. Local relay only. |
| `--local-port N` | 7780 | HTTP port of the local relay. |
| `--open-browser false` | opens | Do not open the page once the local relay is up. |
| `--no-web-build` | builds | Do not rebuild the cockpit and SDK bundles when they are stale (checkouts only, wheels ship them prebuilt). |
| `--jpeg-quality`, `--image-max-hz`, `--odom-max-hz`, `--costmap-max-hz` | 75, 30, 20, 5 | Quality and rates of the built-in channels in the default cockpit. A `cockpit(...)` layout sets them per panel instead. |

## Channels

A channel is one stream crossing the wire in one direction with one encoding. `rx` channels flow from the robot to the browser, `tx` channels from the browser to the robot. The bridge has four built-in ports:

| Channel | Type | Encoding | Delivery |
|---|---|---|---|
| `color_image` | `Image` | `jpeg.v1` | latest |
| `odom` | `PoseStamped` | `pose.json.v1` | reliable |
| `global_costmap` | `OccupancyGrid` | `costmap.zlib.v1` | latest, the last grid is replayed to a new viewer |
| `tele_cmd_vel` (tx) | `Twist` | `twist.json.v1` | latest |

Every other stream needs a `Channel` declaration. It becomes a port typed with the declared message type and wired by `autoconnect` like any module port. The Map2D, Map3D, Chat and Stats panels declare the channels for their own extra streams (`path`, `click`, `stop`, `cloud`, the chat streams, `resource_stats`). A Video panel, or a Map2D costmap or pose, on a stream that is not a built-in port needs a `Channel` from you, see [Panels](/docs/web/cockpit.md#panels) on the cockpit page.

The relay keeps the bridge informed of which channels have at least one subscribed viewer. The bridge encodes and sends a channel only while that holds: it subscribes to the stream when the first viewer arrives and unsubscribes when the last one leaves. Replayable channels (`resend_on_subscribe`, and the built-in `global_costmap`) also keep a raw-message cache, so they stay subscribed to their stream without viewers, but nothing is encoded for them. Two settings shape what crosses the wire:

- `delivery`: `reliable` channels arrive in order and complete (the relay queues per viewer and drops a viewer that cannot keep up). `latest` channels keep only the newest frame and shed stale ones on the way. Video and maps are `latest`.
- `max_hz`: an rx channel is sampled at this rate, so messages that arrive faster are skipped. With `paced=True` the bridge spaces sends instead: every message goes out, one per interval, in order, which suits event streams like chat where a burst must arrive complete. `resend_on_subscribe=True` replays the last message to the first viewer that subscribes, which suits state streams that publish rarely (a map, a path). A second viewer on a channel that is already flowing waits for the next publish.

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/dataflow.svg
color = white
fill = none
boxrad = 5px
margin = 0.06in

M: box "robot modules" "camera, mapper, ..." wid 1.3in ht 0.95in
B: box "Bridge" "encodes subscribed" "channels" wid 1.3in ht 0.95in with .w at M.e + (0.75in, 0)
R: box "Relay" "copies frames to" "each viewer" wid 1.3in ht 0.95in with .w at B.e + (0.75in, 0)
V: box "viewers" "cockpit, SDK pages" wid 1.3in ht 0.95in with .w at R.e + (0.75in, 0)

arrow from M.e + (0, 0.3in) to B.w + (0, 0.3in) "streams" above
arrow from B.e + (0, 0.3in) to R.w + (0, 0.3in) "frames" above
arrow from R.e + (0, 0.3in) to V.w + (0, 0.3in) "frames" above

arrow from V.w to R.e "sub, unsub" above
arrow from R.w to B.e "wanted" above

arrow from V.w - (0, 0.3in) to R.e - (0, 0.3in) "input" below
arrow from R.w - (0, 0.3in) to B.e - (0, 0.3in) "input" below
arrow from B.w - (0, 0.3in) to M.e - (0, 0.3in) "publishes" below
```

</details>

![Three paths between robot modules, bridge, relay and viewers: streams become frames that the bridge encodes and the relay copies to each viewer, subscription changes travel from the viewers through the relay to the bridge, and browser input travels through the relay to the bridge, which publishes it on the modules' streams](assets/dataflow.svg)

### Exposing a stream with Channel

`Channel(stream, message_type, *, dir="rx", encoding=None, delivery="reliable", max_hz=10.0, params=None, publish="none", required_scope=None, paced=False, resend_on_subscribe=False)`

| Parameter | Meaning |
|---|---|
| `stream` | The stream name, matched by `autoconnect`. At most 64 characters, and it must not start with `@`. |
| `message_type` | The stream's Python type. It becomes the generated port's type. |
| `dir` | `rx` (robot to browser) or `tx` (browser to robot). |
| `encoding` | A codec id. `None` picks the default (see [Encodings](#encodings)). |
| `delivery` | `reliable` or `latest`. |
| `max_hz` | The rate cap. For a tx channel it is the publish rate the relay accepts, per viewer and per robot. |
| `params` | Extra JSON for the encoder and the browser, shipped in the manifest. `params["lcm"]` is reserved for the LCM schema. |
| `publish` | `none` (the default) or `shared`. A tx channel must say `shared`: any viewer may publish on it. `exclusive` is not implemented. |
| `required_scope` | Carried in the manifest for a relay with scopes. Nothing checks it today. |
| `paced` | rx only. Space sends instead of sampling. |
| `resend_on_subscribe` | rx only. Replay the last message to the first viewer. |

A `Channel` for a stream that a panel also uses merges with the panel's request: direction, encoding, delivery and params must agree, and the larger `max_hz` wins. Conflicts raise when the blueprint is defined. `cockpit(channels=[...])` with no layout compiles to a manifest with no panels, which is the pure [web SDK](/docs/web/web_sdk.md) setup.

## Encodings

An encoding id names a codec pair: the encoder in the bridge and the decoder in the browser (the reverse for tx). The built-in ids:

| Encoding | Type | Direction | Used by |
|---|---|---|---|
| `jpeg.v1` | `Image` | rx | the Video panel |
| `costmap.zlib.v1` | `OccupancyGrid` | rx | Map2D |
| `voxels.zlib.v1` | `PointCloud2` | rx | Map3D |
| `pose.json.v1` | `PoseStamped` | rx | the Map2D pose marker |
| `path.json.v1` | `Path` | rx | the Map2D path overlay |
| `stats.json.v1` | `dict` | rx | Stats |
| `chat.json.v1` | LangChain `BaseMessage` | rx | Chat |
| `text.json.v1` | `str` | tx | the Chat input |
| `point.json.v1` | `PointStamped` | tx | Map2D clicks |
| `bool.json.v1` | `Bool` | tx | the Map2D cancel button |
| `audio.json.v1` | `AudioChunk` | tx | the Chat microphone |
| `twist.json.v1` | `Twist` | tx | Teleop (its own protocol path, not a codec) |

Two more need no registration:

- `json.v1`: JSON scalars, lists, dicts and plain dataclasses on rx. On tx, scalars, lists and dicts only (a dataclass built from untrusted browser JSON needs an explicit decoder).
- `<package>.<Message>.lcm.v1`, for example `geometry_msgs.PoseStamped.lcm.v1`: any dimOS message with an LCM schema, rx only. The frame is the message's `lcm_encode()` bytes and the manifest carries the schema, so the browser decodes it into a plain object with no registration. A bulk message costs its full size per frame (a `PointCloud2` is 16 bytes per point), so set `max_hz` accordingly or write an encoder that sends less. `voxels.zlib.v1`, the Map3D encoding, is one: it sends a cloud as voxel occupancy bits, about 0.3 bytes per voxel.

When `encoding` is not given, an rx dimOS message gets its LCM encoding and everything else (every tx channel included) gets `json.v1`. `Image` has no default: use `jpeg.v1` or an encoder of your own. The built-in names keep their codecs: `Channel("odom", PoseStamped)` in `cockpit(channels=[...])` raises, because `odom` is `pose.json.v1`. Any other name takes the default:

```python
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.web.cockpit import Channel

print(Channel("pose", PoseStamped).encoding)
print(Channel("health", dict).encoding)
```

```results
geometry_msgs.PoseStamped.lcm.v1
json.v1
```

```python
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.cockpit import Channel

try:
    Channel("camera_2", Image)
except ValueError as e:
    print(e)
```

```results
sensor_msgs.Image has no default web encoding (raw pixel buffers; use encoding='jpeg.v1' or a custom @web_encoder)
```

### Custom encoders

`@web_encoder("id")` registers a function as the encoder for an id. The first parameter's annotation is the message type it accepts (the channel's `message_type` must match). An optional second parameter annotated `Mapping[str, Any]` receives the channel's `params`. It returns `bytes`, an `EncodedPayload(payload, meta)` whose `meta` is a small JSON mapping (16 KiB at most) sent in the frame header, or `None` to skip the sample. The page [`web/examples/custom-path/index.html`](/web/examples/custom-path/index.html#L8) decodes this one:

```python skip
import struct

from dimos.msgs.nav_msgs.Path import Path
from dimos.web.cockpit import Channel, cockpit
from dimos.web.codecs import EncodedPayload, web_encoder


@web_encoder("path.points.v1")
def encode_path_points(msg: Path) -> EncodedPayload:
    payload = b"".join(struct.pack("<ff", p.position.x, p.position.y) for p in msg.poses)
    return EncodedPayload(payload, {"n": len(msg.poses)})


my_ui = cockpit(channels=[Channel("nav_path", Path, encoding="path.points.v1", max_hz=20.0)])
```

Codec functions must be module-level functions that can be imported by name. `cockpit()` resolves the id to the function when the blueprint is defined and ships it to the worker process by reference, so lambdas, nested functions and functions defined in a script running as `__main__` are rejected. Registering an id twice raises (except for the same function after a reload). An encoder that raises is logged, with rate limiting, and only its own channel is affected. The bridge calls an encoder on the stream's transport thread for live messages and on a worker thread when it replays a cached message (`resend_on_subscribe`) to a new viewer. The two can overlap, so an encoder must not keep state between calls. Compute the frame from the message and the params, as the built-in encoders do.

## Publishing from the browser

A `Channel(..., dir="tx", publish="shared")` is a browser input. It must be `reliable` and JSON-encoded. When a value arrives, the bridge decodes it with the registered decoder, publishes it on the generated `Out` port, and only then acknowledges to the browser. A decoder error or a publish error goes back as a rejection (`decode_failed`, `publish_failed`) and leaves other channels alone. The relay caps values at 32 KiB and rate-limits at the channel's `max_hz`, per viewer and per robot.

`@web_decoder("id")` registers the decoder. Its return annotation is the message type it produces (the channel's `message_type`), its first parameter is the JSON value, and an optional second parameter annotated `PublishContext` receives provenance: the robot id, the channel, the relay's receive time and forwarding id, the viewer's name when the relay has an auth file, and the browser's send time when the page passed one. The same module-level rule as for encoders applies. [`builtin_codecs.py`](/dimos/web/relay_bridge/builtin_codecs.py#L15) has the `text.json.v1`, `point.json.v1` and `bool.json.v1` decoders the Chat and Map2D panels use.

Teleop's `tele_cmd_vel` is the one tx channel that is not a publish channel. It uses its own datagram path with the lease and the watchdog below, and `publish()` rejects it.

## Teleop watchdog

The bridge is the last hop of the teleop safety chain described on the [Cockpit](/docs/web/cockpit.md#keyboard-teleop) page. It receives `twist` and `stop` commands as datagrams through the relay, clamps them (`vx` and `vy` to `max_linear * boost`, `wz` to `max_angular * boost`), and publishes `Twist` on `tele_cmd_vel`. A watchdog polls every 50 ms and publishes a zero when the robot is driving and no command has arrived for `watchdog_ms` (300 ms by default). The cockpit repeats commands at `publish_hz`, so silence while driving means the chain broke somewhere (the page is gone, the relay was killed, datagrams were lost), and the robot stops within about the watchdog window whichever hop failed.

Three details keep old or duplicated packets from restarting motion:

- Every lease grant carries a generation number, and the relay stamps it on every command. Commands from an older generation are rejected for good, and the sequence high-water mark survives silence, so a released driver's delayed datagrams cannot move the robot again.
- Zeros are edge-gated: the bridge publishes a zero once after motion, never repeatedly while idle. `MovementManager` cancels the autonomous navigation goal on every teleop message, so repeated idle zeros would cancel navigation.
- The e-stop (Space) is an unconditional zero, published at once even when nothing was moving. It cancels a navigation goal too.

When the manifest has no teleop channel, teleop messages are ignored.

## Connection behaviour

- The bridge fetches `/api/info` on every connect and retries every 2 s. A restarted relay (new QUIC port, new certificate) is transparent, and subscriptions are re-established after each reconnect. At startup it gives up after four failed attempts.
- A bridge killed without a clean close keeps its robot id registered on the relay until the relay's 30 s idle timeout. A restart inside that window waits for the conflict to clear (up to 45 s) instead of failing.
- A wrong or missing key on a relay with an auth file fails with `auth_failed`, logged once. The bridge stops instead of retrying.
- With a local relay, the bridge starts the relay process (downloading Deno on first use and building the web bundles when they are stale), waits for its ready line, and opens the browser.
