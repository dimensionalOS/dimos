# Web SDK

Use the web SDK to build your own robot web page. It connects to a relay, watches a robot and hands you decoded channel data. It can also send input to the robot. The cockpit is built on the same SDK, and the robot side does not change: the same bridge and the same relay serve the cockpit and your page.

You need no build step. The relay serves the SDK as an ES module at `/sdk.js`, so a plain HTML file can `import { connect } from "/sdk.js"`. Inside the dimos repository it is also the `@dimos/sdk` Deno workspace package with React bindings (it is not published to npm yet).

This page starts with "show odom on my own page", then adds a channel of your own, a custom binary encoding, and input to the robot. The reference for every call is at the end.

## Start a robot with a local relay

Any blueprint works, and a recording needs no hardware:

```bash
uv run dimos --replay run unitree-go2 --local-relay
```

`--local-relay` starts a relay on `http://127.0.0.1:7780` and connects the robot to it. The relay serves:

- `/`: the cockpit (or your own files, with `--serve-dir`)
- `/sdk.js`: the SDK as a zero-build ES module
- `/api/info`: what a client needs to connect

The local relay listens on loopback only and deliberately trusts local browser pages (wildcard CORS on those routes), so a page from any local origin connects without configuration. Running a relay by hand, sharing one relay between robots, and relays with TLS and viewer tokens are on the [Relay](/docs/web/relay.md) page. Your page then passes the relay's address and the token to `connect()`, as shown in [Working from another origin](#working-from-another-origin).

## Your first page

Create `ui/index.html`:

```html
<!DOCTYPE html>
<html>
  <body>
    <div id="status">connecting</div>
    <pre id="odom">(no data)</pre>
    <script type="module">
      import { connect } from "/sdk.js";

      const session = connect();

      session.status.subscribe(() => {
        const s = session.status.get();
        document.querySelector("#status").textContent =
          `${s.transport.phase}, robots: ${s.robots.map((r) => r.id).join(", ") || "none"}`;
      });

      session.subscribe("odom", (snapshot) => {
        document.querySelector("#odom").textContent =
          JSON.stringify(snapshot.slot?.value ?? null, null, 2);
      });
    </script>
  </body>
</html>
```

Stop the run above (Ctrl-C) and start it again with your directory served instead of the cockpit:

```bash
uv run dimos --replay run unitree-go2 --local-relay --serve-dir ui
```

`http://127.0.0.1:7780` opens automatically and you get live odometry on your own page.

What the parts do:

- `connect()` returns a `Session` immediately and connects in the background.
- Locally you normally have one robot on the relay, so the session watches it automatically. `session.watch(robotId)` pins a specific robot when there are several.
- `session.subscribe(ch, cb)` does two things: it registers your callback and asks the relay for the channel. The first subscriber for a channel sends `sub` to the relay, the last unsubscribe sends `unsub`. It returns an unsubscribe function (ignored here).
- Subscribing before the robot or its manifest exists is fine. The subscription stays desired and activates when a manifest with that channel arrives.
- `session.close()` tears everything down.

## Snapshots, and the two read paths

Callbacks receive a `ChannelSnapshot` on the session's UI tick (every 500 ms by default). This lets you update many UI elements without an unreasonable number of re-renders.

For canvas or video rendering you want every frame, so read the store directly. `session.store.subscribe(ch, cb)` fires per accepted frame and `session.store.get(ch)` returns the latest slot:

```js
session.subscribe("color_image", () => {}); // keeps the relay subscription
session.store.subscribe("color_image", () => {
  const slot = session.store.get("color_image");
  if (slot) draw(slot.value);
});
```

The store path never asks the relay for anything by itself, so keep one `session.subscribe` handle alive for the channel.

Delivery mode is per channel and declared on the robot side. `reliable` channels arrive ordered and complete. `latest` channels (video, costmap) drop stale frames and keep only the newest.

## Working from another origin

`--serve-dir` is optional. A Vite dev server, or any local page, can point at the relay explicitly:

```js
import { connect } from "http://127.0.0.1:7780/sdk.js";

const session = connect({ url: "http://127.0.0.1:7780" });
```

A relay with auth also takes the viewer token: `connect({ url, token })`. A `file:` page works too (Chromium and Firefox both allow WebTransport there).

Inside the dimos repository you can also import the SDK source directly. `web/sdk` is the `@dimos/sdk` Deno workspace package, and `web/sdk/fixture/` is a small Vite consumer you can copy (`deno task fixture` in `web/sdk`). React bindings live on the `@dimos/sdk/react` subpath and read UI-tick snapshots through `useSyncExternalStore`:

```jsx
import { useChannel, useStatus } from "@dimos/sdk/react";

function Pose({ session }) {
  const status = useStatus(session);
  const odom = useChannel(session, "odom"); // subscribes while mounted
  return <pre>{status.transport.phase}: {JSON.stringify(odom.slot?.value)}</pre>;
}
```

`@dimos/sdk` is not published to npm yet. Outside the repository, use `/sdk.js`.

## Exposing your own channels

So far the page could only subscribe to the bridge's built-in channels (`odom`, `color_image`, `global_costmap`). `cockpit(channels=[...])` exposes any typed stream without a panel. A `Channel` names a stream and its Python type. The defaults matter here: a dimOS message type is sent as its LCM bytes and decoded in the browser without any registration, a plain dataclass or JSON value is sent as JSON, and an `Image` needs an explicit encoding. Every parameter is on the [Bridge](/docs/web/bridge.md#exposing-a-stream-with-channel) page.

The example package below adds a `health` stream from a module of our own to the Go2. The [next section](#a-custom-binary-encoding) adds the Go2's lidar with a custom binary encoding.

```
my-ui/
  pyproject.toml
  my_ui/
    web.py
  page/
    index.html
```

`pyproject.toml` declares the blueprint entry point so `dimos run` can find it:

```toml
[project]
name = "my-ui"
version = "0.1.0"

[project.entry-points."dimos.blueprints"]
go2-web = "my_ui.web:go2_web"

[tool.setuptools]
packages = ["my_ui"]

[build-system]
requires = ["setuptools"]
build-backend = "setuptools.build_meta"
```

`my_ui/web.py`:

```python skip
from dataclasses import dataclass
import time

import reactivex as rx

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.web.cockpit import Channel, cockpit


@dataclass
class Health:
    status: str
    uptime_s: float


class HealthMonitor(Module):
    health: Out[Health]
    _started: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._started = time.time()
        self.register_disposable(
            rx.interval(1.0).subscribe(
                lambda _: self.health.publish(Health("ok", time.time() - self._started))
            )
        )


go2_web = autoconnect(
    unitree_go2,
    HealthMonitor.blueprint(),
    cockpit(channels=[Channel("health", Health, max_hz=1.0)]),
)
```

How the pieces connect:

- A `Channel` whose stream is not a built-in bridge port gets a bridge input port of that name and type. `autoconnect` then wires it like any other stream, here `health` to `HealthMonitor.health`.
- `Channel("health", Health)` uses `json.v1`, the default for JSON scalars, lists, dicts and plain dataclasses. The browser decodes it without registration.
- Encoding is lazy. A channel costs nothing until some viewer subscribes, and stops encoding when the last viewer leaves.
- `cockpit(layout=..., channels=[...])` is fine too. For a pure SDK setup, leave `layout` out and declare only the channels you want to serve.

`page/index.html` shows the value:

```html
<!DOCTYPE html>
<html>
  <body>
    <div id="health">health: (no data)</div>
    <script type="module">
      import { connect } from "/sdk.js";

      const session = connect();
      session.subscribe("health", (snapshot) => {
        document.querySelector("#health").textContent =
          `health: ${JSON.stringify(snapshot.slot?.value ?? null)}`;
      });
    </script>
  </body>
</html>
```

Install the package into the same environment as dimos and run it:

```bash
uv pip install -e ./my-ui
uv run dimos --replay run my-ui.go2-web --local-relay --serve-dir my-ui/page
```

The page opens automatically and the health line updates once a second.

## A custom binary encoding

The Go2's `lidar` stream is a `PointCloud2`. Its default LCM encoding costs 16 bytes per point at the full rate. This step sends a quarter of the points as bare (x, y) pairs instead and draws them as a live 2D scatter. It adds an encoder and a channel on the robot side, and a decoder and a canvas on the page.

In `web.py`, the encoder and the second channel:

```python skip
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.web.codecs import EncodedPayload, web_encoder


# lidar.xy.v1: little-endian float32 (x, y) pairs, point count in meta.
@web_encoder("lidar.xy.v1")
def encode_lidar_xy(msg: PointCloud2) -> EncodedPayload:
    points = msg.points_f32()[::4, :2]  # every 4th point, drop z
    return EncodedPayload(points.tobytes(), {"n": len(points)})
```

```python skip
(
    cockpit(
        channels=[
            Channel("health", Health, max_hz=1.0),
            Channel("lidar", PointCloud2, encoding="lidar.xy.v1", delivery="latest", max_hz=5.0),
        ]
    ),
)
```

`autoconnect` wires `lidar` to the driver's `lidar: Out[PointCloud2]` already inside `unitree_go2`. An encoder returns `bytes`, an `EncodedPayload` (payload plus a small JSON meta mapping sent in the frame header), or `None` to skip a sample. Because dimOS modules live in different processes, codec functions must be importable by name (the registry ships them by reference). A function defined inline in a script or a lambda is rejected.

In `index.html`, the decoder and the canvas. `json.v1`, any `*.json.vN` and any `*.lcm.v1` encoding decode automatically, but `lidar.xy.v1` is opaque bytes to the SDK, so the page registers the matching decoder and passes it to `connect()` in place of the bare `connect()` above:

```html
<canvas id="lidar" width="400" height="400" style="border: 1px solid #888"></canvas>
```

```js
import { connect, createDecoderRegistry } from "/sdk.js";

const decoders = createDecoderRegistry();
decoders.register("lidar.xy.v1", (payload, header) => {
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const points = [];
  for (let i = 0; i + 8 <= payload.byteLength; i += 8) {
    points.push({ x: view.getFloat32(i, true), y: view.getFloat32(i + 4, true) });
  }
  return { value: points };
});

const session = connect({ decoders });

const ctx = document.querySelector("#lidar").getContext("2d");
session.subscribe("lidar", (snapshot) => {
  const points = snapshot.slot?.value;
  if (!points?.length) return;
  ctx.clearRect(0, 0, 400, 400);
  for (const p of points) {
    ctx.fillRect(200 + p.x * 20, 200 - p.y * 20, 2, 2);
  }
});
```

Restart the run and the lidar scatter draws live. The recording is a couple of minutes long. When it ends the picture freezes, so restart the run to see it again. [`web/examples/custom-path/index.html`](/web/examples/custom-path/index.html) in the repository is a minimal codec pair of the same shape.

A decoder is `(payload: Uint8Array, header) => { value, preview? }`, looked up by the channel's manifest encoding. `header.meta` carries the encoder's `EncodedPayload` meta. The rules are in the [Decoders](#decoders) reference below.

## Publishing to the robot

A `dir="tx"` channel with `publish="shared"` is a browser input. Any viewer may publish on it. The bridge decodes the JSON value with the matching decoder and publishes it on a typed `Out` port, and your modules consume it like any other stream.

```python skip
cockpit(
    channels=[
        Channel("human_input", str, dir="tx", encoding="text.json.v1", publish="shared"),
    ]
)
```

```js
try {
  const receipt = await session.publish("human_input", "hello robot");
  // The bridge decoded the value and published it on the dimOS stream.
} catch (e) {
  // e.outcome === "rejected": it definitively did not happen (e.code says why).
  // e.outcome === "unknown": the connection died before the ack. It MAY have
  // been published. Never auto-resend an "unknown" command.
}
```

- Only reliable JSON-encoded tx channels declared `publish="shared"` accept `publish()`. Everything else (rx channels, teleop) rejects locally with a stable code. Values are any JSON value (`null` included), at most 32 KiB serialized and at most 100 nesting levels deep.
- The relay rate-limits per viewer and per robot at the channel's `max_hz`, so extra open tabs never multiply the accepted rate.
- The robot side decodes `json.v1` (strings, numbers, booleans, lists and dicts), `text.json.v1`, `point.json.v1` and `bool.json.v1` without registration, and `audio.json.v1` comes with the Chat panel. Any other encoding needs a `@web_decoder`, see [Publishing from the browser](/docs/web/bridge.md#publishing-from-the-browser) and the [encoding table](/docs/web/bridge.md#encodings).
- [`web/examples/chat-input/index.html`](/web/examples/chat-input/index.html) in the repository is a minimal publish page.

## API reference

### connect

`connect(options?)` returns a `Session` at once and connects in the background.

| Option | Default | Meaning |
|---|---|---|
| `url` | same origin | HTTP address of the relay. The SDK fetches `<url>/api/info` to find the WebTransport endpoint, on every connect. |
| `robot` | none | Robot id to watch. Without it the session watches the robot when it is the only one on the relay. |
| `decoders` | a fresh registry with the built-ins | From `createDecoderRegistry()`. Captured for the session's lifetime. |
| `token` | none | Viewer token for a relay started with `--auth-file`. A rejected token fails the transport for good (`auth_failed`). |
| `uiTickMs` | 500 | Period of the UI tick that feeds `subscribe()` callbacks. |

### Session

| Member | What it does |
|---|---|
| `status` | Store of the session state. `get()` returns a `SessionStatus`, `subscribe(cb)` calls back on every change and returns an unsubscribe function. |
| `store` | The channel store. `get(ch)` returns the latest `Slot` or `null`. `subscribe(ch, cb)` fires on every accepted frame. Neither asks the relay for anything. |
| `watch(robotId)` | Pins the watch to one robot. Resolves with the manifest once it is validated and adopted. Stays pending while the robot is absent. Rejects with `WatchRejectedError` when a newer `watch()` supersedes it or the session closes. |
| `subscribe(ch, cb)` | Registers a UI-tick callback and asks the relay for the channel in one step. Allowed before a robot or manifest exists. Once a manifest is adopted, a missing or non-rx channel raises an `unknown_channel` session error and stays desired for a later manifest. Returns an unsubscribe function. |
| `publish(ch, value, { clientTs? })` | Publishes one JSON value on a `publish="shared"` tx channel. Resolves with `{ ch, relayTs, bridgeTs }` (seconds since the epoch) once the bridge published it. See [Publishing](#publishing). |
| `close()` | Closes the transport, drops every subscription and rejects pending publishes. |

`SessionStatus` has `transport` (the phase below), `robots` (every robot on the relay, `{ id, name, model }`), `watchedRobot`, `manifest`, `manifestUnsupported` (the robot's manifest version is newer than the SDK), `epoch` (bumps whenever the manifest changes) and `lastError` (`{ code, message, ch? }` with code `invalid_manifest`, `unknown_channel` or `relay_error`).

`transport.phase` is `connecting` (with `attempt`), `connected`, `reconnecting` (with `attempt`, `retryAtMs` and a `reason`) or `failed` (with `reason` and a `code` such as `auth_failed`). Reconnects back off from 500 ms to 8 s. A `failed` transport does not retry.

### Channel data

A `Slot` is the latest accepted frame of a channel: `value` (whatever the decoder returned), an optional `preview` string, `seq` and `ts` from the frame header, and `version` (bumps per accepted frame).

A `ChannelSnapshot` is what `subscribe()` callbacks receive: `slot` (or `null` before the first frame) and `stats`.

`ChannelStats` counts `frames` and `decodeErrors`, and reports `hz`, `ageMs` (of the latest frame), `lastFrameAtMs`, `lastSeq`, `lastTs` and `decodeFailing` (the decoder threw on the latest frame). Latest channels can deliver out of order, so the store keeps the newest frame by `seq`.

### Decoders

`createDecoderRegistry()` returns a registry with the built-ins. `register(encoding, decoder, { replace? })` adds one. Registering a taken id throws unless `replace: true` is passed. Each session owns its registry (`connect({ decoders })`), so two apps on one page cannot clobber each other.

A decoder is `(payload: Uint8Array, header: FrameHeader) => { value, preview? }`. The session picks one per manifest channel: a registered `encoding` id first, then the `*.json.vN` convention (JSON-decoded without registration), then `*.lcm.v1` (a dimOS message decoded with the LCM schema the robot put in the channel's `params.lcm`, compiled once per manifest).

Built-in ids:

| Encoding | Value |
|---|---|
| `jpeg.v1` | The raw JPEG bytes (`Uint8Array`). Wrap them in a `Blob` to decode them: `createImageBitmap(new Blob([bytes], { type: "image/jpeg" }))` for a canvas, or `URL.createObjectURL(blob)` for an `<img>` (revoke the URL once the image is shown). |
| `costmap.zlib.v1` | `{ bytes, w, h, res, origin }` with the cells still deflated. `await inflateCostmap(value)` returns the `w * h` cells. |
| `json.v1` (and any `*.json.vN`) | The parsed JSON value. |
| `*.lcm.v1` | A plain object with the LCM fields in wire order (the `*_length` count fields included). Nested structs are plain objects, `byte[]` is a `Uint8Array` and `int8_t[]` an `Int8Array` viewing the frame (a view pins the whole frame, `slice()` copies it out), other primitive arrays are typed arrays, and `int64_t` is a `bigint` (`JSON.stringify` throws on it). |

A frame that would expand into more than 100k struct, string or boolean array elements is reported as oversized instead of decoded, like an oversized `json.v1` payload. A page that needs more registers `lcmDecoder(schema, { maxArrayElements })` for that encoding (the schema is the channel's `params.lcm` in the manifest).

An encoding with no decoder is not an error. The channel still counts frames and its value stays unset. A throwing decoder bumps `decodeErrors` and `decodeFailing` and keeps the last good value. Keep decoders synchronous and cheap, they run on the ingest path. Panel-paced work (inflate, draw) belongs in the consumer.

### Publishing

`publish()` rejects with a `PublishError` carrying `outcome` and `code`. `outcome: "rejected"` means it definitely did not happen. `outcome: "unknown"` means the connection died, or nothing answered, after the send. The SDK never resends on its own.

| Outcome | Codes |
|---|---|
| rejected locally | `closed`, `not_connected`, `not_serializable` (a non-JSON value, or more than 100 nesting levels), `too_large` (over 32 KiB serialized), `unknown_channel`, `not_publishable`, `exclusive_unsupported`, `pending_limit` (64 in flight) |
| rejected by the relay | `no_watch`, `unknown_robot`, `unknown_channel`, `not_publishable`, `publish_too_large`, `duplicate_request`, `pending_limit`, `rate_limited` |
| rejected by the bridge | `unknown_channel`, `decode_failed`, `publish_failed` |
| unknown | `publish_timeout`, `robot_disconnected`, `connection_lost`, `closed` |

### React

`@dimos/sdk/react` exports three hooks built on `useSyncExternalStore`:

- `useStatus(session)` returns the current `SessionStatus`.
- `useChannel(session, ch)` subscribes while the component is mounted and returns the `ChannelSnapshot`.
- `useStoreChannel(store, ch)` observes a channel without asking the relay for it.

## Examples in the repository

| Page | Shows | Run |
|---|---|---|
| [`web/examples/minimal/index.html`](/web/examples/minimal/index.html) | odom on a page, `/sdk.js`, no build | `dimos run <bp> --local-relay --serve-dir web/examples/minimal` |
| [`web/examples/custom-path/index.html`](/web/examples/custom-path/index.html#L2) | a custom binary decoder (`path.points.v1`) | the same with `--serve-dir web/examples/custom-path`, and the Python half from the [Bridge](/docs/web/bridge.md#custom-encoders) page |
| [`web/examples/chat-input/index.html`](/web/examples/chat-input/index.html#L4) | `publish()` with acks and rejections | the same with `--serve-dir web/examples/chat-input` and a `human_input` channel |
| [`web/sdk/fixture/main.ts`](/web/sdk/fixture/main.ts) | a Vite consumer importing the SDK source | a relay first, then `cd web/sdk && deno task fixture`, open `http://localhost:5174/` |
