# Web SDK

We have two ways to construct web interfaces: using the cockpit or using the web SDK. The cockpit is the batteries-included method, where you write a blueprint which describes how pre-built panels are put together to form an interface to control and visualize a particular robot.

This is the second way, the web SDK. You get the raw internals and you build your web app your way.


This tutorial will start with simple stuff like "show odom on my own page" and finish with "stream my own message type with a custom binary encoding".

## Start a robot with a local relay

Any blueprint works. Simulation needs no hardware:

```bash
uv run dimos --simulation run unitree-go2 --local-relay
```

Using `--local-relay` spawns a relay on `http://127.0.0.1:7780` and bridges the robot to it. The relay serves:

- `/` the cockpit (or your own files, with `--serve-dir`)
- `/sdk.js` the SDK as a zero-build ES module
- `/api/info` relay bootstrap info

The local relay binds loopback and deliberately trusts local browser clients (wildcard CORS on the routes above), so a page from any local origin can connect without configuration. This is a local development mode, not the remote deployment story.

## Relay started by hand

`--local-relay` spawns the relay for you. Start one yourself to work on the relay without restarting the robot, or to share one relay between robots. Build the web dists once (and after changes under `web/`), then run the relay from `web/`:

```bash
cd web
deno install --frozen
deno task -r build
deno task dev --cockpit-dir cockpit/dist --sdk-dir sdk/dist
```

Attach a robot with `--relay-url` and the relay's HTTP URL:

```bash
uv run dimos --replay run unitree-go2 --relay-url http://localhost:7780
```

The bridge discovers the WebTransport endpoint (an ephemeral QUIC port and certificate) through `/api/info` on every connect, like the browser does. Open `http://localhost:7780/` for the cockpit. Things to know:

- Restart the relay whenever you like: the bridge and the page reconnect on their own.
- A bridge killed without a clean close keeps its robot id registered until the relay's 30 s idle timeout. Restarting it inside that window waits the conflict out.
- `--serve-dir` belongs to the relay here (`deno task dev --serve-dir DIR`). `dimos run --serve-dir` is rejected together with `--relay-url`.
- A second robot on the same relay needs its own `--robot-id`. A synthetic one: `uv run python -m dimos.web.relay_bridge.demo_smoke --url http://localhost:7780`.
- With several robots on the relay the cockpit lists them; pick one to watch it. "switch robot" in the status bar reopens the list.
- Another machine requires a relay with TLS and auth (next section); pass `--relay-ca` to the robot for a private CA.

## Relay with auth

A relay that other machines reach needs `--cert PEM --key PEM` and `--auth-file auth.json` (`--unsafe-non-loopback` skips both, at your own risk). The file holds robot keys bound to robot ids and viewer tokens, 16 to 256 characters each and no secret twice (`openssl rand -hex 32`):

```json
{
  "robots": { "go2-lab": "<key>" },
  "viewers": { "paul": "<token>" }
}
```

The robot sends its key in hello. Put it in the environment or `.env` as `RELAY_KEY` (the `--relay-key` flag works too, but shows in the process list):

```bash
RELAY_KEY=<key> uv run dimos run unitree-go2 --relay-url https://dimos-relay.example.com --robot-id go2-lab
```

The cockpit asks for the viewer token, keeps it in `localStorage`, and "log out" in the status bar forgets it. Your own page passes it to `connect({ url, token })`. `/api/stats` wants it as `Authorization: Bearer <token>` and drops its CORS header. A wrong key or token fails with `auth_failed` and neither client retries: fix the secret and restart. Edits to the file need a relay restart.

To host one on a VM with Docker and a Let's Encrypt certificate, or on a LAN with mkcert, follow [Relay hosting](/docs/usage/relay_hosting.md).

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

Serve it from the relay instead of the cockpit:

```bash
uv run dimos --simulation run unitree-go2 --local-relay --serve-dir ui
```

You will see `http://127.0.0.1:7780` open automatically and you get live odometry on your own page.

Intro to what some of the parts do:

- `connect()` returns a `Session` immediately and connects in the background.
- Normally, you only have one robot connected to the relay when running locally, so the session watches it automatically. `session.watch(robotId)` pins a specific robot when there are several.
- `session.subscribe(ch, cb)` is one operation for both halves: it registers your callback and creates wire interest. The first subscriber for a channel sends `sub` to the relay, the last unsubscribe sends `unsub`. It returns an unsubscribe function, but ignored here.
- Subscribing before the robot or its manifest exists is fine. The subscription stays desired and activates when a manifest with that channel arrives.
- `session.close()` tears everything down.

## Snapshots, and the two read paths

Callbacks receive a `ChannelSnapshot` on the session's UI tick. The reason for this is so you can update multiple UI elements without forcing an unreasonable number of UI re-renders.

For canvas or video rendering you want every frame, so read the store directly. `session.store.subscribe(ch, cb)` fires per accepted frame and `session.store.get(ch)` returns the latest slot:

```js
session.subscribe("color_image", () => {}); // holds the wire subscription
session.store.subscribe("color_image", () => {
  const slot = session.store.get("color_image");
  if (slot) draw(slot.value);
});
```

The store path never creates wire interest by itself, so keep one `session.subscribe` handle alive for the channel.

Delivery mode is per channel and declared robot-side: `reliable` channels arrive ordered and complete, `latest` channels (video, costmap) drop stale frames and keep only the newest.

## Working from another origin

`--serve-dir` is optional. A Vite dev server, or any local page, can point at the relay explicitly:

```js
import { connect } from "http://127.0.0.1:7780/sdk.js";

const session = connect({ url: "http://127.0.0.1:7780" });
```

A relay with auth also takes the viewer token: `connect({ url, token })`.

Inside the dimos repository you can also import the SDK source directly: `web/sdk` is the `@dimos/sdk` Deno workspace package, and `web/sdk/fixture/` is a small Vite consumer you can copy (`deno task fixture`). React bindings live on the `@dimos/sdk/react` subpath and read UI-tick snapshots through `useSyncExternalStore`:

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

So far the page could only subscribe to the bridge's built-in channels (`odom`, `color_image`, `global_costmap`). `cockpit(channels=[...])` exposes any typed stream without a panel:

```python skip
Channel(
    stream,                  # stream name, matched by autoconnect
    message_type,            # the stream's Python type
    encoding=None,           # default: <msg_name>.lcm.v1 for dimOS messages, json.v1 otherwise
    delivery="reliable",     # or "latest"
    max_hz=10.0,             # encode-rate cap
)
```

The example package below adds two channels to the go2: a `health` stream from a module of our own, and the go2's `lidar` stream with a custom binary encoding, drawn as a live 2D point scatter.

```
my-ui/
  pyproject.toml
  my_ui/
    web.py
  page/
    index.html
```

(`page/index.html` is written in the next section.)

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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.web.cockpit import Channel, cockpit
from dimos.web.codecs import EncodedPayload, web_encoder


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


# lidar.xy.v1: little-endian float32 (x, y) pairs, point count in meta.
@web_encoder("lidar.xy.v1")
def encode_lidar_xy(msg: PointCloud2) -> EncodedPayload:
    points = msg.points_f32()[::4, :2]  # every 4th point, drop z
    return EncodedPayload(points.tobytes(), {"n": len(points)})


go2_web = autoconnect(
    unitree_go2,
    HealthMonitor.blueprint(),
    cockpit(
        channels=[
            Channel("health", Health, max_hz=1.0),
            Channel("lidar", PointCloud2, encoding="lidar.xy.v1", delivery="latest", max_hz=5.0),
        ]
    ),
)
```

How the pieces connect:

- A `Channel` whose stream is not a built-in bridge port generates a bridge input port of that name and type. `autoconnect` then wires it like any other stream: `health` to `HealthMonitor.health`, `lidar` to the driver's `lidar: Out[PointCloud2]` already inside `unitree_go2`.
- `Channel("health", Health)` uses `json.v1`, the default for JSON scalars, lists, dicts and plain dataclasses. The browser decodes it without registration.
- A dimOS message type (`PoseStamped`, `Odometry`, `LaserScan`, ...) defaults to `<msg_name>.lcm.v1`, for example `geometry_msgs.PoseStamped.lcm.v1`: the frame is the message's `lcm_encode()` bytes and the manifest carries its LCM schema, so the browser decodes it to a plain object without registration. `Image` has no default (raw pixels): use `jpeg.v1` or an `@web_encoder`. Types without a `dimos_lcm` schema need an explicit encoding too. A bulk message costs its full size per frame (`PointCloud2` is 16 bytes per point), so set `max_hz`, or send less with an `@web_encoder` like `lidar.xy.v1` below.
- An encoder returns `bytes`, an `EncodedPayload` (payload plus a small JSON meta mapping that rides the frame header), or `None` to skip a sample. The first parameter annotation declares the message type it supports, and the channel's type must match.
- You can normally use `cockpit(layout=..., channels=[...])` to create a cockpit layout UI. But for a pure SDK access, leave "layout" out and just specify which channels you want to serve.
- Encoding is lazy. A channel costs nothing until some viewer subscribes, and stops encoding when the last viewer leaves.

Caveat: because dimOS modules live on different processes, registered functions must be publicly adressable. That is, you can't use `@web_encoder` on an inline function, because such a function cannot be pickled in order to be sent to other processes.

## Decoding custom encodings in the browser

`json.v1`, any `*.json.vN`, and any `*.lcm.v1` encoding decode automatically. `lidar.xy.v1` is opaque bytes to the SDK, so the page registers the matching decoder. `page/index.html`:

```html
<!DOCTYPE html>
<html>
  <body>
    <div id="health">health: (no data)</div>
    <canvas id="lidar" width="400" height="400" style="border: 1px solid #888"></canvas>
    <script type="module">
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

      session.subscribe("health", (snapshot) => {
        document.querySelector("#health").textContent =
          `health: ${JSON.stringify(snapshot.slot?.value ?? null)}`;
      });

      const ctx = document.querySelector("#lidar").getContext("2d");
      session.subscribe("lidar", (snapshot) => {
        const points = snapshot.slot?.value;
        if (!points?.length) return;
        ctx.clearRect(0, 0, 400, 400);
        for (const p of points) {
          ctx.fillRect(200 + p.x * 20, 200 - p.y * 20, 2, 2);
        }
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

The page opens automatically: health updates once a second and the lidar scatter draws live. The replay recording is a couple of minutes long. When it ends the picture freezes, so restart the run to see it again. `web/examples/custom-path/` in the repository is a minimal codec pair of the same shape.

Decoder notes:

- A decoder is `(payload: Uint8Array, header) => { value }`, looked up by the channel's manifest encoding. `header.meta` carries the encoder's `EncodedPayload` meta.
- A `*.lcm.v1` value is a plain object with the LCM fields in wire order: nested structs as objects, `byte[]` and `int8_t[]` as `Uint8Array`/`Int8Array` views into the frame (`slice()` to keep one), other arrays as typed arrays, `int64_t` as `bigint`. A frame above 100k struct or string array elements is reported as oversized instead of decoded.
- Each session owns its registry (`connect({decoders})`), so two apps on one page cannot clobber each other. Registering a taken encoding throws unless you pass `{replace: true}`.
- An encoding with no decoder is not an error. The channel still counts frames and renders as unsupported. A throwing decoder bumps `decodeErrors` and keeps the last good value.
- Decoders run on the ingest path, so keep them synchronous and cheap. Heavy work (inflate, draw) belongs in the consumer.

## Publishing to the robot

A `dir="tx"` channel with `publish="shared"` is a browser input: any viewer may publish, the bridge decodes the JSON value with the matching `@web_decoder` and publishes it on a typed `Out` port, and your modules consume it like any other stream.

```python skip
cockpit(channels=[
    Channel("human_input", str, dir="tx", encoding="text.json.v1", publish="shared"),
])
```

```js
try {
  const receipt = await session.publish("human_input", "hello robot");
  // The bridge decoded the value and published it on the dimOS stream.
} catch (e) {
  // e.outcome === "rejected": it definitively did not happen (e.code says why).
  // e.outcome === "unknown": the connection died before the ack - it MAY have
  // been published. Never auto-resend an "unknown" command.
}
```

Publish notes:

- Only reliable JSON-encoded tx channels declared `publish="shared"` accept `publish()`; everything else (rx channels, teleop) rejects locally with a stable code. Values are any JSON value (`null` included), at most 32 KiB serialized and at most 100 nesting levels deep.
- The relay rate-limits per viewer and per robot at the channel's `max_hz`, so extra open tabs never multiply the accepted rate.
- `text.json.v1` (strings) is built in; other encodings need an `@web_decoder` whose return annotation matches the channel's `message_type`. An optional second `PublishContext` parameter carries provenance (request id, principal, relay/client timestamps).
- `web/examples/chat-input/` in the repository is a minimal publish page.
