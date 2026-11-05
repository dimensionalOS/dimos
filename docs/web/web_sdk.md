# Web SDK

Use the web SDK to build your own robot web page. It connects to a relay, watches a robot and hands you decoded channel data. It can also send input to the robot. The cockpit is built on the same SDK, and the robot side does not change: the same bridge and the same relay serve the cockpit and your page.

You need no build step. The relay serves the SDK as an ES module at `/sdk.js`, so a plain HTML file can `import { connect } from "/sdk.js"`. The same SDK is available as the `@dimos/sdk` Deno workspace package and a buildable Node ESM package with TypeScript declarations and optional React bindings. It is not published to npm yet.

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

For a Node/Vite consumer outside this repository, build and pack a reviewed checkout:

```bash
cd web/sdk
deno task build:package
npm pack
```

Install the resulting `dimos-sdk-0.1.0.tgz` in your application and pin its checksum with the dependency. The package exports `@dimos/sdk` and `@dimos/sdk/react`, including declarations and bundled internal protocol dependencies; no Deno import map is needed by the consumer. Importing the root does not load React. This is a package build, not a Node WebTransport implementation: the application still needs a runtime that supplies WebTransport, normally a supported browser.

## Raw topics

`session.topics` operates alongside the declared-channel API. A topic source has its own lifetime and does not require a watched robot, a robot manifest, or a running blueprint. It needs a topic provider running next to the robot and a relay with open topics: a local relay is open, and a relay with an auth file opens them with `--open-topics`. See [Raw topics](/docs/web/relay.md#raw-topics) on the relay page.

Name the source with `connect({ url, token, topicSource })`, or leave `topicSource` out when a single provider is connected. A subscription to a source that is not connected yet waits (`interrupted` with `source_offline`) and attaches when the provider connects. The provider's Zenoh session defines the reachable network; selecting a robot in the UI does not narrow it.

| Call | Behavior |
|---|---|
| `session.topics.subscribe(keyExpr, options, onMessage)` | Returns a handle immediately. Zenoh expressions such as `workspace/desktop/events/**` retain each sample's concrete key. No catalog entry or existing publisher is needed. |
| `handle.ready()` | Resolves after the provider installs the subscription and the route is ready, even when no publisher exists. |
| `handle.status()` / `handle.onStatus(callback)` | Reports `connecting`, `ready`, `interrupted`, `rejected`, or `closed`, with a structured error when applicable. `onStatus` returns a listener cleanup function. |
| `handle.update(options)` | Changes the settings of this handle and resolves with the settings in force. A changed `source`, `delivery` or `profile` reopens only this handle; other consumers of the old route keep theirs. |
| `handle.settings()` | The settings in force: the profile values until the relay acknowledges the subscription. |
| `handle.close()` | Releases this handle. The source subscription ends after its last consumer leaves. |
| `session.topics.onReconnect(callback)` | Runs after available subscriptions recover; sources that are not connected do not block it. Refetch authoritative HTTP state here after an interrupted event stream. |
| `session.topics.sources()` / `onSources(callback)` | Lists the connected sources with their generation, the provider's read/write capabilities, supported profiles and `profileOptions`, the settings each profile takes with their ranges and defaults. A source that disconnects leaves the list. |
| `session.topics.list({ source, prefix, cursor, limit })` | Returns a bounded page of descriptors plus a continuation cursor. An empty cursor ends pagination. |
| `session.topics.publish(key, bytes, { source, encoding })` | Sends binary bytes to a concrete key and resolves with the provider's local-put receipt. Requires a provider that accepts writes. |
| `session.topics.video(key, options, onTrack)` | A camera as a `MediaStreamTrack` over a direct WebRTC connection to the provider. See [Direct video for topics](#direct-video-for-topics). |

Subscription options are the identity, `source`, `delivery` and `profile`, and the settings below. Select `delivery: "reliable"` for events, with `profile: "raw"`; every delivered sample invokes the callback immediately, outside the UI snapshot tick. Dense event bursts may be coalesced for up to 90 ms while preserving individual frames and callbacks. Reliable overflow interrupts the route explicitly so the application can resynchronize. `delivery: "latest"` keeps the newest sample per concrete key and drops obsolete work; `maxHz` limits this sampling, not reliable event delivery.

| Setting | Profiles | Meaning |
|---|---|---|
| `maxHz` | all | Latest sampling rate, per viewer, in (0, 1000]. Reliable event streams ignore it. |
| `maxResolution` | `image.jpeg.*`, `video.h264.v1` | `[w, h]` box the picture is fitted into, 16 to 4096 a side. |
| `quality` | `image.jpeg.*` | JPEG quality, 1 to 100. |
| `keepEvery` | `cloud.q16.*` | One point in N after the finite filter, 1 to 64. |
| `fields` | `cloud.q16.*` | The optional buffers to encode, a subset of `["intensity", "color"]`. |

The profile is the default and the ceiling of every setting: a larger size, quality or rate comes back clamped to the profile value, and a smaller `keepEvery` comes back raised to it. An omitted or `undefined` key keeps its requested value; `null` resets it to the profile value. For example, `maxHz: null` restores the advertised profile rate, or the protocol default of 1000 Hz for raw topics. `update()` resolves with the normalized settings, so `maxResolution: [3000, 3000]` on `image.jpeg.low.v1` resolves with `[640, 360]`, and `settings()` reports the same values at any time.

`update()` takes one of two paths. A handle alone on its subscription changes it in place: no status change, no missed frame, one acknowledgement. A handle sharing its subscription with other consumers, or asking for settings another subscription already serves, moves to that subscription once it is ready, so the others keep what they had. Changes go to the relay one at a time: an `update()` before the previous one is answered waits for that answer and merges into the settings it left in force. A rejected change leaves the handle untouched and rejects with `unsupported_option` for a name the profile does not take, `option_out_of_range` for a value outside the advertised range, or `variant_limit` for a fifth distinct settings set on one camera and profile. Against a provider older than this SDK, which advertises no ranges, only `maxHz` can change.

The bundled provider shares subscriptions across consumers with the same
source, key expression, delivery and profile. Consumers with the same encode
settings share one transform; different settings on one camera share the
Zenoh subscription but encode separately, up to four variants per route.
Each variant samples at the highest rate among its viewers; the relay applies
each viewer's rate separately and retains that viewer's newest pending sample.

Route faults preserve their error code in `onStatus` and retry only the affected subscription with exponential backoff from 1 to 30 seconds. Other subscriptions and guarded publishers keep their connection. Temporary readiness and capacity rejections also retry; a new subscription, `update()`, or reconnect makes a fresh attempt after a rejection. Unknown option names are rejected. Each route tracks up to 128 concrete keys; a larger wildcard reports `topic_key_limit`, so narrow or split the expression when needed.

An undecodable media sample is dropped without interrupting the route. The provider logs a rate-limited warning for that route and counts `decodeFailures` in diagnostics and subsequent media sample metadata.

Each callback receives `payload: Uint8Array`, `key`, `encoding`, `source`, `generation`, `sequence`, `ingressTs`, optional `sourceTs`, `sentTs`, and `metadata`. Payload bytes are borrowed: copy before mutation or transfer to a worker. The timestamp fields distinguish acquisition, provider ingress and transmission. Preserve that distinction when measuring age. Decode JSON with a reused `TextDecoder` only on a stream whose encoding you know. Unknown encodings remain bytes; descriptor names never trigger imports or Python object construction.

Catalog entries distinguish known, observed and advertised topics. Quiet metadata is not proof that a publisher is currently producing data. Use the exact descriptor key, including remappings and domain/type suffixes. Raw reads remain possible when discovery is empty.

A successful binary publish means the provider's Zenoh put completed locally, not that the robot acted. A `TopicError` exposes `code` and an `outcome` of `rejected`, `unknown`, or `interrupted`. Disconnects and acknowledgement timeouts may have an unknown outcome. Commands are never automatically replayed after reconnect; do not retry an unknown result blindly.

Ordinary puts have a five-second provider deadline including queue time. A blocked put reports `topic_put_timeout` with an unknown outcome; subsequent writes are rejected as `topic_put_busy` until it completes. Expired queued writes are rejected before execution. This preserves subscriptions and the independent guarded-command lane without spawning more blocked put threads.

### Media and custom data

Select an advertised fixed profile with latest delivery. The provider gates the source before conversion and shares identical subscriptions; the SDK's `TopicMediaDecoder` bounds browser work across consumers. Feed raw callbacks to `decoder.push(sample)`, call `decoder.reset()` on subscription interruption or profile change, and `decoder.close()` with the subscription. Its callback receives a decoded media value and the original sample. Render through canvas/WebGL and animation frames instead of React state per frame. The renderer owns and must close any bitmaps it creates.

| Profile | Decoded value |
|---|---|
| `image.jpeg.low.v1` | JPEG bytes, at most 640×360, quality 60, 30 Hz |
| `image.jpeg.balanced.v1` | JPEG bytes, at most 1280×720, quality 75, 30 Hz |
| `image.jpeg.high.v1` | JPEG bytes, at most 1920×1080, quality 85, 30 Hz |
| `video.h264.v1` | No samples: a `MediaStreamTrack` through `topics.video()`, at most 1920×1080, 30 Hz, from a provider with video enabled |
| `depth.lossless.v1` | Lossless `Uint16Array` or `Float32Array`, original shape/encoding, units and invalid values, at most 30 Hz |
| `cloud.q16.low.v1` | Quantized positions, one in four valid points, at most 5 Hz |
| `cloud.q16.balanced.v1` | Quantized positions, all valid points, at most 20 Hz |
| `cloud.q16.full.v1` | Quantized positions, all valid points, at most 30 Hz |

Clouds expose `Float32Array` positions and optional float intensity/RGB buffers, source count, acquisition header and quantization error. Depth is numeric data, not color JPEG. The profiles are ceilings: the [settings](#raw-topics) lower the size, quality, stride or buffers of one subscription without reopening it, and the sample metadata (`w`, `h`, `quality`, `keepEvery`, `intensity`, `color`) reports the values in force. Adaptive bandwidth is not provided.

For explicit `pZenohTransport` reads, `projectCbor(sample.payload)` returns either `{ kind: "value", value }` or `{ kind: "opaque", bytes, reason }`. JSON-like values, tuples and simple dataclass fields have a bounded data-only projection. A dataclass name is inert metadata; unknown Python tags remain opaque. The existing LCM decoder and Desktop's `/msgs.js` wrappers remain available for known schemas and typed publishing.

### Continuous controls

Use `session.topics.publisher(key, safeStopBytes, { source, encoding, group, timeoutMs, takeover })` for a provider that advertises `guardedPublishers`. It requires a provider that accepts writes and an explicit safe-stop payload. `group` identifies shared control ownership; takeover is explicit. The handle has `ready()`, `status()`, `onStatus()`, `publish(bytes)`, and terminal `close()`. Wait for readiness before accepting fresh user input. Commands received while opening are rejected, not queued.

Provider expiry is 100–1000 ms, with 400 ms as the SDK default. The provider sends the safe stop on release, expiry or lost ownership, and the SDK retires a handle on focus/page/connection loss and refuses to open one while the page is hidden. The handle itself expires after `timeoutMs` without an accepted command, including a newly ready handle that never sent a command; heartbeats cannot keep it armed. A provider `guard_expired` status requires a new handle. After failure or reconnect, clear held controls and require a neutral transition and new input before creating another handle.

The robot also needs a watchdog on its side of any link whose loss could leave motion active. The migrated Controller uses the R1 Pro velocity watchdog and guarded joint-jog expiry in the robot-side trajectory task. An older robot runtime does not acquire those protections from a browser update. The dim-app adapter exposes `guardedPublisher`; its legacy `publisher().setDeadman()` API has not been recreated wholesale.

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
    cockpit(
        channels=[
            Channel("health", Health, max_hz=1.0),
            Channel("lidar", PointCloud2, encoding="lidar.xy.v1", delivery="latest", max_hz=5.0),
        ]
    ),
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

## Video as WebRTC tracks

A relay with Cloudflare configured ([Relay hosting](/docs/web/relay_hosting.md#video-through-cloudflare)) delivers every `jpeg.v1` channel as a `video.webrtc.v1` track. The robot encodes H.264 once and the browser pulls it from Cloudflare's SFU, so nothing of the video rides the relay. The SDK handles the signaling. The channel's value is the received `MediaStreamTrack` instead of JPEG bytes:

```js
session.subscribe("color_image", (snapshot) => {
  const track = snapshot.slot?.value;
  if (track instanceof MediaStreamTrack) {
    document.querySelector("video").srcObject = new MediaStream([track]);
  }
});
```

The cockpit's video panel does exactly this. The relay's `/api/info` says whether it offers WebRTC video (`rtc: true`), and the manifest's encoding says which channels use it. A relay without Cloudflare, or a robot without aiortc (`dimos[webrtc]`), keeps `jpeg.v1`. A `rtc_failed` session error means a WebRTC step failed, at the relay (which retries it) or locally (the SDK drops its peer connection and offers again with the transport's backoff, also when the relay reports the SFU session gone or the peer connection fails later). The WebTransport session itself is unaffected.

### Direct video for topics

A topic provider with video enabled ([Raw topics](/docs/web/relay.md#raw-topics)) advertises `video.h264.v1`. `session.topics.video()` subscribes to a camera key with that profile and opens a WebRTC connection straight to the provider, signaled through the relay: the provider encodes each frame once in software H.264 for every viewer of the same settings, the browser decodes in hardware, and no media rides the relay. No Cloudflare account or internet is needed, but the browser must reach the provider's UDP ports (host candidates only, no STUN or TURN).

```js
const video = session.topics.video("camera/color", { maxHz: 30 }, (track) => {
  document.querySelector("video").srcObject = track === null ? null : new MediaStream([track]);
});
await video.ready();
console.log(video.mode()); // "rtc"
await video.update({ maxResolution: [1280, 720] });
```

`onTrack` fires with the track when it arrives, again with a new track after a reconnect or a settings change that moved the viewer to another encoder, and with `null` when the track ends. Pass `fallback: "jpeg"` to open `image.jpeg.balanced.v1` instead when direct video is unavailable or has failed for good; the handle then reports `mode()` `jpeg`, keeps the reason in `status().error`, and delivers the JPEG samples to `options.onSample`.

| Member | What it does |
|---|---|
| `ready()` | Resolves once a track plays (or the JPEG fallback is ready). |
| `status()` / `onStatus(cb)` | `connecting`, `ready`, `interrupted`, `rejected` or `closed`, with a `TopicError`. Codes: `rtc_unavailable` (the source offers no video, or the provider is at its peer or encoder limit), `rtc_failed` (ICE, the answer timeout, a failed connection; the message says which, `no_direct_path` when no candidate pair connects), `encoder_failed`. |
| `mode()` | `rtc` while a track is live, `jpeg` on the fallback, `none` otherwise. |
| `track()` / `receiver()` | The live `MediaStreamTrack` and its `RTCRtpReceiver` (for `jitterBufferTarget` and `getStats()`). |
| `settings()` / `update(options)` | As on a subscription: `maxHz` and `maxResolution` (at most 1920×1080, 30 Hz). `profile` cannot change. |
| `close()` | Ends the peer and the subscription. |

A peer that fails is offered again after 1, 2 and 4 seconds; the third failure is final. Each `video()` handle has its own relay route and peer; two handles on one camera cost the provider one encode and two packetizations, so share the track on the page instead.

## Publishing to the robot

A `dir="tx"` channel with `publish="shared"` is a browser input. Any viewer may publish on it. The bridge decodes the JSON value with the matching decoder and publishes it on a typed `Out` port, and your modules consume it like any other stream.

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

`SessionStatus` has `transport` (the phase below), `robots` (every robot on the relay, `{ id, name, model }`), `watchedRobot`, `manifest`, `manifestUnsupported` (the robot's manifest version is newer than the SDK), `epoch` (bumps whenever the manifest changes) and `lastError` (`{ code, message, ch? }` with code `invalid_manifest`, `unknown_channel`, `rtc_failed` or `relay_error`).

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
| `video.webrtc.v1` | The received `MediaStreamTrack`, not a decoded frame. See [Video as WebRTC tracks](#video-as-webrtc-tracks). |
| `costmap.zlib.v1` | `{ bytes, w, h, res, origin }` with the cells still deflated. `await inflateCostmap(value)` returns the `w * h` cells. |
| `voxels.zlib.v1` | `{ bytes, res, n, chunks }` with the chunk records still deflated. `await inflateVoxels(value)` returns a `Float32Array` of the `n` voxel centres as x, y, z triplets. `n` is 0 for an empty cloud. |
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
