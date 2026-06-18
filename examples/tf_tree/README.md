# Live TF Tree

A browser visualization of a live TF (transform) tree, end-to-end over the
DimOS **ts_bridge**.

```
FakeTfPublisher ──/tf──▶ LCM bus ──▶ TsBridgeModule ──websocket──▶ browser
                                          ▲   │                       │
                            web_register  │   └── serves /TfWebUi,    │
                            (WebModule)    │       /dimos.js, /ws  ────┘
                                       TfWebUi              @dimos/client + @dimos/msgs
```

The bridge is the **only** web server: it streams data *and* hosts every
`WebModule`'s page. `TfWebUi` is a `WebModule` — it has no server of its own; in
`start()` it calls `web_register` (via a `WebHostSpec` reference) to hand the
bridge its HTML, served at `/TfWebUi`.

## Run

```bash
python examples/tf_tree/tf_tree.py
```

A browser opens at <http://localhost:8999/TfWebUi> (the bridge's `ts_api_port`).
The page draws the transform tree and updates each frame's translation and a yaw
needle in place as `/tf` streams in.

## Binary (no JSON) + QoS

The bridge is a **byte pipe**: it forwards each message in the bus's own wire
format (`pubsub.encode`) as a binary frame and never transcodes — no bespoke
server-side serialization, nothing LCM-specific. The page imports
[`@dimos/msgs`](https://jsr.io/@dimos/msgs) and passes its `decode` to
`Dimos.connect({ decode, ... })` to decode those bytes client-side. Without a
`decode` you receive the raw `Uint8Array` and decode it however you like. To use
a different wire format, swap the bus codec — both sides then speak it.

Each subscription carries a QoS profile (ROS2-flavoured):

```js
qos: { "/tf": { rate: 30, durability: "transient_local", reliability: "best_effort" } }
```

- `rate` — max messages/sec (server-side throttle; `rateLimit` is sugar for this).
- `durability: transient_local` — a late-joining client immediately gets the
  cached newest message (no waiting for the next publish).
- `reliability` — `best_effort` coalesces to newest (low latency); `reliable`
  buffers up to `depth` messages in order.
- `depth` — outbox/history buffer size (default 1, or 10 for `reliable`).

**TF special case:** `/tf` publishers each send only their own transforms, so
naive coalescing would drop frames. The bridge aggregates TF by `(parent, child)`
and forwards the **complete** tree, so every transform reaches the web (and a
late joiner gets the whole tree at once via `transient_local`).

## Pieces

| File | Role |
|------|------|
| `tf_tree.py` | `FakeTfPublisher` (animated tree on `/tf`) + `TfWebUi` (a `WebModule`), composed with `TsBridgeModule` via `autoconnect`. |
| `index.html` | The viewer. `import { Dimos } from "/dimos.js"`, connects same-origin, subscribes to `/tf`, renders the SVG tree. |
| `logo.ico` | Favicon, registered via `web_icon` and served at `/TfWebUi/icon`. |
| `demo_smoke.py` | Headless end-to-end check (page + client + binary + QoS). |

A `WebModule` only implements `web_init(self) -> str` (returns its page HTML,
run once at start) and optionally sets `web_icon`. The base lives in
`dimos/core/module.py`; the bridge (`TsBridgeModule`) implements `WebHostSpec`
and serves `/dimos.js` (the `@dimos/client` package) and every registered page.

## How the bridge filters

The browser sends its own `whitelist` / `blacklist` / `qos` on connect; the
bridge only forwards what that client asked for, at the requested QoS. Here the
page whitelists just `/tf`. Patterns are globs (`*` stops at `/`, `**` spans it).

## Dynamic client API

```js
const stop = app.subscribeAll((m) => console.log(m.stream, m.data))  // streams not known up front
app.setQos("/tf", { rate: 10, durability: "transient_local" })       // change QoS at runtime
```

`subscribeAll` fires for every message the client is allowed to receive (connect
with no whitelist to see everything the bridge exposes); `m.stream` says which
stream it came from. `setQos` updates a stream's QoS without reconnecting.

## Backend exposure control (safety)

The operator restricts what any client may ever see or call, independent of each
client's own filter, on `TsBridgeModule`:

```python
TsBridgeModule.blueprint(
    topic_whitelist=["/tf", "/odom"],          # or topic_blacklist=[...]
    rpc_whitelist=["GO2Connection.standup"],   # "Module", "Module.method", or "Module.*"
)
```

Framework-internal RPCs (`start`, `stop`, `set_transport`, `web_register`, …)
and any `_private` method are never reachable from the web, regardless of lists.
Hidden RPCs aren't even advertised in `app.modules`.
