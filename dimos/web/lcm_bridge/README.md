# LCM ↔ WebSocket bridge

The dimos bus, in the browser. A subscribe-all bridge that forwards LCM
small-message packets to WebSocket clients and republishes packets clients
send back — so browser code publishes and subscribes like any other bus
peer, using [`@dimos/msgs`](https://jsr.io/@dimos/msgs) to encode/decode.

This is the load-tested extraction of the Babylon scene viewer's `/lcm-ws`
endpoint, contributed as a concrete v0 for the Dimos Web discussion:

- [#2710 Dimos Web Proposal](https://github.com/dimensionalOS/dimos/issues/2710) —
  this is a working data point for "the bridge is a DimOS module" (question 2)
  and the local/LAN case (question 3).
- [#2502 TS API Spec](https://github.com/dimensionalOS/dimos/issues/2502) —
  the central `DimosWebsocket` server sketched there subsumes this; until it
  exists, this module is the smallest thing that gives any blueprint a
  browser-reachable bus.
- [#2708 Dimos Web SDK](https://github.com/dimensionalOS/dimos/issues/2708) —
  the flow-control here (latest-wins buffering, per-channel rate caps) is
  the operational evidence behind the per-stream QoS requirements.

## Usage

Standalone, in any blueprint:

```python
from dimos.web.lcm_bridge.module import LcmWebSocketBridgeModule

autoconnect(
    my_robot_blueprint,
    LcmWebSocketBridgeModule.blueprint(
        port=9669,
        channel_rate_hz={"/global_map": 1.0},   # throttle the WS leg only
        topic_blocklist=["/camera_image_raw*"],  # never forward these
    ),
)
```

Browser:

```html
<script type="module" src="http://robot:9669/lcm_client.js"></script>
<script type="module">
  const { subscribe, publish } = window.dimosLcm;
  const { geometry_msgs } = window.dimosMsgs;
  subscribe("/odom", geometry_msgs.PoseStamped, (msg) => console.log(msg));
  publish("/cmd_vel", new geometry_msgs.Twist(...));
</script>
```

Embedded in an existing Starlette app (the Babylon viewer pattern):

```python
from dimos.web.lcm_bridge.bridge import LcmWebSocketBridge

bridge = LcmWebSocketBridge(channel_rate_hz={"/global_map": 1.0})
bridge.start()
app = Starlette(routes=[*my_routes, *bridge.routes()])
```

`GET /` on the standalone module returns JSON debug counters
(clients, forwarded, rate_capped, filtered, published_from_clients).

Requires the `web` extra (`starlette` + `uvicorn`).

## Flow control (why this isn't a naive forwarder)

Learned running multi-MB pointclouds at 10 Hz into browser tabs:

- **Latest-wins buffering** — one slot per (client, channel); when the bus
  outpaces a client's socket, stale packets are overwritten, never queued.
  The naive queue-per-packet design put browsers 10–15 s behind real time.
- **Single drain task per client** — exactly one in-flight send per socket;
  a stalled client is closed (its JS auto-reconnects) instead of holding
  buffers hostage.
- **Per-channel rate caps** — fnmatch pattern → max Hz, applied to the
  WebSocket leg only; in-process bus consumers are unaffected.

## Known limitation: LCM only

The bridge taps the **LCM** bus. Runs using the Zenoh transport default
(`--transport zenoh` / `DIMOS_TRANSPORT`) publish nothing on LCM, so the
bridge forwards nothing — run with `--transport lcm` for now. A Zenoh
subscribe-all twin is the natural follow-up (payloads are the same
encoded bytes; only the tap changes), and is exactly the
transport-agnostic central bridge #2502 sketches.

## Deliberately out of scope (v0)

- **Per-connection QoS** (client-requested rates/filters) — that's the
  `setQos` API in #2502; here filtering and caps are server config.
- **Fragmented LCM messages** (>~64 KB) — not bridged. Big payloads (video,
  raw pointclouds) should be encoded/decimated server-side first, per #2708.
- **WebTransport / brokered remote access** — #2710's transport and broker
  questions; this module is the same-host/LAN case.
- **Auth** — LAN-trust, like every other dimos listener today.
