# DimOS web

Deno workspace for the robot web stack. `shared/` holds the wire protocol and its golden vectors;
`relay/` is the WebTransport relay; `sdk/` is the viewer SDK (`@dimos/sdk`: transport, session,
stores, decoders, React hooks); `cockpit/` is the browser app (Vite + React + TS) built on the SDK.
The Python mirror + WebTransport client live in `dimos/web/relay_bridge/`.

Everything runs on Deno 2.6.10, pinned in `dimos/utils/deno.py` (CI reads the pin from there). No
node/npm anywhere: vite, vitest, and tsc run as npm packages under Deno (`nodeModulesDir: auto`),
and `dimos --local-relay` auto-downloads Deno via `ensure_deno()`.

```bash
deno task dev            # relay on http://127.0.0.1:7780 (add --cockpit-dir cockpit/dist for the UI,
                         # --sdk-dir sdk/dist for /sdk.js, --serve-dir DIR for a custom page at /)
deno task test           # relay + shared tests (unit + loopback e2e)
deno task check          # type-check relay + shared; deno fmt + deno lint for style (all of web/)
```

The local relay deliberately answers `/api/info`, `/api/stats`, `/sdk.js`, and served JavaScript
modules with wildcard CORS so any local origin (a Vite dev server, a `file:` page) can bootstrap
against it; a remotely reachable relay is a different, fail-closed mode (W10) and must not inherit
that. For the same reason `startRelay` refuses to bind a non-loopback host unless
`--unsafe-non-loopback` explicitly acknowledges it (only sensible behind your own TLS and access
control).

## SDK

`sdk/` is the read-only viewer library the cockpit is built on: reconnecting WebTransport, session
state, channel/status stores, per-session decoder registries, refcounted subscriptions (`connect`,
`Session.watch/subscribe/close`), and React hooks on the `@dimos/sdk/react` subpath (the root entry
is React-free). The SDK subscribes to nothing by itself; the cockpit's panel policy lives in
`cockpit/src/subscriptions.ts`.

```bash
cd sdk
deno task test           # vitest
deno task check          # tsc --noEmit
deno task build          # dist/sdk.js, the zero-build ESM bundle the relay serves at /sdk.js
deno task fixture        # demo consumer on http://localhost:5174 (run a relay first)
```

`fixture/` is a minimal non-cockpit consumer: it lists robots, auto-watches the lone robot, and
subscribes to `odom` by hand. Run `dimos run <bp> --local-relay` and `deno task fixture` side by
side; like the cockpit dev server it proxies `/api` to the relay on `:7780`.

### Decoders

Each session owns a `DecoderRegistry` (pass one via `connect({decoders})`; the default is a fresh
registry with the built-ins). A decoder is
`(payload: Uint8Array, header: FrameHeader) => { value, preview? }`, looked up by the channel's
manifest `encoding`. Built-ins: `jpeg.v1`, `costmap.zlib.v1`, `json.v1`; any other `*.json.vN`
encoding JSON-decodes without registration. An exact registration wins over that convention, and a
duplicate `register()` throws unless `{ replace: true }`. An encoding with no decoder is not an
error: the channel still counts frames and renders as unsupported. A throwing decoder bumps the
channel's `decodeErrors`/`decodeFailing` stats and keeps the last good value. Keep decoders
synchronous and cheap - they run on the ingest path; panel-paced work (inflate, draw) belongs in the
consumer.

The Python half is `@web_encoder` (`dimos.web.codecs`) plus `Channel` (`dimos.web.cockpit`);
`examples/custom-path/` is the end-to-end pair for this exact codec:

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

### Zero-build page

`examples/minimal/` is a single HTML file importing `/sdk.js` - no bundler, no npm. Serve it from
the relay itself:

```bash
dimos run <bp> --local-relay --serve-dir web/examples/minimal
```

`--serve-dir` replaces the cockpit at `/` with the given directory (`/api/*` and `/sdk.js` keep
precedence, and the relay's traversal/symlink guards apply); it needs the spawned local relay and is
rejected with `--relay-url`. A page can also import the absolute `http://127.0.0.1:7780/sdk.js` and
pass that base to `connect({url})` - from another local origin or straight from a `file:` page (both
supported browsers permit WebTransport there; `dimos/e2e_tests/test_sdk_browser.py` pins all three
forms).

## Cockpit

```bash
cd cockpit
deno task dev            # vite dev server on http://localhost:5173 with HMR
deno task test           # vitest
deno task check          # tsc --noEmit
deno task build          # dist/ (what the relay serves at /)
```

Panels are authored in Python (`dimos.web.cockpit`: `Video`, `Map2D`, `Teleop`, `Chat`, `Stats`) and
compiled into the manifest; `cockpit(pages=[...])` panels render as full-page tabs in the header,
next to Overview and the panels/channels toggle. `Stats()` is dtop as a tab: the bridge re-encodes
the resource monitor's `/resource_stats` dict as `stats.json.v1`, and the blueprint switches the
monitor on (`GlobalConfig.dtop`) by itself.

Dev workflow: run the relay (`deno task dev` in `web/`, or just `dimos run <bp> --local-relay`) and
the vite server side by side. `localhost:5173` is a secure context; vite proxies `/api` to the relay
on `:7780` and the WebTransport connection goes straight to the advertised `wtUrl`.

Without vite, `--local-relay` serves the built `cockpit/dist` at `/` and `sdk/dist/sdk.js` at
`/sdk.js`, building both first when either is missing or older than the sources (`ensure_web_dist`
in `relay_process.py`; one stamp, both products swapped together). Release wheels ship both
pre-built dists inside `_relay_dist` (built by the release workflow; see `setup.py`), so a
pip-installed dimos never builds or downloads npm packages.

After changing cockpit or sdk dependencies run `deno install` in `web/` and commit the `deno.lock`
update; CI validates it with `deno install --frozen`. The cockpit's fonts (Inter, JetBrains Mono)
are npm packages (`@fontsource-variable/*`) bundled into `dist/` by vite, so nothing is fetched at
runtime. If vitest ever misbehaves under a new Deno, the fallback ladder is `--no-file-parallelism`,
then `--pool=threads`, then pinning a different vitest minor.

The browser e2e (`dimos/e2e_tests/test_cockpit_browser.py` for the cockpit, `test_sdk_browser.py`
for the SDK's zero-build/cross-origin/file: pages; marker `web_browser`) drives the whole stack
against the go2 replay dataset in both Playwright Chromium and Firefox (their WebTransport stacks
differ; see bug 11). The CI `web` job runs them; the pinned browser builds auto-install on first
run.

## Teleop safety chain

Keyboard teleop (the `teleop` panel; WASD drive, Q/E strafe, Shift boost, Space e-stop, key
semantics from `dimos/robot/unitree/keyboard_teleop.py`) sends `twist` datagrams cockpit -> relay ->
bridge, which publishes on `tele_cmd_vel`. Driving needs the per-robot exclusive lease
(`teleop_start` on the control stream, acked with `teleop_started`; a second viewer gets error
`teleop_held`); the relay forwards twist/stop datagrams robot-ward only from the lease holder.
Datagram loss is fine: commands repeat at the channel's `maxHz` and silence trips the bridge
deadman. Each hop covers the failure of the previous one:

1. The cockpit zeroes on every disarm trigger: last key release (zero + two repeats over 200 ms),
   Esc, focus loss, window blur, hidden tab, unmount, disconnect. Armed only while the panel has
   focus. Space is the e-stop: a `stop` burst on datagrams plus one on the control stream, which the
   bridge publishes as an unconditional zero (it also cancels an autonomous nav goal).
2. The relay releases the lease and sends `teleop_stop` robot-ward when the holder disconnects or
   watches away. It stamps every robot-bound teleop message with the lease generation `gen` (bumped
   per grant, announced with a robot-ward `teleop_start`).
3. The bridge watchdog is authoritative: it publishes `Twist.zero()` on stop, `teleop_stop`, session
   loss, or `watchdogMs` (default 300 ms) of twist silence - so a SIGKILLed relay stops the robot
   without any goodbye reaching either end. Older generations are rejected permanently and the seq
   high-water mark survives silence, so a released holder's delayed datagrams cannot restart motion
   after a stop. Zeros are edge-gated (only after a nonzero twist): MovementManager cancels the nav
   goal on every teleop message, so idle zeros must never repeat.

## Protocol shape, and why it is odd

The framing is defined once in `shared/protocol.ts`, mirrored in Python, and pinned by golden
vectors in `shared/fixtures/` (regenerate via
`deno run --allow-write=shared/fixtures shared/fixtures/gen.ts`; tested from both `deno test` and
pytest). The one exception is `costmap_frames.json`: its payloads pin the Python encoder's zlib
bytes, so it is generated by `uv run python -m dimos.web.relay_bridge.gen_costmap_fixtures`.

The transport per leg is deliberately asymmetric (the numbered workarounds below explain why):

| Leg             | What                          | Transport                                                                                                                                 |
| --------------- | ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| robot -> relay  | hello, publish acks           | `@control` data frame on a one-shot bidi stream (hello resent until welcomed)                                                             |
| robot -> relay  | channel data                  | one-shot bidi stream per frame                                                                                                            |
| robot -> relay  | ping                          | datagram                                                                                                                                  |
| relay -> robot  | welcome, errors, pong, teleop | datagrams (lossy; teleop is loss-tolerant by design)                                                                                      |
| relay -> robot  | subs snapshots, publishes     | `@control` (subs) and tx data frames (forwarded publishes) on the robot control carrier: ONE relay-opened reliable uni stream per session |
| viewer -> relay | control                       | viewer-opened bidi control stream (browser/SDK) or datagrams (Python test viewer)                                                         |
| relay -> viewer | control replies + pushes      | the same control stream, or datagrams                                                                                                     |
| relay -> viewer | channel data                  | relay-opened uni streams: per-frame for latest, one persistent per reliable channel                                                       |

Relay-opened uni streams are the proven direction on both legs: Deno's server->client uni delivery
works to browsers, Deno's own client, and aioquic alike; only client->Deno-server uni receive is
broken (bug 1).

Several choices are workarounds for upstream bugs, verified 2026-07-10..15 on Deno 2.6.10 + aioquic
1.3 (details and probes in the spike branch `paul/experiment/webtransport`):

1. **Robot data rides one-shot bidi streams, not uni.** Deno never delivers payloads of incoming uni
   streams (server-side receive; even from Deno's own client). Relay->viewer uni streams are
   unaffected.
2. **Every message is length-prefixed; EOF is never a message boundary.** Deno's `writer.close()`
   sends FIN lazily (~1 s, on GC). Receivers count bytes:
   `u32-LE headerLen | u32-LE payloadLen | header JSON | payload`.
3. **The relay never writes on robot-opened streams** and aborts its send half (RESET). aioquic
   parses server bytes on client-initiated bidi WT streams as H3 frames and kills the connection
   (H3_FRAME_UNEXPECTED). Relay->robot handshake and teleop control (welcome/error/pong/teleop)
   rides datagrams. Subs snapshots ride `@control` frames on the robot control carrier, one
   relay-OPENED reliable uni stream per robot session - the direction bug 1 does not break - so
   snapshots are ordered, unbounded by the old ~1200 B datagram budget, and never need a periodic
   resend. The carrier is a control dependency: a relay-side write failure or overflow fails the
   whole robot session (`carrier_failed` error + close), and the bridge treats corrupt carrier
   framing, a carrier reset, or a carrier end while the connection lives the same way (the relay
   never replaces a carrier); either way the bridge reconnects and the fresh registration
   re-baselines subs. Since protocol v5 the robot's hello rides an `@control` data frame (payload =
   the datagram encoding, capped at 64 KiB) on a fresh one-shot bidi stream, resent until the lossy
   welcome datagram lands - which freed the manifest from the ~1100 B datagram budget. Channel ids
   beginning with `@` are reserved for control frames and never forwarded to viewers.
4. **aioquic must set `max_datagram_frame_size=65536`** or the session dies at SETTINGS time.
5. **Relay installs an `unhandledrejection` guard** (deno#28406) or it dies ~30 s after a browser
   tab closes.
6. **WT URLs use `https://127.0.0.1`, never `localhost`** (Chrome resolves localhost to ::1 first;
   the endpoint binds IPv4).
7. **Relay->viewer uni streams use `waitUntilAvailable` + decreasing `sendOrder`.** Without the
   former, a slow page exhausts stream credit and the create call throws; without the latter, quinn
   round-robins in-flight streams and completions arrive in ~1 s waves.
8. **Reading incoming streams server-side needs a BYOB reader**; default readers never deliver on
   Deno 2.6.10.
9. **aioquic `reset_stream()` on an already-discarded stream corrupts the stream-id allocator**
   (`_get_or_create_stream_for_send` recreates the stream and rewinds `_local_next_stream_id_*`, so
   the next stream reuses a FIN'd id). The bridge only resets ids still present in `_quic._streams`,
   checked and reset in the same event-loop turn.
10. **The relay accepts robot data streams from the raw `Deno.QuicConn`, not
    `wt.incomingBidirectionalStreams`.** A reset that races stream acceptance (a stale latest-wins
    write reset before the relay read the stream's preamble; quinn discards buffered data on reset)
    makes the preamble read inside Deno's `incomingBidirectionalStreams` `pull` throw, which errors
    that ReadableStream permanently and silently kills the accept loop (`ext/web/webtransport.js`,
    still present on Deno main 2026-07). The QUIC-level accept only fails with the connection; the
    relay parses the WebTransport preamble itself (`readWebTransportPreamble`) and a bad/reset
    stream drops alone.
11. **Reliable channels ride ONE persistent uni stream per (viewer, channel), not a stream per
    frame** (verified 2026-07-24, Firefox 142/Playwright). Firefox grants a WebTransport session
    ~100 incoming uni streams and only replenishes the credit as streams complete - but the relay's
    FIN goes out lazily (bug 2), so with stream-per-frame the relay's
    `createUnidirectionalStream({waitUntilAvailable})` hangs after ~100 frames, the reliable FIFO
    overflows, and the relay kicks the viewer every ~8 s. Chromium's much larger window masks this.
    Latest channels keep per-frame streams (their reset semantics need them); bug 12 is what keeps
    their credit pressure bounded.
12. **Relay->viewer latest streams are never FIN'd: every one ends in a RESET.** JS WebTransport
    exposes no delivery signal (`getStats()` is a zeros stub in Deno 2.6.10) and quinn buffers
    writes without bound, so "write accepted" says nothing about delivery - and a closed
    WritableStream can no longer be aborted. The relay therefore keeps each latest stream open and
    reaps it: streams older than `LATEST_STALE_MS` (500 ms, matching the Python leg's `stale_after`)
    are reset, discarding buffered-but-undelivered bytes on both ends and returning Firefox's stream
    credit far faster than the lazy FIN would. Reaping fires from newer offers AND from a periodic
    reap every `LATEST_STALE_MS`: an idle input stops offering and would otherwise leave up to about
    100 open streams pinning Firefox's uni-stream credit. The one send still wedged in
    `createUnidirectionalStream` is never reset - resets do not replenish stream credit while the
    viewer's application is not reading (verified against Deno's client; a frozen tab behaves the
    same) - instead newer offers supersede its payload in place (the payload binds only when the
    write starts), so a resuming viewer receives the newest frame with zero stream churn. Once the
    write has started the payload can no longer change, so a newer offer resets a write-wedged
    carrier once it is `LATEST_STALE_MS` old and resends the newest on a fresh stream - bounded to
    one reset per stale window, and once stream credit runs out the wedge moves back to creation,
    where superseding is churn-free. In `/api/stats`, `aborted` counts backpressure resets - stale
    accepted streams reaped while the newest send is unaccepted, plus stale write-wedged carriers
    (the suspended-viewer signal; 0 when healthy); `expired` counts routine end-of-life resets (~=
    `sent` on a healthy latest channel); superseded and displaced payloads count as `dropped`.
    Receivers dispatch frames on byte count (bug 2) and treat the reset as end-of-stream; a reset
    mid-frame drops a stale partial by design.

Latest streams deliver out of order by design; consumers keep the newest frame by `seq` (a reliable
channel's persistent stream is ordered) and loss metrics are span-based
(`maxSeq - minSeq + 1 - received`).
