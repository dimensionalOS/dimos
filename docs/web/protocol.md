# Wire protocol

One protocol connects the bridge, the relay and the browser. It is defined once in [`web/shared/protocol.ts`](/web/shared/protocol.ts), mirrored in [`dimos/web/relay_bridge/protocol.py`](/dimos/web/relay_bridge/protocol.py), and pinned by golden fixtures in `web/shared/fixtures/` that both test suites check. The version is 7, and there is no compatibility mode: a client with another version gets `version_mismatch` and is closed. Nothing on this page is needed to use the cockpit or the SDK. It is for people changing the relay, the bridge or the SDK, and it records why the transport looks the way it does.

## Framing

Three framings, all over WebTransport:

- Control stream frame: `u32-LE length | UTF-8 JSON`. The viewer's bidirectional control stream.
- Datagram: raw UTF-8 JSON, no prefix. Teleop commands, and the relay's replies to the robot.
- Data frame: `u32-LE headerLen | u32-LE payloadLen | header JSON | payload`. Channel data in both directions, and robot-side control (`@control`).

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/data_frame.svg
color = white
fill = none
boxht = 0.55in
margin = 0.06in

A: box "headerLen" "u32, 4 bytes, LE" wid 1.5in
B: box "payloadLen" "u32, 4 bytes, LE" wid 1.5in with .w at A.e
C: box "header JSON" "headerLen bytes" wid 1.5in with .w at B.e
D: box "payload" "payloadLen bytes" wid 1.9in with .w at C.e
```

</details>

![A data frame in wire order: a 4-byte little-endian header length, a 4-byte little-endian payload length, the header JSON, then the payload](assets/data_frame.svg)

The header is `{ch, seq, ts, delivery, meta?}`: the channel id, a per-channel sequence number, a timestamp in seconds, `latest` or `reliable` (the manifest wins when the channel is declared there), and the encoder's meta, which is optional. The widths in the picture are not to scale. Limits: a header is at most 64 KiB, a frame 64 MiB, a control payload 64 KiB, and a published value 32 KiB. Receivers count bytes and never treat end-of-stream as a message boundary (workaround 2 below). Channel ids beginning with `@` are reserved for control frames and are never forwarded to viewers.

```python
from dimos.web.relay_bridge.protocol import FrameHeader, decode_data_frame, encode_data_frame

frame = encode_data_frame(FrameHeader(ch="odom", seq=7, ts=1.5, delivery="reliable"), b"payload")
print(frame[:8].hex(), len(frame))
decoded = decode_data_frame(frame)
print(decoded.header.ch, decoded.header.seq, decoded.payload)
```

```results
3400000007000000 67
odom 7 b'payload'
```

## Sessions and messages

A robot connects to `/robot`, a viewer to `/viewer`. Both start with `hello`, which carries the protocol version and the role (and a token when the relay has an auth file). The relay answers `welcome` or an `error` and closes.

The robot's `hello` also carries its `{id, name, model}` and its manifest. It is sent as an `@control` data frame on a one-shot bidirectional stream, and the bridge resends it until the relay's `welcome` datagram arrives. The relay then opens the carrier: one persistent reliable unidirectional stream to each registered robot. Everything else the relay says to the robot is a datagram (`welcome`, `error`, `pong`, `teleop_start {gen}`, `teleop_stop {gen}`, `twist`, `stop`). The carrier carries `subs {chs, n}` snapshots (the set of channels some viewer wants, numbered so a stale one is ignored) and the viewers' published values as tx data frames. Channel data from the robot rides bidirectional streams: a `reliable` channel's frames back to back on one persistent stream per channel (v8), a `latest` frame on a one-shot stream of its own. The bridge's `pub_ack` and `pub_nack` are `@control` frames on one-shot streams like hello, as are its `rtc_offer` and `rtc_stalled` (v7). The relay's `rtc_ice` and `rtc_answer` ride the carrier. The robot pings with datagrams: one every 3 s once registered, and three unanswered in a row end the session on the bridge's side. The relay pings a robot whose id a new session claims and the bridge answers; no pong within a second means the holder is dead or frozen, and the new session supersedes it.

A viewer opens one bidirectional control stream and sends length-prefixed JSON on it:

| Viewer sends | Relay answers |
|---|---|
| `hello {v, role: "viewer", token?}` | `welcome {v}`, then `robots {robots}` whenever the robot list changes |
| `watch {robotId}` | `manifest {robotId, manifest}` |
| `sub {ch}`, `unsub {ch}` | channel data on relay-opened unidirectional streams, frames back to back: one per latest channel, one per bulk reliable channel, one shared by the other reliable channels |
| `ack {ch, seq}` (v8) | nothing. The newest `latest` frame the viewer has read on the channel; the relay sends at most a budget of frames a viewer has not acknowledged (two, growing to ten with the viewer's delivery rate, never more than half a second of it) |
| `ping` | `pong {n, ts}`. The SDK pings every 3 s and reconnects after two unanswered pings |
| `teleop_start` | `teleop_started`, or `error teleop_held` |
| `teleop_stop` | nothing (idempotent) |
| `twist {vx, vy, wz, seq, ts}`, `stop {seq, ts}` (datagrams) | forwarded to the robot with the lease generation when the sender holds the lease, dropped otherwise |
| `pub {id, ch, data, clientTs?}` | `pub_ack {id, ch, relayTs, bridgeTs}`, or `error {code, message, requestId}` |
| `rtc_offer {sdp}` (v7, after the relay's `rtc_ice {iceServers}`) | `rtc_answer {sdp}`. Every later pull arrives as a relay `rtc_offer {sdp, tracks, robotId}` that the viewer answers with `rtc_answer {sdp}`. See [WebRTC video](#webrtc-video) |

Error codes: `hello_required`, `version_mismatch`, `role_mismatch`, `missing_robot_id`, `auth_failed`, `invalid_manifest`, `rtc_unavailable` (a track channel on a relay without `--rtc-file`), `hello_mismatch` (a repeated hello changed identity or manifest), `robot_id_conflict` (the id's holder answered the relay's probe ping), `robot_superseded` (sent to a holder that did not, as a new session took its id), `invalid_control`, `control_too_large`, `carrier_failed`, `unknown_robot`, `unknown_channel`, `no_watch`, `teleop_held`, `duplicate_request`, `publish_too_large`, `not_publishable`, `pending_limit`, `rate_limited`, `publish_timeout`, `robot_disconnected`, `rtc_failed`, `rtc_session_gone`. `auth_failed` is terminal on both sides. An invalid message is dropped, and corrupt framing kills only its stream.

## Delivery modes

On the robot leg a `latest` channel has a pool of three persistent streams with one frame in flight per stream (a loss on one never holds the others, and no stream is created per frame). The bridge waits for an acknowledgement before reusing a slot, so frames can arrive at the relay out of order, and a frame still unacknowledged with a newer one waiting has its stream reset once it is older than 500 ms, or than three times the channel's recent delivery time on a link slower than that, so a slow uplink still completes frames (a reset in the middle of a frame drops the partial by design, and the slot opens a fresh stream). Consumers keep the newest frame by `seq`. A `reliable` channel's frames ride one persistent stream per channel, in order and never shed, except that a stream holding over 16 MiB the relay has not acknowledged is reset (dropping what it held, logged) and the channel continues on a fresh one.

On the viewer leg every channel has one persistent stream per viewer with frames packed back to back (workaround 12). The relay writes a `latest` frame only while fewer than the budget's worth of frames of that channel await the viewer's `ack` (v8): two frames, growing with the rate the viewer's acks show it delivers, up to ten and never more than half a second of that rate, so a long round trip does not cap a fast link at two frames per round trip while the backlog stays under half a second. It keeps at most one pending frame beyond that: a frame that arrives while the budget is spent replaces the pending one, so a slow viewer sees only the newest, a slow link carries at most the budget of backlog, and a viewer whose acks stop (a suspended tab) holds at most the budget's frames. An ack that never comes releases its frame after 10 s (`ackTimeouts` in `/api/stats`), a safety net rather than pacing: releasing earlier on a slow link only rebuilds the backlog the budget bounds. The acks also feed the robot's rate. A viewer whose pending frame the relay keeps replacing (more than a tenth of what it is offered over the trailing 5 s) is behind, and the frames its acks release per second are its rate. Every 500 ms the relay drops a latest channel's cap to the slowest such viewer's rate, or raises it by a quarter toward the manifest's `maxHz` when no viewer is behind, and sends the robot `rate {ch, hz}` on the carrier when the cap moves by more than a tenth; the bridge samples the channel at that rate and lowers a JPEG channel's quality with it (square root of the share of `maxHz`, never under 30), so a slow link no longer costs the robot the encodes it sheds and carries more of the frames it does get. A viewer counts once it has acknowledged for a full 5 s and within the last 2 s. A `reliable` channel queues up to 64 frames or 16 MiB per viewer and channel (a lone frame over the byte cap still queues) and the relay disconnects a viewer that overflows it.

## Manifest

Manifest version 1 is `{version, channels, panels, layout, pages}`:

- A channel is `{ch, dir, encoding, delivery, maxHz, params, publish, requiredScope}`. Ids are at most 64 characters and must not start with `@`.
- A panel is `{id, kind, title, channels, params}`. Panel channels are indexes into the channel list by id.
- A layout node is a panel id, `{row: [...], shares?}` or `{col: [...], shares?}`. `pages` is a list of panel ids.

Panel kinds are validated by their channels: `video` (one `jpeg.v1` or `video.webrtc.v1` latest rx), `map2d` (a `costmap.zlib.v1` latest rx and optionally a `pose.json.v1` rx), `map3d` (the same with a `voxels.zlib.v1` latest rx), `teleop` (one `twist.json.v1` latest tx), `chat` (four, in order: `text.json.v1` reliable shared tx, `chat.json.v1` reliable rx, `json.v1` latest rx, `audio.json.v1` reliable shared tx), `stats` (one `stats.json.v1` latest rx). Unknown kinds pass through, and the cockpit renders them as unknown panels. The robot's `{id, name, model}` is not part of the manifest. It travels in `hello`. An unsupported manifest version makes the SDK report `manifestUnsupported`.

## Transport per leg

The transport is deliberately asymmetric. The numbered workarounds below explain why.

| Leg | What | Transport |
|---|---|---|
| robot to relay | hello, publish acks | `@control` data frame on a one-shot bidirectional stream (hello resent until welcomed) |
| robot to relay | channel data | bidirectional streams: one persistent per reliable channel, a pool of three persistent per latest channel (one frame in flight each) |
| robot to relay | ping | datagram |
| relay to robot | welcome, errors, pong, teleop | datagrams (lossy, and teleop is loss-tolerant by design) |
| relay to robot | subscription snapshots, publishes, rate feedback | `@control` and tx data frames on the carrier, the one relay-opened reliable unidirectional stream per robot session |
| viewer to relay | control | viewer-opened bidirectional control stream (browser and SDK), or datagrams (the Python test viewer) |
| relay to viewer | control replies and pushes | the same control stream, or datagrams |
| relay to viewer | channel data | relay-opened unidirectional streams: one per latest channel, one per bulk reliable channel (`jpeg.v1`, `costmap.zlib.v1`, `voxels.zlib.v1`), one shared by the viewer's other reliable channels |
| robot to relay | rtc_offer, rtc_stalled (v7) | `@control` data frame on a one-shot bidirectional stream, like hello |
| relay to robot | rtc_ice, rtc_answer (v7) | `@control` frames on the carrier |
| viewer and relay | rtc_ice, rtc_offer, rtc_answer (v7) | the viewer's control stream (the SFU carries the media itself) |

Relay-opened unidirectional streams are the proven direction on both legs. Deno's server-to-client unidirectional delivery works to browsers, to Deno's own client and to aioquic alike. Only client-to-Deno-server unidirectional receive is broken (workaround 1).

## Workarounds for Deno and aioquic

Verified on Deno 2.6.10 and aioquic 1.3. Code comments cite these by number, so the numbering stays stable.

1. Robot data is sent on bidirectional streams, not unidirectional ones. Deno never delivers the payload of an incoming unidirectional stream on the server side, not even from Deno's own client. Relay-to-viewer unidirectional streams are unaffected.
2. Every message is length-prefixed, and end-of-stream is never a message boundary. Deno's `writer.close()` sends FIN lazily (about a second later, on garbage collection). Receivers count bytes.
3. The relay never writes on a robot-opened stream, and aborts its send half with a RESET.

    What fails: aioquic parses server bytes on client-initiated bidirectional WebTransport streams as HTTP/3 frames and kills the connection (`H3_FRAME_UNEXPECTED`).

    The workaround: the relay's replies to the robot (welcome, error, pong, teleop) are datagrams. Subscription snapshots and published values go on the carrier, one relay-opened reliable unidirectional stream per robot session, the direction that item 1 does not break. Snapshots are therefore ordered, unbounded by the datagram size, and never need a periodic resend. The robot's hello is an `@control` data frame (the payload is the datagram encoding, capped at 64 KiB) on a fresh one-shot stream, resent until the lossy welcome datagram lands, which frees the manifest from any datagram size budget.

    On failure: the carrier is a control dependency. A write failure or an overflow on the relay side fails the whole robot session (`carrier_failed`), and the bridge treats corrupt framing, a reset, or an early end of the carrier the same way. Either way the bridge reconnects, and the fresh registration re-baselines the subscriptions.

4. aioquic must set `max_datagram_frame_size=65536`, or the session dies at SETTINGS time.
5. The relay installs an `unhandledrejection` guard (Deno issue 28406), or it dies about 30 s after a browser tab closes.
6. WebTransport URLs use `https://127.0.0.1`, never `localhost`. Chrome resolves `localhost` to `::1` first, and the endpoint binds IPv4.
7. Relay-to-viewer unidirectional streams use `waitUntilAvailable` and a `sendOrder` per band: control 2, reliable 1, latest 0, so a saturated video channel never delays a pose or a reply. Without `waitUntilAvailable` a page that has used up its stream credit makes the create call throw; with it the create waits. A create still pending after 1 s means the viewer's credit is exhausted (every client allows about 100 incoming streams): the relay counts it (`streamCreditStalls` in `/api/stats`) and forces a garbage collection, which is what returns the credit of an unsubscribed channel's stream on Deno 2.6.10 (item 12), and a create still pending after 10 s disconnects the viewer (`stream credit exhausted`).
8. Reading incoming streams on the server side needs a BYOB reader. Default readers never deliver on Deno 2.6.10, and a BYOB read into a view at a non-zero offset never resolves, so the relay reads every chunk into a whole buffer and copies it into place.
9. aioquic's `reset_stream()` on an already-discarded stream corrupts the stream-id allocator (`_get_or_create_stream_for_send` recreates the stream and rewinds `_local_next_stream_id_*`, so the next stream reuses a finished id). The bridge only resets ids still present in `_quic._streams`, checked and reset in the same event-loop turn.
10. The relay accepts robot data streams from the raw `Deno.QuicConn`, not from `wt.incomingBidirectionalStreams`. A reset that races stream acceptance (a stale latest-wins write reset before the relay read the stream's preamble, and quinn discards buffered data on reset) makes the preamble read inside Deno's `pull` throw, which errors that ReadableStream permanently and silently kills the accept loop. The QUIC-level accept only fails with the connection. The relay parses the WebTransport preamble itself, and a bad or reset stream drops alone.
11. Every latest channel and every bulk reliable channel (`jpeg.v1`, `costmap.zlib.v1`, `voxels.zlib.v1`) uses one persistent unidirectional stream per viewer, not a stream per frame, and a viewer's other reliable channels share one stream, whole frames interleaved and dispatched by the header's `ch`, so a viewer costs about one stream per panel rather than one per channel. The small channels sharing a stream queue behind each other, which is the trade for the budget below. Browsers and Deno's client all grant a WebTransport session about 100 incoming unidirectional streams (Chromium's and Firefox's defaults are both 100) and replenish the credit only as streams end. The relay's FIN goes out lazily (item 2), so with a stream per frame `createUnidirectionalStream({waitUntilAvailable})` hangs after about 100 frames, the reliable queue overflows, and the relay kicks the viewer every 8 s or so.
12. Relay-to-viewer streams are never finished with FIN, and on Deno 2.6.10 they cannot be reset either.

    What fails: JavaScript WebTransport exposes no delivery signal (`getStats()` is a zeros stub in Deno 2.6.10), and `WebTransportSendStream.close()` and `abort()` never reach QUIC: the stream is delivered in full and ended with FIN only when V8 finalizes its wrapper. Each stream also holds about 400 B of heap until its connection closes. A stream per latest frame therefore leaked 10 MB per minute per watched camera, needed a forced GC every 500 ms to return viewer credit (growing to 25 ms pauses), and cost 60 us of main-thread time per frame and viewer.

    The workaround: one persistent stream per channel, latest channels included, with frames packed back to back. The relay keeps one pending frame per latest channel and writes it as soon as the previous write is accepted. `write()` resolves when quinn took the bytes into the stream's send window, which the viewer's reads replenish, so a viewer that stops reading (a suspended tab) stalls the write once its window is full while newer frames replace the pending one. When it resumes it reads the window's worth of stale frames oldest first and keeps the newest by seq; the relay cannot discard what it already buffered. In `/api/stats`, `stalled` counts writes accepted later than 500 ms (`LATEST_STALL_MS`) and `writing` says whether one is pending. An unsubscribed channel's stream goes back to the viewer's pool and carries its next channel at that priority (frames carry `ch`, receivers route by it), since an abandoned stream keeps its native state in quinn for the connection's life; only a stream with a write still in flight is aborted, which ends it at the next finalization. The robot leg's latest frames ride a pool of persistent streams per channel too (one frame in flight per stream), so neither leg creates a stream per frame and the only one-shot streams left are the robot's rare `@control` frames, whose slots in the relay's incoming-stream budget (raised to 10,000) return at finalization. The relay forces a garbage collection only at the credit stall of item 7 (`gcRuns` and `gcPauseMs` in `/api/stats`). The ~400 B of heap a stream wrapper leaks for the connection's life is now paid per channel, not per frame.

## WebRTC video

Protocol v7 adds `rtc_ice`, `rtc_offer` and `rtc_answer` for `video.webrtc.v1` track channels. A relay started with `--rtc-file` (a Cloudflare Realtime SFU app id and secret, plus an optional TURN key) brokers one SFU session per peer and never touches media. It hands each peer its ICE servers in `rtc_ice` right after `welcome`, and again after every TURN credential refresh (hourly, half the credentials' lifetime): a peer's next peer connection uses the latest set it received, so a connection that dies when its TURN allocation expires with its credentials comes back with fresh ones. The robot offers once, with its sendonly video transceivers and a `tracks` list saying which mid carries which channel, and the relay answers. The viewer offers once with a recvonly transceiver and the relay answers. Every later pull of a subscribed track channel arrives as a relay-sent `rtc_offer` carrying the pulled tracks and the robot they belong to (`robotId`), which the viewer answers; a viewer that switched robots meanwhile answers it but ignores the tracks. The SFU collects a track after 30 s without media, across every session pulling it, so a bridge whose track carried no media that long sends `rtc_stalled {ch}` when its video resumes; the relay closes that track's pulls, declares it again and pulls it onto its viewers, who receive it as a new `rtc_offer`. An SDP is at most `MAX_SDP_LEN` (48 KiB) and an offer names at most `MAX_RTC_TRACKS` (8) tracks. A frame a bridge sends on a track channel anyway is dropped and counted (`rtc.framesOnTrack` in `/api/stats`), never forwarded. A relay without Cloudflare rejects a robot that declares a track channel (`rtc_unavailable`). A failed Cloudflare call surfaces to the viewer as the non-terminal `rtc_failed` error and the relay retries it with backoff; `rtc_session_gone` means the viewer's SFU session no longer exists and the viewer must send a new `rtc_offer`.

## Golden fixtures

`web/shared/fixtures/` pins the wire bytes from both sides. `control_frames.json`, `datagrams.json`, `data_frames.json` and `manifests.json` are generated from TypeScript:

```bash
cd web
deno run --allow-write=shared/fixtures shared/fixtures/gen.ts
```

Three files pin Python encoder output and are generated from Python. `costmap_frames.json` and `voxel_frames.json` hold the zlib bytes of the costmap and voxel encoders, and `lcm_frames.json` holds `lcm_encode()` bytes plus the exported schemas:

```bash
uv run python -m dimos.web.relay_bridge.gen_costmap_fixtures
uv run python -m dimos.web.relay_bridge.gen_voxel_fixtures
uv run python -m dimos.web.relay_bridge.gen_lcm_fixtures
```

`deno test`, vitest and pytest all read them. A wire change regenerates them on both sides in the same commit.
