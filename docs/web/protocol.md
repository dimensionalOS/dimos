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

The robot's `hello` also carries its `{id, name, model}` and its manifest. It is sent as an `@control` data frame on a one-shot bidirectional stream, and the bridge resends it until the relay's `welcome` datagram arrives. The relay then opens the carrier: one persistent reliable unidirectional stream to each registered robot. Everything else the relay says to the robot is a datagram (`welcome`, `error`, `pong`, `teleop_start {gen}`, `teleop_stop {gen}`, `twist`, `stop`). The carrier carries `subs {chs, n}` snapshots (the set of channels some viewer wants, numbered so a stale one is ignored) and the viewers' published values as tx data frames. Channel data from the robot is sent on one-shot bidirectional streams, one per frame, and the bridge's `pub_ack` and `pub_nack` are `@control` frames sent the same way, as are its `rtc_offer` and `rtc_stalled` (v7). The relay's `rtc_ice` and `rtc_answer` ride the carrier. The robot pings with datagrams.

A viewer opens one bidirectional control stream and sends length-prefixed JSON on it:

| Viewer sends | Relay answers |
|---|---|
| `hello {v, role: "viewer", token?}` | `welcome {v}`, then `robots {robots}` whenever the robot list changes |
| `watch {robotId}` | `manifest {robotId, manifest}` |
| `sub {ch}`, `unsub {ch}` | channel data on relay-opened unidirectional streams: one per frame on a latest channel, one persistent stream per reliable channel |
| `ping` | `pong {n, ts}` |
| `teleop_start` | `teleop_started`, or `error teleop_held` |
| `teleop_stop` | nothing (idempotent) |
| `twist {vx, vy, wz, seq, ts}`, `stop {seq, ts}` (datagrams) | forwarded to the robot with the lease generation when the sender holds the lease, dropped otherwise |
| `pub {id, ch, data, clientTs?}` | `pub_ack {id, ch, relayTs, bridgeTs}`, or `error {code, message, requestId}` |
| `rtc_offer {sdp}` (v7, after the relay's `rtc_ice {iceServers}`) | `rtc_answer {sdp}`. Every later pull arrives as a relay `rtc_offer {sdp, tracks, robotId}` that the viewer answers with `rtc_answer {sdp}`. See [WebRTC video](#webrtc-video) |

Error codes: `hello_required`, `version_mismatch`, `role_mismatch`, `missing_robot_id`, `auth_failed`, `invalid_manifest`, `rtc_unavailable` (a track channel on a relay without `--rtc-file`), `hello_mismatch` (a repeated hello changed identity or manifest), `robot_id_conflict`, `invalid_control`, `control_too_large`, `carrier_failed`, `unknown_robot`, `unknown_channel`, `no_watch`, `teleop_held`, `duplicate_request`, `publish_too_large`, `not_publishable`, `pending_limit`, `rate_limited`, `publish_timeout`, `robot_disconnected`, `rtc_failed`, `rtc_session_gone`. `auth_failed` is terminal on both sides. An invalid message is dropped, and corrupt framing kills only its stream.

## Delivery modes

A `latest` channel sends each frame on its own stream, so frames can arrive out of order and a slow viewer sees only the newest. Consumers keep the newest frame by `seq`. The relay resets streams older than 500 ms, and a reset in the middle of a frame drops a stale partial by design (workaround 12).

A `reliable` channel uses one persistent, ordered stream per viewer and channel with frames packed back to back. The relay queues up to 64 frames or 16 MiB per viewer and channel (a lone frame over the byte cap still queues) and disconnects a viewer that overflows it.

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
| robot to relay | channel data | one-shot bidirectional stream per frame |
| robot to relay | ping | datagram |
| relay to robot | welcome, errors, pong, teleop | datagrams (lossy, and teleop is loss-tolerant by design) |
| relay to robot | subscription snapshots, publishes | `@control` and tx data frames on the carrier, the one relay-opened reliable unidirectional stream per robot session |
| viewer to relay | control | viewer-opened bidirectional control stream (browser and SDK), or datagrams (the Python test viewer) |
| relay to viewer | control replies and pushes | the same control stream, or datagrams |
| relay to viewer | channel data | relay-opened unidirectional streams: per frame for latest, one persistent per reliable channel |
| robot to relay | rtc_offer, rtc_stalled (v7) | `@control` data frame on a one-shot bidirectional stream, like hello |
| relay to robot | rtc_ice, rtc_answer (v7) | `@control` frames on the carrier |
| viewer and relay | rtc_ice, rtc_offer, rtc_answer (v7) | the viewer's control stream (the SFU carries the media itself) |

Relay-opened unidirectional streams are the proven direction on both legs. Deno's server-to-client unidirectional delivery works to browsers, to Deno's own client and to aioquic alike. Only client-to-Deno-server unidirectional receive is broken (workaround 1).

## Workarounds for Deno and aioquic

Verified on Deno 2.6.10 and aioquic 1.3. Code comments cite these by number, so the numbering stays stable.

1. Robot data is sent on one-shot bidirectional streams, not unidirectional ones. Deno never delivers the payload of an incoming unidirectional stream on the server side, not even from Deno's own client. Relay-to-viewer unidirectional streams are unaffected.
2. Every message is length-prefixed, and end-of-stream is never a message boundary. Deno's `writer.close()` sends FIN lazily (about a second later, on garbage collection). Receivers count bytes.
3. The relay never writes on a robot-opened stream, and aborts its send half with a RESET.

    What fails: aioquic parses server bytes on client-initiated bidirectional WebTransport streams as HTTP/3 frames and kills the connection (`H3_FRAME_UNEXPECTED`).

    The workaround: the relay's replies to the robot (welcome, error, pong, teleop) are datagrams. Subscription snapshots and published values go on the carrier, one relay-opened reliable unidirectional stream per robot session, the direction that item 1 does not break. Snapshots are therefore ordered, unbounded by the datagram size, and never need a periodic resend. The robot's hello is an `@control` data frame (the payload is the datagram encoding, capped at 64 KiB) on a fresh one-shot stream, resent until the lossy welcome datagram lands, which frees the manifest from any datagram size budget.

    On failure: the carrier is a control dependency. A write failure or an overflow on the relay side fails the whole robot session (`carrier_failed`), and the bridge treats corrupt framing, a reset, or an early end of the carrier the same way. Either way the bridge reconnects, and the fresh registration re-baselines the subscriptions.

4. aioquic must set `max_datagram_frame_size=65536`, or the session dies at SETTINGS time.
5. The relay installs an `unhandledrejection` guard (Deno issue 28406), or it dies about 30 s after a browser tab closes.
6. WebTransport URLs use `https://127.0.0.1`, never `localhost`. Chrome resolves `localhost` to `::1` first, and the endpoint binds IPv4.
7. Relay-to-viewer unidirectional streams use `waitUntilAvailable` and a decreasing `sendOrder`. Without the former, a slow page exhausts stream credit and the create call throws. Without the latter, quinn round-robins in-flight streams and completions arrive in one-second waves.
8. Reading incoming streams on the server side needs a BYOB reader. Default readers never deliver on Deno 2.6.10.
9. aioquic's `reset_stream()` on an already-discarded stream corrupts the stream-id allocator (`_get_or_create_stream_for_send` recreates the stream and rewinds `_local_next_stream_id_*`, so the next stream reuses a finished id). The bridge only resets ids still present in `_quic._streams`, checked and reset in the same event-loop turn.
10. The relay accepts robot data streams from the raw `Deno.QuicConn`, not from `wt.incomingBidirectionalStreams`. A reset that races stream acceptance (a stale latest-wins write reset before the relay read the stream's preamble, and quinn discards buffered data on reset) makes the preamble read inside Deno's `pull` throw, which errors that ReadableStream permanently and silently kills the accept loop. The QUIC-level accept only fails with the connection. The relay parses the WebTransport preamble itself, and a bad or reset stream drops alone.
11. Reliable channels use one persistent unidirectional stream per viewer and channel, not a stream per frame. Firefox grants a WebTransport session about 100 incoming unidirectional streams and replenishes the credit only as streams complete. The relay's FIN goes out lazily (item 2), so with a stream per frame `createUnidirectionalStream({waitUntilAvailable})` hangs after about 100 frames, the reliable queue overflows, and the relay kicks the viewer every 8 s or so. Chromium's much larger window masks this. Latest channels keep per-frame streams because their reset semantics need them, and item 12 keeps their credit pressure bounded.
12. Relay-to-viewer latest streams are never finished with FIN: every one ends in a RESET.

    What fails: JavaScript WebTransport exposes no delivery signal (`getStats()` is a zeros stub in Deno 2.6.10) and quinn buffers writes without bound, so "write accepted" says nothing about delivery, and a closed WritableStream can no longer be aborted.

    The workaround: the relay keeps each latest stream open and reaps it. Streams older than 500 ms (`LATEST_STALE_MS`, matching the Python leg's `stale_after`) are reset, which discards buffered but undelivered bytes on both ends and returns Firefox's stream credit far faster than the lazy FIN would. Reaping fires from newer offers and from a periodic reap every 500 ms, because an idle input stops offering and would otherwise leave about 100 open streams pinning Firefox's credit.

    The wedged send: the one send still stuck in `createUnidirectionalStream` is never reset, since resets do not replenish stream credit while the viewer's application is not reading (a frozen tab behaves the same). Instead, newer offers supersede its payload in place (the payload binds only when the write starts), so a resuming viewer receives the newest frame with no stream churn. Once a write has started the payload can no longer change, so a newer offer resets a write-wedged stream once it is 500 ms old and resends the newest on a fresh stream, bounded to one reset per stale window. Once stream credit runs out the wedge moves back to creation, where superseding is churn-free.

    In `/api/stats`, `aborted` counts these backpressure resets and `expired` counts routine end-of-life resets. Receivers dispatch frames on byte count (item 2) and treat the reset as end-of-stream.

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
