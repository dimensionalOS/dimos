# WebTransport spike (throwaway)

Tests the cockpit plan's riskiest bet (T1): Python (aioquic) -> Deno relay ->
browsers over WebTransport, with real Go2 recordings (video/odom/lidar from
`go2_short`). All code here is disposable; the findings below are the point.

## Verdict

WebTransport works end to end at real robot data rates, **but both the Deno
server API and the aioquic client have sharp edges that dictate protocol
design** (see findings). Browser-side (Chrome, Firefox) it is solid.

## Run

```bash
# terminal 1: relay (Deno 2.6.10, HTTP :8000 + QUIC/WT :4433)
cd experimental/webtransport_spike && deno run -A --unstable-net relay/main.ts

# terminal 2: bridge (real replay data; --synthetic for fake data)
uv run --with aioquic python experimental/webtransport_spike/bridge.py

# browser(s)
google-chrome http://localhost:8000/    # and/or firefox
# machine-readable stats the page reports: curl localhost:8000/api/reports

# after editing cockpit/main.ts:
deno bundle --platform browser -o relay/static/main.js cockpit/main.ts
```

WASD on the page sends teleop datagrams to the bridge (logged there). The page
shows per-channel Hz / KB/frame / lost / out-of-order, datagram RTT
(browser->relay->bridge->back), video, odom trace, lidar top-down.

## Results (2026-07-10, this machine)

| Check | Result |
|---|---|
| aioquic 1.3.0 -> Deno 2.6.10 WT session | works: `:status 200`, `sec-webtransport-http3-draft: draft02` |
| Robot data at real rates | video 14.5 Hz jpeg 1280x720 (~1 MB/s), odom 19 Hz, lidar 8 Hz 25k pts (~2.2 MB/s) |
| Chrome <-> relay (serverCertificateHashes) | works, no flags (Chrome 147) |
| Firefox <-> relay | works (Firefox 150), zero loss |
| Loss robot->browser | 0 transport loss over 140 s at full rate (both browsers) |
| Datagrams viewer->robot (teleop 20 Hz) | works, 0 gaps single-viewer; 40 Hz aggregate with 2 viewers |
| Datagram RTT browser->bridge->browser | 0.5-1 ms loopback |
| Multi-viewer isolation | works: slow viewer (headless sw-render Chrome) shed 18/~4900 frames via the latest-wins policy; Firefox next to it lost 0 |
| Sustained run | 140 s, 2 viewers, ~3.2 MB/s: no session drops, no relay errors |
| In-browser render rate | video decoded+drawn at full 14.3/s (decode 3 ms, cycle 70 ms); arrival-gap max 82 ms after the FIN fix below |
| Safari | untested (no macOS box); MDN BCD says Safari 26.4+ |

## Findings (the actual deliverable)

Numbered by severity for the real T1-T3 implementation:

1. **Deno 2.6.10 bug: incoming WT *unidirectional* stream payloads never reach
   the app.** The stream objects arrive, reads hang forever until session
   close. Reproduced with Deno's own WT client against `upgradeWebTransport`
   (`relay/probe_client.ts`), default and BYOB readers, FIN'd or held-open.
   Deno's `webtransport.js` parses the `0x54 + session id` preamble with a
   BYOB reader, then re-wraps the underlying rid
   (`readableStreamForRid`); payload is lost in that handoff. The bidi path
   passes the original stream through and works.
   **Workaround: robot->relay data goes on one-shot *bidirectional* streams.**
   (Relay->browser uni streams are fine: the send side is unaffected, and
   Chrome/Firefox receive correctly.)

2. **Deno 2.6.10 bug: `writer.close()` on a QUIC/WT send stream never sends
   FIN promptly.** `SendStreamResource::close` in `ext/net/quic.rs` is an
   empty function - `quinn::SendStream::finish()` is never called; the FIN
   only escapes when the resource is dropped/GC'd, ~1 s later (Chrome netlog:
   stream data arrives instantly, its FIN 0.5-1.2 s later, in ~1.15 s waves).
   Any EOF-delimited message framing therefore completes in ~1 s clumps -
   this is what made video render at ~1 fps in BOTH Chrome and Firefox while
   arrival stats looked full-rate. **Workaround: length-prefix every message
   (`u32 headerLen | u32 payloadLen | header | payload`) and have receivers
   count bytes; never use stream EOF as a message boundary.** (The late FIN
   still closes the stream eventually, so streams don't leak.)

3. **aioquic limitation: it mis-parses server bytes on client-initiated bidi
   WT streams as HTTP/3 frames.** `create_webtransport_stream()` never
   registers the receive direction, so any relay reply (even a bare FIN risks
   the same path) hits `FrameUnexpected("DATA frame is not allowed in this
   state")` and aioquic closes the whole connection (H3_FRAME_UNEXPECTED,
   code 261). Workarounds baked into this spike:
   - the relay never writes on robot streams, and closes its send half with
     `writable.abort()` (RESET is invisible to aioquic's h3 layer; a FIN via
     `close()` is not) - this also releases QUIC stream credit;
   - no hello/welcome exchange on a stream; the robot hello is a datagram.

4. **aioquic config must set `max_datagram_frame_size=65536`** or the session
   dies at SETTINGS time (`H3_DATAGRAM requires max_datagram_frame_size`).

5. **deno#28406 is real**: without a global `unhandledrejection` guard the
   relay process dies ~30 s after any browser tab closes (idle-timeout
   rejection on an internal promise nobody holds).

6. **Use `https://127.0.0.1:...` not `localhost` in the WT URL**: Chrome
   resolves localhost to `::1` first, the Deno QUIC endpoint binds IPv4.
   With `serverCertificateHashes`, hostname verification is skipped anyway.

7. **Slow viewers are the relay's problem to survive**: a busy page stops
   granting stream credit; `createUnidirectionalStream()` then throws unless
   called with `{waitUntilAvailable: true}`. Combined with per-(viewer,
   channel) skip-if-busy, a slow viewer sheds its own frames and never stalls
   others. Symmetrically, the page must never render per-arrival: draw latest
   in a rAF loop, keep canvas work cheap (lidar is a putImageData pixel-buffer
   blit, not 25k fillRect calls), decode video in a latest-wins queue.

8. **One-stream-per-message delivers out of order** (by design - no
   head-of-line blocking). Consumers need latest-wins by `seq`; loss metrics
   must be span-based, not gap-based.

9. Interop handshake detail: Deno (web-transport-proto 0.2.7) accepts
   aioquic's draft-02 `ENABLE_WEBTRANSPORT` settings; response carries
   `sec-webtransport-http3-draft: draft02`. No settings fight.

## Implications for the real T1

- The T1 plan's "one unidirectional stream per message" data plane must become
  **one bidi stream per message on the Python leg** (or aioquic/Deno get fixed
  upstream first; the bugs are upstream-reportable with the probes here).
  Browser leg keeps uni streams.
- **Frame headers must carry an explicit payload length** (finding 2): message
  boundaries by byte count, never by stream EOF, as long as the relay runs on
  Deno 2.6.x.
- The control-plane bidi stream robot<->relay as specced (hello/welcome both
  ways) does not work with aioquic today; keep robot control on datagrams or
  make it strictly one-directional.
- Everything else in T1 (certs, /api/info, browser connect, datagrams, rates)
  is validated as designed.

## Files

- `relay/main.ts` - QUIC accept/upgrade, robot/viewer sessions by URL path,
  payload-blind fan-out, HTTP static + /api/info + /api/report(s), stats.
- `relay/cert.ts` - ephemeral ECDSA P-256 self-signed cert (9 days), SHA-256.
- `relay/probe_client.ts` - Deno-native WT client used to isolate finding #1.
- `cockpit/main.ts` - browser page (bundled to `relay/static/main.js`).
- `bridge.py` - aioquic WT client; ReplayConnection (or synthetic) pumps with
  per-channel latest-wins queues.
