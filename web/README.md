# DimOS web

Deno workspace for the robot web stack. `shared/` holds the wire protocol and its golden vectors;
the WebTransport relay (`relay/`, with `deno task dev`), its Python mirror + client
(`dimos/web/relay_bridge/`), and the Cockpit browser app arrive in follow-up PRs.

Everything runs on Deno 2.6.10, pinned in `dimos/utils/deno.py` (CI reads the pin from there).

```bash
deno task test           # unit tests
deno task check          # type-check; deno fmt + deno lint for style
```

## Protocol shape, and why it is odd

The framing is defined once in `shared/protocol.ts`, mirrored in Python, and pinned by golden
vectors in `shared/fixtures/` (regenerate via
`deno run --allow-write=shared/fixtures shared/fixtures/gen.ts`; tested from both `deno test` and
pytest).

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
   (H3_FRAME_UNEXPECTED). Robot-leg control (hello/welcome/ping/pong) rides datagrams instead; the
   robot retries hello until welcomed (datagrams are lossy).
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

One-stream-per-message delivers out of order by design; consumers keep the newest frame by `seq` and
loss metrics are span-based (`maxSeq - minSeq + 1 - received`).
