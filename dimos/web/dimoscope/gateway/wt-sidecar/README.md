# wt-sidecar — native WebTransport egress (Rust)

QUIC muscle, Python brain. With `WT_EXTERNAL=1` this crate owns ONLY the UDP `:8443` WebTransport
listener (instead of the in-process aioquic server, whose pure-Python packetization caps bulk at
~2.4 MB/s on a real WAN). The Python gateway keeps everything stateful and safety-critical — the single
bus tap (LCM + Zenoh), `SafetyEgress` (velocity clamp, TTL deadman, RPC whitelist), and topic
discovery — and feeds this process over a unix socket (`gateway/pipe.py`). The sidecar speaks the
**same WT wire protocol as the aioquic server**, so browser clients need zero changes; its hello label
is `dimoscope/WT-rs`.

## Run

```sh
deno task serve:wt-rs     # the gateway with WT_EXTERNAL=1 (pipe plane instead of aioquic)
deno task wt-sidecar      # this crate (cargo run --release)
```

Env: `WT_PORT` (8443) · `WT_PIPE` (/tmp/dimoscope-wt.sock) · `WT_CERT_HASH_FILE`
(/tmp/dimoscope-wt-cert.hash — the gateway's `/cert` serves this file, 503 until it exists) ·
`EGRESS_KBPS` (optional per-session drain pacer; unset = rely on QUIC flow-control backpressure).

## Design notes

- **Pipe framing** `[u32be len][u8 kind][payload]`: kind 1 = DATA (raw LC02 packet; topic/type live in
  its channel), kind 2 = JSON (hello/topic/rpc-res down; subs/teleop/stop/goal/rpc/disconnect up).
  DATA is stamped `[f64be now-ms]` once at pipe ingress — the same place aioquic stamps, so bench
  latency includes outbox queue time.
- **Safety**: teleop/goal/rpc are forwarded to the gateway's `SafetyEgress` keyed by this session's
  `sid`; a session closing sends `disconnect{sid}` (deadman cancel + robot stop), and the gateway
  stops ALL sids if the pipe itself drops.
- **QoS**: `src/outbox.rs` is a line-for-line port of `gateway/qos.py` (WRR weights, lane tuples,
  regex heuristic) — KEEP IN SYNC; qos.py is the source of truth. The operator glob-rule layer
  (`QOS_RULES`) is deliberately not ported: it is off by default and Python-side only.
- **Wire**: frames ≤1100 B go as QUIC datagrams, larger ride one persistent uni stream
  (`[u32be len]` prefix) set to a lower quinn priority than the control stream.

## Why Rust is in this repo

Bulk >2 MB/s and real stream priorities over one QUIC connection need a native QUIC stack (measured in
docs/benchmarks.md §3: 19.1 MB/s clean, 9.3 MB/s at 5% loss). The crate is isolated — no other part of
dimoscope depends on it; delete this directory and unset `WT_EXTERNAL` to fall back to the aioquic
server unchanged.
