#!/usr/bin/env bash
# WebTransport under REAL packet loss — the payoff, isolated. Boots the WebTransport stack with a
# pose-only (small-frame → all DATAGRAMS) load, then runs the headless aioquic probe through
# bench/netsim-udp.ts at rising loss. QUIC datagrams are never retransmitted (RFC 9221), so a lost one
# simply never arrives: delivered-datagram latency (dgP95) stays ~flat while throughput (hz) degrades
# ~linearly with loss — NO head-of-line blocking. Contrast: a reliable+ordered TCP stream (WebSocket)
# turns the same loss into retransmit→stall (see the bench:matrix 3g/2g columns: p95 1400–1900 ms).
#
# The relay runs at LOW latency (10 ms ± 5) on purpose, to isolate the effect of LOSS from the effect of
# latency/bandwidth (the lossy *profile* bundles 80 ms + 5 Mbps; here we want just the drops). No sudo /
# no dummynet — netsim-udp.ts drops real UDP datagrams in userspace.  → deno task bench:loss
#
# All hosts pinned to 127.0.0.1 so the QUIC client, the UDP relay and the aioquic server agree on IPv4
# (a localhost→::1 split would leave the client retransmitting into the void).
set -uo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="$REPO/.venv/bin/python"
DENO="$(command -v deno || echo "$HOME/.deno/bin/deno")"
cd "$HERE"

GW_PORT="${GW_PORT:-8090}"
WT_PORT="${WT_PORT:-8093}"
DUR="${DUR:-4}"
LOG="${TMPDIR:-/tmp}/dimoscope_loss"
mkdir -p "$LOG"

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

echo "[loss] gateway + pose-only load (4×PoseStamped @ 100Hz → ~400 datagrams/s, no streams)…"
GATEWAY_PORT="$GW_PORT" "$DENO" run -A servers/gateway.ts >"$LOG/gw.log" 2>&1 &
pids+=($!)
DIMOS_TRANSPORT=lcm BENCH_HZ=100 BENCH_IMG_HZ=0 BENCH_GRID_HZ=0 \
  PYTHONPATH=bench "$PY" bench/bench_source.py >"$LOG/pub.log" 2>&1 &
pids+=($!)
echo "[loss] warming up load (cold dimos import)…"
sleep 15
WEBTRANSPORT_HOST=127.0.0.1 WEBTRANSPORT_PORT="$WT_PORT" GATEWAY_URL="ws://localhost:$GW_PORT" \
  "$PY" servers/webtransport.py >"$LOG/wt.log" 2>&1 &
pids+=($!)
sleep 3

echo
echo "=== WebTransport (QUIC datagrams) under REAL packet loss — relay @ 10ms±5, varying drop% ==="
echo "    hz = datagrams/s delivered  ·  dgP95 = delivered-datagram p95 latency (ms)"
# baseline: direct to webtransport.py, no relay
out=$(WT_HOST=127.0.0.1 WT_PORT="$WT_PORT" DUR="$DUR" "$PY" bench/webtransport_client_probe.py)
printf "  %-22s %s\n" "0% (direct)" "$out"
# through the UDP relay at increasing real loss (latency forced low to isolate the drop effect)
i=0
for spec in "0.05:5%" "0.10:10%" "0.20:20%"; do
  plr="${spec%%:*}"; label="${spec##*:}"
  port=$((8193 + i)); i=$((i + 1))
  NETSIM_PLR="$plr" NETSIM_LATENCY_MS=10 NETSIM_JITTER_MS=5 \
    NETSIM_UDP_LISTEN="$port" NETSIM_UDP_TARGET="127.0.0.1:$WT_PORT" \
    "$DENO" run -A --unstable-net bench/netsim-udp.ts >"$LOG/udp_$i.log" 2>&1 &
  rpid=$!; pids+=("$rpid")
  sleep 1
  out=$(WT_HOST=127.0.0.1 WT_PORT="$port" DUR="$DUR" "$PY" bench/webtransport_client_probe.py)
  printf "  %-22s %s\n" "$label (netsim-udp)" "$out"
  kill "$rpid" 2>/dev/null || true
done

echo
echo "Takeaway: dgP95 stays ~flat as drop% rises (a lost datagram just doesn't arrive — no head-of-line"
echo "blocking); hz falls ~linearly with the drop rate. A reliable WebSocket/TCP stream instead turns loss"
echo "into retransmit→stall — bench:matrix shows WS p95 climbing to 1400–1900 ms on impaired links."
echo "(netsim-udp drops REAL datagrams; the TCP netsim can only model loss as jitter — TCP can't raw-drop.)"
