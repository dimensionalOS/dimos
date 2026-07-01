# Real-WAN testing: MacBook (client) ↔ VPS (robot/gateway) over the actual internet

This is the production topology in miniature: the **VPS** has a public IP and opens a port (= the
robot), your **Mac browser/CLI** connects out to it over the real internet (= the operator). Instead of
*simulating* bad networks with `netsim`, you measure the real path — and you can validate that the
`netsim` profiles (`bench/RESULTS-mechanisms*.md`) actually predict reality.

The JS SDK does **not** change: the app reads `?gw=host:port` and the headless bench reads
`GATEWAY_URL` — both already exist. You only point them at the VPS.

> **All five transports work by raw IP — no domain, no CA-issued TLS.** See the table below. A domain
> (via Coolify/Caddy) is only needed to serve the *app itself* over public `https`, not for testing.

---

## Transports over a raw IP (what each needs)

| transport | by IP? | requirement |
|---|---|---|
| **WebSocket** `ws://` | ✅ | TCP origin port open. Keep the **client page on `http://localhost`** (so it may reach `ws://`/`http://` on the VPS IP without a mixed-content block). |
| **SSE / HTTP-poll** `http://` | ✅ | same TCP port |
| **WebRTC DataChannel** | ✅ | WebRTC brings its **own DTLS** (self-signed, fingerprint in SDP) — no domain. Needs the VPS **UDP ports open**; a public STUN (or just the VPS public-IP host candidate) is enough — usually no TURN. |
| **WebTransport / QUIC** | ✅ | QUIC is always encrypted but needs **no CA/domain**: the service serves a self-signed **ECDSA (≤14-day)** cert and its hash at `/cert`; the client pins it via `serverCertificateHashes` (already wired: `main.tsx certHashUrl` ↔ service `/cert`). Needs **UDP :8443 open**. **Chrome/Edge only.** |

---

## 1. VPS — run the service (Ubuntu, has git/python/node/docker; needs `uv`)

```bash
# one-time
curl -LsSf https://astral.sh/uv/install.sh | sh           # install uv (python pkg mgr)
cd ~/code/dimos && git fetch && git checkout kris/research-29-06-2026 && git pull
uv sync --extra web        # light: zenoh + aioquic + aiortc (WS/SSE/poll/WebRTC/WebTransport).
                           # do NOT `uv sync --all-extras` — that pulls torch/mujoco (GBs, not needed).

# run the single service — ONE process: the app + every transport
cd dimos/web/dimoscope
uv run python -m gateway        # http://0.0.0.0:8080  (app / + /ws /sse /poll /rtc /media /cert) · QUIC UDP :8443
# + a synthetic data source so there's something to stream (or run a real robot / a sim):
DIMOS_TRANSPORT=lcm BENCH_HZ=100 uv run python bench/bench_publisher.py
```
(No deno needed on the VPS — the server is all Python now. The gateway taps **both** LCM and Zenoh, so
either `DIMOS_TRANSPORT` for the source works.)

### Open the firewall (the part people forget)
- **TCP**: the HTTP/WS origin port (serves `/ws` `/sse` `/poll` `/media` `/cert`).
- **UDP**: the WebRTC port range **and** `:8443` (WebTransport/QUIC).
- On this Coolify host that's the provider security-group + any `ufw`. (Docker already maps ports like
  minio :9000 / postgres :54324, so the pattern is in place.)

## 2. Mac — point the existing tools at the VPS (zero new code)

```bash
# headless numbers (WS data plane is on the /ws path now):
GATEWAY_URL=ws://37.60.232.68:8080/ws deno task bench

# real browser, all 5 transports (Chrome):
deno task app                                   # Vite on :5173 (the app is served from your Mac)
#   open  http://localhost:5173/?gw=37.60.232.68:8080
#   then flip the topbar dropdown: WebSocket · SSE · HTTP poll · WebRTC data · WebTransport
# (or, if app/dist was built on the VPS, just open http://37.60.232.68:8080/ directly — same-origin)
```

## 3. Read it
Put the VPS numbers next to the localhost `netsim` table (`bench/RESULTS-mechanisms*.md`): real RTT to
the VPS region vs `netsim 3g/4g`, and which transports stay smooth. That real-vs-simulated comparison
is the client-credible result. (Save it as `bench/RESULTS-realwan.md`.)

---

## Caveats
- **Mixed content:** the above assumes the app is served from `http://localhost`. If you ever serve the
  app over `https`, the browser blocks `ws://`/`http://` to the VPS IP → you then need `wss://`/`https`
  → a domain + TLS (the Coolify appendix).
- **WebTransport:** Chrome/Edge only; the self-signed cert is **ECDSA, ≤14 days** — the service
  regenerates it, and the client always fetches the current hash from `/cert`.
- **WebRTC NAT:** the VPS public IP is a directly-reachable host candidate, so browser→VPS usually
  connects with just a public STUN and no TURN. If a restrictive network blocks it, add coturn (later).
- **Install weight:** stick to `--extra web`. The ML/sim extras (torch, mujoco) are irrelevant to a
  network test and huge.

## Appendix — optional public endpoint via Coolify + Caddy
Only if you want clients to hit a real `wss://dimos.<domain>` (app over https, valid CA cert, no
hash-pinning): a small Dockerfile (`uv sync --extra web` → `python -m gateway`) deployed as a
Coolify app; Caddy fronts TCP with auto Let's Encrypt; map **UDP :8443** through for QUIC. Not required
for the testing above.

---

## Currently deployed (2026-07-01 benchmark session) + TEARDOWN

A live benchmark deployment is running on the VPS so you can open it in a browser. **Close it with the
steps below when done.**

**What's deployed (VPS `37.60.232.68`, user `kristjan`):**
- Isolated clone `~/dimos-bench` (branch `kris/research-29-06-2026`) — nothing else on the box touched.
- the gateway + a single `bench_source` running in **tmux session `dimos-realwan-65329`** (serve on
  `:8080`, WT on `:8443`); source PID in `~/realwan-source.pid`; logs `~/realwan-serve.log` /
  `~/realwan-source.log`.
- **ufw** rules added: `8080/tcp` + `8443/udp` (only these two — other rules e.g. 6001/6002/2222 belong
  to other work, leave them).
- Results in `~/dimos-bench/dimos/web/dimoscope/bench/RESULTS-*.md`; agent status file
  `~/DIMOS-BENCH-AGENT-65329a9d.md`.
- Inspect anytime: `ssh -i ~/.ssh/vps-coolify kristjan@37.60.232.68` then `tmux ls` /
  `tmux attach -t dimos-realwan-65329`.

**Open it in your browser now:**
- Local same-origin (works fully): `http://localhost:8080/bench.html` (needs the local serve; or the VPS
  one via the line below).
- Real-WAN (SSE/poll today): `deno task app` on the Mac → `http://localhost:5173/?gw=37.60.232.68:8080`.

**TEARDOWN (one block, reversible, non-destructive):**
```bash
ssh -i ~/.ssh/vps-coolify kristjan@37.60.232.68 '
  tmux kill-session -t dimos-realwan-65329 2>/dev/null   # stop the gateway
  kill $(cat ~/realwan-source.pid) 2>/dev/null           # stop the source
  pkill -f "dimos-bench.*gateway"; pkill -f "dimos-bench.*bench_source"   # belt-and-suspenders (scoped to ~/dimos-bench)
  sudo tc qdisc del dev lo root 2>/dev/null               # remove any leftover netem
  sudo ufw delete allow 8080/tcp; sudo ufw delete allow 8443/udp; sudo ufw delete allow 32768:60999/udp   # close the ports (incl. WebRTC ICE range)
  rm -rf ~/dimos-bench ~/realwan-*.log ~/realwan-source.pid ~/DIMOS-BENCH-AGENT-65329a9d.md
  cd ~/code/dimos && git checkout -- . 2>/dev/null       # undo the earlier v1 rsync overlay on main
'
# uv + deno installs on the VPS are harmless to leave; remove with: rm -rf ~/.local/bin/uv ~/.deno
```
Nothing here touches the Coolify containers. `kill`/`pkill` are scoped to `~/dimos-bench` so they won't
hit other agents' processes.
