# PACK MIND — LIVE RUNBOOK (2 dogs / 2 laptops / AP-per-dog + Tailscale)

The on-site page for the 2-dog live demo. Documents the topology the README's "Live demo"
section does NOT cover: each Go2 stays in its own **AP mode**, each laptop is **dual-homed**
(Wi-Fi → its dog's AP, a USB-tether uplink → the internet), and the two laptops reach the
shared coordinator over a **Tailscale** tunnel across the internet.

> README's live section assumes one shared router + dogs in STA (rejected: no trusted net on
> site) and its `--robot-ip` flag is stale. `live.py`'s LAN_IP only works on one LAN. Neither
> fits this topology — use THIS page.

```
 Go2-A ──AP 192.168.12.1── Laptop-A(alpha) ── USB-tether ─→ INTERNET ─┐
                                │ runs coordinator :8090               │  Tailscale
                                │ tailscale 100.x.A                    │  100.64/10
 Go2-B ──AP 192.168.12.1── Laptop-B(bravo) ── USB-tether ─→ INTERNET ─┘
                                  PACK_COORDINATOR_URL=http://100.x.A:8090
```
Both dogs share IP `192.168.12.1` — two separate Wi-Fi L2 nets, each laptop sees only its
own dog. No conflict. The dog never touches Tailscale; only laptop↔laptop coordinator HTTP does.

### Honesty red line (from PLAN §4 — Q&A will probe this)
The live demo proves **embodiment + coordination + memory-outlives-the-robot** (no-overlap,
inheritance). It does **NOT** prove the coverage-speed A/B claim — that's the sim's job.
Narrate: "two bodies, one shared memory; lose one, the mission survives."

---

## 0. NIGHT BEFORE (cannot be debugged inside the demo window)

1. **USB-tether uplink on both laptops** (Wi-Fi is taken by the dog AP). A single-Wi-Fi Mac
   with no second uplink CANNOT do this topology.
2. **Tailscale on both, same tailnet.** From Laptop-B: `ping <Laptop-A-tailscale-ip>` over
   the internet (no dogs needed).
3. **Service order: uplink ABOVE Wi-Fi** (System Settings → Network → ⋯ → Set Service
   Order). Else macOS routes WAN through the dog AP → Tailscale dies. Verify
   `route -n get 8.8.8.8` shows the uplink iface.
4. **Cross-net smoke (no dogs)** — prove the ledger works over Tailscale end to end:
   ```bash
   # Laptop-A:
   uv run python -m dimos.experimental.pack_mind.pack_coordinator_server \
     --zones north,east,south,west --prefs "alpha:south,north,east,west;bravo:north,east,west,south"
   # Laptop-B (A's tailscale ip):
   uv run python -m dimos.experimental.pack_mind.mock_dog --dog bravo \
     --url http://<Laptop-A-tailscale-ip>:8090 --reset --target "red kit"
   ```
   Dashboard at `http://<A-tailscale-ip>:8090` animates from Laptop-B's browser → brain works.
5. **Per-laptop `~/.packmind.env`** pre-written (§2). Confirm `OPENAI_API_KEY` valid on uplink.
6. **Prefetch models** (insurance): `uv run python -m dimos.experimental.pack_mind.prefetch_live_models`.

---

## 1. MORNING — preflight each laptop FIRST (fix every FAIL before starting dogs)

```bash
# Laptop-A:
uv run python -m dimos.experimental.pack_mind.preflight \
  --role alpha --peer <Laptop-B-tailscale-ip> --coordinator http://127.0.0.1:8090
# Laptop-B:
uv run python -m dimos.experimental.pack_mind.preflight \
  --role bravo --peer <Laptop-A-tailscale-ip> --coordinator http://<Laptop-A-tailscale-ip>:8090
```

---

## 2. Per-laptop env (`~/.packmind.env`, `source` before anything)

```bash
# BOTH:
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export ROBOT_IP=192.168.12.1            # each laptop's OWN dog over its AP
export LISTEN_HOST=0.0.0.0
export OPENAI_API_KEY=<key>             # agent brain (hosted) — needs the uplink
# Laptop-A only:
export PACK_DOG_NAME=alpha
export PACK_COORDINATOR_URL=http://127.0.0.1:8090
# Laptop-B only:
export PACK_DOG_NAME=bravo
export PACK_COORDINATOR_URL=http://<Laptop-A-tailscale-ip>:8090
```
`dimos run` has **no `--robot-ip` flag** in this build — it reads `ROBOT_IP`; verify with
`dimos show-config`. `ROBOT_IP=192.168.12.1` = dog's-own-AP WebRTC path (one laptop per dog).

---

## 3. Bring-up (Laptop-A first)

**Laptop-A — coordinator + dashboard + dog alpha:**
```bash
source ~/.packmind.env
uv run python -m dimos.experimental.pack_mind.pack_coordinator_server \
  --zones north,east,south,west \
  --prefs "alpha:south,north,east,west;bravo:north,east,west,south" &
#   → open http://<Laptop-A-tailscale-ip>:8090 on the projector NOW
uv run dimos run unitree-go2-pack --daemon
```
**Laptop-B — dog bravo:**
```bash
source ~/.packmind.env
uv run dimos run unitree-go2-pack --daemon
```
Per-dog gate before demo: ping dog → `dimos show-config` → run to "running" → `mcp
list-tools` + `speak` (proves WebRTC) → a `drive` burst → `look_for_red` → search skills.

---

## 4. THE DEMO — STABLE first, SHOWCASE only if stable lands

### 4a. STABLE (scripted, can't-miss) — `scripted_pack_run`
Real dogs move via velocity teleop; the **real coordinator** drives no-overlap + inheritance;
the dashboard animates. No LLM/nav-RPC on the critical path.
```bash
# Laptop-A (alpha = leader, holds target zone, you drop it on cue):
uv run python -m dimos.experimental.pack_mind.scripted_pack_run \
  --role alpha --leader --target "red kit" --target-zone south \
  --hold-zone --drop-on-enter --drive
# Laptop-B (bravo = sweeper, inherits + finds):
uv run python -m dimos.experimental.pack_mind.scripted_pack_run \
  --role bravo --target "red kit" --target-zone south \
  --find --keep-polling --drive
```
Beat: alpha claims `south` and holds it → bravo clears `north/east/west` **zero overlap** →
on the host's cue press **Enter** on Laptop-A ("we lose Alpha") → alpha OFFLINE, `south`
returns to the pool → bravo **inherits** `south`, finds the kit → pack stops. Mission
outlived the robot. Drop `--drive` to run the pure-ledger story if the floor is tight.

### 4b. SHOWCASE (full autonomy, higher risk)
Voice/`mcp` to each dog: *"find the red kit."* Agent runs `start_search → next_zone →
navigate → look_for_red → report_*`. Fragile: `navigate_with_text`/`relative_move` stall on
CPU/LCM-RPC. If a dog hangs, fall back to 4a — coordinator state is shared, so scripted and
autonomous dogs can even mix.

---

## 5. FAILURE → FIX

| Symptom | Cause | Fix |
|---|---|---|
| Laptop-B can't reach coordinator | NAT / wrong URL / service order | `tailscale ping <A>`; uplink is default route; URL uses A's **tailscale** ip not 192.168.x |
| Internet dead on dog AP | Wi-Fi above uplink in service order | reorder uplink above Wi-Fi; `route -n get default` = uplink iface |
| `tailscale` not found (App Store) | CLI off PATH | `/Applications/Tailscale.app/Contents/MacOS/Tailscale` |
| Dog unreachable | not on its AP / wrong ROBOT_IP | join dog Wi-Fi; `ping 192.168.12.1`; `dimos show-config` |
| `dimos run` ignores robot ip | used a flag | unset; set `ROBOT_IP` env; `dimos show-config` |
| EdgeTAM/CUDA crash on deploy | GPU-only modules | none needed — `unitree-go2-pack` already disables SecurityModule + PersonFollow |
| Dog reaches goal but driver freezes | `relative_move` waits on flaky `is_goal_reached` RPC | use `scripted_pack_run --drive` (VelocityTeleop lane) |
| `look_out_for`/moondream slow 10–60s | VLM on CPU | use `look_for_red` (GPU-free) on the critical path |
| bravo stops before inheriting | next_zone empty while alpha still held south | run bravo with `--keep-polling` |
| projector state stale | dashboard cache | refresh `http://<A-tailscale-ip>:8090` (it polls `/state`) |

---

## 6. Rehearse with ZERO dogs (do the night before)
```bash
uv run python -m dimos.experimental.pack_mind.demo_pack_live --pace 2     # http://localhost:8090
# OR the real two-process story locally (no --drive):
uv run python -m dimos.experimental.pack_mind.pack_coordinator_server \
  --prefs "alpha:south,north,east,west;bravo:north,east,west,south" &
uv run python -m dimos.experimental.pack_mind.scripted_pack_run --role alpha --leader \
  --target-zone south --hold-zone --drop-on-enter
uv run python -m dimos.experimental.pack_mind.scripted_pack_run --role bravo \
  --target-zone south --find --keep-polling
```

## 7. Teardown
`kill` the coordinator (holds the only shared state), Ctrl-C the `dimos run` daemons,
`tailscale down` to leave the tailnet. Nothing persists server-side.
