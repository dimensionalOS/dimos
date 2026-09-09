# FRANK task board

Everything for the demo lives in this directory. One line per task, move it between sections.
Contract for all app work: `../app/API.md`. Handoff prompts for other agents: `handoffs/`. Run instructions: `../README.md`.

## Blocked

- Robot connection: Go2 at 10.0.0.79 needs its per-device AES key (firmware ≥ 1.1.15). `unitree-fetch-aes-key --device-type Go2 --email <unitree app account>`, then `DIMOS_UNITREE_AES_128_KEY=...` in `.env`.

## Doing

## Todo

- [ ] `scan.py`: spin, capture, identify, one-line result (needs robot + identify.py)
- [ ] `sightings.py`: search recent recorded frames for a person (needs robot recording)
- [ ] SKILL.md: add the chat, find-now, and follow-up procedures once the pieces exist
- [ ] Pair Bluetooth speaker, pin as default sink, confirm `speak.py --device`
- [x] Pi as the first client harness: `loop.py --harness pi` + `persona.md`; end-to-end pass 2026-09-03 (enroll → greet, chat → reply, ~20 s per turn, spoke via speak.py)
- [x] `up.py`: one-command up / down / status for the whole stack; `down --wipe` is the end-of-day script

## Done

- [x] Hardware-verified 2026-09-03: Euler pitch negative = nose-up but momentary (nods back level in ~1 s, Pose/BalanceStand don't help); `sit` holds ~50° nose-up and the watcher recognized a face at 0.9 m from it; odom carries pitch. Watcher found Henry live with lidar range.
- [x] Stable public URL: Tailscale Funnel `https://<machine>.<tailnet>.ts.net` → 7790 (`tailscale funnel --bg 7790`, needs one-time enable on the tailnet); QR rendered for it; server rejects forwarded requests on agent routes.
- [x] Rolling face watcher + world state + found/seen events — `watch.py`, `app/lib/world.py`, `/agent/world` + `/agent/watch`, `inbox.py world` / `watch-for` / `unwatch`, `loop.py` `[watcher]` headers. Tested against the server with curl and against `watch.py`'s real loop with rendered frames; the laptop webcam opens but nobody was in front of it, so the walk-in/walk-out test is still owed on hardware.

- [x] Ephemeral world state: `loop.py` writes `cache/world.txt` per turn, `pi_world.ts` appends it as the last message at LLM-call time (Pi `context` hook); verified not stored in the session. Claude Code harness has no equivalent hook yet.

- [x] Feasibility spec (`docs/development/frank_go2_demo_feasibility.md`)
- [x] `robot.py`: tilt / level / sit / rise / observe / pose / move / stop over the running DimOS instance
- [x] `speak.py`: ElevenLabs streaming → default audio output, cached
- [x] `app/API.md` contract
- [x] Chat server + PWA (`app/server.py`, `app/static/`) — `handoffs/chat-app.md`
- [x] Idle scheduler inside the server (`app/tasks.yaml`) — part of `handoffs/chat-app.md`
- [x] Agent-side CLI (`inbox.py`) — plus `app/fake_server.py`, a test double to delete when the real server lands
- [x] `qr.py`: printable QR page for the ngrok URL, `--print` sends it to `lp` (needs the `qrcode` package or the `qrencode` CLI — neither is installed yet)
- [x] `grid.py`: occupancy-grid queries + `test_grid.py`, checked against a replay costmap — `handoffs/grid.md`
- [x] Face identification (`identify.py`) — YuNet + SFace via OpenCV, `test_identify.py` harness
