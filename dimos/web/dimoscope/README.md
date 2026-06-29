# dimoscope

A lightweight, npm-native **web viewer + teleop for the dimos LCM bus**. Open a
browser, it **auto-discovers topics**, **auto-picks a visualization per message
type**, and lets you **drive the robot** — everything decoded client-side with
[`@dimos/msgs`](https://jsr.io/@dimos/msgs).

Built for the Dimensional trial task ("an easy-to-use web framework to subscribe
to topics and visualize"). No Deno — the bridge runs on **Bun** (`node:dgram`),
the app is **Vite + React + TS**. Runs against **real dimos** (no mocks).

```
dimos (LCM/UDP)  ──►  bridge.ts (Bun)  ──►  WebSocket  ──►  browser (@dimos/msgs decode → widgets)
   ▲  /cmd_vel        node:dgram ⇄ WS                          DimosBus + React hooks
   └──────────────────────────────────────────────────────────────── teleop ◄──┘
```
The bridge also **reassembles LCM fragments** (dimos fragments any message over
~1.4KB), so messages like OccupancyGrid arrive whole — the browser is unchanged.

This lives at `dimos/web/dimoscope/`; the demo publisher is `examples/fakesensors.py`.
All paths below are relative to the **dimos repo root**.

## Prerequisites (one-time)
- `brew install git-lfs libjpeg-turbo`
- A dimos venv (bare wheel is enough for simplerobot + fakesensors):
  ```bash
  uv venv --python 3.12        # uv fetches its own 3.12 (bypasses pyenv)
  uv pip install dimos         # prebuilt wheel — NOT `uv sync` (that compiles + pulls torch)
  ```

## Quickstart (real dimos)

Four terminals, from the dimos repo root:

```bash
# 1) the bridge (LCM bus ⇄ WebSocket)
cd dimos/web/dimoscope && bun install && bun run bridge.ts

# 2) the robot — real dimos example module
.venv/bin/python examples/simplerobot/simplerobot.py --headless

# 3) sensors — a hand-written real dimos Module that publishes /map
PYTHONUNBUFFERED=1 .venv/bin/python examples/fakesensors.py
#   (first run builds the matplotlib font cache once — give it a moment)

# 4) the web app
cd dimos/web/dimoscope/app && bun install && bun run dev      # open http://localhost:5173
```

WorldView shows the robot pose + trail + the occupancy grid. Drive it with
**WASD / arrow keys**.

Verify the pipe headlessly (no browser): `bun run wsprobe.ts` → prints the topics
it received (e.g. `/odom`, `/map`).

## Notes
- **macOS multicast:** if the bridge shows no traffic, move the multicast route to
  loopback once: `sudo route -n delete -net 224.0.0.0/4 2>/dev/null; sudo route -n add -net 224.0.0.0/4 -interface lo0`. (Not needed on all setups.)
- dimos LCM default = `udpm://239.255.76.67:7667`; the bridge listens there.

## Layout
| Path | What it is |
|---|---|
| `bridge.ts` | Bun: `node:dgram` multicast ⇄ WebSocket relay + LCM fragment reassembly |
| `wsprobe.ts` | headless WS client for verification |
| `examples/fakesensors.py` *(repo root)* | a real dimos `Module` demo publisher (subscribes /odom, publishes /map OccupancyGrid) |
| `app/src/bus.ts` | `DimosBus` — decode, topic discovery, subscribe, publish |
| `app/src/useBus.tsx` | React hooks: `useBus`, `useTopics`, `useTopic` |
| `app/src/widgets/` | `WorldView` (auto-detects pose/map/scan/path by type), `TeleopPad`, `JsonInspector`, `PoseReadout`, `registry` |

See **WALKTHROUGH.md** for the guided, concept-by-concept tour.
