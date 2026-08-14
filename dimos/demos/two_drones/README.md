# Two-Drone Coordinated Search Demo (dimsim)

Two **fully isolated** dimensional agent stacks — one LLM agent per drone —
share ONE dimsim physics world, coordinate a search over a radio channel,
find a moving red target, and pursue it together.

https://localhost:8090 shows the shared world; each drone knows about its
partner **only** what arrives over the radio.

## What you're looking at

- **Two real drones in one world**: both are physically simulated (flight
  embodiment, collisions, per-drone lidar). They can see each other in
  lidar and bump into each other.
- **Epistemic isolation**: each drone runs its own dimensional process
  (`dimos run dimsim-drone`, namespaced `droneA`/`droneB`). Sensors, maps,
  planner and the LLM agent are private. The ONLY shared channel is
  `/radio`.
- **The radio** is Buzz/Nostr-inspired: signed JSON events
  `{id, pubkey, created_at, kind, tags, content, sig}` with per-drone
  Ed25519 device keys. Tags carry machine facts (position, claimed sector,
  target sightings); `content` carries agent-to-agent prose. See
  `comms_comparison.py` for the bandwidth study (~1 kbit/s vs a 20 kbit/s
  SiK telemetry budget).
- **Ghosts**: the translucent drone + wireframe cube are a drone's *belief*
  about its partner and the target, built exclusively from radio messages.
  Watch them lag behind the real avatars — that gap is the whole point.
- **The referee** (`referee.py`) is the only process that reads sim truth:
  it moves the target and simulates each drone's line-of-sight sensor
  (range 12 m, FOV 140°, occlusion raycast), publishing results only to
  that drone's private topics.

## Coordination loop (all agent-decided, not scripted)

1. Mission briefing arrives → agents negotiate sector split over the radio
   (`claim_sector`) and sweep their own sectors (`sweep_area`, lawnmower
   pattern with obstacle avoidance via the A* replanning stack).
2. A drone's sensor spots the target → it broadcasts `report_sighting` →
   the partner abandons its sweep and converges.
3. Target lost behind walls → drones tell each other and split the
   re-search around the last known position.

## Run it

```bash
# Requires: macOS (or Linux), ollama with the gemma4 model pulled, uv sync done.
uv run python -m dimos.demos.two_drones.run_demo --scenario open
uv run python -m dimos.demos.two_drones.run_demo --scenario obstacles --record out/obstacles.webm
uv run python -m dimos.demos.two_drones.run_demo --scenario hide --duration 420
```

Scenarios: `open` (no obstacles), `obstacles` (interior walls), `hide`
(target weaves behind walls and drones lose/reacquire it).

The launcher starts: the dimsim bridge (`--robots droneA,droneB`), one
recording/viewer browser page, the referee, and both agent stacks. Radio
traffic prints live to the console. Logs land in `out/two_drones/<scenario>/`.

Agents default to a local LLM (`ollama:gemma4`) — no API key required.
Override with `--model` (e.g. `--model gpt-5.6-luna` with `OPENAI_API_KEY`).

To talk to a drone mid-run (humancli style):

```python
from dimos.core.transport_factory import make_transport
t = make_transport("/droneA/human_input"); t.start()
t.publish("Status report, please. Then continue your mission.")
```

## Engine changes this demo rides on

- dimsim multi-robot worlds: `dimsim dev --robots a,b` puts N physical
  robots in ONE world with per-robot topics `/a/cmd_vel`, `/a/odom`,
  `/a/lidar` and robot-addressed teleport/embodiment (`misc/DimSim`).
- `SceneClient` grew a `robot=` parameter; `dimos --dimsim-external`
  attaches a stack to an already-running sim; `Blueprint.namespace()` MCP
  skill dispatch fixed (SkillInfo.rpc_name).
