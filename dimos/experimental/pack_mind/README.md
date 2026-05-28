# PACK MIND

**One brain, many bodies, one memory that outlives any single dog.**

A team of robot dogs explores an unknown space. The question the demo answers:
**does *sharing memory* across the pack actually help?** We A/B it head-to-head — same
dogs, same start, same map; the *only* difference is whether they share one discovered
map or each keep a private one.

## Pitch deck & diagrams

**🎤 Pitch deck (browser):** <https://pack-mind.pages.dev/> — the idea, the two magic beats
(handoff + inheritance), the A/B proof, and the fleet-memory-layer business case.

**Share meaning, not maps** — a static zone partition is just a brittle special case of
shared memory (it can't adapt to failure, discovery, or rebalancing):

![Static partition vs shared memory](docs/pack_mind_partition_vs_memory.png)

**How it works** — every dog, every tick: sense → remember → choose → plan → move, all
over one shared memory; the A/B knob is whether dogs share that memory or each keep their own:

![PACK MIND search loop and shared memory](docs/packmind_how_it_works.png)

System design — one coordination layer, two substrates (sim shares *cells*, live shares
*zones*, never coordinates): [`docs/packmind_system_design.png`](docs/packmind_system_design.png).

## Thesis (and honest limits)

- **Speed:** a pack sharing coverage/discovery memory avoids redundant re-searching, so
  it searches the whole area faster.
- **Resilience (the strongest point):** when a dog goes offline, its discoveries persist
  in shared memory and the survivors keep using them. With private memory, that dog's
  knowledge dies with it.
- **Honest caveats:** this is a 2D-grid sim (rendered in 3D); point robots, scripted
  sensing, no real SLAM/perception. The win is *complete-area search speed + knowledge
  persistence* — **not** "finds victims faster" (redundant independent robots can stumble
  on victims sooner). Keep the narration honest.

## Two simulations

### 1. Fog-of-war exploration (the current demo) — `explore_sim.py`
Unknown maze, revealed by a raycast sensor (walls block line of sight, RTS-style). Dogs
navigate to **frontiers** (known-free / unknown boundary). The discovered map is **shared**
(one map all dogs read+write) or **independent** (one per dog; the team only sees the union
of *online* dogs, so killing a dog erases its territory). Measured (seed 0, 3 dogs):
shared reaches 95% revealed at **tick 246** vs **335** independent; killing a dog drops
independent's team knowledge **0.80 → 0.32**, shared **drops 0**.

### 2. Coverage race (earlier model) — `sim.py`
Known map; dogs sweep with `CoveragePatrolRouter`; A/B = shared vs private
`VisitationHistory`. Shared hits 75% coverage at tick 1745 vs 3275; final 96.5% vs 88.9%.

Both reuse DimOS navigation primitives (`min_cost_astar`, patrol routers,
`VisitationHistory`) on a fabricated `OccupancyGrid` — pure numpy/scipy, **no CUDA/ROS/sim**.

## Run it

```bash
# Web 3D demo (FastAPI + Three.js) — the centerpiece. Side-by-side fog-of-war race,
# 3-level fog (visible / remembered / unknown), kill-a-dog + reset controls.
uv run python -m dimos.experimental.pack_mind.server      # → http://localhost:8000

# DimOS Viewer (Rerun) — one maze, the pack exploring on the shared map. Scrub the
# "tick" timeline; add --kill DOG TICK for the resilience beat, --save run.rrd to skip the GUI.
uv run python -m dimos.experimental.pack_mind.view_explore_rerun --dogs 3 --seed 0

# Native DimOS blueprint — coverage A/B as two live OccupancyGrid streams in Rerun.
dimos --viewer rerun run pack-mind-sim

# Standalone A/B video of the coverage race.
uv run python -m dimos.experimental.pack_mind.render --out pack_mind_ab.mp4

# Tests
bin/pytest-fast dimos/experimental/pack_mind/test_explore_sim.py -v
bin/pytest-fast dimos/experimental/pack_mind/test_pack_mind_sim.py -v
```

## Live demo — 2 dogs / 2 laptops

The intelligence (shared memory) runs on the laptops; the dogs are bodies. Movement
is teleop-assisted; the agent + coordinator own the memory layer.

**1. One-time per laptop (with internet):** cache the runtime models so the demo runs
offline (a dog's WiFi AP has no internet, and venue WiFi is flaky):
```bash
uv run python -m dimos.experimental.pack_mind.prefetch_live_models   # moondream2 + whisper
```
CLIP/YOLO ship in `data/` (git-lfs). The agent brain (GPT-4o) is a hosted API — keep
the coordinator laptop on a router with WAN.

**2. Network:** one travel router; put each Go2 in STA mode (Unitree app) onto it; both
laptops + both dogs on that subnet. DimOS picks the WebRTC method purely from
`ROBOT_IP` — `192.168.12.1` → the dog's own AP (single-laptop only), any other IP →
LocalSTA (shared router, required for 2 laptops).

**3. Laptop A (Alpha + coordinator + dashboard):**
```bash
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export ROBOT_IP=<dog-A-ip> LISTEN_HOST=0.0.0.0 OPENAI_API_KEY=<key>
uv run python -m dimos.experimental.pack_mind.pack_coordinator_server \
  --zones north,east,south,west --prefs "alpha:north,east;bravo:south,west" &   # dashboard: http://<laptopA-ip>:8090
PACK_DOG_NAME=alpha PACK_COORDINATOR_URL=http://127.0.0.1:8090 \
  uv run dimos run unitree-go2-pack --daemon
```

**4. Laptop B (Bravo):**
```bash
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export ROBOT_IP=<dog-B-ip> LISTEN_HOST=0.0.0.0 OPENAI_API_KEY=<key>
PACK_DOG_NAME=bravo PACK_COORDINATOR_URL=http://<laptopA-ip>:8090 \
  uv run dimos run unitree-go2-pack --daemon
```

**Gotchas (hard-won on site):**
- `unitree-go2-pack` already disables EdgeTAM modules (SecurityModule, PersonFollow) —
  they hard-require CUDA. No `--disable` flag needed.
- `dimos run` has **no `--robot-ip` flag** in this build — set the `ROBOT_IP` env and
  verify with `dimos show-config`.
- `HF_HUB_OFFLINE=1` is required on a no-internet AP; run prefetch first.
- **Detection: use `look_for_red`** (RedObjectDetector) — a fast, GPU-free colour check
  that reports the finding to the coordinator instantly. moondream/`look_out_for` also
  works but is slow on a CPU host (~10–60s first call); keep it off the critical path.
- Movement: `relative_move` (skill) or keyboard teleop. `navigate_with_text` works but is
  slow on CPU (Qwen).
- Bring-up order: ping dog → `dimos show-config` → run to "running" → `mcp list-tools` +
  `speak` → `relative_move` → `look_for_red` → `start_search`/`next_zone` (dashboard at :8090).

**Hardware-free rehearsal (no dogs):**
`uv run python -m dimos.experimental.pack_mind.demo_pack_live --pace 2`, open http://localhost:8090.

## File map

| File | Role |
|---|---|
| `world.py` | Fabricate the maze `OccupancyGrid` + plant survivors |
| `explore_sim.py` | **Fog-of-war exploration engine** (raycast reveal, frontier, shared/private discovered map, offline persistence) |
| `server.py` | FastAPI WebSocket backend streaming both explore sims |
| `static/explore.html` | Three.js 3D fog-of-war frontend (side-by-side, kill/reset) |
| `view_explore_rerun.py` | **DimOS Viewer (Rerun)** view of one shared-memory maze search (fog + dogs + trails + coverage scalar) |
| `live.py` | **Live blueprint** `unitree-go2-pack` (one dog per laptop, EdgeTAM disabled) + PACK system prompt |
| `pack_coordinator.py` / `pack_coordinator_server.py` | Shared zone ledger (no-overlap, find, **inheritance**) + JSON/HTTP API + dashboard route |
| `pack_dashboard.html` | Projector dashboard — zones, finding, offline/inheritance, causal chain |
| `pack_search_skills.py` | Dog agent tools: `start_search` / `next_zone` / `report_*` / `where_is` |
| `red_detector.py` | **Fast GPU-free red-object detector** (`look_for_red`) — HSV-free colour test → auto-reports the find |
| `pack_search_runner.py` | `RobotDriver`-protocol search loop + `MockDriver` |
| `mock_dog.py` / `demo_pack_live.py` / `demo_pack_scene.py` | Hardware-free test + projector rehearsal of both magic beats |
| `prefetch_live_models.py` | Pre-cache moondream2 + whisper so the live stack runs offline |
| `sim.py` / `sim_robot.py` | Coverage-race sim (known map) |
| `blueprint.py` | `pack-mind-sim` native DimOS blueprint (publishes coverage as `OccupancyGrid`) |
| `render.py` | Standalone matplotlib → mp4 A/B render |
| `view_rerun.py` | Log the coverage A/B into a Rerun `.rrd` / live viewer |
| `demo_spike.py` | Feasibility spike (kept as a smoke test of the reused primitives) |
| `test_explore_sim.py` / `test_pack_mind_sim.py` | Engine tests |
| `sim_perception.py` | MuJoCo+VLM single-frame perception probe (optional, off the critical path) |
