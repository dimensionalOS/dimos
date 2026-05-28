# PACK MIND

**One brain, many bodies, one memory that outlives any single dog.**

A team of robot dogs explores an unknown space. The question the demo answers:
**does *sharing memory* across the pack actually help?** We A/B it head-to-head — same
dogs, same start, same map; the *only* difference is whether they share one discovered
map or each keep a private one.

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

## File map

| File | Role |
|---|---|
| `world.py` | Fabricate the maze `OccupancyGrid` + plant survivors |
| `explore_sim.py` | **Fog-of-war exploration engine** (raycast reveal, frontier, shared/private discovered map, offline persistence) |
| `server.py` | FastAPI WebSocket backend streaming both explore sims |
| `static/explore.html` | Three.js 3D fog-of-war frontend (side-by-side, kill/reset) |
| `view_explore_rerun.py` | **DimOS Viewer (Rerun)** view of one shared-memory maze search (fog + dogs + trails + coverage scalar) |
| `sim.py` / `sim_robot.py` | Coverage-race sim (known map) |
| `blueprint.py` | `pack-mind-sim` native DimOS blueprint (publishes coverage as `OccupancyGrid`) |
| `render.py` | Standalone matplotlib → mp4 A/B render |
| `view_rerun.py` | Log the coverage A/B into a Rerun `.rrd` / live viewer |
| `demo_spike.py` | Feasibility spike (kept as a smoke test of the reused primitives) |
| `test_explore_sim.py` / `test_pack_mind_sim.py` | Engine tests |
| `sim_perception.py` | MuJoCo+VLM single-frame perception probe (optional, off the critical path) |

### Legacy (the old "backpack handoff" demo — superseded, kept for reference)
`conductor.py`, `dashboard.html`, `static/style.css`, `sim_harness.py`, `venue_go2.sh`,
`RUNBOOK.md`, `test_conductor.py`. That demo was a single message relay dressed up as
"shared memory"; the exploration A/B above replaces it. Safe to delete in a cleanup pass.
