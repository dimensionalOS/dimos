# Twist-base controller tuning

Three blueprints. Run in order.

1. `unitree-go2-characterization` — drives the robot, fits a velocity model per axis, writes a JSON.
2. `unitree-go2-benchmark` — replays the JSON against fixed test paths, scores path error, writes a results table.
3. `unitree-go2-precision-nav` — click-to-goal nav using both, with live keys to tighten/loosen path tracking.

Each is one terminal. Pygame window needs focus; `WASD/QE` reposition, `Enter` advance, `K` skip, `Backspace` quit.

## Prereqs

```
cd ~/dimos && source .venv/bin/activate
```

Robot on, network reachable. Step 1 wants a corridor or basketball-court-sized space (high-amplitude steps cover several meters before settling). Steps 2-3 are fine in ~3m × 3m.

## Step 1 — characterization

```
dimos run unitree-go2-characterization
```

Per axis (`vx`, `vy`, `wz`), three loops with one Enter per step:

```
floor_probe(amps=[0.02, 0.05, 0.10, 0.15])    # smallest command that actually moves the robot
dense_sweep(amps=[0.2, 0.5, 1.0, 1.5, 2.0])   # main fit: per-amp first-order-plus-deadtime (FOPDT) model
ceiling_probe(amps=[2.5, 3.0])                # over-the-top, measures the real top output
```

"Amp" = the velocity command magnitude for one step. m/s for `vx`/`vy`, rad/s for `wz`.

The canonical model used downstream is the lowest-amplitude fit with `r² > 0.9` (linear regime, before the actuator saturates). The operational ceiling is the max steady-state output velocity actually reached, clamped to the platform's hard envelope.

After Enter the tool waits ~1.5s before commanding the step (settling). Each phase prints a banner.

### Output

```
data/characterization/go2/
├── go2_config_hw_concrete_<date>_<sha>.json   # the artifact (Step 2 + 3 read this)
├── go2_config_hw_concrete_<date>_<sha>.png    # fit-quality plot
└── go2_recording_<date>_<sha>.db              # raw streams
```

The `.json` carries `plant` (the fitted FOPDT), `dynamics_by_amplitude` (full per-amp table for interpolation), `velocity_envelope` (floor/ceiling per axis), `feedforward` (`1/K` per axis), `velocity_profile` (curvature-aware speed profile), and `valid_for_tuning` (`false` for self-test → downstream tools refuse it).

## Step 2 — benchmark

```
dimos run unitree-go2-benchmark    -o benchmarker.config=<path>.json
dimos run unitree-go2-benchmark-rg -o benchmarker.config=<path>.json
```

Replays fixed paths (`straight_line`, `single_corner`, `square`, `circle`) at multiple speeds, scores cross-track error. Bare baseline by default. Enable comparison arms one at a time:

- `-o benchmarker.ff=true` — adds the derived feedforward.
- `-o benchmarker.profile=true` — adds the curvature speed profile.
- `-o benchmarker.rg=true -o benchmarker.e_max=0.20` — adds the reference-governor per-waypoint cap (path-tracking budget = `e_max` meters off the path).

The `-rg` blueprint is the same thing with RG enabled by default. The Go2 stalls below ~0.2 m/s commanded, so `benchmarker.min_speed` defaults to 0.2; set `None` to use the artifact's floor.

Outputs land in `data/benchmark/go2/` (XY plot + per-arm JSON + recording). The bare run also appends an `operating_point_map` to the artifact.

## Step 3 — precision-nav

```
dimos run unitree-go2-precision-nav
```

Bundles the Go2 coord + rerun viewer + voxel/cost/A* planner chain + the pygame window (now repurposed as a 0-9 slider for live corridor width, no WASD). Click a goal in rerun; planner emits a path; coord broadcasts it to the precision follower task, which loads the artifact, solves a per-waypoint speed profile honoring the current `e_max`, and drives the robot.

Keys `0-9` set the corridor half-width 0.0-0.9 m. Each keypress re-solves the profile mid-path.

Output: `data/precision_nav/go2/*.db`.

## Reading recordings

```python
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState

store = SqliteStore(path="<.db>")
store.start()
for obs in store.stream("joint_state", JointState):
    ts, msg = obs.ts, obs.data
```

Streams: `cmd_vel`, `joint_state` (positions = x/y/yaw, velocities = commanded body vel), `odom` (raw), `gate` (operator events).

## Troubleshooting

- Pygame won't open → X11 not reachable. `xeyes` test, then `export DISPLAY=:1; export XAUTHORITY=/run/user/$(id -u)/.Xauthority`.
- Enter does nothing → pygame window unfocused; click on it.
- Flooded with TF warnings → pipe through `grep --line-buffered -E 'Benchmarker|Characterizer|reject|aborted|arrived|timeout|configure|start_path|reposition'`.
- RG arm robot won't move → see `min_speed` above.
- No robot? `uv run python -m dimos.utils.benchmarking.characterization --mode self-test` (artifact is stamped `valid_for_tuning=false`).
