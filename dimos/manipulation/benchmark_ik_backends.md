# IK Backend Benchmark: RoboPlan OInK vs Pink

Benchmark comparing the two manipulation IK backends on representative DimOS
workloads, for [issue #3232](https://github.com/dimensionalOS/dimos/issues/3232).
Related implementation: [PR #3230](https://github.com/dimensionalOS/dimos/pull/3230)
(RoboPlan-native OInK backend).

## Methodology

`dimos/manipulation/benchmark_ik_backends.py`:

- Robots: xArm6 and xArm7 (`make_xarm6_model_config()` / `make_xarm7_model_config()`,
  with gripper, `limited=true`).
- One `RoboPlanWorld` per robot is shared by both solvers. Pink is world-agnostic
  (builds its own Pinocchio model); RoboPlan OInK *is* the world
  (`RoboPlanKinematicsConfig` returns the world cast to `KinematicsSpec`). Same
  world, same collision model, same FK for both.
- Targets: joint configurations sampled uniformly within limits, filtered to
  collision-free ones, mapped through world FK. Every target is reachable by
  construction, so failures measure solver behavior, not unreachable goals.
- Both backends get identical targets and the same seed joint state (live robot
  state), `check_collision=True`, `max_attempts=10`, default tolerances
  (1 mm / 0.01 rad).
- Latency: `time.perf_counter` around `KinematicsSpec.solve`. 10 warmup solves
  per backend discarded.
- Accuracy: successful solutions are independently re-verified by pushing them
  through the world's FK and scoring against the target with
  `compute_pose_error` (same metric for both backends).
- Resource usage: coarse peak-RSS delta (`resource.ru_maxrss`) across each
  backend's measured window; model construction happens in warmup.
- Hardware: 16-core x86_64 (Ubuntu 20.04), Python 3.12, `pin-pink 4.2.0`,
  `roboplan 0.0.100`. 200 timed solves per (robot, backend), seed 0.

## Results (seed 0, 200 samples)

| robot | backend | success | p50 ms | p95 ms | mean ms | pos err p50 (mm) | pos err p95 (mm) |
|-------|---------|---------|--------|--------|---------|------------------|-------------------|
| xarm6 | pink | 87.5% | 29.6 | 512.2 | 123.8 | 0.74 | 0.97 |
| xarm6 | roboplan_oink | 84.5% | 31.3 | 99.8 | 38.4 | 0.27 | 0.86 |
| xarm7 | pink | 99.0% | 8.6 | 207.5 | 38.9 | 0.74 | 0.96 |
| xarm7 | roboplan_oink | 97.0% | 5.1 | 66.5 | 16.4 | 0.53 | 0.97 |

Orientation error means are comparable (0.5–1.2 mrad; tolerance is 10 mrad).
Verified max position error is ~1.0 mm for both backends, i.e. both honor the
position tolerance exactly when they report success. Peak RSS delta was ≤ 2 MB
for both backends — per-solve memory pressure is negligible once models are
loaded.

Failure breakdown:

| robot | backend | status counts |
|-------|---------|---------------|
| xarm6 | pink | 19 NO_SOLUTION, 3 COLLISION, 3 JOINT_LIMITS |
| xarm6 | roboplan_oink | 31 NO_SOLUTION |
| xarm7 | pink | 2 NO_SOLUTION |
| xarm7 | roboplan_oink | 3 NO_SOLUTION, 3 COLLISION (converged only to colliding endpoints) |

## Observed tradeoffs

- **OInK has a much tighter latency tail.** Mean 2–3.2x faster, p95 3–5x lower.
  Pink's slow solves are its failures: on xarm6, failed solves average 343 ms
  (burnt on restarts that exhaust the 200-iteration budget) vs 92 ms for
  successes. OInK failures cost ~100 ms (10 bounded attempts x 100 iterations).
  For planning loops that call IK repeatedly, OInK's worst case is ~5x cheaper.
- **Pink succeeds slightly more often.** +3 pp on xarm6 (87.5 vs 84.5), +2 pp on
  xarm7 (99.0 vs 97.0). Pink's QP formulation with per-attempt restarts within
  joint limits rescues some targets OInK gives up on.
- **The backends fail on different targets.** Only 6 of 200 xarm6 targets fail
  under both; zero overlap on xarm7. They are complementary: a fallback chain
  (OInK first, Pink on NO_SOLUTION) would beat either alone on both latency and
  success rate.
- **Failure modes differ in kind.** Pink fails three ways (no convergence, QP
  failure, collision/joint-limit rejection of a converged solution). OInK only
  ever returns NO_SOLUTION — it never reports success for an endpoint that
  fails collision, and FK verification confirms every OInK success lands inside
  tolerance.
- **Accuracy is a wash.** Both meet the 1 mm tolerance on every reported
  success; OInK's median position error is somewhat lower (0.27 vs 0.74 mm on
  xarm6).
- **Caveats.** Single-machine run; Pink solver config was the default
  (`proxqp`, `dt=0.05`, 200 iterations) and OInK's 100 iterations/attempt is
  currently hardcoded; targets are uniform-random reachable poses with no
  obstacles beyond self-collision, so results don't cover cluttered-scene
  workloads; RSS delta is process-wide and coarse.

A rerun with `--seed 1` reproduced the same pattern (OInK mean 2–3.5x faster,
p95 ~5x tighter; success rates within ±3 pp, including one robot where OInK
scored higher), so the tradeoffs above are not seed-specific.

## Reproduce

```bash
uv sync --extra manipulation --inexact
uv run python dimos/manipulation/benchmark_ik_backends.py \
    --samples 200 --warmup 10 --seed 0 --output /tmp/ik_bench.json
```

Requires the `xarm_description` LFS data (`get_data("xarm_description")`).
