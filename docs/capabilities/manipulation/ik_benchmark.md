---
title: "IK Backend Benchmark"
---

`misc/ik_benchmark/ik_backends.py` compares manipulation IK backends (Pink, RoboPlan
OInK, and any future `KinematicsSpec` implementation) on representative DimOS
workloads: latency, convergence reliability, solution accuracy, and coarse
resource usage, across supported robot configurations.

## How it works

- **Scenarios** are drawn from `SCENARIOS` — one or more robots sharing a
  world (currently `xarm6`, `xarm7`, and `dual_xarm6`, a two-arm setup
  mirroring the `dual-xarm6-planner-coordinator` blueprint). Multi-robot
  scenarios are solved as joint multi-target `solve_pose_targets` calls. Each
  backend run builds its **own fresh `RoboPlanWorld`** — no shared mutable
  scene between backends.
- **Targets** are sampled by drawing joint configurations uniformly within
  limits for every robot, keeping only scene-wide collision-free ones, and
  mapping them through the world's FK. Every target is reachable by
  construction, so failures measure solver behavior rather than unreachable
  goals.
- **Fairness**: all backends solve identical targets with an equivalent seed
  joint state, `check_collision=True`, and the same `max_attempts`.
- **Latency** is `time.perf_counter` around `KinematicsSpec.solve`, after
  warmup solves are discarded.
- **Accuracy**: every successful solution is independently re-verified by
  pushing it through the world's FK and scoring against the target with
  `compute_pose_error` — the same metric for every backend.
- **Resource usage**: coarse current-RSS delta (`/proc/self/statm`) across each
  backend's measured window; model construction happens during warmup.

## Running

```bash
uv sync --extra manipulation --inexact

# Full benchmark (all scenarios, both backends):
uv run python misc/ik_benchmark/ik_backends.py --samples 200 --warmup 10 --output /tmp/ik.json

# Single scenario / backend, custom attempt budget:
uv run python misc/ik_benchmark/ik_backends.py --scenario dual_xarm6 --solver pink --max-attempts 3

# Sweep the attempt budget to trace the success-rate vs latency tradeoff:
for n in 1 2 5 10; do
  uv run python misc/ik_benchmark/ik_backends.py --max-attempts "$n" --output "/tmp/ik_a$n.json"
done
```

Key options: `--scenario` / `--solver` (repeatable), `--samples`, `--warmup`,
`--max-attempts`, `--pink-max-iterations`, `--seed`, `--output`.

Requires the `xarm_description` LFS data (fetched automatically by
`get_data("xarm_description")`).

## Extending

- **New scenario**: add a `ScenarioSpec` to `SCENARIOS` — a name plus a factory
  returning the `RobotModelConfig` list for one world (one entry per arm for
  multi-robot setups).
- **New backend**: add a `SolverSpec` to `_solver_registry()` — a name plus a
  constructor from a fresh `WorldSpec` to a `KinematicsSpec`. Backends that are
  world-agnostic (like Pink) may ignore the world; world-native backends (like
  OInK) take it as their solver.

## Results

Benchmark numbers are machine- and version-dependent, so they are not checked
into the repo. Reference results and the observed tradeoffs for OInK vs Pink
are discussed on
[issue #3232](https://github.com/dimensionalOS/dimos/issues/3232).
