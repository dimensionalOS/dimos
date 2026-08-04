---
title: "IK Backend Benchmark"
---

`benchmarks/ik_backends.py` compares manipulation IK backends (Pink, RoboPlan
OInK, and any future `KinematicsSpec` implementation) on representative DimOS
workloads: latency, convergence reliability, solution accuracy, and coarse
resource usage, across supported robot configurations.

## How it works

- **Robots** are drawn from `ROBOT_CONFIG_FACTORIES` (currently xArm6 and
  xArm7). Each backend run builds its **own fresh `RoboPlanWorld`** — no shared
  mutable scene between backends.
- **Targets** are sampled by drawing joint configurations uniformly within
  limits, keeping only collision-free ones, and mapping them through the
  world's FK. Every target is reachable by construction, so failures measure
  solver behavior rather than unreachable goals.
- **Fairness**: all backends solve identical targets with an equivalent seed
  joint state, `check_collision=True`, and the same `max_attempts`.
- **Latency** is `time.perf_counter` around `KinematicsSpec.solve`, after
  warmup solves are discarded.
- **Accuracy**: every successful solution is independently re-verified by
  pushing it through the world's FK and scoring against the target with
  `compute_pose_error` — the same metric for every backend.
- **Resource usage**: coarse peak-RSS delta (`resource.ru_maxrss`) across each
  backend's measured window; model construction happens during warmup.

## Running

```bash
uv sync --extra manipulation --inexact

# Full benchmark (both robots, both backends):
uv run python benchmarks/ik_backends.py --samples 200 --warmup 10 --output /tmp/ik.json

# Single robot / backend, custom attempt budget:
uv run python benchmarks/ik_backends.py --robot xarm7 --solver pink --max-attempts 3

# Sweep the attempt budget to trace the success-rate vs latency tradeoff:
for n in 1 2 5 10; do
  uv run python benchmarks/ik_backends.py --max-attempts "$n" --output "/tmp/ik_a$n.json"
done
```

Key options: `--robot` / `--solver` (repeatable), `--samples`, `--warmup`,
`--max-attempts`, `--pink-max-iterations`, `--seed`, `--output`.

Requires the `xarm_description` LFS data (fetched automatically by
`get_data("xarm_description")`).

## Extending

- **New robot**: add a `RobotModelConfig` factory to `ROBOT_CONFIG_FACTORIES`.
- **New backend**: add a `SolverSpec` to `_solver_registry()` — a name plus a
  constructor from a fresh `WorldSpec` to a `KinematicsSpec`. Backends that are
  world-agnostic (like Pink) may ignore the world; world-native backends (like
  OInK) take it as their solver.

## Results

Benchmark numbers are machine- and version-dependent, so they are not checked
into the repo. Reference results and the observed tradeoffs for OInK vs Pink
are discussed on
[issue #3232](https://github.com/dimensionalOS/dimos/issues/3232).
