# Trajectory control tick benchmark (P4-2)

This documents how to reproduce **Python CPU cost per control tick** for the holonomic path follower and for building one **JSONL-ready** tick record. Numbers are **machine-specific**; the committed tests only enforce very loose ceilings so shared CI stays stable.

## What is timed

1. **`HolonomicPathController.advance`** - lookahead plus odom in, `Twist` out. This is the issue 921 holonomic tracking path used from `LocalPlanner` when `local_planner_path_controller` is `"holonomic"`.
2. **`trajectory_control_tick_from_samples`** - scalar errors and one `TrajectoryControlTick` dataclass when telemetry export is enabled.

These are **single-threaded, in-process** medians. They do not include planner, ROS, or I/O. Use them to see whether a chosen control rate (see `GlobalConfig.local_planner_control_rate_hz`) leaves headroom on your Mac or Linux dev box.

## How to run

From the `dimos` repository root (with the project env active, same as other tests):

```bash
uv run pytest -m slow -sv dimos/navigation/test_trajectory_control_tick_benchmark.py
```

Optional: print medians and implied headroom in Hz (still runs the same assertions):

```bash
DIMOS_TRAJECTORY_BENCH_VERBOSE=1 uv run pytest -m slow -sv dimos/navigation/test_trajectory_control_tick_benchmark.py
```

For CI parity including slow tests:

```bash
CI=1 ./bin/pytest-slow dimos/navigation/test_trajectory_control_tick_benchmark.py
```

## Interpreting results

- **Median latency** from `perf_counter_ns` per call is less noisy than the mean for this kind of micro-benchmark.
- **Implied max rate** (single-thread, ignoring everything else) is about `1e9 / median_ns` Hz. If that is well above your configured `local_planner_control_rate_hz`, the Python controller and tick record are unlikely to be the bottleneck; if it is close, profile end-to-end before raising the rate.

## Example numbers (not guarantees)

One developer run (Apple Silicon, CPython, April 2026) showed median `HolonomicPathController.advance` around **0.3-0.4 ms** and `trajectory_control_tick_from_samples` around **0.015 ms**. Your laptop or CI VM will differ; re-run the commands above after code changes.

Related: tick JSONL fields and plotting are in `dimos/navigation/trajectory_control_tick_jsonl.md` and [`trajectory_control_tick_plots.md`](trajectory_control_tick_plots.md). For choosing a rate from telemetry and plant delay (P4-3), see [`trajectory_control_rate_from_logs.md`](trajectory_control_rate_from_logs.md).
