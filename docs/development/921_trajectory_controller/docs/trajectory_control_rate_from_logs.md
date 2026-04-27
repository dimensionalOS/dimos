# Choosing control rate (Hz) from logs (P4-3)

The holonomic local path follower uses a single config knob, `GlobalConfig.local_planner_control_rate_hz`, to pace `LocalPlanner` and to pass a consistent `dt_s` into the controller. That rate should follow **evidence** (issue 921): telemetry, achieved scheduling, and rough plant or actuator delay - not a fixed "high enough" number.

This note ties together: JSONL fields in `dimos/navigation/trajectory_control_tick_jsonl.md`, the plot recipe in [`trajectory_control_tick_plots.md`](trajectory_control_tick_plots.md), and Python headroom in [`trajectory_control_tick_benchmark.md`](trajectory_control_tick_benchmark.md).

## 1. What the export contains

Each line has at least:

- **`dt_s`** - Control period the producer intended for that tick. When pacing matches config, it should sit near `1.0 / local_planner_control_rate_hz`.
- **`wall_time_s`** (optional) - If your integration supplies monotonic or wall time per tick, consecutive differences are the **achieved** inter-tick time; compare them to `dt_s` to see schedule slip and jitter.
- **`ref_time_s` / `meas_time_s`** - Trajectory and measurement time bases. Large stable gaps can indicate coarser sensing than the control loop, which caps how much faster the inner loop can help for that measurement set.

If `wall_time_s` is absent, you still know the **nominal** rate from `dt_s` or from config, but you cannot read jitter or overload from the log alone.

## 2. Reconstruct achieved Hz from a JSONL file

**Nominal average rate (always available):** take the mean of `1.0 / dt_s` for lines with `dt_s > 0`, or use the reciprocal of the mean of `dt_s` if you prefer one number per run.

**Achieved inter-arrival (if `wall_time_s` is present):** sort by `wall_time_s` and use successive differences. Report mean and standard deviation of the gaps. Mean gap close to `1/rate_config` and low spread mean the process is really running near the intended Hz. Wide spread or a mean much larger than `1/rate` mean the configured rate is optimistic: fix scheduling or load first before pushing Hz up.

A minimal check with [pandas](https://pandas.pydata.org/):

```python
import pandas as pd

df = pd.read_json("ticks.jsonl", lines=True)
nominal_hz = (1.0 / df["dt_s"].replace(0, float("nan"))).mean()
if df["wall_time_s"].notna().sum() > 1:
    g = df.sort_values("wall_time_s")
    inter = g["wall_time_s"].diff().dropna()
    actual_hz = 1.0 / inter.mean()
    print("nominal_Hz", nominal_hz, "wall_clock_Hz", actual_hz, "jitter_s_std", inter.std())
else:
    print("nominal_Hz", nominal_hz, "(no wall_time_s: cannot see jitter)")
```

## 3. Use plant or actuator delay (not the log) as a cap

The JSONL file does not measure motor lag or network RTT by itself. You combine logs with a **separate** estimate of delay, for example:

- Manufacturer or driver doc for the base, or
- A step on the bench: time from a step in commanded twist to a clear change in odom (same order of magnitude as the "effective" delay **D** the controller sees end to end).

Rules of thumb (intentionally loose: use them to avoid nonsense rates, not to prove optimality):

- If **D** is a large fraction of your notional control period, raising Hz far above about **1 / D** often yields **diminishing** improvement, because the loop still reacts to information that is **D** seconds old.
- If the dominant mechanical time constant **τ** of the open-loop plant is large compared to your period, very fast control may still help little until sensing and actuation are updated more often. Match expectations to the slowest part of the path (sensors, bridge, base).

In short: if logs show you truly achieve **f** Hz, but the plant cannot respond meaningfully in less than about **D** seconds, a rate much larger than about **1/D** to **5/D** is a candidate to **test down**, not up, unless the benchmark shows spare CPU and **speed vs divergence** improves when you try it.

## 4. A/B a candidate rate (same motion, two exports)

1. Set `local_planner_control_rate_hz` to value **A** (e.g. current default or conservative).
2. Run the same test track, export JSONL, build the **speed vs planar position divergence** plot (see [`trajectory_control_tick_plots.md`](trajectory_control_tick_plots.md)).
3. Repeat with rate **B**; keep other gains and max speed the same.
4. Prefer the rate with **better** divergence at comparable commanded speed, **if** the benchmark in [`trajectory_control_tick_benchmark.md`](trajectory_control_tick_benchmark.md) shows the Python work still has headroom at that rate, and if Section 2 shows your process actually attains the nominal schedule.

A higher number that does not show up in `dt_s` / `wall_time_s` or that does not improve the plot is not a win.

## 5. Config reference

- **`GlobalConfig.local_planner_control_rate_hz`** in `dimos/dimos/core/global_config.py` - default is conservative and validation rejects 100 Hz-style settings on this path; adjust within the allowed range after measurements.

Related: P4-2 cost per tick in [`trajectory_control_tick_benchmark.md`](trajectory_control_tick_benchmark.md), field list in `dimos/navigation/trajectory_control_tick_jsonl.md`.
