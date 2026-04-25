# Trajectory control tick plots

How to turn trajectory control tick **JSONL** exports into figures (speed vs planar divergence and time-series sanity panels). Field definitions and units are in `dimos/navigation/trajectory_control_tick_jsonl.md`.

For live `LocalPlanner` runs, set `GlobalConfig.local_planner_trajectory_tick_log_path` to the JSONL file you want to capture, then run the plot command below on that file.

From the repository root, using the bundled sample fixture:

```bash
make plot-trajectory-ticks
```

That invokes `scripts/plot_trajectory_control_ticks.py` with `uv run --with matplotlib` so matplotlib is not part of the default install.

For your own export:

```bash
uv run --with matplotlib python scripts/plot_trajectory_control_ticks.py /path/to/ticks.jsonl -o plot.png
```

If you omit `-o`, the script writes `<input_stem>_921_plot.png` next to the JSONL file. The same flow works on macOS when `uv` is available.

For Python CPU cost per tick and how to compare rates on a developer machine, see `docs/development/trajectory_control_tick_benchmark.md`. For how to **choose** a control rate using JSONL timestamps and plant delay, see `docs/development/trajectory_control_rate_from_logs.md` (P4-3).
