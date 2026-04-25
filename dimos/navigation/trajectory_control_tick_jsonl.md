# Trajectory control tick JSONL export

## Format

- **Encoding:** UTF-8.
- **Layout:** one JSON object per line (JSON Lines, `.jsonl`). Empty lines are ignored when reading.
- **Schema version:** integer field `schema_version` on every object. Current value is `1`. Consumers should reject or quarantine unknown major versions.

## Key order and names

After `schema_version`, object keys follow the field order of `TrajectoryControlTick` in `trajectory_control_tick_log.py`. Producers use that order for stable diffs and stream inspection; parsers must not rely on key order.

## Fields

| Key | Type | Unit | Meaning |
|-----|------|------|---------|
| `schema_version` | int | - | Export schema; `1` for this revision. |
| `ref_time_s` | number | s | Reference sample time basis (producer-defined; see trajectory types). |
| `ref_x_m` | number | m | Reference pose x in plan horizontal frame. |
| `ref_y_m` | number | m | Reference pose y in plan horizontal frame. |
| `ref_yaw_rad` | number | rad | Reference yaw in plan frame. |
| `ref_twist_linear_x_m_s` | number | m/s | Reference twist linear x (body). |
| `ref_twist_linear_y_m_s` | number | m/s | Reference twist linear y (body). |
| `ref_twist_linear_z_m_s` | number | m/s | Reference twist linear z (body). |
| `ref_twist_angular_x_rad_s` | number | rad/s | Reference twist angular x (body). |
| `ref_twist_angular_y_rad_s` | number | rad/s | Reference twist angular y (body). |
| `ref_twist_angular_z_rad_s` | number | rad/s | Reference twist angular z (body). |
| `meas_time_s` | number | s | Measured sample time basis. |
| `meas_x_m` | number | m | Measured pose x (plan frame). |
| `meas_y_m` | number | m | Measured pose y (plan frame). |
| `meas_yaw_rad` | number | rad | Measured yaw (plan frame). |
| `meas_twist_linear_x_m_s` | number | m/s | Measured twist linear x (body). |
| `meas_twist_linear_y_m_s` | number | m/s | Measured twist linear y (body). |
| `meas_twist_linear_z_m_s` | number | m/s | Measured twist linear z (body). |
| `meas_twist_angular_x_rad_s` | number | rad/s | Measured twist angular x (body). |
| `meas_twist_angular_y_rad_s` | number | rad/s | Measured twist angular y (body). |
| `meas_twist_angular_z_rad_s` | number | rad/s | Measured twist angular z (body). |
| `e_along_track_m` | number | m | Along-track error (see `trajectory_metrics`). |
| `e_cross_track_m` | number | m | Cross-track error. |
| `e_heading_rad` | number | rad | Heading error. |
| `planar_position_divergence_m` | number | m | Planar position divergence (issue 921 speed-vs-divergence plots). |
| `cmd_linear_x_m_s` | number | m/s | Commanded linear x (published this tick). |
| `cmd_linear_y_m_s` | number | m/s | Commanded linear y. |
| `cmd_linear_z_m_s` | number | m/s | Commanded linear z. |
| `cmd_angular_x_rad_s` | number | rad/s | Commanded angular x. |
| `cmd_angular_y_rad_s` | number | rad/s | Commanded angular y. |
| `cmd_angular_z_rad_s` | number | rad/s | Commanded angular z. |
| `commanded_planar_speed_m_s` | number | m/s | Planar speed from command (`hypot(linear.x, linear.y)` in body). |
| `dt_s` | number | s | Control period for this tick. |
| `wall_time_s` | number or null | s | Optional wall clock. |
| `sim_time_s` | number or null | s | Optional simulation time. |

## API

- `trajectory_control_tick_to_jsonl_dict`, `trajectory_control_ticks_to_jsonl_lines`, `write_trajectory_control_ticks_jsonl`, `JsonlTrajectoryControlTickSink`, and `iter_trajectory_control_tick_jsonl` in `dimos.navigation.trajectory_control_tick_export`.
- `LocalPlanner` writes live JSONL when `GlobalConfig.local_planner_trajectory_tick_log_path` is set to a file path.

## Plotting

Use `planar_position_divergence_m` vs `commanded_planar_speed_m_s` (and time series on `ref_time_s` or `meas_time_s`) per issue 921 guidance; `pandas.read_json(..., lines=True)` accepts this format if you use pandas.

**Control rate (P4-3):** after you have exports, see `docs/development/trajectory_control_rate_from_logs.md` for how to relate `dt_s` and optional `wall_time_s` to a sensible `local_planner_control_rate_hz` and to plant delay.

**Live navigation export:** set `local_planner_trajectory_tick_log_path` in `GlobalConfig`, run the holonomic path follower, then plot the generated file. This is the normal path for comparing speed vs divergence on a robot or replay harness.

**Built-in recipe (P2-3):** from the repository root, `make plot-trajectory-ticks` writes `dimos/navigation/fixtures/trajectory_control_ticks_sample_921_plot.png` using `uv run --with matplotlib`. For your own export:

```bash
uv run --with matplotlib python scripts/plot_trajectory_control_ticks.py path/to/ticks.jsonl -o out.png
```

See [docs/development/trajectory_control_tick_plots.md](../../docs/development/trajectory_control_tick_plots.md) for commands and defaults.
