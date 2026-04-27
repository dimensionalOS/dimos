# Simulation Matrix Presets And Scenarios

This note summarizes the presets and scenarios exercised by:

- `run_sim04_initial_matrix.py`
- `run_sim_speed_max_matrix.py`

Both scripts are matrix wrappers around `simulate_trajectory_controller.py`. Each matrix expands plant presets, path scenarios, target speeds, and control rates into individual simulation runs. Each run writes artifacts under `docs/development/921_trajectory_controller/simulation/simulation_runs/` (paths relative to the `dimos/` repo root), including `config.yaml`, `summary.json`, `ticks.jsonl`, `command.log`, and usually `plot.png`. Each matrix also writes `matrix_results.json` and `matrix_summary.md`.

## Shared Plant Presets

Both matrices use the same response-curve presets:

- `ideal`: mathematical baseline with effectively instant response and very high acceleration limits.
- `synthetic_nominal`: synthetic first-order lag and acceleration-limited plant, not a Go2 hardware claim.
- `synthetic_sluggish`: slower synthetic response with lower acceleration and saturation limits, used for margin checks.
- `synthetic_asymmetric`: direction-specific synthetic response across linear and yaw axes.
- `synthetic_noisy`: nominal-like synthetic response with bounded linear and yaw noise.

The scripts use seed `7`, so noisy runs are repeatable.

## SIM-04 Initial Matrix

`run_sim04_initial_matrix.py` tests the original trajectory-controller envelope across walking-to-early-running speeds and compact path geometries.

Default sweep:

- Target speeds: `0.55`, `1.0`, `1.5`, and `2.0` m/s.
- Control rates: `5`, `10`, `20`, and `30` Hz.
- Plant presets: all five shared presets listed above.
- Scenarios: `line`, `circle`, `s_curve`, and `right_angle_turn`.
- Reference mode: `direct` by default, with an optional `path_speed_profile` mode.
- Gains: `k_position = 2.2` and `k_yaw = 2.5` by default, with optional gain sweeps.

Scenarios:

- `line`: an 8 m straight path. This is the basic tracking and speed response check.
- `circle`: a 1.5 m radius closed circle. This stresses continuous curvature and yaw tracking.
- `s_curve`: an 8 m S-curve with 1 m lateral amplitude. This checks reversing lateral curvature and coupled x/y tracking.
- `right_angle_turn`: a compact 4 m by 4 m 90 degree polyline. This is the tight-corner stress case.

How it tests:

- For every plant, scenario, speed, rate, and gain combination, the wrapper runs `simulate_trajectory_controller.py`.
- The simulator's own verdict gates are used without relaxing divergence thresholds.
- Matrix classification is `pass`, `fail`, or `invalid` from the simulator verdict, with a `borderline` label added when a passing numeric gate reaches at least 80 percent of its threshold.
- For `synthetic_nominal`, the matrix also reports stricter PR-confidence checks: at `1.5 m/s`, p95 planar divergence must be at most `0.20 m` on line, circle, and S-curve; at `2.0 m/s`, max planar divergence must be at most `0.35 m` except on right-angle turns.

Use this matrix to answer: "Does the controller track the original compact SIM-04 paths across plant response envelopes, target speeds, and control rates?"

## Speed-Max Matrix

`run_sim_speed_max_matrix.py` tests whether the controller stack can operate near the running-speed target, defaulting up to `3.7 m/s`, on geometry designed to let the path-speed profile be the limiter.

Default sweep:

- Target speeds: `2.0`, `2.5`, `3.0`, `3.5`, and `3.7` m/s.
- Control rates: `10` and `20` Hz.
- Plant presets: all five shared presets listed above.
- Scenarios: `speed_max_line`, `speed_max_arc`, `speed_max_s_curve`, `speed_max_stop_distance`, and `speed_max_right_angle_turn`.
- Reference mode: always `path_speed_profile`.
- Path-speed caps: tangent acceleration `0.7 m/s^2`, normal acceleration `1.0 m/s^2`, and goal deceleration `0.7 m/s^2` by default.
- Response wrapper: synthetic saturation limits are scaled by `3.0` by default so synthetic plants can physically reach the high-speed targets. This is recorded in each run config.

Scenarios:

- `speed_max_line`: a 40 m straight path. This checks whether the system can accelerate to and sustain the requested running speed.
- `speed_max_arc`: a 25 m radius, 90 degree arc. This checks high-speed tracking with a broad curvature cap.
- `speed_max_s_curve`: a 40 m S-curve with 2 m lateral amplitude and approximate minimum radius near 20 m. This adds gentle lateral motion at running speed.
- `speed_max_stop_distance`: a 30 m straight path. This checks acceleration to speed and deceleration to a stop under the goal-deceleration cap.
- `speed_max_right_angle_turn`: 20 m straight legs joined by a 1.5 m radius fillet. This verifies that the path-speed profile slows for a sharp corner while still allowing enough approach distance to reach running speed before braking.

How it tests:

- For every plant, scenario, speed, and rate combination, the wrapper runs `simulate_trajectory_controller.py` with `path_speed_profile`.
- The runner's max divergence gate is passed through as `2.0 m` so the matrix can apply its own speed-scaled divergence gate.
- The matrix computes extra metrics from `ticks.jsonl`: achieved peak, p95, and mean planar speed; max observed lateral acceleration; max observed yaw rate; max observed command jerk; and the minimum speed near the right-angle fillet.
- The matrix applies extra gates: max divergence must be at most `0.30 + 0.15 * target_speed`; lateral acceleration must be at most `1.5 m/s^2`; command jerk must be at most `30.0 m/s^3`.
- On line, arc, S-curve, and stop-distance scenarios, achieved peak speed must reach at least 85 percent of the geometry-bounded expected peak speed.
- On `speed_max_right_angle_turn`, speed inside a 2 m window around the fillet midpoint must drop to within `0.4 m/s` of the geometry-bounded corner cap.
- Passing runs are labeled `borderline` when max-type gates reach at least 80 percent of their threshold.

Use this matrix to answer: "Can the controller reach and regulate running speed while respecting path-speed, lateral-acceleration, command-jerk, and corner-slowdown constraints?"
