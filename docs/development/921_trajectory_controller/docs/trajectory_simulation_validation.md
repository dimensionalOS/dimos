# Trajectory Simulation Validation

This is the PR-ready simulation report for issue 921. It explains what the branch can claim before real Unitree Go2 P8 hardware sign-off, what evidence supports that claim, and what must remain a hardware validation step.

## Review

The simulation evidence supports the software claims that the holonomic trajectory controller is pluggable, observable, curvature-aware, rate-configurable, not Pure Pursuit, and stable across the documented synthetic envelopes when the path speed profile is used. The strongest running-speed claim is still labeled as synthetic: no `fitted_go2_*` response-curve preset exists in this workspace, so the validation does not claim empirical Go2 hardware behavior.

Supported pre-P8 envelope:

- `2.0 m/s` target speed across line, circle, S-curve, and right-angle paths for `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` plants with `path_speed_profile`, default gains, tangent cap `0.5 m/s^2`, normal cap `0.1 m/s^2`, and goal decel `0.5 m/s^2`.
- `1.0 m/s` right-angle support for `synthetic_sluggish` under the same conservative caps. The sluggish envelope is not a supported `2.0 m/s` tight-turn claim.
- `3.7 m/s` target speed on the separate speed-max geometry set for `ideal`, `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy`, with wide enough geometry and running-envelope caps. The sluggish envelope fails the speed-max divergence gates and is explicitly excluded.

## Issue 921 Requirements Covered

| Requirement | Validation evidence |
| --- | --- |
| Pluggable controller and plant abstractions | The Level 0 tests cover `TrajectoryController`, immutable reference and measurement samples, command limits, integrated and actuated plants, calibration, replay loading, and controller-swap regression. |
| Speed-vs-divergence graph | All matrix runs write `ticks.jsonl`, `summary.json`, `config.yaml`, and, when plotting dependencies are available, `plot.png`. The plot script uses commanded planar speed vs planar position divergence as its primary panel. |
| Rate chosen from evidence, not arbitrary 100 Hz | Matrices compare `5`, `10`, `20`, and `30 Hz` for the 2.0 m/s envelope and `10` vs `20 Hz` for speed-max. Higher rate did not unlock failing cases; the live default remains `10 Hz`. |
| No Pure Pursuit for holonomic | Validation uses the holonomic feedforward plus proportional pose-tracking law through the runner and live `LocalPlanner` harness. |
| Curvature and stopping behavior | Direct constant-speed stress tests failed at running speed; path-speed-profile validation passes the supported envelopes by slowing for curvature and near-goal decel before the controller sees the reference. |
| Planner-side obstacle policy remains separate | The simulation validates trajectory control, speed profiling, telemetry, and tolerance/gate behavior. It does not rewrite obstacle clearance or planner policy. |
| P8 remains post-merge hardware sign-off | The hardware checklist remains in [`trajectory_operator_guide_and_hardware_checklist.md`](trajectory_operator_guide_and_hardware_checklist.md#hardware-checklist). |

## Simulation Levels Run

| Level | Scope | Result |
| --- | --- | --- |
| Level 0 - Unit and analytic regressions | Focused 921 tests for types, metrics, command limits, tick export, plotting inputs, calibration, replay, analytic paths, controller swap, and live planner path controller behavior. | Passing in the recorded SIM-08 proof: `44 passed` for the runner plus local planner focused modules. |
| Level 1 - Controller plant sweep | Standalone runner matrix across path scenario, target speed, plant response preset, rate, and path-speed mode. | Direct stress matrix exposed failures; supported profiled-reference matrices pass the stated 2.0 m/s envelope. |
| Level 2 - `LocalPlanner` in the loop | Test-only harness subscribes to live `cmd_vel`, steps an ideal plant, feeds odom through `handle_odom`, and writes live JSONL through `local_planner_trajectory_tick_log_path`. | Covers straight arrival, curvature speed cap, near-goal decel, initial rotation, and conservative 2.0 m/s right-angle profile. |
| Level 3 - Replay, delay, and disturbance stress | Runner supports measurement delay, jitter, stale sample reuse, dropped samples, nonlinear response, lateral asymmetry, and bounded noise. | Bad delayed runs fail gates instead of passing silently; noisy/asymmetric envelopes are included in the supported matrices. |
| Speed-max extension | Separate long-geometry matrix for `3.7 m/s` running target. | Supports synthetic nominal/asymmetric/noisy and ideal on speed-max geometries; rejects sluggish as a running-speed plant. |

## Plant Presets And Response Curves

The artifact schema and response-curve model are defined in [`trajectory_simulation_artifacts.md`](trajectory_simulation_artifacts.md#response-curve-and-inertia-model). Every simulation run records the response-curve source, preset id, time constants, acceleration caps, command gain, deadband, saturation, noise, latency, and seed where applicable.

| Preset | Source | Purpose | PR interpretation |
| --- | --- | --- | --- |
| `ideal` | `ideal` | Mathematical baseline with instant or near-instant response. | Confirms controller math and geometry handling, not hardware behavior. |
| `synthetic_nominal` | `synthetic` | Default synthetic lag, acceleration, deadband, and saturation envelope. | Main synthetic confidence preset. |
| `synthetic_asymmetric` | `synthetic` | Slower or direction-specific lateral and yaw response. | Checks that lateral asymmetry does not silently break the holonomic controller. |
| `synthetic_noisy` | `synthetic` | Bounded command noise with recorded seed. | Checks deterministic noisy-envelope behavior and speed-vs-divergence margins. |
| `synthetic_sluggish` | `synthetic` | Slow response and low acceleration stress preset. | Lower-speed safety envelope; not a supported running-speed claim. |
| `fitted_go2_*` | `fitted_from_go2_log` | Future command-aligned Go2 fit. | Not available in this workspace, so no empirical Go2 response-curve claim is made. |

## Scenario Matrix Summary

| Matrix | Scope | Result | Reviewer artifact |
| --- | --- | --- | --- |
| SIM-04 direct stress | `320` runs: speeds `0.55`, `1.0`, `1.5`, `2.0`; rates `5`, `10`, `20`, `30 Hz`; five plants; line, circle, S-curve, right-angle; direct references. | Runner verdicts: `240` pass, `80` fail. At `2.0 m/s`, only ideal mostly looked acceptable; synthetic nominal/sluggish/noisy failed heavily. This proved direct constant-speed references are not the PR-confidence path. | [`matrix_summary.md`](../simulation/simulation_runs/2026-04-27T111304Z_sim04_initial_matrix/matrix_summary.md) |
| SIM-08 2.0 m/s conservative profile | `48` runs: `2.0 m/s`; rates `5`, `10`, `20`, `30 Hz`; `synthetic_nominal`, `synthetic_asymmetric`, `synthetic_noisy`; four path scenarios; `path_speed_profile`. | Runner verdicts: `48` pass, `0` fail. Matrix classifications: `36` pass, `12` borderline. Borderlines are circle-policy cases with max divergence about `0.02-0.048 m`, not gate failures. | [`matrix_summary.md`](../simulation/simulation_runs/sim08_verify_3plant_2mps_conservative/matrix_summary.md) |
| SIM-08 sluggish right-angle | `4` runs: `1.0 m/s`; `synthetic_sluggish`; right-angle path; rates `5`, `10`, `20`, `30 Hz`; `path_speed_profile`. | Runner verdicts: `4` pass, `0` fail. Confirms sluggish tight-turn support at `1.0 m/s`, while preserving the no-`2.0 m/s` sluggish claim. | [`matrix_summary.md`](../simulation/simulation_runs/sim08_verify_sluggish_ra_1mps/matrix_summary.md) |
| SIM-11 speed-max | `250` runs: speeds `2.0`, `2.5`, `3.0`, `3.5`, `3.7`; rates `10`, `20 Hz`; five plants; five speed-max geometries. | Matrix classifications: `194` pass, `2` borderline, `54` fail. At `3.7 m/s`, ideal, synthetic nominal, asymmetric, and noisy pass all five speed-max scenarios at both rates; sluggish fails all scenarios. | [`matrix_summary.md`](../simulation/simulation_runs/sim11_speed_max_matrix_2026-04-27_verify/matrix_summary.md) |

## Speed-Vs-Divergence Plots

The simulation runner emits per-run `ticks.jsonl` plus optional `plot.png` artifacts. The representative plot paths are listed inside each matrix summary. The run directories under `simulation_runs/` are generated artifacts and may be regenerated locally from the commands recorded in [`robot_running_simulation_plan.md`](robot_running_simulation_plan.md).

For reviewers, the most useful plot sets are:

- SIM-08 2.0 m/s conservative profile: representative speed-vs-divergence plots for every supported path and plant are listed in the matrix summary.
- SIM-08 sluggish 1.0 m/s right-angle: representative sluggish-envelope plot is listed in the matrix summary.
- SIM-11 speed-max: inspect `matrix_results.json` plus per-run `plot.png` after rerunning without `--no-plot` if image artifacts are needed for review.

Plot command for any generated tick file:

```bash
uv run --with matplotlib python docs/development/921_trajectory_controller/simulation/plot_trajectory_control_ticks.py path/to/ticks.jsonl -o plot.png
```

## Control-Rate Comparison

The 2.0 m/s conservative profile matrix tested `5`, `10`, `20`, and `30 Hz`. Every supported nominal/asymmetric/noisy run passed at every rate, and the worst right-angle divergences slightly increased at higher rates in the synthetic envelopes. Higher rate did not fix the failing clusters in earlier default-cap tests.

The speed-max matrix tested `10` and `20 Hz`. At `3.7 m/s`, both rates pass for ideal, synthetic nominal, asymmetric, and noisy across all speed-max scenarios. Sluggish fails at both rates, which points to plant response and path envelope limits rather than Python loop frequency.

Conclusion: keep the live default `local_planner_control_rate_hz=10.0` before hardware. Do not raise control rate for the PR without fitted Go2 timing data showing a measured benefit on speed-vs-divergence plots.

## Implementation Changes Made Because Of Simulation

Simulation changed the software in targeted ways rather than by hiding failures behind looser gates:

- Added `path_speed_profile` reference mode to the controller simulation runner so matrix runs can exercise live-like tangent, normal, and goal deceleration caps.
- Added runner config input, calibration gain loading, gain sweep knobs, and live-like acceleration cap knobs.
- Surfaced live holonomic command acceleration and yaw limits through `GlobalConfig` instead of keeping them hard-coded in `HolonomicPathController`.
- Added `local_planner_goal_decel_m_s2` so near-goal deceleration is no longer implicitly tied to the tangent acceleration cap.
- Added Level 2 `LocalPlanner` in-loop harness coverage for live `cmd_vel`, odom feedback, arrival, curvature capping, near-goal decel, initial rotation, and JSONL sink behavior.
- Added disturbance wrappers and tests for delayed, jittered, stale, dropped, noisy, and asymmetric measurements.
- Added speed-max scenario geometries and the `run_sim_speed_max_matrix.py` driver so a `3.7 m/s` claim is tested on geometry sized for running speed.

The matrix gates were not relaxed to make failures disappear. Direct constant-speed failures remain documented as stress evidence, and sluggish response remains excluded from running-speed support.

## Known Limits Before P8

- This is a synthetic-envelope report, not empirical Go2 sign-off. A fitted Go2 claim requires command-aligned Go2 logs and a `fitted_go2_*` preset.
- The synthetic sluggish envelope does not support `2.0 m/s` tight turns or `3.7 m/s` speed-max behavior. If P8 data resembles this plant, lower tangent caps or lower speed are required.
- The `3.7 m/s` speed-max claim only applies to long straight, wide arc, wide S-curve, stop-distance, and filleted-corner geometries. It does not apply to bare sharp polyline corners or sub-9 m approaches before a sharp turn.
- Obstacle clearance policy remains planner-side and is not validated by these simulations.
- Generated `simulation_runs/` artifacts are reproducible but not all committed; reviewers should use the matrix summaries and rerun commands to recreate PNGs or inspect raw JSONL locally.
- Hardware validation must start with slow-speed bring-up, live JSONL capture, speed-vs-divergence plots, and the P8 checklist in [`trajectory_operator_guide_and_hardware_checklist.md`](trajectory_operator_guide_and_hardware_checklist.md#hardware-checklist).

## Reproduction Commands

Focused tests:

```bash
uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py dimos/navigation/replanning_a_star/test_local_planner_path_controller.py -q
```

Supported 2.0 m/s synthetic-envelope matrix:

```bash
uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim08_verify_3plant_2mps_conservative --speeds 2.0 --plants synthetic_nominal,synthetic_asymmetric,synthetic_noisy --scenarios line,circle,s_curve,right_angle_turn --rates 5,10,20,30 --reference-mode path_speed_profile --local-planner-max-tangent-accel-m-s2 0.5 --local-planner-max-normal-accel-m-s2 0.1 --local-planner-goal-decel-m-s2 0.5
```

Sluggish lower-speed check:

```bash
uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim08_verify_sluggish_ra_1mps --speeds 1.0 --plants synthetic_sluggish --scenarios right_angle_turn --rates 5,10,20,30 --reference-mode path_speed_profile --local-planner-max-tangent-accel-m-s2 0.5 --local-planner-max-normal-accel-m-s2 0.1 --local-planner-goal-decel-m-s2 0.5
```

Speed-max matrix:

```bash
uv run python docs/development/921_trajectory_controller/simulation/run_sim_speed_max_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim11_speed_max_matrix_2026-04-27_verify --no-plot
```
