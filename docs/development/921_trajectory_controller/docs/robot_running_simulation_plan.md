# Issue 921 Robot Running Simulation Plan

**Status:** simulation planning document for `feat/921-trajectory-controllers`. (https://github.com/dimensionalOS/dimos/issues/921)

**Goal:** define how to simulate the robot running behavior leshy asked about before opening the issue 921 PR, using the implementation already present on the branch, and define a trackable plan that an LLM can execute, monitor, and update.

**Core claim to prove:** with `planner_robot_speed` raised above the conservative default, the holonomic local follower remains stable, bounded, observable, and tunable across a conservative response-curve envelope. If real Go2 data is available, that envelope should include fitted Go2 response curves; otherwise it must be labeled synthetic. Simulation should give PR-confidence that the software is correct before real P8 hardware validation.

## Part A - Simulation Process And Gap Analysis

### 1. What Leshy Asked For

Issue 921 asks for Unitree Go2 to "run not walk" without the current speed-up failure mode: oscillation, overshoot, and guessed controller numbers. The binding requirements from leshy are:

- Use abstractions that let us plug in different plants or controllers, run calibration, take measurements, graph controller performance, and compare controllers.
- Use a primary performance graph like "speed vs divergence from target".
- Decide speed, precision, and control rate from measured data, not from a hard-coded or arbitrary high Python loop rate.
- Do not use Pure Pursuit or other car-style path laws for a holonomic base.
- Keep obstacle and clearance policy planner-side; issue 921 provides controller telemetry and tolerance concepts, not a full obstacle-planner rewrite.

The recent task from leshy adds one specific missing validation mode: simulate the running behavior of the robot using inertia, response curves, and related plant effects before opening the PR.

### 2. What "Simulation" Means For This Branch

For this branch, simulation should be treated as a closed-loop software-in-the-loop harness around the same surfaces used by the live robot:

```text
reference path sample
  -> holonomic trajectory controller
  -> command limits and slew
  -> plant model with response curve
  -> measured pose and twist
  -> trajectory tick JSONL
  -> speed-vs-divergence plots and pass/fail metrics
```

The useful plant model is not a full rigid-body simulator. The local planner commands high-level body-frame `Twist`, so the model needs to represent the effective input-output behavior that matters to this controller:

- Body-frame velocity lag from commanded `vx`, `vy`, `wz` to realized velocity.
- Acceleration and yaw acceleration limits.
- Optional bounded command or measurement noise.
- Optional latency, odom sample delay, packet jitter, and dropped measurement updates.
- Optional lateral-response asymmetry, because Go2 may not respond identically to `vx` and `vy`.
- Optional ground-slip or speed-dependent degradation for high-speed trials.

In leshy's "inertia, response curves etc." framing, "inertia" should be modeled here as the effective response of the Go2 velocity servo, not as a mass-matrix or contact simulator. At this control layer the useful model is:

```text
u_cmd = commanded body-frame twist
u_eff = response_curve(u_cmd, speed, axis, direction)
v_realized[t + dt] = v_realized[t] + clamp((u_eff - v_realized[t]) * dt / tau, accel_limit * dt)
pose[t + dt] = integrate_body_twist(pose[t], v_realized[t + dt], dt)
```

The response curve can start simple, but it must be explicit and recorded per run:

- Per-axis time constants for forward, lateral, and yaw response.
- Per-axis acceleration limits that stand in for effective inertia.
- Optional command gain, deadband, saturation, and direction asymmetry.
- Optional speed-dependent slip or degradation above a selected running-speed threshold.
- Optional transport and sensing timing effects outside the plant: delay, jitter, stale samples, and dropped odom updates.

The existing branch already models lag, acceleration limits, and command noise through `ActuatedHolonomicPlant`. The important audit correction is that the current code does **not** yet model identified Go2 response curves; it only gives the primitives for a synthetic first pass. Any PR-confidence claim must therefore separate "synthetic plant envelope passed" from "empirically fitted Go2 envelope passed".

### 3. What We Already Have

The branch has the essential issue 921 software pieces:

- `TrajectoryController` protocol, frozen reference and measurement samples, and explicit command limits.
- `HolonomicTrackingController`, which is a Cartesian feedforward plus proportional pose-tracking law and is not Pure Pursuit.
- `HolonomicPathController` wired into `LocalPlanner`, with holonomic as the default path controller.
- `planner_robot_speed` plumbing for high-speed trials without code edits.
- Live path speed capping from distance-to-goal and local curvature via tangent and normal acceleration limits.
- Per-tick slew limiting on the command emitted by `HolonomicPathController`.
- `IntegratedHolonomicPlant`, an ideal body-frame twist integrator.
- `ActuatedHolonomicPlant`, a deterministic first-order lag plus per-axis acceleration-limit plant with optional bounded noise.
- Closed-loop analytic path tests for line and arc references against ideal and actuated plants.
- A controller-swap regression proving the harness can compare `HolonomicTrackingController` against another protocol-shaped controller.
- A tiny in-tree odom replay fixture and golden replay regression that avoid LFS and network.
- Live `LocalPlanner` JSONL telemetry through `local_planner_trajectory_tick_log_path`.
- A plotting script whose primary panel is `commanded_planar_speed_m_s` vs `planar_position_divergence_m`.
- A rate-selection guide that compares nominal `dt_s`, actual `wall_time_s` gaps, plant delay, and speed-vs-divergence plots.

This means the branch can already test the controller law, command limiting, calibration output, replay loading, and the live logging path.

### 4. What Is Missing For PR-Confidence Simulation

The remaining gap is not "no simulator". It is that the simulator is not yet packaged as a PR-facing running-validation harness.

Missing items:

- No single script or test target runs the complete simulation matrix: speed, control rate, gains, path shape, and plant response parameters.
- No generated artifact bundle combines JSONL, PNG plots, summary metrics, config, and pass/fail verdict.
- No pass/fail gates are defined for "ready to open PR" at running speeds.
- No response-curve schema exists. A run cannot yet record whether its plant preset is ideal, synthetic, fitted from a Go2 log, or merely copied from a unit test.
- No explicit Go2-like response parameter set exists. Current tests use simple values such as `linear_lag_time_constant_s=0.08`, but they are not documented as "synthetic nominal", "fitted Go2 conservative", or "bad but plausible".
- No curve-fitting workflow exists to turn a bounded step, dwell, or replay log into per-axis `tau`, acceleration, deadband, gain, saturation, and asymmetry parameters.
- No simulated transport or sensing delay exists in `ActuatedHolonomicPlant`.
- No simulated odom jitter, stale odom, or dropped samples exist.
- No lateral asymmetry exists for `vx` vs `vy`.
- No speed-dependent slip or tracking degradation exists.
- The live `LocalPlanner` path speed and state machine are tested, but there is no reusable offline harness that drives `LocalPlanner` end-to-end with a plant and an updating odom loop.
- Calibration YAML is generated, but not automatically loaded into `GlobalConfig`; simulation must explicitly apply suggested gains.
- The yaw-rate cap and acceleration limits inside `HolonomicPathController` are still partly hard-coded. This is acceptable for the current implementation audit, but a simulation sweep may expose a need to surface them through config.
- The mini replay is too small to validate running behavior. It validates replay wiring and golden math, not a sustained path-following run.

### 5. Simulation Levels

Use four levels. Do not skip directly to the most complicated level.

#### Level 0 - Unit And Analytic Regressions

Purpose: prove the math and local harness still work.

Inputs:

- Existing analytic line and arc tests.
- Existing command-limit, path-speed-profile, controller-swap, tick-export, replay, and calibration tests.

Outputs:

- Focused 921 pytest results.
- No generated running confidence artifact.

Gate:

- All focused 921 tests pass.

#### Level 1 - Controller Plant Sweep

Purpose: answer "if the controller sees a known reference and a documented response-curve plant, does it remain stable at running speeds?"

Harness shape:

- Generate analytic references: line, circle, S-curve or two joined arcs, right-angle turn, stop-at-goal.
- For each tick, sample reference, sample plant measurement, compute controller command, clamp, step plant, and append a trajectory control tick.
- Repeat across a matrix of speed, rate, gains, lag, acceleration limit, yaw lag, noise, and initial offset.

Required outputs per run:

- `ticks.jsonl`
- `plot.png`
- `summary.json`
- `config.yaml`

Suggested initial scenario matrix:

- Speeds: `0.55`, `1.0`, `1.5`, `2.0` m/s.
- Rates: `5`, `10`, `20`, `30` Hz.
- Position gains: current default, calibration suggested, lower by 25%, higher by 25%.
- Paths: 4 m line with lateral offset, 1 m radius circle, 1.5 m radius S-curve, 90 degree corner with 1 m approach and exit.
- Plants:
  - ideal: no lag, high acceleration.
  - synthetic_nominal: seed values such as `linear_lag_time_constant_s=0.08`, `yaw_lag_time_constant_s=0.08`, `max_linear_accel_m_s2=4.0`, `max_yaw_accel_rad_s2=4.0`; do not call this "Go2-like" until a source or fit supports it.
  - synthetic_sluggish: seed values such as `linear_lag_time_constant_s=0.18`, `yaw_lag_time_constant_s=0.15`, `max_linear_accel_m_s2=2.0`, `max_yaw_accel_rad_s2=2.0`.
  - synthetic_asymmetric: nominal forward response with slower lateral and yaw response.
  - noisy: synthetic nominal plus bounded linear and yaw noise.
  - fitted_go2_nominal or fitted_go2_conservative: only after a real calibration or replay source exists.

Minimum metrics:

- `max_planar_position_divergence_m`
- `p95_planar_position_divergence_m`
- `final_planar_position_divergence_m`
- `max_abs_heading_error_rad`
- `p95_abs_cross_track_m`
- `max_commanded_planar_speed_m_s`
- `mean_commanded_planar_speed_m_s`
- `max_linear_accel_command_delta_m_s2`
- `max_yaw_accel_command_delta_rad_s2`
- `nominal_hz`
- `simulated_success`

Initial PR-confidence gates:

- At `1.5 m/s` on synthetic nominal or fitted nominal response curves, p95 planar divergence stays below `0.20 m` for line, circle, and S-curve.
- At `2.0 m/s` on synthetic nominal or fitted nominal response curves, no divergence blow-up: max planar divergence stays below `0.35 m` on all non-corner paths.
- At all rates tested, 10 Hz is not obviously dominated unless plots and metrics show a clear improvement at a higher achieved rate.
- Synthetic sluggish and fitted conservative plants either pass lower-speed gates or fail in a controlled, explainable way that recommends lower speed, lower normal acceleration, or different gains.
- Controller-swap baseline still shows feedback controller beating feedforward-only on the same scenario.

The numeric gates are starting points. The first LLM pass should make them explicit and adjust only if the generated plots show they are either too strict for a documented fitted or conservative synthetic envelope, or too loose to catch instability.

#### Level 2 - LocalPlanner-In-The-Loop Simulation

Purpose: test the actual issue 921 integration path, not only the inner controller.

Harness shape:

- Construct a `LocalPlanner` with a fake or lightweight `NavigationMap`, path, odom stream, and a writable trajectory tick log path.
- Subscribe to `cmd_vel`.
- Feed each emitted command into a plant.
- Feed the next plant pose back into `LocalPlanner.handle_odom`.
- Let `LocalPlanner` run through initial rotation, path following, final rotation, and arrived.

Required observations:

- `LocalPlanner` writes JSONL through the same sink used in live runs.
- `LocalPlanner._path_speed_for_index` slows tight curves and near-goal approach.
- The controller sees the configured `planner_robot_speed`.
- The loop exits with `arrived` for pass scenarios.
- `wall_time_s` exists, and achieved inter-arrival does not show gross scheduling slip in the offline harness.

This level is the most important missing bridge before PR open, because it proves the path-speed cap, state machine, telemetry, controller wrapper, and plant feedback cooperate.

#### Level 3 - Replay And Delay Stress

Purpose: test that the code remains interpretable under non-ideal measurement timing.

Harness shape:

- Add a plant wrapper or measurement wrapper that can delay odom by N ticks, reuse stale odom, add timestamp jitter, or drop updates.
- Run Level 1 and Level 2 scenarios with delay values that represent plausible WebRTC or odom latency.

Initial stress cases:

- 1 tick odom delay at 10 Hz.
- 2 tick odom delay at 10 Hz.
- Random 0 to 1 tick jitter.
- 5% dropped odom updates.

Pass criteria:

- The harness detects degraded performance in metrics and plots.
- It does not silently claim a bad run is good.
- The recommended action is clear: lower speed, lower gains, lower normal acceleration, or change implementation if no conservative setting passes.

### 6. What "100% Confidence" Can Mean

Simulation cannot prove real hardware behavior with mathematical certainty unless the model is identified from real Go2 data. It can provide PR-level confidence if:

- The implemented software contracts are covered by tests.
- The controller remains stable across a conservative response-curve envelope.
- The full live integration path is simulated with a plant and telemetry.
- The plots and summary metrics are generated automatically and are easy for reviewers to inspect.
- Known missing physics are explicit and deferred to P8 hardware validation, not hidden.

For this PR, "100% confidence before opening PR" should mean "no known software-level blocker remains, and the simulation harness would have caught the failure modes issue 921 describes: oscillation, overshoot, wrong holonomic law, arbitrary Hz, missing telemetry, and bad speed-vs-divergence behavior."

## Part B - LLM-Executable Trackable Plan

### Execution Rules For The LLM

- Shell examples and paths such as `docs/development/921_trajectory_controller/simulation/` are relative to the `dimos/` repository root unless stated otherwise. Issue 921 plotting and matrix driver scripts live next to `simulation_runs/` under `dimos/docs/development/921_trajectory_controller/simulation/`.
- Work on `feat/921-trajectory-controllers` in the `dimos` repo.
- Do not remove or weaken existing issue 921 tests.
- Keep new validation artifacts deterministic unless a seed is explicitly recorded.
- Store reusable code, tests, fixtures, and review-worthy docs under `dimos/`.
- Every generated simulation run must record commit, config, seed, scenario name, plant parameters, speed, gains, rate, and pass/fail result.
- Prefer additive harnesses and focused tests over changing controller behavior first. Only change implementation after a failing simulation points to a specific code issue.

### SIM-00 - Baseline Repository Check

Status: `[x]`

Goal: confirm the branch is in a known state.

Steps:

- Run `cd dimos && git status`.
- Confirm current branch is `feat/921-trajectory-controllers` or the intended local equivalent.
- Run the focused 921 test suite

Command (from `dimos/` root):

```bash
source .venv/bin/activate
uv run -m pytest -q --tb=short \
  dimos/navigation/test_trajectory_*.py \
  dimos/navigation/replanning_a_star/test_local_planner_path_controller.py
```

- Run the existing trajectory smoke subset.

```bash
source .venv/bin/activate
uv run -m pytest -q --tb=short --durations=5 \
  -m "not (tool or mujoco or lfs_data)" \
  dimos/navigation/test_trajectory_holonomic_calibration.py \
  dimos/navigation/test_trajectory_replay_loader.py \
  dimos/navigation/test_trajectory_golden_replay.py
```

- Run `ruff check` on touched issue 921 Python files if any edits are made.

Deliverables:
- None

Done when:

- Existing tests pass or failures are recorded as pre-existing blockers.

### SIM-01 - Define Simulation Artifact Schema

Status: `[x]`

Goal: define machine-readable outputs before writing the runner.

Steps:

- Create a schema note for `summary.json` and `config.yaml`.
- Include scenario, plant, response-curve source, controller, limits, rate, seed, metrics, gates, and verdict fields.
- Reuse existing JSONL tick schema for per-tick data.

Deliverables:

- `dimos/docs/development/921_trajectory_controller/docs/trajectory_simulation_artifacts.md`

Done when:

- An LLM can read the doc and know exactly which files a simulation run must emit.

### SIM-01A - Define Response-Curve And Inertia Model

Status: `[x]`

Goal: make leshy's "inertia, response curves etc." request executable before tuning the controller.

Steps:

- Define a versioned response-curve config shape. At minimum include source, axis, direction, `tau_s`, max acceleration, command gain, deadband, saturation, noise caps, latency, and notes.
- State explicitly whether each preset is `ideal`, `synthetic`, `fitted_from_go2_log`, or `unknown`. The runner should reject `unknown` for PR-gate runs.
- Add synthetic presets for ideal, nominal, sluggish, asymmetric, noisy, and conservative timing stress.
- If a real Go2 step, dwell, or replay log is available, fit the measured command-to-velocity response into a `fitted_go2_*` preset. If not, keep all Go2-language claims out of the verdict and call the run a synthetic envelope simulation.
- Document the equations used by the plant so another LLM can change the implementation without changing the meaning of historical results.

Deliverables:

- Response-curve section in `dimos/docs/development/921_trajectory_controller/docs/trajectory_simulation_artifacts.md`, or a dedicated `dimos/docs/development/921_trajectory_controller/docs/trajectory_response_curve_model.md`.
- Preset config file or Python mapping used by the runner.

Done when:

- Every simulation run records enough plant parameters to reproduce the same response curve.
- The final report can distinguish synthetic confidence from empirically fitted Go2 confidence.

Verified:
`uv run pytest dimos/navigation/test_trajectory_response_curve_presets.py`

### SIM-02 - Build Controller Plant Simulation Runner

Status: `[x]`

Goal: package Level 1 simulation as a reusable script.

Suggested location:

- `dimos/docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py`

Required features:

- CLI args for scenario, speed, rate, gains, plant preset, output directory, and random seed.
- Built-in path scenarios: line, circle, S-curve, right-angle turn, stop-at-goal.
- Built-in plant or response-curve presets: ideal, synthetic nominal, synthetic sluggish, synthetic asymmetric, noisy, and fitted Go2 presets when data exists.
- Writes existing trajectory tick JSONL.
- Calls or documents calling `docs/development/921_trajectory_controller/simulation/plot_trajectory_control_ticks.py`.
- Writes `summary.json` with metrics and gate verdict.

Done when:

- One command can produce `ticks.jsonl`, `plot.png`, `summary.json`, and `config.yaml` for a synthetic nominal `1.5 m/s`, `10 Hz`, S-curve run.

Verified (from `dimos/` root):

`uv run python -m py_compile docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py`

`uv run --with matplotlib python docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py --scenario s_curve --speed 1.5 --rate 10 --plant-preset synthetic_nominal --output-dir docs/development/921_trajectory_controller/simulation/simulation_runs/sim02_synthetic_nominal_s_curve_1p5_10hz --seed 7`

Nominal S-curve run produced all four artifacts under `docs/development/921_trajectory_controller/simulation/simulation_runs/sim02_synthetic_nominal_s_curve_1p5_10hz/` with verdict pass.

### SIM-03 - Add Focused Tests For The Runner

Status: `[x]`

Goal: keep the runner PR-safe and deterministic.

Steps:

- Add a fast pytest module for the runner.
- Test that a synthetic nominal line or circle produces JSONL and summary output.
- Test that a deliberately bad feedforward-only or low-gain case fails a gate.
- Test that a seeded noisy run is reproducible.

Done when:

- Tests pass in default Linux CI without network, LFS, or matplotlib if possible.
- Plot generation can remain an optional integration path if matplotlib is not available by default.

Verified (from `dimos/` root):

`uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py -q`

Focused runner tests passed: synthetic nominal line/circle artifact output, low-gain gate failure, and seeded noisy-run reproducibility.

### SIM-04 - Run Initial Matrix And Record Results

> Note: you may want to read only this step and run the matrix with tuned parameters that much more correctly describe the sim environment. This step had to be initial to have a base to work with. Check SIM-04-02 - Tune Plan To Make Current Go2 Matrix Pass At `2.0 m/s`.

Status: `[x]`

Goal: establish the first simulation confidence report.

Preset plants, scenario geometry for the initial SIM-04 sweep, and how matrix runs lay out under `run_sim04_initial_matrix.py` are summarized in [`matrix_presets_and_scenarios.md`](../simulation/matrix_presets_and_scenarios.md#sim-04-initial-matrix).

Matrix:

- Speeds: `0.55`, `1.0`, `1.5`, `2.0`.
- Rates: `5`, `10`, `20`, `30`.
- Plants or response curves: ideal, synthetic nominal, synthetic sluggish, synthetic asymmetric, noisy, and fitted Go2 presets if available.
- Paths: line, circle, S-curve, right-angle turn.
- Gains: default and calibration-suggested if calibration output exists.

Deliverables (paths relative to the `dimos/` repo root):

- `docs/development/921_trajectory_controller/simulation/simulation_runs/<timestamp>_sim04_initial_matrix/matrix_summary.md`
- One subdirectory per scenario run containing artifacts.

Done when:

- The matrix summary identifies pass, fail, and borderline cases.
- It includes at least one speed-vs-divergence plot per path and plant preset.

Verified (from `dimos/` root):

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py`

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/<timestamp>_sim04_initial_matrix --only-missing`

`uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py -q`

> The direct SIM-04 2.0 m/s matrix is a stress test: "what happens if we force the reference to stay fast through all geometry?"

Exact SIM-04 reproduction command (320 runs; allow on the order of a few minutes; each subprocess may invoke plotting if matplotlib is available):

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --speeds 0.55,1.0,1.5,2.0 --plants ideal,synthetic_nominal,synthetic_sluggish,synthetic_asymmetric,synthetic_noisy --scenarios line,circle,s_curve,right_angle_turn --rates 5,10,20,30 --reference-mode direct --k-position-values 2.2 --k-yaw-values 2.5`

**SIM-04 direct matrix (320 runs)** reproduced with the command above (same runner and matrix driver as in this doc). Latest run in this workspace: run root `docs/development/921_trajectory_controller/simulation/simulation_runs/<timestamp>_sim04_initial_matrix/` (Started `<timestamp>`, Finished `<timestamp>`). Aggregated report: [`matrix_summary.md`](../simulation/simulation_runs/<timestamp>T111304Z_sim04_initial_matrix/matrix_summary.md). Runner verdicts: 240 pass, 80 fail, 0 invalid. Matrix classifications: 195 pass, 45 borderline, 80 fail, 0 invalid. That folder is listed in `.gitignore`; clone the repo and rerun the same command to regenerate artifacts locally.
The report includes representative speed-vs-divergence plots for every path and plant preset.
No fitted Go2 presets or generated calibration output were available, so the matrix uses default gains and remains a synthetic-envelope confidence report.

### SIM-04-results - Matrix Results By Speed

Status: `[x]`

Goal: make the SIM-04 result readable as speed bands, not only aggregate pass/fail counts.

Summary by target speed (aggregated from direct SIM-04 matrix run `<run_timestamp>_sim04_initial_matrix`; same counts as the historical <timestamp> snapshot):

| Speed | Runner verdict pass | Runner verdict fail | Matrix pass | Borderline | Matrix fail | Interpretation |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `0.55 m/s` | 80 | 0 | 68 | 12 | 0 | Conservative walking baseline. Every run completed under broad gates; sluggish and noisy plants explain the borderline cases. |
| `1.0 m/s` | 79 | 1 | 68 | 11 | 1 | Fast-walk band. Only one broad-gate failure, in the sluggish plant. |
| `1.5 m/s` | 62 | 18 | 45 | 17 | 18 | First running-speed target. Ideal and nominal mostly complete, but stricter PR-confidence checks miss and sluggish/noisy plants start failing. |
| `2.0 m/s` | 19 | 61 | 14 | 5 | 61 | Aggressive running stress target. It is acceptable mostly only in the ideal plant; realistic synthetic envelopes fail heavily. |

Important `2.0 m/s` detail:

- `ideal`: 14 pass, 1 borderline, 1 fail.
- `synthetic_nominal`: 16 fail out of 16.
- `synthetic_sluggish`: 16 fail out of 16.
- `synthetic_noisy`: 16 fail out of 16.
- `synthetic_asymmetric`: 4 borderline, 12 fail.

So the correct conclusion is not "all `2.0 m/s` runs failed". The correct conclusion is: `2.0 m/s` only looks acceptable in the near-perfect ideal plant. Under the synthetic plants that are meant to approximate inertia, lag, noise, and asymmetry, `2.0 m/s` is not ready with **the current default gains and current scenario geometry** and need tuning. See step SIM-04-02 - Tune Plan To Make Current Go2 Matrix Pass At `2.0 m/s`.

Backlog usage check:

- SIM-04 uses the P1/P3 controller contract and command limits through `HolonomicTrackingController` and `clamp_holonomic_cmd_vel`.
- SIM-04 uses the P2 telemetry stack by writing `ticks.jsonl` with `write_trajectory_control_ticks_jsonl`.
- SIM-04 uses the P2 speed-vs-divergence plotting script through `docs/development/921_trajectory_controller/simulation/plot_trajectory_control_ticks.py`.
- SIM-04 uses the P5 response-curve plant family through `trajectory_response_curve_presets.py` and the runner's `ResponseCurvePlant`.
- SIM-04 records schema-compliant `config.yaml` and `summary.json` per `dimos/docs/development/921_trajectory_controller/docs/trajectory_simulation_artifacts.md`.
- SIM-04 does not yet exercise the P3-3 live `LocalPlanner` wrapper, `HolonomicPathController`, obstacle handling, initial/final rotation states, or `LocalPlanner._path_speed_for_index`. That is intentionally deferred to SIM-06, but it means direct SIM-04 failures on tight high-speed curves should not be interpreted as the final live planner result.

Verified (from `dimos/` root):

[`matrix_results.json`](../simulation/simulation_runs/<run_timestamp>_sim04_initial_matrix/matrix_results.json) (320 rows; re-aggregate by `speed_m_s`, `classification`, and `summary.verdict.status` as needed). Example:

`uv run python -c "import json,collections; p='docs/development/921_trajectory_controller/simulation/simulation_runs/<run_timestamp>_sim04_initial_matrix/matrix_results.json'; r=json.load(open(p)); print(len(r),'runs')"`

### SIM-04-02 - Tune Plan To Make Current Go2 Matrix Pass At `2.0 m/s`

Status: `[x]`

Goal: identify the concrete code and parameter changes to try so all current Go2-relevant SIM-04 scenarios can pass at `2.0 m/s` before making any hardware claim.

Default SIM-04 sweep dimensions and scenario definitions are documented in [`matrix_presets_and_scenarios.md`](../simulation/matrix_presets_and_scenarios.md#sim-04-initial-matrix).

Current blocker:

- In SIM-04, every `synthetic_nominal`, `synthetic_sluggish`, and `synthetic_noisy` run at `2.0 m/s` failed.
- The strict synthetic nominal PR checks also failed at `2.0 m/s`: max planar divergence was roughly `1.06-1.45 m` on line, circle, and S-curve where the target was `0.35 m`.
- Worst broad-gate failures were dominated by `synthetic_sluggish` at `2.0 m/s`, especially straight line and S-curve cases, with max divergence over `2.6-3.8 m`.

Likely causes to investigate:

- The original direct SIM-04 runner tracks analytic references at the requested speed. That remains available as `--reference-mode direct`, but the runner now also has `--reference-mode path_speed_profile` to apply live-like tangent, normal, and goal deceleration caps before the controller sees the reference.
- The default runner gains are fixed at `k_position_per_s=2.2` and `k_yaw_per_s=2.5`, while live defaults are `local_planner_holonomic_kp=2.0` and `local_planner_holonomic_ky=1.5`. The matrix driver can now sweep `--k-position-values` and `--k-yaw-values`, and the runner can load calibration-suggested gains from `--calibration-params-yaml`.
- Runner command limits are still explicit CLI inputs: `--max-planar-accel`, `--max-yaw-accel`, and `--max-yaw-rate`. Live `HolonomicPathController` no longer hard-codes planar/yaw acceleration limits or yaw-rate policy; it reads `GlobalConfig.local_planner_max_planar_cmd_accel_m_s2`, `local_planner_max_yaw_accel_rad_s2`, and optional `local_planner_max_yaw_rate_rad_s`.
- The synthetic nominal plant itself has `linear_x/linear_y tau_s=0.18`, max acceleration `1.5 m/s^2`, deadband `0.02`, and saturation `2.0`. A plant with a `1.5 m/s^2` acceleration limit is marginal for aggressive `2.0 m/s` path changes unless the reference speed profile is conservative.
- The current controller is proportional plus feedforward. It has no integral action, no derivative damping, no model-based lag compensation, and no curvature/yaw feedforward in the direct runner beyond the reference twist for circle yaw.

Experiment plan:

- First separate "software issue" from "unreasonable reference" by adding a path-speed-profile-aware SIM-04 variant. Use `trajectory_path_speed_profile.py` semantics in the runner so circle, S-curve, and right-angle turn can be capped by tangent and normal acceleration before the controller sees them. Compare against the direct constant-speed reference.
- Sweep gains around default in SIM-04: for example `k_position_per_s` in `1.5, 2.2, 3.0, 4.0, 5.0` and `k_yaw_per_s` in `1.5, 2.5, 3.5, 5.0`, then select the lowest-gain region that reduces p95 divergence without oscillation.
- Add calibration-suggested gain loading to the runner once real or replay-backed calibration output exists. The code path is `trajectory_holonomic_calibration.py` -> `read_holonomic_calibration_params_yaml()` -> runner `--k-position` / `--k-yaw` or config-file support.
- Add runner CLI parameters or scenario presets for live-like local-planner caps: tangent acceleration, normal acceleration, yaw-rate cap, planar acceleration cap, and goal deceleration. Use the same names as `GlobalConfig.local_planner_max_tangent_accel_m_s2`, `local_planner_max_normal_accel_m_s2`, and `local_planner_control_rate_hz` where possible.
- Surface the live `HolonomicPathController` hard-coded limits as `GlobalConfig` fields: max planar command acceleration, max yaw acceleration, and max yaw rate. The code to change is `dimos/navigation/replanning_a_star/controllers.py` in `HolonomicPathController.__init__()` and `set_speed()`.
- Keep the speed-vs-divergence gate strict enough to catch the current misses: do not simply raise `--gate-max-divergence` from `1.0 m` unless the report clearly labels the run as a stress pass rather than PR confidence.
- Re-run the matrix after each tuning family and compare by speed, plant, path, and rate. A successful `2.0 m/s` claim needs all non-ideal Go2-relevant presets to pass broad gates and the synthetic nominal PR confidence checks to pass or be explicitly revised with evidence.

Files likely to change:

- `dimos/docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py`: add path-speed-profile-aware references, config-file input, gain sweeps, and live-like speed/acceleration knobs.
- `dimos/docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py`: add reference-mode and gain-sweep knobs while preserving the original direct/default-gain run directory names.
- `dimos/navigation/replanning_a_star/controllers.py`: expose `HolonomicPathController` slew/yaw limits instead of hard-coded `5.0` and yaw-rate equals speed.
- `dimos/core/global_config.py`: add validated fields for holonomic command acceleration and yaw limits.
- `dimos/navigation/trajectory_response_curve_presets.py`: add fitted Go2 presets once command-aligned logs exist; do not tune synthetic presets to hide controller problems.
- `dimos/navigation/trajectory_holonomic_calibration.py`: extend calibration beyond x/y lag if yaw, lateral asymmetry, saturation, or latency need fitted output.
- `dimos/docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py` and focused navigation tests: add regression coverage for any new runner mode and config knobs.

Implementation status:

- Implemented `--reference-mode path_speed_profile` in `docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py` for line, circle, S-curve, right-angle turn, and stop-at-goal scenarios. The profile records tangent acceleration, normal acceleration, goal deceleration, profile sample count, peak profiled speed, and travel time in `config.yaml`.
- Added runner input YAML support through `--config-yaml`; explicit CLI flags override file values.
- Added `--calibration-params-yaml` gain loading from `read_holonomic_calibration_params_yaml()`. Explicit CLI or config-file gains still win over calibration suggestions.
- Added SIM-04 matrix sweep knobs: `--reference-mode`, `--k-position-values`, `--k-yaw-values`, and live-like path-speed-profile acceleration caps.
- Added `GlobalConfig` fields for live holonomic command acceleration and yaw limits, and wired `HolonomicPathController` to use them in `__init__()` and `set_speed()`.
- Added focused regression coverage for path-speed-profile artifacts, config-file overrides, calibration gain loading, and live controller limit wiring.

Verified (from `dimos/` root):

`uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py dimos/navigation/replanning_a_star/test_local_planner_path_controller.py -q`

Matrix record (re-run <timestamp> with stable `--run-root` names; artifacts under `docs/development/921_trajectory_controller/simulation/simulation_runs/` are gitignored):

- Default live-like profile caps were not enough at `2.0 m/s`. Path-speed-profile submatrix at default matrix caps (tangent `1.0 m/s^2`, normal `0.6 m/s^2`, goal decel `1.0 m/s^2`, matching `run_sim04_initial_matrix.py` defaults): [`matrix_summary.md`](../simulation/simulation_runs/sim04_02_pph_default_2mps/matrix_summary.md). Runner verdicts pass `44`, fail `20`; matrix classifications pass `44`, borderline `0`, fail `20`. `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` pass line, circle, and S-curve but fail every right-angle turn; `synthetic_sluggish` fails every line and right-angle turn (and passes circle and S-curve only).
- Conservative path-speed-profile submatrix (tangent `0.5 m/s^2`, normal `0.1 m/s^2`, goal decel `0.5 m/s^2`): [`matrix_summary.md`](../simulation/simulation_runs/sim04_02_pph_conservative_2mps/matrix_summary.md). Runner verdicts pass `61`, fail `3`; matrix classifications pass `40`, borderline `21`, fail `3`.
- In the conservative submatrix, `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` pass runner gates across line, circle, S-curve, and right-angle turn at `2.0 m/s` target speed. Their circle runs classify as borderline by matrix policy with runner pass and peak planar divergence about `0.02-0.048 m`; `synthetic_sluggish` circle borderlines reach about `0.06-0.08 m` but still pass the broad gate.
- In the conservative submatrix, `synthetic_sluggish` passes or classifies borderline for line, circle, and S-curve, but fails right-angle turns at `10`, `20`, and `30 Hz` with max planar divergence about `1.023-1.114 m` against the `1.0 m` broad gate (see fail list in that summary).
- Lower-speed sluggish right-angle check under the same conservative caps: [`matrix_summary.md`](../simulation/simulation_runs/sim04_02_sluggish_ra_1_1p5mps/matrix_summary.md). Runner verdicts pass `6`, fail `2` (both fails are `1.5 m/s` at `20` and `30 Hz` only). `1.0 m/s` right-angle runs pass the runner gate at all four rates (two classify as matrix borderline at `20` and `30 Hz`). Therefore the documented safe runner-gate target for this synthetic sluggish right-angle envelope remains `1.0 m/s`, not `2.0 m/s`.
- Implementation changes under test: path-speed-profile reference generation and live-like speed caps. Tuning-only changes under test: conservative profile caps. Gains remained default (`k_position_per_s=2.2`, `k_yaw_per_s=2.5`), and divergence gates were not relaxed.

Verified commands (same matrices as above; `--run-root` keeps directory names stable for docs):

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim04_02_pph_default_2mps --speeds 2.0 --plants synthetic_nominal,synthetic_asymmetric,synthetic_noisy,synthetic_sluggish --scenarios line,circle,s_curve,right_angle_turn --rates 5,10,20,30 --reference-mode path_speed_profile`

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim04_02_pph_conservative_2mps --speeds 2.0 --plants synthetic_nominal,synthetic_asymmetric,synthetic_noisy,synthetic_sluggish --scenarios line,circle,s_curve,right_angle_turn --rates 5,10,20,30 --reference-mode path_speed_profile --local-planner-max-tangent-accel-m-s2 0.5 --local-planner-max-normal-accel-m-s2 0.1 --local-planner-goal-decel-m-s2 0.5`

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim04_02_sluggish_ra_1_1p5mps --speeds 1.0,1.5 --plants synthetic_sluggish --scenarios right_angle_turn --rates 5,10,20,30 --reference-mode path_speed_profile --local-planner-max-tangent-accel-m-s2 0.5 --local-planner-max-normal-accel-m-s2 0.1 --local-planner-goal-decel-m-s2 0.5`

Done when:

- A new SIM-04 matrix or submatrix shows `2.0 m/s` pass for `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` across line, circle, S-curve, and right-angle turn.
- `synthetic_sluggish` either passes at `2.0 m/s` after speed profiling or is explicitly documented as requiring a lower safe speed because the plant envelope cannot physically track the requested references.
- The report states which changes were tuning-only and which were implementation changes.

### SIM-05 - Add Delay, Jitter, And Nonlinear Response Wrappers

Status: `[x]`

Goal: cover non-ideal plant and measurement behavior that a pure lag-plus-acceleration model misses.

Implementation options:

- Add a wrapper around `TrajectoryMeasuredSample` generation, not necessarily into `ActuatedHolonomicPlant`.
- Support fixed tick delay, random jitter, stale measurement reuse, and dropped measurements.
- Support response-curve nonlinearities not already in `ActuatedHolonomicPlant`: deadband, saturation, command gain error, lateral asymmetry, and speed-dependent slip.
- Record delay settings in `config.yaml` and `summary.json`.

Done when:

- The runner can reproduce a controlled degradation when odom delay increases.
- The runner can reproduce a controlled degradation when lateral response is slower than forward response.
- A bad delayed run fails gates instead of passing silently.

Verified (from `dimos/` root):

`uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py -k "measurement_delay_records_disturbance_and_fails_gate or measurement_jitter_drop_and_stale_are_reproducible or slower_lateral_response_causes_controlled_degradation" -q`

`uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py -q`

CLI proof examples:

- Delay degradation and gate failure: pass `--measurement-delay-ticks 20 --gate-max-divergence 0.25` on a nominal line run. The delay makes the controller act on old odom, and the tighter divergence gate proves the runner reports the degraded run as a failure instead of silently passing. **This combination is expected to fail gates:** `summary.json` has `verdict.status` `fail`, and the process **exits with code `1`** (not an uncaught error). Automated proof: `test_measurement_delay_records_disturbance_and_fails_gate` asserts `rc == 1`, `verdict.status == "fail"`, and `max_planar_position_divergence_m` in `failed_gates`.
- Jitter, stale reuse, and drops: pass `--measurement-jitter-ticks 3 --measurement-stale-probability 0.25 --measurement-drop-probability 0.25 --seed 99`. The seed makes the random disturbance sequence reproducible while `summary.json` records the disturbance stats.
- Slower lateral response: compare a nominal `s_curve` run with one using `--response-linear-y-scale 0.35 --response-linear-y-command-gain-scale 0.65 --response-speed-dependent-slip-per-mps 0.3`. These flags reduce lateral acceleration/gain and effective lateral motion, so cross-track error should increase versus nominal.

Example commands (from `dimos/` root; `/tmp/...` is illustrative):

```bash
uv run python docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py \
  --scenario line \
  --speed 1.0 \
  --rate 10 \
  --plant-preset synthetic_nominal \
  --measurement-delay-ticks 20 \
  --gate-max-divergence 0.25 \
  --output-dir /tmp/sim05_delay \
  --no-plot
```

Expect **exit code `1`** and **Verdict: fail** in the log. To assert that in shell after a one-line copy, append `; test $? -eq 1`.

```bash
uv run python docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py \
  --scenario circle \
  --speed 0.8 \
  --rate 10 \
  --plant-preset synthetic_nominal \
  --measurement-jitter-ticks 3 \
  --measurement-stale-probability 0.25 \
  --measurement-drop-probability 0.25 \
  --seed 99 \
  --output-dir /tmp/sim05_jitter_stale_drop \
  --no-plot
```

```bash
uv run python docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py \
  --scenario s_curve \
  --speed 1.0 \
  --rate 10 \
  --plant-preset synthetic_nominal \
  --output-dir /tmp/sim05_lateral_nominal \
  --no-plot

uv run python docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py \
  --scenario s_curve \
  --speed 1.0 \
  --rate 10 \
  --plant-preset synthetic_nominal \
  --response-linear-y-scale 0.35 \
  --response-linear-y-command-gain-scale 0.65 \
  --response-speed-dependent-slip-per-mps 0.3 \
  --output-dir /tmp/sim05_lateral_slow \
  --no-plot
```

### SIM-06 - Build LocalPlanner-In-The-Loop Harness

Status: `[x]`

Goal: simulate the live path-following integration path.

Implementation options:

- Prefer a test-only harness first.
- Construct `LocalPlanner` with a simple path and fake or minimal `NavigationMap`.
- Subscribe to `cmd_vel`, step a plant, and feed `PoseStamped` odom back through `handle_odom`.
- Use a writable `local_planner_trajectory_tick_log_path` (`GlobalConfig.local_planner_trajectory_tick_log_path`).

Required scenarios:

- Straight line arrival.
- Curved path where curvature speed cap binds.
- Near-goal deceleration.
- Initial yaw offset that exercises initial rotation.

Done when:

- The harness reaches `arrived` in pass scenarios (`stopped_navigating` includes `arrived`).
- The generated JSONL uses the live `LocalPlanner` tick sink (same trajectory tick schema as production).
- Tests assert reasonable envelopes on tick fields (divergence, reference and commanded speeds, body-frame quantities), not the Level 1 runner's standalone `summary.json` bundle.

Verified (from `dimos/` root):

`uv run pytest dimos/navigation/replanning_a_star/test_local_planner_path_controller.py -q`

Implemented a test-only LocalPlanner-in-the-loop harness that subscribes to 
live `cmd_vel`, steps an ideal holonomic plant, feeds `PoseStamped` odom 
through `handle_odom`, and verifies the live JSONL sink for straight-line 
arrival, curvature speed capping, near-goal deceleration, and initial 
rotation.

Implemented in `dimos/navigation/replanning_a_star/test_local_planner_path_controller.py` as `_run_local_planner_harness`: `LocalPlanner` with a free-space `NavigationMap`, `cmd_vel.subscribe`, `IntegratedHolonomicPlant`, `PoseStamped` through `handle_odom`, and `local_planner_trajectory_tick_log_path`. Covered by `test_local_planner_in_the_loop_harness_reaches_arrival_on_straight_line`, `test_local_planner_in_the_loop_harness_exercises_curvature_speed_cap`, `test_local_planner_in_the_loop_harness_decelerates_near_goal`, `test_local_planner_in_the_loop_harness_exercises_initial_rotation`, plus a conservative right-angle case at `2.0 m/s` (`test_local_planner_in_the_loop_harness_supports_2mps_right_angle_with_conservative_profile`).

### SIM-07 - Decide Whether Implementation Changes Are Needed

Status: `[x]`

Goal: convert simulation findings into code changes only when justified.

Decision rules:

- If high-speed divergence is dominated by tight curves, first lower or tune `local_planner_max_normal_accel_m_s2`.
- If overshoot is dominated by acceleration slew, consider surfacing `HolonomicPathController` planar and yaw acceleration caps in `GlobalConfig`.
- If yaw behavior is limiting speed, consider a dedicated `local_planner_max_yaw_rate_rad_s` knob instead of reusing linear speed numerically.
- If higher Hz does not improve speed-vs-divergence, keep the default at 10 Hz and document the evidence.
- If delayed odom causes instability at all reasonable speeds, do not tune around it blindly; identify whether measured twist, timestamp use, or command timing needs a code change.

Done when:

- Each failing cluster has one of: config recommendation, code change task, or explicit P8 hardware-only follow-up.

Decision:

- Current default live-like tuning is not enough for a blanket `2.0 m/s` robot-speed increase before hardware. With path-speed profiling enabled but default caps (`local_planner_max_tangent_accel_m_s2=1.0`, `local_planner_max_normal_accel_m_s2=0.6`, goal decel `1.0`), the `2.0 m/s` synthetic matrix produced 44 pass and 20 fail. Every right-angle turn failed across synthetic nominal, sluggish, asymmetric, and noisy plants, and sluggish straight-line runs also failed. Worst max divergence was about `2.15 m` on sluggish line runs and about `1.15-1.54 m` on right-angle turns.
- The first required implementation changes have already been made by SIM-04-02: path-speed-profile references exist in the direct runner, `HolonomicPathController` command acceleration and yaw limits are surfaced through `GlobalConfig`, and live `LocalPlanner` in-loop tests exercise arrival, curvature capping, near-goal deceleration, initial rotation, and JSONL telemetry through the real sink.
- The remaining `2.0 m/s` blocker is primarily a config and physical-envelope issue, not evidence for a new controller law. With conservative profile caps (`local_planner_max_tangent_accel_m_s2=0.5`, `local_planner_max_normal_accel_m_s2=0.1`, goal decel `0.5`) and unchanged default gains, the `2.0 m/s` synthetic nominal, asymmetric, and noisy plants passed line, circle, S-curve, and right-angle turns across `5`, `10`, `20`, and `30 Hz`. Synthetic sluggish still failed tight right-angle turns at `10-30 Hz` and only stayed within gates at `1.0 m/s` under the same conservative caps.
- Higher control rate is not the fix for the failing clusters. In the default-cap `2.0 m/s` matrix, each rate had five failures and worst divergence slightly increased from `5 Hz` to `30 Hz`. In the conservative-cap matrix, the only remaining failures were sluggish right-angle turns at higher rates, while `10-30 Hz` did not improve the result. Keep the live default at `10 Hz` unless fitted Go2 timing data later shows a measured benefit.
- Yaw behavior is not the current limiting evidence. A dedicated `local_planner_max_yaw_rate_rad_s` knob already exists and is wired, but the failing clusters are dominated by planar divergence on tight geometry and sluggish response. Do not add another yaw feature before fitted command-response data or a yaw-specific failing run points to it.
- Delayed odom remains a stress/testing concern, not a tuning target. SIM-05 demonstrates that large fixed delay fails gates instead of passing silently. No current result says to tune around delayed odom before hardware; if real delayed odom is unstable at reasonable speeds, the next task is timestamp/measured-twist/command-timing diagnosis.

Failing clusters and action:

- Direct constant-speed `2.0 m/s` synthetic nominal/sluggish/asymmetric/noisy failures: resolved by using live-like path-speed profiling instead of forcing impossible speed through tight geometry. Keep direct mode as a stress test, not a PR-confidence claim.
- Default live-like cap `2.0 m/s` right-angle failures: config recommendation. Do not try higher robot speeds with the current `1.0/0.6` tangent/normal defaults on tight paths. For the next `2.0 m/s` simulation and any cautious P8 trial, start from conservative caps near `0.5 m/s^2` tangent and `0.1 m/s^2` normal, with equally conservative goal deceleration.
- Synthetic sluggish `2.0 m/s` line and right-angle failures: envelope recommendation. Treat sluggish response as a lower-speed safety envelope; document `1.0 m/s` as the current passing right-angle target under conservative caps. Do not claim `2.0 m/s` support for sluggish/fitted-conservative behavior until fitted Go2 command-response data or additional controller work justifies it.
- Conservative-cap circle borderline classifications: no code change. Matrix policy can mark `borderline` while the runner verdict stays `pass`. In the re-run under `docs/development/921_trajectory_controller/simulation/simulation_runs/sim04_02_pph_conservative_2mps/`, nominal, asymmetric, and noisy circle borderlines stay at or just above about `0.047 m` peak planar divergence; sluggish circle borderlines reach about `0.06-0.08 m` but still pass the `1.0 m` broad gate. Keep them as review-visible evidence, not blockers.
- Live `LocalPlanner` integration risk: covered enough for PR confidence at current scope by SIM-06. The harness reaches `arrived`, writes live JSONL, and exercises path speed caps and initial rotation, so no additional live-planner code hook is required before the next matrix.
- `5.0 m/s` or other maximum-speed claims: explicit P8/hardware-only follow-up plus separate speed-max simulation. Current tight-path matrix is not a basis for raising max speed beyond the synthetic-envelope `2.0 m/s` experiments.

Verified (from `dimos/` root):

Aggregates checked against [`sim04_02_pph_default_2mps/matrix_results.json`](../simulation/simulation_runs/sim04_02_pph_default_2mps/matrix_results.json), [`sim04_02_pph_conservative_2mps/matrix_results.json`](../simulation/simulation_runs/sim04_02_pph_conservative_2mps/matrix_results.json), and [`sim04_02_sluggish_ra_1_1p5mps/matrix_results.json`](../simulation/simulation_runs/sim04_02_sluggish_ra_1_1p5mps/matrix_results.json) (<timestamp> re-runs; same commands as SIM-04-02).

`uv run pytest dimos/navigation/replanning_a_star/test_local_planner_path_controller.py -q`

### SIM-08 - Implement Required Fixes

Status: `[x]`

Goal: apply only changes proven necessary by simulation.

SIM-08 reruns use `run_sim04_initial_matrix.py` with narrowed CLI flags; shared presets and compact scenario meanings stay as in [`matrix_presets_and_scenarios.md`](../simulation/matrix_presets_and_scenarios.md#sim-04-initial-matrix).

Possible fixes:

- Add config knobs for yaw rate and controller slew caps.
- Add delay-aware or stale-measurement detection if the harness reveals silent bad behavior.
- Improve `LocalPlanner` simulation hooks if the live integration is hard to test.
- Tighten docs around recommended `planner_robot_speed`, gains, normal acceleration, and rate.

Done when:

- Each code change has a focused regression test.
- The matrix from SIM-04 is re-run after changes.

Verified:

Implemented the remaining required live-planner fix from the simulation findings: `GlobalConfig` now exposes `local_planner_goal_decel_m_s2`, and `LocalPlanner._path_speed_for_index` uses that cap for near-goal distance-based deceleration instead of implicitly reusing the tangent acceleration cap. Focused regression coverage was added for the configured goal-deceleration cap, for a live `LocalPlanner` harness run with `planner_robot_speed=2.0` through a conservative right-angle profile, and for runner-level `2.0 m/s` conservative-profile S-curve and right-angle support across `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy`.

Focused tests (from `dimos/` root):

`uv run pytest -q docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py dimos/navigation/replanning_a_star/test_local_planner_path_controller.py`

Result: `44 passed` (22 runner tests + 22 `LocalPlanner` / controller tests; counts drift if either module grows).

Supported SIM-04 matrix rerun after the code change (stable `--run-root`; <timestamp> verification run):

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim08_verify_3plant_2mps_conservative --speeds 2.0 --plants synthetic_nominal,synthetic_asymmetric,synthetic_noisy --scenarios line,circle,s_curve,right_angle_turn --rates 5,10,20,30 --reference-mode path_speed_profile --local-planner-max-tangent-accel-m-s2 0.5 --local-planner-max-normal-accel-m-s2 0.1 --local-planner-goal-decel-m-s2 0.5`

Result: [`matrix_summary.md`](../simulation/simulation_runs/sim08_verify_3plant_2mps_conservative/matrix_summary.md) recorded runner verdicts pass `48`, fail `0`; matrix classifications pass `36`, borderline `12`, fail `0`. Borderlines are the same circle-policy pattern as SIM-07 (runner pass; peak planar divergence about `0.02-0.048 m` on the three plants in this 48-run subset).

`uv run python docs/development/921_trajectory_controller/simulation/run_sim04_initial_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim08_verify_sluggish_ra_1mps --speeds 1.0 --plants synthetic_sluggish --scenarios right_angle_turn --rates 5,10,20,30 --reference-mode path_speed_profile --local-planner-max-tangent-accel-m-s2 0.5 --local-planner-max-normal-accel-m-s2 0.1 --local-planner-goal-decel-m-s2 0.5`

Result: [`matrix_summary.md`](../simulation/simulation_runs/sim08_verify_sluggish_ra_1mps/matrix_summary.md) recorded runner verdicts pass `4`, fail `0`; matrix classifications pass `2`, borderline `2`, fail `0`.

> SIM-08 is closer to real planner behavior: "2.0 m/s is the target ceiling, but speed drops for curvature, stopping distance, and near-goal decel."

### SIM-08-results - Matrix Results By Speed

Status: `[x]`

Goal: make the SIM-08 post-fix rerun readable in the same speed-band format as SIM-04, while keeping the scope difference clear.

Summary by target speed:

| Speed | Runner verdict pass | Runner verdict fail | Matrix pass | Borderline | Matrix fail | Interpretation |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `1.0 m/s` | 4 | 0 | 2 | 2 | 0 | Sluggish right-angle support check under conservative path-speed caps. This preserves the SIM-07 conclusion that sluggish tight turns are a lower-speed envelope, not a `2.0 m/s` claim. |
| `2.0 m/s` | 48 | 0 | 36 | 12 | 0 | Supported running target for `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` with `path_speed_profile` and conservative caps. Line, S-curve, and right-angle runs pass cleanly; circle runs classify borderline by matrix policy with runner pass (peak planar divergence about `0.02-0.048 m` on nominal/asymmetric/noisy in `sim08_verify_3plant_2mps_conservative`). |

Important comparison with SIM-04-results:

- SIM-04-results used the original direct/default-cap matrix across `ideal`, `synthetic_nominal`, `synthetic_sluggish`, `synthetic_asymmetric`, and `synthetic_noisy`. In that matrix, `2.0 m/s` produced runner verdicts pass `19`, fail `61`, and the synthetic plants that approximate lag, noise, and asymmetry failed heavily.
- SIM-08 is not a blanket replacement for those direct stress results. It reruns the supported post-tuning envelope: `path_speed_profile`, default gains, tangent cap `0.5 m/s^2`, normal cap `0.1 m/s^2`, and goal decel `0.5 m/s^2`.
- The `2.0 m/s` conclusion changed only for the supported synthetic nominal/asymmetric/noisy profiled-reference envelope: those plants now have `48/48` runner passes across line, circle, S-curve, and right-angle turns at `5`, `10`, `20`, and `30 Hz`.
- The sluggish conclusion did not change to `2.0 m/s`. SIM-08 keeps sluggish tight right-angle turns at the documented `1.0 m/s` support target under the same conservative caps, where `4/4` runner verdicts pass.

Simulation outcome: the Level 1 runner matrix was not materially improved by this code change because the supported conservative-profile runner cases were already passing after SIM-04-02 tuning. The improvement is in live `LocalPlanner` parity and coverage: near-goal deceleration is no longer tied to the tangent acceleration cap, and the 2.0 m/s live harness now exercises the same conservative path-profile envelope as the runner.

### SIM-11 - Speed-Max Matrix For 3.7 m/s Running Target

Status: `[x]`

Goal: build a separate speed-max matrix that can support or reject a `3.7 m/s` running-speed claim, instead of misusing the SIM-04 tight-geometry matrix as a high-speed validator. SIM-04-03 listed the requirements for a real speed-max matrix (long straight lines, larger-radius arcs, wide S-curves, stop-distance tests, achieved-speed reporting, and hardware safety gates). This task implements those requirements and runs the matrix.

Speed-max defaults, `speed_max_*` scenarios, and matrix-only gates are documented in [`matrix_presets_and_scenarios.md`](../simulation/matrix_presets_and_scenarios.md#speed-max-matrix).

Why the SIM-04 matrix is not enough at `3.7 m/s`:

- SIM-04 only swept up to `2.0 m/s`, and the direct/default-cap `2.0 m/s` results failed heavily under realistic synthetic plants.
- SIM-08 only proves the narrower `2.0 m/s` envelope with `path_speed_profile` and conservative caps tangent `0.5`, normal `0.1`, goal decel `0.5`.
- Lateral acceleration on the `1.5 m` SIM-04 circle at `3.7 m/s` would be `v^2 / r = 3.7^2 / 1.5 ~= 9.1 m/s^2`, far above any reasonable cap. So SIM-04 cannot validate `3.7 m/s`; it can only inform the design of a speed-max matrix.

Web inputs that informed corner geometry:

- Unitree publishes Go2 max running speeds per variant (Air `2.5`, Pro `3.5`, EDU `3.7 m/s`, lab peak `~ 5 m/s`) in the user manual and product brochures, but does not publish a max deceleration or stop-distance number. So corner approach distance must be sized from the configured `goal_decel` cap, not from a Go2 spec.
- A Go2 ODD-observer reference project assumes a `10 m/s^2` indoor max-acceleration ceiling for the platform, with `12.6 m/s^2` flagged out-of-envelope. So `10 m/s^2` is the platform headroom number, not a safe `goal_decel` cap; this matrix uses the live default running envelope of `0.7 m/s^2`.
- At `goal_decel = 0.7 m/s^2`, brake distance from `3.7 m/s` to `0` is `v^2 / (2 a) = 9.78 m`, and from `3.7 m/s` to the geometry-bounded corner cap of `1.22 m/s` (a 1.5 m fillet under `normal_accel = 1.0 m/s^2`) is `8.71 m`. So a sharp-corner test for running speed needs `>= ~ 9 m` of straight approach plus an accel ramp from rest. The original `right_angle_turn` (4 m legs) is correctly sized for slow-speed validation only.

Implementation:

- Added speed-max scenarios to `dimos/docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py`:
  - `speed_max_line`: 40 m straight line.
  - `speed_max_arc`: 90 degree sweep at radius 25 m. Geometry-bound peak is `sqrt(normal_accel * 25)` m/s; with the chosen running envelope this is `5.0 m/s`, so the arc does not cap the `3.7 m/s` target.
  - `speed_max_s_curve`: 40 m length, 2.0 m amplitude, R_min `~ 20.3 m`. Geometry-bound peak is `sqrt(normal_accel * 20.3) ~= 4.5 m/s`, so the S-curve does not cap the `3.7 m/s` target either.
  - `speed_max_stop_distance`: 30 m straight line used as a stop-distance test; with goal decel `0.7 m/s^2` the stop distance from `3.7 m/s` is about `9.78 m`, well inside the 30 m runway.
  - `speed_max_right_angle_turn`: 90 degree turn modeled as a polyline with two `20 m` straight legs joined by a small-radius fillet (`R = 1.5 m`, sampled densely). The fillet replaces a bare polyline vertex because the path-speed profile in this codebase computes the corner curvature cap from the circumscribed circle of three adjacent waypoints; for symmetric long legs that radius is `~ L / sqrt(2)`, so a bare 20 m polyline corner gets treated as a `~ 14 m` arc and the profile does not slow down at running speed. A small fillet matches the output of a real planner and gives the speed profile a well-defined local curvature to brake into. Under `normal_accel = 1.0 m/s^2` the corner cap is `sqrt(1.0 * 1.5) ~= 1.22 m/s`. The 20 m straight leg accommodates accel from 0 to `3.7 m/s` (`9.78 m`), brake from `3.7 m/s` to `1.22 m/s` (`8.71 m`), plus `~ 1.5 m` of cruise margin. The original 4 m `right_angle_turn` is retained for SIM-04 slow-speed validation and is no longer used by this matrix.
- Added focused regressions in `dimos/docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py`: every `speed_max_*` scenario is exercised at `3.7 m/s` against the ideal plant under `path_speed_profile`, asserting the geometry-bounded peak speed and arrival. `speed_max_right_angle_turn` additionally asserts the measured corner-window minimum speed reaches the geometry cap within `0.1 m/s`.
- Added `dimos/docs/development/921_trajectory_controller/simulation/run_sim_speed_max_matrix.py`, modeled on `run_sim04_initial_matrix.py`, that drives the simulator across speed, plant, scenario, and rate, then post-processes each `ticks.jsonl` to compute speed-max metrics and apply hardware safety gates.

Speed-max metrics computed per run:

- `achieved_peak_planar_speed_m_s`, `achieved_p95_planar_speed_m_s`, `achieved_mean_planar_speed_m_s`.
- `max_observed_lateral_accel_m_s2` (measured planar speed times measured yaw rate).
- `max_observed_yaw_rate_rad_s`.
- `max_observed_command_jerk_m_s3` (per-tick change in commanded planar speed divided by `dt`).
- `min_speed_during_corner_window_m_s` for `speed_max_right_angle_turn` (a 2 m window around the fillet midpoint).

Speed-max gates added on top of the runner verdict:

- Speed-scaled max planar divergence: `0.30 + 0.15 * target_speed_m_s` (so `0.86 m` at `3.7 m/s`).
- Hardware lateral acceleration cap: `<= 1.5 m/s^2`.
- Hardware command jerk cap: `<= 30 m/s^3`.
- Achieved peak speed must be `>= 85%` of the geometry-bounded expected peak speed (only on the four `speed_max_*` scenarios; `speed_max_right_angle_turn` is target-limited because the leg is sized to reach `3.7 m/s`).
- On `speed_max_right_angle_turn`, the corner-window minimum measured speed must be within `0.4 m/s` of the geometry-bounded corner cap `sqrt(normal_accel * fillet_radius)`. This gate replaces the earlier "drop below `0.5 m/s` inside the corner" gate, which was geometry-incorrect for any fillet with cap `> 0.5 m/s`.

Matrix:

- Speeds: `2.0, 2.5, 3.0, 3.5, 3.7` m/s.
- Rates: `10, 20` Hz.
- Plants: `ideal, synthetic_nominal, synthetic_sluggish, synthetic_asymmetric, synthetic_noisy`.
- Scenarios: the five `speed_max_*` scenarios listed above. The legacy 4 m `right_angle_turn` is no longer used here.
- Reference mode: `path_speed_profile` only.
- Path speed caps (running envelope): tangent `0.7 m/s^2`, normal `1.0 m/s^2`, goal decel `0.7 m/s^2`.
- Response saturation scale: `3.0`. Synthetic presets cap at `1.4-2.0 m/s` saturation by default; the wrapper widens that so synthetic plants can physically reach the running target. The wrapper is recorded in each run's `config.yaml` so the matrix is auditable as a documented high-speed envelope, not a hidden tuning change.
- Total runs: `250`.

Verified:

`uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py -q`
`uv run python docs/development/921_trajectory_controller/simulation/run_sim_speed_max_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim11_speed_max_matrix_<timestamp>_verify --no-plot`

Speed-max matrix recorded at `docs/development/921_trajectory_controller/simulation/simulation_runs/sim11_speed_max_matrix_<timestamp>_verify/matrix_summary.md` (paths relative to `dimos/`). Matrix classifications: pass `194`, borderline `2`, fail `54`, invalid `0`. an even earlier `<timestamp>T163229Z_sim_speed_max_matrix` run used the legacy 4 m polyline `right_angle_turn` and is superseded.

### SIM-11-results - Speed-Max Matrix Findings

Status: `[x]`

Goal: state what the speed-max matrix supports and rejects, in the same speed-band shape as `SIM-04-results` and `SIM-08-results`, so the report can be used as direct evidence for or against a `3.7 m/s` running claim.

Summary by target speed (250 runs total, 50 per target speed):

| Speed | Pass | Borderline | Fail | Interpretation |
| ---: | ---: | ---: | ---: | --- |
| `2.0 m/s` | 34 | 2 | 14 | Synthetic noise floor of about `0.6 m` divergence narrowly fails the `0.6 m` speed-scaled gate at `2.0 m/s` for `synthetic_noisy` on `speed_max_line` (`20 Hz`), `speed_max_arc` (`20 Hz`), and `speed_max_s_curve` (both rates; four failing runs, three wide scenario kinds); the sluggish plant fails on every scenario. The corner scenario passes for all non-sluggish plants. |
| `2.5 m/s` | 40 | 0 | 10 | Sluggish fails on every scenario; the four other plants pass every scenario including the corner. |
| `3.0 m/s` | 40 | 0 | 10 | Same pattern. |
| `3.5 m/s` | 40 | 0 | 10 | Same pattern. |
| `3.7 m/s` | 40 | 0 | 10 | Same pattern. All five speed-max scenarios pass for `ideal`, `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` at both `10` and `20 Hz`; sluggish fails everywhere. |

`3.7 m/s` plant breakdown across all five `speed_max_*` scenarios at `20 Hz`:

- `ideal`: peak `3.70 m/s`, max divergence `0.026-0.032 m`, max lateral accel `0.585 m/s^2` on the wide arc and `1.072 m/s^2` on the right-angle fillet (matches `v^2 / r = 1.22^2 / 1.5 ~= 0.99 m/s^2` plus measurement margin), well under the `1.5 m/s^2` hardware cap.
- `synthetic_nominal`: peak `3.68 m/s`, max divergence `0.076-0.198 m`, max lateral accel `1.057 m/s^2` on the corner fillet.
- `synthetic_asymmetric`: peak `3.70 m/s`, max divergence `0.063-0.070 m`, max lateral accel `1.060 m/s^2` on the corner fillet.
- `synthetic_noisy`: peak `3.67-3.69 m/s`, max divergence `0.098-0.281 m`, max lateral accel `1.087 m/s^2` on the corner fillet.
- `synthetic_sluggish`: peak `3.66 m/s` but max divergence `1.461-2.912 m`. The plant has `linear_x/y tau_s = 0.35` and `max_accel = 0.75 m/s^2`, so even with a tangent cap of `0.7 m/s^2` the lag accumulates into a large along-track divergence at running speed; the corner fillet is the worst case at `2.912 m`.

Supported envelope for `3.7 m/s`:

- Plants: `ideal`, `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` (four of five plant presets; `synthetic_sluggish` excluded).
- Scenarios: `speed_max_line`, `speed_max_arc` (R `>= 25 m`), `speed_max_s_curve` (R_min `>= 20 m`), `speed_max_stop_distance` (`>= 14 m` runway from running speed at goal decel `0.7 m/s^2`), and `speed_max_right_angle_turn` (`>= 20 m` straight approach with a `>= 1.5 m` fillet under `normal_accel = 1.0 m/s^2`).
- Path speed caps: tangent `0.7 m/s^2`, normal `1.0 m/s^2`, goal decel `0.7 m/s^2`. Lateral acceleration stays at or below `0.7 m/s^2` on the wide arc and S-curve and at or below `1.10 m/s^2` on the corner fillet, all under the `1.5 m/s^2` hardware cap.
- Control rates: `10` and `20 Hz` both pass; higher rate does not unlock new behavior.

Not supported:

- `synthetic_sluggish` is not a `3.7 m/s` plant under the running-envelope caps. The slow time constant and low max acceleration break the speed-scaled divergence gate on every scenario at every tested speed from `2.0 m/s` upward; at `3.7 m/s` max planar divergence reaches about `1.5-1.8 m` on line, arc, and S-curve and about `2.9 m` on the filleted corner (`20 Hz` worst case). At `2.0 m/s` the stop-distance scenario can still show order `1.2-1.3 m` divergence, below the `1.4 m` wording that only fits the higher-speed sluggish wide paths. To support a Go2-conservative envelope at running speed, the path-speed profile would need a tangent cap closer to `0.3-0.4 m/s^2` (a separate matrix; the current speed-scaled `0.86 m` divergence gate at `3.7 m/s` is not the issue). Alternatively, treat sluggish as a lower-speed envelope, the same conclusion as in SIM-07.
- The synthetic noise floor of about `0.6 m` divergence narrowly fails the `0.6 m` speed-scaled gate at `2.0 m/s`. The noisy plant comfortably passes the `0.86 m` gate at `3.7 m/s`, which means the speed-scaled gate is correctly tracking the realistic envelope at running speed but is borderline tight at lower speeds. Future iterations may switch the noisy `2.0 m/s` gate to a documented per-plant noise allowance.

Sharp-corner geometry insight (worth carrying into hardware tests):

- A sharp 90 degree corner at running speed must be supplied to the controller as a small-radius fillet in the path geometry, not as a bare polyline vertex. The path-speed profile in this codebase computes vertex curvature from the circumscribed circle of three adjacent waypoints, which scales with leg length and stops slowing the reference at running speed for legs `>= ~ 12 m`. A real navigation stack producing dense smoothed paths (e.g., a planner output filtered through a smoothing pass) will give the speed profile something to brake into; a hand-crafted polyline with long legs and a bare corner will not.
- The required straight approach distance for a sharp corner at running speed is set by the configured `goal_decel` cap and the local fillet radius: `(v_target^2 - v_corner_cap^2) / (2 * goal_decel)`. At the running envelope (`goal_decel = 0.7 m/s^2`, `normal_accel = 1.0 m/s^2`, `fillet_radius = 1.5 m`) this is `8.71 m` for `3.7 m/s -> 1.22 m/s`. The 4 m legs in the original SIM-04 `right_angle_turn` cannot satisfy this and are correctly excluded from the speed-max matrix.

Implementation vs tuning under test:

- Implementation under test: speed-max scenario geometries (new, in `docs/development/921_trajectory_controller/simulation/simulate_trajectory_controller.py`), the existing path-speed-profile reference generation, the existing `HolonomicPathController` command acceleration and yaw config, and post-hoc speed-max metric extraction with hardware safety gates.
- Tuning under test: running-envelope speed caps (`tangent=0.7, normal=1.0, goal_decel=0.7`) and `--response-saturation-scale 3.0`. Both are explicitly recorded in the run config and the report.

Conclusion for the `3.7 m/s` claim:

- The controller, command-limit, and path-speed-profile stack on this branch supports `3.7 m/s` on long straight, large-radius arc, wide S-curve, stop-distance, and 90 degree filleted-corner geometries under `synthetic_nominal`, `synthetic_asymmetric`, and `synthetic_noisy` envelopes, plus the `ideal` baseline. Achieved peak speed reaches the geometry-bounded target, lateral acceleration stays at or below `1.10 m/s^2` (corner) and `0.70 m/s^2` (wide curves), and divergence stays under the speed-scaled `0.86 m` gate at `3.7 m/s`.
- It does not support `3.7 m/s` for the `synthetic_sluggish` envelope or for sharp corners that arrive as a bare polyline vertex (no fillet) or with a sub-9 m straight approach. Both of these are envelope/geometry limits, not controller correctness defects, and they should be carried into hardware as explicit safety gates: hardware tests should not request `3.7 m/s` along a sub-9 m straight before a sharp corner, hardware paths should be smoothed before being handed to the path-speed profile, and the live `local_planner_max_tangent_accel_m_s2` should be lowered if measured Go2 behavior matches the synthetic sluggish envelope.
- This matrix is still a synthetic-envelope confidence report. A `3.7 m/s` claim against fitted Go2 response curves still requires command-aligned Go2 logs, per `SIM-04A`. Until a `fitted_go2_*` preset exists, the supported envelope must be labeled "synthetic envelope at running speed", and hardware validation must include a slow-speed bring-up before any `3.7 m/s` field test.

Verified:

`uv run pytest docs/development/921_trajectory_controller/simulation/test_simulate_trajectory_controller.py dimos/navigation/replanning_a_star/test_local_planner_path_controller.py -q`
`uv run python docs/development/921_trajectory_controller/simulation/run_sim_speed_max_matrix.py --run-root docs/development/921_trajectory_controller/simulation/simulation_runs/sim11_speed_max_matrix_<timestamp>_verify --no-plot`

Findings above are cross-checked against `docs/development/921_trajectory_controller/simulation/simulation_runs/sim11_speed_max_matrix_<timestamp>_verify/matrix_summary.md` (paths relative to `dimos/`).

## Part C - PR related

### SIM-09 - Produce PR-Ready Simulation Report

Status: `[x]`

Goal: produce a concise reviewer artifact.

Suggested location:

- `dimos/docs/development/921_trajectory_controller/docs/trajectory_simulation_validation.md` if review-facing.

Report must include:

- Issue 921 requirements covered.
- Simulation levels run.
- Plant presets and response curves.
- Scenario matrix summary.
- Speed-vs-divergence plots.
- Control-rate comparison.
- Implementation changes made because of simulation, if any.
- Known limits of simulation and P8 hardware checklist pointer.

Done when:

- A reviewer can see why the branch is ready to open as a PR before real robot P8 sign-off.

Verified:

Added [`trajectory_simulation_validation.md`](trajectory_simulation_validation.md), the reviewer-facing validation artifact. It covers issue 921 requirements, simulation levels, plant presets and response-curve sources, SIM-04/SIM-08/SIM-11 matrix summaries, speed-vs-divergence plot locations and regeneration command, control-rate comparison, implementation changes made because of simulation, known synthetic-envelope limits, and the P8 hardware checklist pointer.

### SIM-10 - PR Gate

Status: `[ ]`

Open the PR only when all of the following are true:

- Existing focused 921 tests pass.
- New simulation runner tests pass.
- Level 1 matrix passes the synthetic or fitted nominal response-curve gates at `1.5 m/s` and produces interpretable results at `2.0 m/s`.
- Level 2 `LocalPlanner` harness passes at least line, curve, and initial-yaw scenarios.
- Delay or jitter stress either passes conservative gates or produces a documented config/code recommendation.
- Every simulation artifact needed for review is generated and linked from the PR body or review-facing docs.
- Remaining hardware uncertainty is explicitly labeled as P8 field validation, not claimed as solved by simulation.

### Monitoring Template For Each Simulation Run

Copy this block into each run report:

```markdown
## Run

- Commit:
- Date:
- Runner command:
- Scenario:
- Plant preset:
- Response curve source:
- Response curve parameters:
- Speed:
- Rate:
- Gains:
- Limits:
- Delay and jitter:
- Seed:
- Output directory:

## Metrics

- max_planar_position_divergence_m:
- p95_planar_position_divergence_m:
- final_planar_position_divergence_m:
- max_abs_heading_error_rad:
- p95_abs_cross_track_m:
- max_commanded_planar_speed_m_s:
- mean_commanded_planar_speed_m_s:
- nominal_hz:
- wall_clock_hz:
- jitter_s_std:

## Verdict

- Pass or fail:
- Gate that failed, if any:
- Plot path:
- Recommended action:
- Follow-up task:
```

### Suggested First Command Sequence For The Next LLM

```bash
cd /Users/9q7834rg/try/dimensional/dimos
git status
uv run pytest -q \
  dimos/navigation/test_trajectory_metrics.py \
  dimos/navigation/test_trajectory_types.py \
  dimos/navigation/test_trajectory_command_limits.py \
  dimos/navigation/test_trajectory_control_tick_log.py \
  dimos/navigation/test_trajectory_control_tick_export.py \
  dimos/navigation/test_trajectory_path_speed_profile.py \
  dimos/navigation/test_trajectory_holonomic_tracking_controller.py \
  dimos/navigation/test_trajectory_holonomic_plant.py \
  dimos/navigation/test_trajectory_replay_loader.py \
  dimos/navigation/test_trajectory_holonomic_calibration.py \
  dimos/navigation/test_trajectory_holonomic_analytic_paths.py \
  dimos/navigation/test_trajectory_controller_swap_regression.py \
  dimos/navigation/test_trajectory_golden_replay.py \
  dimos/navigation/replanning_a_star/test_local_planner_path_controller.py
```
