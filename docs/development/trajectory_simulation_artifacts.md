# Trajectory Simulation Artifacts

This note defines the machine-readable artifact bundle for issue 921 trajectory controller simulations. A runner that claims this format must emit the files below in a single run directory, with deterministic values when the same config, commit, seed, and code version are used.

## Run Directory

Each simulation run writes one directory containing:

- `config.yaml` - the complete input configuration needed to reproduce the run.
- `summary.json` - machine-readable metrics, gate results, and final verdict.
- `ticks.jsonl` - one trajectory control tick per line, using the existing trajectory tick JSONL schema.
- `plot.png` - optional speed-vs-divergence or time-series plot. The runner may skip this when plotting dependencies are unavailable, but `summary.json` must record whether it was produced.

The directory name is not part of the schema, but should include a timestamp or stable run id when runs are generated in bulk.

## Common Conventions

- Encoding is UTF-8.
- All physical units must be encoded in field names where practical, for example `rate_hz`, `speed_m_s`, `max_linear_speed_m_s`, and `max_planar_accel_m_s2`.
- Numeric values must be JSON/YAML numbers, not strings.
- Unknown, NaN, and infinity values are not valid in `summary.json`. Use `null` only for explicitly optional values.
- Field order is for readability only. Consumers must use field names.
- Schema versions are integers. A consumer should reject an unsupported major version rather than silently interpreting a changed format.
- `seed` must be recorded for every run. Use `null` only when the run is fully deterministic and consumes no random source.
- `commit` must identify the code revision under test. A dirty tree may be recorded, but the dirty state must be explicit.

## `config.yaml`

`config.yaml` is the source-of-truth input document for a run. It should be possible to pass this file back to the runner and get the same scenario, plant, controller, limits, rate, seed, and gates.

Required top-level fields:

- `schema_version`: integer config schema version. Current value: `1`.
- `run`: run identity and provenance.
- `scenario`: path scenario and speed target.
- `plant`: simulated plant parameters and response-curve source.
- `controller`: controller implementation and gains.
- `limits`: speed, acceleration, yaw, and path-following limits applied during the run.
- `rate`: nominal control loop timing.
- `random`: seed and noise policy.
- `outputs`: filenames the runner should write.
- `gates`: pass/fail criteria.

Required `run` fields:

- `id`: stable run id, unique within a matrix when possible.
- `created_at`: ISO 8601 timestamp, or `null` when the config is a reusable template.
- `commit`: git commit SHA or equivalent source revision.
- `dirty`: boolean indicating whether uncommitted changes were present.
- `description`: short human-readable label.

Required `scenario` fields:

- `name`: one of the runner-supported scenario names, for example `line`, `circle`, `s_curve`, `right_angle_turn`, or `stop_at_goal`.
- `variant`: optional scenario variant string, or `null`.
- `target_speed_m_s`: nominal commanded or requested planar speed.
- `duration_s`: maximum simulated duration.
- `initial_pose`: object with `x_m`, `y_m`, and `yaw_rad`.
- `goal_tolerance`: object with `position_m` and `yaw_rad`.
- `path`: scenario-specific parameters. This may include values such as `length_m`, `radius_m`, `turn_angle_rad`, or waypoint lists.

Required `plant` fields:

- `model`: plant model name used by the runner.
- `preset`: preset name, for example `ideal`, `synthetic_nominal`, `synthetic_sluggish`, `synthetic_asymmetric`, `synthetic_noisy`, or a fitted Go2 preset.
- `response_curve_source`: one of `ideal`, `synthetic`, `fitted_from_go2_log`, or `unknown`.
- `response_curve_id`: stable id for the response curve or fitted dataset.
- `response_curve_version`: integer or string version for the curve definition.
- `parameters`: concrete plant parameters used in the run. At minimum this must include response time constants, acceleration caps, command gains, deadbands, saturation limits, latency, and noise caps when the plant model supports them.
- `notes`: optional human-readable text.

PR-gate runs must not use `response_curve_source: unknown`.

Required `controller` fields:

- `name`: controller implementation name, for example `HolonomicPathController`.
- `mode`: controller mode or integration path, for example `direct` or `local_planner_in_loop`.
- `gains`: object containing the gain values used by the controller.
- `calibration_source`: path, preset id, or `null`.
- `parameters`: other controller parameters that affect behavior.

Required `limits` fields:

- `max_linear_speed_m_s`: maximum planar linear speed.
- `max_angular_speed_rad_s`: maximum yaw rate or angular speed limit.
- `max_planar_accel_m_s2`: maximum planar acceleration.
- `max_yaw_accel_rad_s2`: maximum yaw acceleration, or `null` if the controller has no such limit.
- `max_normal_accel_m_s2`: path curvature acceleration cap, or `null` if not applied.
- `goal_tolerance_position_m`: arrival position tolerance.
- `goal_tolerance_yaw_rad`: arrival yaw tolerance.

Required `rate` fields:

- `control_rate_hz`: nominal controller tick rate.
- `dt_s`: nominal tick period.
- `jitter_s`: deterministic jitter setting, or `0.0` when disabled.
- `max_ticks`: maximum number of ticks before timeout.

Required `random` fields:

- `seed`: integer seed, or `null` only for fully deterministic runs.
- `noise_enabled`: boolean.
- `rng`: random number generator name, or `null` when no random source is used.

Required `outputs` fields:

- `ticks_jsonl`: relative path to `ticks.jsonl`.
- `summary_json`: relative path to `summary.json`.
- `plot_png`: relative path to `plot.png`, or `null` when not requested.

Required `gates` fields:

- `max_planar_position_divergence_m`: pass threshold for the maximum observed planar divergence.
- `max_final_position_error_m`: pass threshold for final planar position error.
- `max_final_heading_error_rad`: pass threshold for final heading error.
- `max_settle_time_s`: pass threshold for time to satisfy arrival tolerances, or `null`.
- `require_arrival`: boolean.
- `require_no_timeout`: boolean.
- `require_finite_metrics`: boolean.

Example:

```yaml
schema_version: 1
run:
  id: synthetic_nominal_s_curve_1p5mps_10hz_seed7
  created_at: "2026-04-26T17:00:00+09:00"
  commit: "abcdef1234567890"
  dirty: false
  description: "Synthetic nominal S-curve at 1.5 m/s and 10 Hz"
scenario:
  name: s_curve
  variant: null
  target_speed_m_s: 1.5
  duration_s: 20.0
  initial_pose:
    x_m: 0.0
    y_m: 0.0
    yaw_rad: 0.0
  goal_tolerance:
    position_m: 0.15
    yaw_rad: 0.2
  path:
    length_m: 8.0
    lateral_amplitude_m: 1.0
plant:
  model: actuated_holonomic
  preset: synthetic_nominal
  response_curve_source: synthetic
  response_curve_id: synthetic_nominal_v1
  response_curve_version: 1
  parameters:
    schema: holonomic_response_curve
    version: 1
    curves:
      - axis: linear_x
        direction: bidirectional
        tau_s: 0.18
        max_accel: 1.5
        command_gain: 1.0
        deadband: 0.02
        saturation: 2.0
        noise_max: 0.0
        latency_s: 0.0
        notes: ""
      - axis: linear_y
        direction: bidirectional
        tau_s: 0.18
        max_accel: 1.5
        command_gain: 1.0
        deadband: 0.02
        saturation: 2.0
        noise_max: 0.0
        latency_s: 0.0
        notes: ""
      - axis: yaw
        direction: bidirectional
        tau_s: 0.16
        max_accel: 2.0
        command_gain: 1.0
        deadband: 0.02
        saturation: 1.5
        noise_max: 0.0
        latency_s: 0.0
        notes: ""
  notes: "Synthetic envelope, not a Go2 hardware claim."
controller:
  name: HolonomicPathController
  mode: direct
  gains:
    k_cross_track: 1.0
    k_heading: 1.0
  calibration_source: null
  parameters: {}
limits:
  max_linear_speed_m_s: 1.5
  max_angular_speed_rad_s: 1.0
  max_planar_accel_m_s2: 1.5
  max_yaw_accel_rad_s2: null
  max_normal_accel_m_s2: 1.0
  goal_tolerance_position_m: 0.15
  goal_tolerance_yaw_rad: 0.2
rate:
  control_rate_hz: 10.0
  dt_s: 0.1
  jitter_s: 0.0
  max_ticks: 200
random:
  seed: 7
  noise_enabled: false
  rng: null
outputs:
  ticks_jsonl: ticks.jsonl
  summary_json: summary.json
  plot_png: plot.png
gates:
  max_planar_position_divergence_m: 0.35
  max_final_position_error_m: 0.15
  max_final_heading_error_rad: 0.2
  max_settle_time_s: 20.0
  require_arrival: true
  require_no_timeout: true
  require_finite_metrics: true
```

## `ticks.jsonl`

Per-tick data must reuse the existing trajectory control tick JSONL export documented in `dimos/navigation/trajectory_control_tick_jsonl.md`.

Runner-specific requirements:

- The file must be named by `outputs.ticks_jsonl`, normally `ticks.jsonl`.
- Every line must be a valid schema version `1` trajectory control tick object.
- `dt_s` must match the simulated controller period for that tick.
- `sim_time_s` should be populated for simulation runs.
- `wall_time_s` should be `null` unless the runner is measuring real elapsed time.
- Command fields must record the command published by the controller for that tick.
- Measured fields must record the plant state after applying the model's timing convention for that tick. The runner documentation or code should keep this convention stable.

Do not add simulation-only fields to tick objects unless the tick JSONL schema is intentionally revised. Put run-level and aggregate simulation data in `summary.json`.

## Response-Curve And Inertia Model

The `plant.parameters` object must use the versioned response-curve schema below for simulation runs that model command-to-motion dynamics:

- `schema`: string schema id. Current value: `holonomic_response_curve`.
- `version`: integer response-curve schema version. Current value: `1`.
- `curves`: list of axis and direction response curves. A preset must cover `linear_x`, `linear_y`, and `yaw`; asymmetric presets may have separate `positive` and `negative` entries instead of one `bidirectional` entry.

Each curve entry requires:

- `axis`: one of `linear_x`, `linear_y`, or `yaw`.
- `direction`: one of `bidirectional`, `positive`, or `negative`.
- `tau_s`: first-order response time constant in seconds. `0.0` means instant velocity tracking before acceleration limiting.
- `max_accel`: acceleration cap for that axis. Units are `m_s2` for linear axes and `rad_s2` for yaw.
- `command_gain`: scalar applied to the incoming command before deadband and saturation.
- `deadband`: absolute command magnitude below which the command is treated as zero. Units match the command axis: `m_s` for linear axes and `rad_s` for yaw.
- `saturation`: maximum absolute post-gain, post-deadband command for that axis. Units match the command axis.
- `noise_max`: bounded uniform command noise cap. Units match the command axis. The runner must record the seed whenever any curve has non-zero noise.
- `latency_s`: command latency in seconds. `0.0` means the current tick command is applied with no delay.
- `notes`: human-readable curve notes.

Version `1` plant equations are defined per axis. Let `u_raw[k]` be the controller command for tick `k`, `dt_s` be the tick duration, and `v[k]` be the realized body-frame velocity after the tick. A runner that supports latency chooses the delayed command `u_delay[k]` from the command history by `latency_s`; a runner without latency support must reject presets with non-zero `latency_s` rather than silently ignoring it. The effective command is:

```text
u_gain = command_gain * u_delay[k]
u_deadband = 0 if abs(u_gain) < deadband else u_gain
u_sat = clamp(u_deadband, -saturation, saturation)
u_eff = u_sat + uniform(-noise_max, noise_max)
```

The velocity update is:

```text
error = u_eff - v[k - 1]
dv_lag = error if tau_s == 0 else (dt_s / tau_s) * error
dv = clamp(dv_lag, -max_accel * dt_s, max_accel * dt_s)
v[k] = v[k - 1] + dv
```

After updating body-frame `linear_x`, `linear_y`, and `yaw`, the plant integrates pose using the realized body velocity:

```text
vx_world = cos(yaw) * linear_x - sin(yaw) * linear_y
vy_world = sin(yaw) * linear_x + cos(yaw) * linear_y
x[k] = x[k - 1] + vx_world * dt_s
y[k] = y[k - 1] + vy_world * dt_s
yaw[k] = wrap_to_pi(yaw[k - 1] + yaw_rate * dt_s)
```

Built-in preset definitions live in `dimos/navigation/trajectory_response_curve_presets.py`. The required preset names are:

- `ideal`: `response_curve_source: ideal`; mathematical integration baseline with effectively instant response.
- `synthetic_nominal`: `response_curve_source: synthetic`; default synthetic envelope, not a Go2 hardware claim.
- `synthetic_sluggish`: `response_curve_source: synthetic`; slow response and low acceleration for controller margin checks.
- `synthetic_asymmetric`: `response_curve_source: synthetic`; direction-specific linear and yaw response, requiring a runner that supports directional curves.
- `synthetic_noisy`: `response_curve_source: synthetic`; bounded command noise, requiring deterministic seed recording.
- `synthetic_conservative_timing_stress`: `response_curve_source: synthetic`; low acceleration and fixed latency for timing validation.

`response_curve_source` must always be one of:

- `ideal`: a mathematical baseline that makes no hardware claim.
- `synthetic`: a hand-authored envelope for simulation confidence only.
- `fitted_from_go2_log`: parameters fitted from a named Go2 step, dwell, or replay log. The `response_curve_id`, `notes`, and run attachments must identify the source log or fixture.
- `unknown`: a draft or incomplete response curve. PR-gate runs must reject this source and set `summary.verdict.status` to `invalid` if it appears.

When no real Go2 step, dwell, or replay log is available, reports must describe the run as a synthetic envelope simulation. They must not describe the verdict as Go2 hardware confidence. A report may claim empirically fitted Go2 confidence only when the preset source is `fitted_from_go2_log` and the source log is recorded.

## `summary.json`

`summary.json` is the source-of-truth output document for automated review. It records what ran, which artifacts were produced, aggregate metrics, gate results, and a final verdict.

Required top-level fields:

- `schema_version`: integer summary schema version. Current value: `1`.
- `run`: run identity and provenance copied from `config.yaml`, plus output timing.
- `config`: digest and relative path for the config used by the runner.
- `artifacts`: emitted file paths and availability.
- `scenario`: normalized scenario identity.
- `plant`: normalized plant and response-curve identity.
- `controller`: normalized controller identity.
- `limits`: normalized limits used by the controller and plant.
- `rate`: achieved and nominal timing metrics.
- `seed`: random seed used by the run.
- `metrics`: aggregate behavior metrics computed from the tick stream and final state.
- `gates`: each pass/fail criterion and measured value.
- `verdict`: final machine-readable result.

Required `run` fields:

- `id`: run id.
- `started_at`: ISO 8601 timestamp.
- `finished_at`: ISO 8601 timestamp.
- `duration_wall_s`: wall-clock runner duration, or `null` if unavailable.
- `commit`: commit SHA or source revision.
- `dirty`: boolean.

Required `config` fields:

- `path`: relative path to `config.yaml`.
- `sha256`: SHA-256 digest of the exact config file used for the run.

Required `artifacts` fields:

- `ticks_jsonl`: object with `path`, `sha256`, and `line_count`.
- `summary_json`: object with `path`.
- `plot_png`: object with `path`, `sha256`, and `produced`, or `null` when not requested.

Required `scenario`, `plant`, `controller`, `limits`, and `seed` fields must mirror the corresponding effective values from `config.yaml`. The summary may omit verbose plant parameter details only if it records a stable `response_curve_id` and the full values remain in `config.yaml`.

Required `rate` fields:

- `control_rate_hz`: nominal control rate.
- `dt_s`: nominal tick period.
- `tick_count`: number of simulated control ticks.
- `sim_duration_s`: simulated duration covered by the run.
- `achieved_rate_hz`: achieved simulated tick rate, computed from tick count and simulated duration.
- `max_dt_error_s`: maximum absolute tick period deviation from nominal.

Required `metrics` fields:

- `arrived`: boolean.
- `timed_out`: boolean.
- `final_position_error_m`: final planar distance to goal.
- `final_heading_error_rad`: final heading error to goal.
- `max_planar_position_divergence_m`: maximum `planar_position_divergence_m` observed in `ticks.jsonl`.
- `mean_planar_position_divergence_m`: mean `planar_position_divergence_m` over the run.
- `p95_planar_position_divergence_m`: 95th percentile `planar_position_divergence_m`.
- `max_cross_track_error_m`: maximum absolute `e_cross_track_m`.
- `max_heading_error_rad`: maximum absolute `e_heading_rad`.
- `max_commanded_planar_speed_m_s`: maximum `commanded_planar_speed_m_s`.
- `settle_time_s`: first simulated time where arrival tolerances are satisfied and remain satisfied, or `null`.

Required `gates` fields:

- `max_planar_position_divergence_m`: gate object.
- `max_final_position_error_m`: gate object.
- `max_final_heading_error_rad`: gate object.
- `max_settle_time_s`: gate object, or `null` when disabled.
- `require_arrival`: gate object.
- `require_no_timeout`: gate object.
- `require_finite_metrics`: gate object.

Each gate object must contain:

- `threshold`: configured threshold, boolean requirement, or `null`.
- `actual`: measured value or boolean.
- `passed`: boolean.

Required `verdict` fields:

- `status`: one of `pass`, `fail`, or `invalid`.
- `reason`: short stable reason string.
- `failed_gates`: list of failed gate names.
- `invalid_reasons`: list of schema, artifact, or runtime problems that prevent trusting the metrics.

Use `invalid` when the run did not produce required artifacts, produced non-finite metrics, used an unsupported response-curve source for a PR gate, or otherwise cannot be interpreted. Use `fail` when the run is valid but one or more behavioral gates failed.

Example:

```json
{
  "schema_version": 1,
  "run": {
    "id": "synthetic_nominal_s_curve_1p5mps_10hz_seed7",
    "started_at": "2026-04-26T17:00:00+09:00",
    "finished_at": "2026-04-26T17:00:01+09:00",
    "duration_wall_s": 1.0,
    "commit": "abcdef1234567890",
    "dirty": false
  },
  "config": {
    "path": "config.yaml",
    "sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
  },
  "artifacts": {
    "ticks_jsonl": {
      "path": "ticks.jsonl",
      "sha256": "1111111111111111111111111111111111111111111111111111111111111111",
      "line_count": 200
    },
    "summary_json": {
      "path": "summary.json"
    },
    "plot_png": {
      "path": "plot.png",
      "sha256": "2222222222222222222222222222222222222222222222222222222222222222",
      "produced": true
    }
  },
  "scenario": {
    "name": "s_curve",
    "variant": null,
    "target_speed_m_s": 1.5
  },
  "plant": {
    "model": "actuated_holonomic",
    "preset": "synthetic_nominal",
    "response_curve_source": "synthetic",
    "response_curve_id": "synthetic_nominal_v1"
  },
  "controller": {
    "name": "HolonomicPathController",
    "mode": "direct",
    "calibration_source": null
  },
  "limits": {
    "max_linear_speed_m_s": 1.5,
    "max_angular_speed_rad_s": 1.0,
    "max_planar_accel_m_s2": 1.5,
    "max_yaw_accel_rad_s2": null,
    "max_normal_accel_m_s2": 1.0
  },
  "rate": {
    "control_rate_hz": 10.0,
    "dt_s": 0.1,
    "tick_count": 200,
    "sim_duration_s": 20.0,
    "achieved_rate_hz": 10.0,
    "max_dt_error_s": 0.0
  },
  "seed": 7,
  "metrics": {
    "arrived": true,
    "timed_out": false,
    "final_position_error_m": 0.08,
    "final_heading_error_rad": 0.05,
    "max_planar_position_divergence_m": 0.21,
    "mean_planar_position_divergence_m": 0.06,
    "p95_planar_position_divergence_m": 0.14,
    "max_cross_track_error_m": 0.12,
    "max_heading_error_rad": 0.18,
    "max_commanded_planar_speed_m_s": 1.5,
    "settle_time_s": 18.4
  },
  "gates": {
    "max_planar_position_divergence_m": {
      "threshold": 0.35,
      "actual": 0.21,
      "passed": true
    },
    "max_final_position_error_m": {
      "threshold": 0.15,
      "actual": 0.08,
      "passed": true
    },
    "max_final_heading_error_rad": {
      "threshold": 0.2,
      "actual": 0.05,
      "passed": true
    },
    "max_settle_time_s": {
      "threshold": 20.0,
      "actual": 18.4,
      "passed": true
    },
    "require_arrival": {
      "threshold": true,
      "actual": true,
      "passed": true
    },
    "require_no_timeout": {
      "threshold": true,
      "actual": true,
      "passed": true
    },
    "require_finite_metrics": {
      "threshold": true,
      "actual": true,
      "passed": true
    }
  },
  "verdict": {
    "status": "pass",
    "reason": "all_gates_passed",
    "failed_gates": [],
    "invalid_reasons": []
  }
}
```

## Minimal Consumer Checklist

An automated consumer can treat a run as interpretable when all of these are true:

- `config.yaml`, `summary.json`, and `ticks.jsonl` exist.
- `config.schema_version == 1` and `summary.schema_version == 1`.
- `summary.config.sha256` matches `config.yaml`.
- `summary.artifacts.ticks_jsonl.line_count` equals the number of non-empty lines in `ticks.jsonl`.
- Every tick line is valid trajectory control tick JSONL schema version `1`.
- `summary.verdict.status` is one of `pass`, `fail`, or `invalid`.
- PR-gate runs do not use `plant.response_curve_source: unknown`.
