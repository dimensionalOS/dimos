# Holonomic trajectory calibration (YAML, v1)

## Purpose

- **Encoding:** UTF-8, YAML 1.1.
- **Producer:** `dimos.navigation.trajectory_holonomic_calibration.write_holonomic_calibration_params_yaml` after a plant (or, later, replay) calibration run.
- **Consumer:** `read_holonomic_calibration_params_yaml`. Files are **generated artifacts** (analog to Frigate `movement_weights`), not hand-tuned in normal use.

## Top-level fields

| Key | Type | Meaning |
|-----|------|--------|
| `schema` | str | Must be `holonomic_trajectory_calibration` for v1. |
| `version` | int | Schema revision; v1 is `1`. Bumps break compatibility. |
| `kind` | str | `plant` (deterministic or recorded plant harness) or `replay` (reserved for odom replays). |
| `generated_utc` | str | ISO-8601 UTC timestamp. |
| `scenario` | map | Input parameters: `max_displacement_m`, `step_displacement_m`, `peak_body_speed_m_s`, `dwell_s`, `dt_s`, and `body_axis` (`x` or `y`). |
| `results` | map | Counters, optional `estimated_linear_lag_s`, and `return_plan_error_m` after the maneuver. |
| `suggested_holonomic_gains` | map | Suggested `k_position_per_s` and `k_yaw_per_s` for `HolonomicTrackingController` (heuristic from the run). |

Callers that require strict validation should reject unknown `version` or `schema` values.
