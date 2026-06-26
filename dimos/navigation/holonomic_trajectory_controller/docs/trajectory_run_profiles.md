# Operator run-profile contract

## Purpose

- **What:** Named movement envelopes (`walk`, `trot`, `run_conservative`,
  `run_verified`) that an operator, agent, CLI, or MCP caller can request
  instead of a loose `planner_robot_speed` number plus a fistful of
  `--local-planner-*` flags.
- **Where:** `dimos.navigation.holonomic_trajectory_controller.trajectory_run_profiles`.
- **Status:** data + validation, plus Go2 locomotion-mode wiring. The profile's
  `required_locomotion_mode` is wired into `GO2Connection` start-up via
  `GlobalConfig.go2_run_profile`. The live stack applies profile speed/accel/yaw
  limits through `LocalPlanner`.

## Data model — `RunProfile`

All numeric fields are **upper bounds in SI units** in the same conventions as
the live limit types (`HolonomicCommandLimits`, `PathSpeedProfileLimits`):
planar speed is `hypot(vx, vy)` in the body frame; yaw rate is `wz`.

| Field | Unit | Meaning |
|-------|------|---------|
| `name` | str | Profile identity (registry key must match). |
| `requested_planner_speed_m_s` | m/s | Requested cruise speed (still curvature/decel-capped downstream). |
| `max_tangent_accel_m_s2` | m/s² | Along-path acceleration cap for the speed profile. |
| `max_normal_accel_m_s2` | m/s² | Centripetal (curvature) acceleration cap. |
| `goal_decel_m_s2` | m/s² | Deceleration approaching the goal. |
| `max_planar_cmd_accel_m_s2` | m/s² | Command slew cap on planar `cmd_vel`. |
| `max_yaw_rate_rad_s` | rad/s | Yaw-rate cap. |
| `max_yaw_accel_rad_s2` | rad/s² | Yaw-acceleration cap. |
| `required_locomotion_mode` | str | Embodiment mode to activate before high-speed motion (Go2: `default` or `rage`). |
| `description` | str | Human-readable note. |

Adapters onto the existing validated limit types:

- `command_limits() -> HolonomicCommandLimits`
- `path_speed_profile_limits_at(max_speed_m_s) -> PathSpeedProfileLimits`

### Validation (rejected at construction)

- Every speed/acceleration/yaw field must be **finite and strictly positive**.
- `name` and `required_locomotion_mode` must be non-empty.

## Profile resolution

`GO2_RUN_PROFILES.get(name)` looks up a profile by name. Unknown names raise
`RunProfileError` with a message listing known profiles.

Per-goal overrides are resolved in `LocalPlanner._resolve_run_envelope`: the goal
profile name wins over `GlobalConfig.go2_run_profile`. The registry default name
(`walk`) keeps legacy walking behavior via `_default_run_envelope()`.

## Go2 profiles (`GO2_RUN_PROFILES`)

Caps are **conservative nominal engineering envelopes, not measured hardware
performance**. `walk` reproduces today's `LocalPlanner` default (0.55 m/s and
the `local_planner_*` defaults). Default `relative_move(...)` stays `walk`.

| Profile | Speed (m/s) | Go2 mode |
|---------|-------------|----------|
| `walk` | 0.55 | `default` |
| `trot` | 1.0 | `default` |
| `run_conservative` | 1.5 | `default` |
| `run_verified` | 2.5 | `rage` |

Set `GO2_RUN_PROFILE=<name>` to select any profile at startup.
