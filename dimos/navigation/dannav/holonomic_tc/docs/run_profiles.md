# Operator run-profile contract

## Purpose

Named movement envelopes (`walk`, `trot`, `run_conservative`) bundle
cruise speed and limit caps in one registry entry. `DanHolonomicTC` is the only live
consumer: it resolves a profile name into path-speed limits and command saturation
limits on each follow.

Registry data and validation live in
`dimos.navigation.dannav.holonomic_tc.run_profiles`.
Tuning limits means editing `GO2_RUN_PROFILES` there, or picking a different profile
name at deploy time.

## What is wired today


| Surface                               | How you set the profile                                                     | Status                                                   | Code                                                                                                              |
| ------------------------------------- | --------------------------------------------------------------------------- | -------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| Blueprint                             | `DanHolonomicTC.blueprint(run_profile="walk")` on the `autoconnect` chain   | Wired at deploy                                          | e.g. `unitree_go2_mls_htc.py`                                                                                     |
| CLI                                   | `dimos run <blueprint> -o danholonomictc.run_profile=trot`                  | Wired at deploy                                          | `dimos/robot/cli/dimos.py` (`load_config_args` -> module `config`)                                                |
| Module default                        | Omit `run_profile`; `DanHolonomicTCConfig` defaults to `"walk"`             | Wired                                                    | `module.py` (`DanHolonomicTCConfig.run_profile`)                                                                  |
| Live RPC                              | `DanHolonomicTC.set_run_profile("trot")` on a running module                | Wired in module; no production caller yet                | `module.py` (`set_run_profile` RPC -> `_HolonomicPathFollower.set_run_profile`)                                   |
| Cruise override                       | `DanHolonomicTC.blueprint(speed_m_s=1.0)` or `danholonomictc.speed_m_s=1.0` | Wired; overrides profile cruise only, not accel/yaw caps | `module.py` (`_cruise_speed_override` in `_profile_run_envelope`)                                                 |
| Agent skills (`relative_move`, etc.)  | N/A                                                                         | Not wired to run profiles                                | Skills use `NavigationInterface`; MLS+HTC stack uses click goals via `MovementManager`                            |
| MCP tools                             | N/A                                                                         | Not wired to run profiles                                | No MCP tool calls `set_run_profile`                                                                               |
| `GO2_RUN_PROFILE` env var             | N/A                                                                         | Removed                                                  | Was `GlobalConfig.go2_run_profile`; use blueprint / CLI module config instead                                     |
| Go2 locomotion mode                   | `GO2Connection.blueprint(motion_mode="mcf")`                                | Separate from run profiles                               | e.g. `unitree_go2_mls_htc.py`; not driven by `run_profile`                                                      |




### Limit flow (when wired)

1. `DanHolonomicTCConfig.run_profile` names a registry entry.
2. `_HolonomicPathFollower._resolve_run_envelope()` calls `GO2_RUN_PROFILES.get(name)`.
3. `_apply_run_envelope()` sets path-speed limits and passes the `RunProfile` into
  `HolonomicPathController` (`set_profile`, `set_speed`).
4. Each control tick: path reference speed from the profile's path limits; `cmd_vel`
  slew from the profile's yaw/accel caps (`holonomic_path_controller.py`).



## Data model - `RunProfile`

All numeric fields are **upper bounds in SI units** in the same conventions as
the live limit types (`HolonomicCommandLimits`, `PathSpeedProfileLimits`):
planar speed is `hypot(vx, vy)` in the body frame; yaw rate is `wz`.


| Field                         | Unit   | Meaning                                                                                                     |
| ----------------------------- | ------ | ----------------------------------------------------------------------------------------------------------- |
| `name`                        | str    | Profile identity (registry key must match).                                                                 |
| `requested_planner_speed_m_s` | m/s    | Requested cruise speed (still curvature/decel-capped downstream).                                           |
| `max_tangent_accel_m_s2`      | m/s²   | Along-path acceleration cap for the speed profile.                                                          |
| `max_normal_accel_m_s2`       | m/s²   | Centripetal (curvature) acceleration cap.                                                                   |
| `goal_decel_m_s2`             | m/s²   | Deceleration approaching the goal.                                                                          |
| `max_planar_cmd_accel_m_s2`   | m/s²   | Command slew cap on planar `cmd_vel`.                                                                       |
| `max_yaw_rate_rad_s`          | rad/s  | Yaw-rate cap.                                                                                               |
| `max_yaw_accel_rad_s2`        | rad/s² | Yaw-acceleration cap.                                                                                       |


Adapter onto the existing validated limit type:

- `path_speed_profile_limits_at(max_speed_m_s) -> PathSpeedProfileLimits`



### Validation (rejected at construction)

- Every speed/acceleration/yaw field must be **finite and strictly positive**.
- `name` must be non-empty.



## Profile resolution

`GO2_RUN_PROFILES.get(name)` looks up a profile by name. Unknown names raise
`RunProfileError` with a message listing known profiles.

The session profile is `DanHolonomicTCConfig.run_profile`, resolved in
`_HolonomicPathFollower._resolve_run_envelope` at module construction. The
`set_run_profile` RPC re-resolves and applies a new profile live. The registry is
the single envelope source: even `walk` reads its caps from `GO2_RUN_PROFILES`,
not from removed `GlobalConfig.local_planner_*` fields.

## Go2 profiles (`GO2_RUN_PROFILES`)

Caps are **conservative nominal engineering envelopes, not measured hardware
performance**.


| Profile            | Cruise speed (m/s) |
| ------------------ | ------------------ |
| `walk`             | 0.55               |
| `trot`             | 1.0                |
| `run_conservative` | 1.5                |


Example blueprint line:

```python
DanHolonomicTC.blueprint(run_profile="walk"),
```

Example CLI override:

```bash
dimos run unitree-go2-mls-htc -o danholonomictc.run_profile=trot
```
