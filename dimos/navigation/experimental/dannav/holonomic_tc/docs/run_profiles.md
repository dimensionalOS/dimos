# Operator run-profile contract

Named movement envelopes (`walk`, `trot`, `run_conservative`) bundle a cruise speed
and its limit caps in one entry of `GO2_RUN_PROFILES`
(`dimos.navigation.experimental.dannav.holonomic_tc.run_profiles`). `DanHolonomicTC`
is the only consumer.

## Setting the profile

| Surface         | How                                                         |
| --------------- | ----------------------------------------------------------- |
| Blueprint       | `DanHolonomicTC.blueprint(run_profile="walk")`              |
| CLI             | `dimos run <blueprint> --run-profile=trot`                  |
| Default         | `DanHolonomicTCConfig.run_profile` is `"walk"`              |
| Live RPC        | `DanHolonomicTC.set_run_profile("trot")`                    |
| Cruise override | `speed_m_s=1.0`; overrides the cruise speed only, not caps  |

`get_run_profile(name)` resolves a name; unknown names raise `RunProfileError`
listing the known profiles. The resolved profile goes to
`HolonomicPathController.configure`, which profiles the speed along every path it is
given (`geometry/path_speed_profile.py`) and caps the commanded speed and yaw rate.

## `RunProfile`

All numeric fields are finite, strictly positive upper bounds in SI units.

| Field                         | Unit  | Meaning                                                       |
| ----------------------------- | ----- | ------------------------------------------------------------- |
| `name`                        | str   | Profile identity, the `GO2_RUN_PROFILES` key.                 |
| `requested_planner_speed_m_s` | m/s   | Cruise speed, and the cap on commanded planar speed.          |
| `max_tangent_accel_m_s2`      | m/s²  | Along-path acceleration from the start of a path.             |
| `max_normal_accel_m_s2`       | m/s²  | Centripetal cap, `v <= sqrt(a_n / kappa)`.                    |
| `goal_decel_m_s2`             | m/s²  | Deceleration into the end of a path.                          |
| `max_yaw_rate_rad_s`          | rad/s | Yaw-rate cap; also slows turns, `v <= w / kappa`.             |

## Go2 profiles

Conservative nominal envelopes, not measured hardware performance.

| Profile            | Cruise speed (m/s) |
| ------------------ | ------------------ |
| `walk`             | 0.55               |
| `trot`             | 1.0                |
| `run_conservative` | 1.5                |
