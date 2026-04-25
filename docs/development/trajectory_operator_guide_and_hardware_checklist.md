# Trajectory Operator Guide and Hardware Checklist

This guide covers issue 921 holonomic trajectory control: bounded calibration, tick export, speed-vs-divergence plots, and Unitree Go2 hardware validation. The software PR can be reviewed with deterministic plants and tiny in-tree replay fixtures; real-robot sign-off remains a field validation step.

## Operator Guide

Trajectory tuning follows a structured loop: bounded excitation, measure, plot, adjust, and re-test. The Klipper, ArduPilot, and Frigate references in issue 921 are useful workflow analogies, not physics or file-format specs for DimOS.

Before commanding motion:

- Clear the floor area and keep people, loose objects, cords, and trip hazards outside the test envelope.
- Match speed and displacement to the room. The calibration API caps commanded displacement, but the physical workspace must still include stopping margin.
- Keep a physical e-stop, deadman, or battery disconnect path ready before the first `cmd_vel`.
- Record the commit, robot ID, `GlobalConfig`, and any generated calibration YAML for every run you may compare later.

Holonomic path following is selected through `GlobalConfig`:

- `local_planner_path_controller` defaults to `"holonomic"` for the issue 921 path follower; set `"differential"` only for legacy comparison runs.
- `planner_robot_speed` sets the requested local planner speed. Use this for higher-speed Go2 trials instead of changing code constants.
- `local_planner_holonomic_kp` and `local_planner_holonomic_ky` set pose tracking gains. Start from calibration output when available, then validate on the target base.
- `local_planner_max_tangent_accel_m_s2` and `local_planner_max_normal_accel_m_s2` bound the live path speed profile so tight turns slow down before the command reaches the controller.
- `local_planner_control_rate_hz` is the local loop rate. Do not pick a high value because it sounds safer or faster; choose it from tick logs, plant delay, and speed-vs-divergence plots.
- `local_planner_trajectory_tick_log_path="path/to/ticks.jsonl"` streams live `LocalPlanner` ticks to JSONL for plotting and rate analysis.

Run plant-backed calibration with `dimos.navigation.trajectory_holonomic_calibration`. A minimal synthetic run looks like this:

```python
from pathlib import Path

from dimos.navigation.trajectory_holonomic_calibration import (
    HolonomicStepDwellReturnConfig,
    plant_result_to_params_v1,
    run_step_dwell_return_on_plant,
    write_holonomic_calibration_params_yaml,
)
from dimos.navigation.trajectory_holonomic_plant import ActuatedHolonomicPlant

plant = ActuatedHolonomicPlant(
    linear_lag_time_constant_s=0.12,
    yaw_lag_time_constant_s=0.1,
    max_linear_accel_m_s2=50.0,
    max_yaw_accel_rad_s2=50.0,
)
cfg = HolonomicStepDwellReturnConfig(
    max_displacement_m=0.5,
    step_displacement_m=0.1,
    peak_body_speed_m_s=0.3,
    dwell_s=0.05,
    dt_s=0.01,
    body_axis="x",
)
result = run_step_dwell_return_on_plant(plant, cfg)
params = plant_result_to_params_v1(result)
write_holonomic_calibration_params_yaml(params, Path("holonomic_calibration_out.yaml"))
```

The YAML is a generated artifact with a versioned schema. Treat it as calibration output, not as a hand-edited config file.

For plots, collect JSONL from the live `LocalPlanner` config above or from a test harness, then run:

```bash
uv run --with matplotlib python scripts/plot_trajectory_control_ticks.py path/to/ticks.jsonl -o plot.png
```

Use the primary `commanded_planar_speed_m_s` vs `planar_position_divergence_m` view to compare controller gains, control rates, and speed limits. Use the time-series panels to catch schedule jitter, yaw mistakes, and obvious odometry discontinuities.

Re-run calibration or a full A/B plot when gains, control rate, max speed, firmware, payload, flooring, or actuator behavior changes.

## Hardware Checklist

Before motion:

- [ ] Floor area and height clearance fit the planned maneuver and stopping distance.
- [ ] No loose rugs, power cords, wet patches, or small objects are in the path.
- [ ] E-stop, deadman, or battery disconnect path was tested before the run.
- [ ] Cables and tethers cannot wrap legs, snag joints, or pull the robot over.
- [ ] A second person is present when lab policy or risk level calls for a spotter.

Configuration:

- [ ] `local_planner_path_controller` is `"holonomic"` for issue 921 validation.
- [ ] `planner_robot_speed`, holonomic gains, control rate, tangent and normal acceleration caps, and speed limits are recorded.
- [ ] `local_planner_trajectory_tick_log_path` points to a writable JSONL file.
- [ ] Git commit and dirty or clean state are recorded.
- [ ] Calibration YAML is attached if one was generated.

During and after the run:

- [ ] JSONL tick log was captured, or an agreed substitute was recorded with a mapping to the JSONL schema.
- [ ] At least one speed-vs-divergence PNG was generated from the run.
- [ ] Slip, odom jumps, foot collisions, aborts, or operator interventions were noted.
- [ ] Artifact bundle includes JSONL or substitute log, PNG plots, calibration YAML if any, `GlobalConfig` summary, commit, date, and robot ID.
- [ ] Field sign-off results are posted to issue 921 or the team tracker when this is a real P8 validation run.
