# dimos.control.autotune

Characterize a mobile base's velocity plant and emit tuned controller gains from
a declared robot profile. Supersedes `dimos/utils/characterization` and
`dimos/utils/benchmarking` for plant identification and PI tuning; those trees
are deprecated pending parity.

User guide: [docs/capabilities/control-autotune.md](../../../docs/capabilities/control-autotune.md).

## Layout

| File | Role |
|---|---|
| `profile.py` | `RobotProfile` — declared inputs + measured-properties slot |
| `probe.py` | passive stream timing/noise probe (advisory; no motion) |
| `excitation.py` | step / ramp / deadzone signal generators + batteries |
| `drive.py` | sequencer: one excitation run = one episode (protocol-based) |
| `live.py` | live adapters binding the sequencer to coordinator + recorder |
| `fit/` | velocity + pose FOPDT fitters + synthetic ground truth |
| `properties.py` | direction asymmetry, gain schedule, coupling, deadzone |
| `bandwidth.py` | bandwidth + Bode + pole-zero from the fit (no python-control) |
| `tune.py` | lambda-PI tuning, closed-loop scoring, robustness sweep, verdict |
| `report.py` | tuned artifact (task `from_artifact` shape) + characterization report |
| `runner.py` | offline pipeline: episodes → fit → bandwidth → tune → emit |

## Property triage

Measured: FOPDT `K/tau/L` per axis, bandwidth, saturation, gain-vs-amplitude
nonlinearity, direction asymmetry, cross-axis coupling, deadzone, stream rate /
jitter / noise floor.

Deliberately **not** measured (marginal benefit): isolated measurement latency
(lumped into `L`), hysteresis, and automatic controller-structure selection.

## Notes for contributors

- The FOPDT fitters and the lambda-tune math were ported from the
  characterization/benchmark branches and made robot-agnostic: Go2-specific
  constants (saturation limits, control rate, cost weights, verdict thresholds)
  are parameters or named module constants, not baked-in values.
- The tuned artifact's `vx/vy/wz` slots are a fixed contract the consuming task
  reads. `report.py` projects the channel-list profile onto those slots and
  rejects channels outside `{vx, vy, wz}`.
- The closed-loop scorer is O(n²) by construction (re-simulates the full command
  history each control tick), preserved exactly from the source. Vectorizing it
  is a future optimization, not a behavior change.
