# FOPDT modeling — file-by-file guide

A short walkthrough of what each file in this package does, written for
someone learning controls. Read top to bottom; each section is a few
sentences.

## What we're doing and why

The "plant" is everything between the Twist we command and the body
velocity the robot actually achieves: WebRTC link → onboard locomotion
policy → leg dynamics → odom. We model that whole thing as a single
transfer function:

```
            K · e^(-L·s)
   G(s) = ────────────────
              τ·s + 1
```

Three numbers per channel:

- **K**  — steady-state gain. Command 1.0 m/s, eventually you measure K m/s.
  K=1 means the plant tracks. K<1 means it under-tracks (saturation).
- **τ**  — time constant. After the deadtime, the response reaches ~63% of
  its final value at t=τ. Smaller τ = faster plant.
- **L**  — deadtime (pure delay). The cmd has changed, but nothing has
  happened yet for L seconds.

That's First-Order Plus Deadtime. It's the simplest thing that captures
the three things that actually matter for control design (gain,
bandwidth, delay), and standard tuning rules (IMC, lambda-tuning) take
K/τ/L directly as input. If FOPDT doesn't fit, the residuals tell you
what's missing — but it's the right place to start for >80% of process
plants.

## The pipeline

```
recording.db + cmd_monotonic.jsonl   (one run, raw data)
            │
            ▼  per_run.py
       RunFit (rise + fall fit)
            │
            ▼  aggregate.py (one bucket per recipe)
       GroupFit (K/τ/L mean ± CI per cell)
            │
            ▼  pool.py (one bucket per channel)
       model_summary.json (pooled K/τ/L, gain schedule, asymmetries)
            │
            ▼  report.py + session.py
       model_report.md + plots/
```

`compare-models` is a separate optional step that takes two
`model_summary.json` files (default mode + rage mode) and produces a
verdict per parameter.

## Files

### `fopdt.py` — the model and the fitter

Defines `fopdt_step_response(t, K, tau, L, u_step)` (the math) and
`fit_fopdt(t, y, u_step, ...)` (least-squares fit using `scipy.curve_fit`).

The fit does three things you should know about:

1. **Initial guesses come from the data.** K from the steady-state span,
   L from when the response first leaves the noise band, τ from the
   time-to-63%. Bad initial guesses send the optimizer to bad local
   minima — for nonlinear least squares, this matters more than the
   bounds.
2. **Weighted least squares using the per-channel noise floor σ.** The
   noise floor was computed during pre-roll by `processing/noise.py`. A
   noisier channel gets less aggressive fitting, which makes the CIs
   honest.
3. **95% CIs come from the covariance matrix** (`pcov` from `curve_fit`,
   `±1.96σ` per parameter). When the covariance is singular ("degenerate"),
   we report point estimates with NaN CIs and flag the result.

### `per_run.py` — fit one run

Loads the run via `analyze_run.load_run` (already does Savitzky-Golay
smoothing + central-difference + body-frame rotation to recover
`meas_vx` and `meas_wz` from raw odom), parses the recipe name, then
fits TWO FOPDTs:

- **rise**: cmd 0 → amplitude. The slice is `[step_t, active_end_t]`.
  Baseline = mean of `meas` during pre-roll.
- **fall**: cmd amplitude → 0. The slice is `[active_end_t, end_of_post_roll]`.
  Baseline = mean of `meas` during the last 30% of the active window
  (i.e. the steady-state right before the cmd dropped). `u_step` is
  `-amplitude` (the change in command).

Both fits land on the same `RunFit` dataclass: `params` (rise) and
`params_down` (fall). `select_edge(...)` projects either onto a single
list so the rest of the pipeline stays edge-agnostic.

### `aggregate.py` — one bucket per recipe

Each recipe (e.g. `e1_vx_+1.0`) is repeated 3–5 times per session. This
file pools those repeats:

1. Drop fits where `converged=False` or `degenerate=True`.
2. Compute the median K/τ/L across what's left, then drop any fit that
   sits >2σ off the median on any of K, τ, L (logged with the parameter
   that triggered rejection — useful for forensics).
3. Inverse-variance weighted mean. Each fit's σ comes from its own CI
   width; tight fits get more weight than sloppy ones. The pooled CI
   shrinks as `1/√(Σ wᵢ)` — exactly what you'd expect from independent
   measurements.

### `pool.py` — one bucket per channel + cross-mode comparison

Within a session, for each `(channel, mode)`:

- **Direction symmetry**: at each amplitude, do the forward and reverse
  GroupFits agree (CIs overlap on K/τ/L)? If yes → pool fwd+rev; if no →
  set `direction_asymmetric=True` and keep them separate.
- **Linear-in-amplitude check**: regress each parameter against
  `|amplitude|`. If the slope's 95% CI contains 0, the parameter is
  effectively constant in amplitude. If it doesn't, we have a *gain
  schedule* — the plant behaves differently at different amplitudes (e.g.
  K saturates at high commands), and we report `slope`, `intercept`, and
  CIs so a future controller can compensate.

`compare_models(default, rage)` and `compare_rise_fall(rise, fall)` use
the same verdict scheme: `identical` (<5%), `equivalent` (CIs overlap or
<20%), `differs` (otherwise), `missing`.

### `session.py` — top-level orchestration

Discovers runs, calls `fit_run` on each, pools rise and fall pipelines
separately via `select_edge`, then writes the four output files in
`<session>/modeling/`:

- `fits_per_run.json` — every run's fit, traceable
- `fits_per_group.json` — per-recipe aggregated stats
- `model_summary.json` — pooled per-channel K/τ/L + gain schedules + rise/fall comparison
- `model_report.md` — human-readable summary

Plus per-recipe overlay plots and per-channel parameter-vs-amplitude
plots in `plots/`. `compare_two_sessions` is the cross-mode entry point
used by the `compare-models` subcommand.

### `report.py` — markdown + matplotlib

Builds the markdown report in four sections (per-channel summary,
per-cell table, pooling decisions, diagnostics) plus a step-down
section comparing rise and fall fits. The plots are matplotlib SVGs:
overlay plots show measured traces + group-mean FOPDT curve, parameter
plots show K/τ/L vs |amplitude| with CI error bars.

### `_io.py`

One function: `atomic_write_json(path, data)`. Same pattern as
`processing/noise.py` — writes to a `.tmp` file then `os.replace`s, so
if the process dies mid-write you don't get a half-written JSON.

## Reading a `model_summary.json`

Top-level shape:

```python
{
  "session_dir": "...",
  "mode": "default" | "rage",         # session-level — one mode per session
  "channels": {                       # rise fits (the primary plant model)
    "vx": {
      "direction_asymmetric": bool,
      "linear_in_amplitude": {"K": bool, "tau": bool, "L": bool},
      "pooled":         {"K": {...}, "tau": {...}, "L": {...}},
      "gain_schedule":  {"K": {slope, intercept, slope_ci, ...} | None, ...},
      "per_amplitude":  [...]         # per-cell stats
    },
    ...
  },
  "rise":         {channels: {...}},  # full rise summary (mirror of "channels")
  "fall":         {channels: {...}},  # full fall summary, same shape
  "rise_vs_fall": {channels: {... per-param verdicts ...}},
  "diagnostics":  {n_groups_total, n_groups_with_fit, n_groups_without_fit}
}
```

Use `pooled.K.mean` as your nominal gain, `pooled.tau.mean` as your
nominal time constant, `pooled.L.mean` as your nominal deadtime. Check
`linear_in_amplitude` first — if any are False, prefer the gain
schedule. Check `direction_asymmetric` and `rise_vs_fall` next — if
either flags meaningful differences, your controller may need
direction-aware or rise/fall-aware feedforward.
