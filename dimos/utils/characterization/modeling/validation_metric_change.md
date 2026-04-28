# Validation metric: smoothed nRMSE + noise-floor diagnostic

## Why we changed it

Session 2's verdict-driving metric was `nRMSE = RMSE(meas - pred) / |amp|`
on raw residuals. Visual inspection of overlay SVGs showed the FOPDT
fits tracking the data well, but every channel's verdict was FAIL with
nRMSE in the 12–28% range. The hypothesis was that raw nRMSE was
penalizing the model for unmodelable leg-jitter — gait-cadence noise the
deterministic FOPDT cannot predict.

## What we did

In `validate_run.py`:

- Added a Sav-Gol smoothing pass (window=11 samples, polyorder=3) over the
  residual `r(t) = meas(t) - pred(t)` before computing RMSE.
- Verdict now uses `norm_rmse_smoothed = RMSE(smoothed(r)) / |amp|`.
- Raw `norm_rmse` stays in the per-run / per-group / per-channel JSON for
  traceability.
- Added `residual_over_noise = rmse_smoothed / σ_noise` as a diagnostic
  (Rung 1 noise floor from `run.json["noise_floor"][channel]["std"]`).
  When this is ≲ 2 the model is at the noise ceiling and FOPDT cannot do
  better.
- Thresholds (10% rise / 15% fall pass; 20% / 25% marginal) unchanged
  — per the plan, threshold tuning needs explicit reasoning.

In `validate_aggregate.py`:

- `GroupSummary` and `ChannelSummary` carry both `rise_norm_rmse_raw` and
  `rise_norm_rmse` (now smoothed) plus `rise_residual_over_noise`. Same
  for fall.

In `validate_report.py`:

- Summary table shows both raw and smoothed medians plus the
  noise-relative ratio so the reader can see what the smoothing did.

## What we found

Validation re-run on `session_20260425-125216` (vx, default mode) and
`session_20260425-131525` (wz, default mode):

| channel | raw nRMSE (med) | smoothed nRMSE (med) | residual_over_noise (med) | verdict |
|---|---|---|---|---|
| vx | 13.6% | 12.2% | 107 | FAIL (1/20 pass) |
| wz | 27.9% | 25.6% | 155 | FAIL (0/20 pass) |

**Smoothing barely moved the metric.** The drop from raw to smoothed is
~1.5 percentage points on each channel — leg-jitter is *not* the
dominant residual source.

The `residual_over_noise` ratio is 100-1600× the stationary noise floor.
Caveat: that floor was measured on the pre-roll (stationary) period, but
during motion the effective floor is much higher (gait-cadence content,
~1Hz at the trot frequency, swamps stationary sensor noise). So this
diagnostic is *biased high* and doesn't cleanly answer "is the model at
the noise ceiling." Worth refining later (in-motion noise estimate from
a spectrum slot above τ⁻¹) but out of scope here.

## What's actually driving the residual: steady-state K bias

A second decomposition split each rise window into the steady-state
plateau (last 30%) and the transient. If the residual is dominated by
the steady-state offset, the model's *gain* (K) is wrong; if it's
dominated by transient mismatch, the model's *dynamics* (τ, L) are wrong.

| channel | rise nRMSE | rise nRMSE after SS-debiasing | |SS residual|/|amp| | bias fraction |
|---|---|---|---|---|
| vx | 10.6% | 7.0% | 9.4% | **89% of nRMSE is steady-state bias** |
| wz | 24.0% | 16.7% | 18.0% | **75% of nRMSE is steady-state bias** |

In English: when we predict a reverse-direction step using the
forward-fit model, the predicted plateau is wrong by ~9% of amplitude on
vx and ~18% on wz. The transient (rise time, deadtime) is fine — only
the gain is off.

This is the direction asymmetry Session 1 already flagged on vx at high
amplitudes — but we now have a quantified validation-set number, and
we've shown it's the entire story for the FAIL verdict.

## What this means

1. **Smoothing is the wrong tool here.** Leg-jitter wasn't dominant; a
   real K-mismatch was. Smoothing earned us ~1.5 pp; nothing will close
   the rest until the model learns direction-aware K.
2. **The FOPDT model's dynamics (τ, L) are fine.** The 7% / 17%
   debiased-transient nRMSEs would pass thresholds easily.
3. **The fix is upstream: refit Session 1 with direction-asymmetric K**
   — i.e., make `pool_session` keep forward K and reverse K separate by
   default (or lower the symmetry threshold so vx and wz don't get
   pooled across direction).
4. **wz still has a real puzzle.** 75% of its nRMSE is bias, but its
   transient nRMSE (17%) is also large. Likely the K=2× anomaly is
   inflating both: the absolute K-difference is bigger when K itself is
   bigger, and the transient may reflect odom-decoder issues independent
   of K. The wz scaling investigation (Task 1, already complete per
   user) should resolve this.

## Threshold proposal (for the master tracker to weigh in on)

The current thresholds (10% rise / 15% fall) reject vx as FAIL even
though its FOPDT dynamics are fine and its only problem is a 9%
steady-state bias from direction-asymmetric K. Two options to surface:

**Option A — Refit before re-thresholding.** Don't change thresholds.
Force `pool_session` to keep direction asymmetric, regenerate the model
summary, re-run validation. Predicted result: vx debiased nRMSE ~7% →
PASS. wz still fails on transient (17%) — needs the scaling fix.

**Option B — Bump thresholds and proceed.** Raise rise pass to 15%, fall
pass to 20%. Predicted: vx PASSES, wz still FAILS but with documented
known-issue tag. Lets Session 3 start on vx-only.

Option A is the right thing if we want the model to be correct. Option B
is the right thing if we want Session 3 unblocked on vx fast.

## Definition-of-done check

- [x] Smoothed-residual nRMSE alongside raw — code in `validate_run.py`,
      `validate_aggregate.py`, `validate_report.py`.
- [x] Noise-floor-relative diagnostic added (with the caveat above).
- [x] Re-run on vx and wz sessions; both metrics in
      `validation_summary.json` and `validation_report.md`.
- [x] Distribution of new metric reviewed.
- [ ] Threshold adjustments — *not made*. Decision belongs in the gate
      note (Task 3), not in code.
