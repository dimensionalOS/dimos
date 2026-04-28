# Pre-Session-3 decision gate

## Inputs

- Task 1 (wz K=2× investigation): **done by user** — outcome to be
  inserted by tracker. Flag below if the bug is `decoder` /
  `cmd-path-codec` / `cmd-path-adapter` / `firmware`.
- Task 2 (validation metric replacement): **done in this branch** — see
  `validation_metric_change.md`.

## wz status

> _**TODO (master-tracker):** insert Task 1 conclusion. Was the K=2×
> from a decoder bug, a cmd-path scaling bug, or firmware behaviour? Is
> a fix planned? On what timeline?_

If the bug is in the cmd path or codec, a fix + small re-collection
batch is required before wz validation results mean anything — the
current data is being captured through a broken codec. **vx is unaffected**
since vx values aren't suspect.

## Validation status

Smoothed-nRMSE metric replaced raw nRMSE. Both channels still FAIL on
the verdict, but the *structure* of the residual is now clear:

- **vx**: 89% of nRMSE is steady-state K bias (~9% of amplitude).
  Transient nRMSE (debiased) is ~7%. **The FOPDT dynamics are fine; only
  K is direction-asymmetric.**
- **wz**: 75% of nRMSE is steady-state K bias. Transient nRMSE is ~17%
  — also large, and possibly entangled with the K=2× scaling issue.

In short: the validation metric is honest and the model's *dynamics* are
correct. What's failing is the *gain* in the held-out direction.

## Recommendation

**Do not start Session 3 yet.** Two cheap unlocks first:

1. **Resolve wz** (gated on Task 1 outcome).
   - If decoder/cmd-path bug → fix it, re-collect E2 (3 amps × 3 reps),
     refit.
   - If firmware behaviour → document, accept K=2.19 as truth.

2. **Refit with direction-aware K** for both channels.
   - Force `pool_session` to keep forward and reverse K separate (i.e.,
     mark `direction_asymmetric=True` more aggressively, or lower the CI-
     overlap threshold that currently lets fwd/rev get pooled).
   - Validation should then look up K by direction. Predicted result:
     vx PASSES (transient nRMSE was 7%); wz pass-rate depends on Task 1
     outcome.

If the team wants to start Session 3 on vx-only in parallel, that's
defensible: vx's 7% transient nRMSE is well within the existing
thresholds, the gain bias is direction-asymmetric (controller can absorb
it via a sign-aware K table), and the lambda-tuning math is the same on
either side.

## Concrete next-tickets (if accepted)

1. *(quick)* Refit Session 1 with `direction_asymmetric` forced on vx
   and wz; re-run Session 2 validation; expect vx to flip to PASS.
2. *(blocked on Task 1)* Fix wz scaling bug if cmd-path; re-collect
   small E2 batch.
3. *(blocked on 1+2)* Session 3 start: closed-loop simulator,
   lambda-tuning for vx and wz with direction-aware K.

## Branch state

`task/mustafa/go2-modelling`:

- Session 2 implementation (modeling/validate_*.py + validate-model CLI)
- Pre-Session-3 metric change (this folder's
  `validation_metric_change.md`)
- This decision note

Uncommitted on top of merged Session 1 work. No dependency on the wz
investigation (which is data-side, not code-side).
