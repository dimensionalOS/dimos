# control/research/ml — the learned trajectory controller

Empty on purpose. The seam and the entry conditions are written down here so
that starting is a decision, not a design exercise.

## The seam

A learned controller is a `TrajectoryController` like any other — nothing in
the harness needs to change to score one:

```python
# control/research/ml/candidate.py, when it exists
def make() -> TrajectoryController: ...   # reset(), update(pose, path, t, clearance) -> Twist
```

```bash
python -m dimos.navigation.motion.control --score \
    --controller dimos.navigation.motion.control.research.ml.candidate:make
```

`controller.py:load()` resolves any `module:factory` string, so the net is
judged by the identical battery, tracks and judge as the shipped laws. Add a
short `ml` name to `REGISTRY` once a candidate actually exists — not before, or
`--ls` starts advertising a module that fails to import.

## Why it is not started

The referee is already saturated: autoresearch took hinted to 115.23 of 115.5
and blind to 110.77 of 111, with zero collisions across 228 reality-mode
episodes ([`../auto/README.md`](../auto/README.md)). A perfectly trained net
ties. **The binding constraint is the benchmark, not the algorithm.**

What would reopen it is closing the gap between the referee's world and the
robot's: the referee samples analytic box surfaces on a 0.05 m grid — no
dropout, no range noise, no occlusion, no ego-motion smear, full 360° coverage
including the far side of every obstacle — and hands the follower a 29 Hz
zero-order-hold pose. The real stack sees a partial, occluded `local_map` and a
Point-LIO pose that drifts. That regime — many weak, noisy, partially observed
cues instead of a few clean geometric ones — is where a learned law earns its
keep and a hand-shaped one does not.

So the order is: harden the referee, re-baseline both laws on it, and only then
is there a number worth training against.

## The cost to weigh first

The shipped laws carry an explicit gait calibration (`walk_gain`, `walk_slip`,
measured by `referee/probe_walk_slip.py`) that is a property of the policy
blob, not of the law — on a new gait it is **re-keyed**. A net absorbs the gait
into its weights and has to be **retrained**. That is a real operational cost of
going learned here, not a detail, and it should be priced before starting.

## If it does start

Train on generated worlds only; the curated 16 and the OOD rules are held out
(`referee/battery.py:OOD_RULES`). Report `judge.score_episode`, never the
training reward. Keep the judge's lexicographic ordering — precision above pace
— in the reward, or the net will learn to rush gaps. Ship via a rust port under
`control/rust/` held to `test_rust_parity.py`'s 1e-9 discipline, as the
hand-written laws are.
