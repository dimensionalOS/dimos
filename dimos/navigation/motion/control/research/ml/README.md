# control/research/ml — the learned trajectory controller

PPO on the referee's own closed-loop episodes. The policy is a
`TrajectoryController` like any other, so it is scored by the identical
battery, tracks and judge as the shipped laws.

## Status, 2026-08-06 — pipeline works, PPO did not pay

Built and validated end to end on the `hinted` track. **Behaviour cloning
works; PPO fine-tuning on top of it did not beat the clone**, and the policy
search was handed to autoresearch instead. Everything below is the record of
what was measured, so the next attempt starts from it rather than from zero.

| candidate | curated 16 | gen 40 | battery | in-distribution val |
|---|---|---|---|---|
| `hinted` (shipped law) | 115.36 | 115.21 | **115.25** | — |
| `ml` (BC clone of it) | 81.84 | 93.07 | **89.86** | 115.05 |

The clone reaches the teacher's level *on the teacher's own distribution* and
loses ~25 points off it, by stalling rather than crashing (13 timeouts of 56,
zero collisions, zero falls). PPO from that start, once it stopped collapsing
outright (see [Fine-tuning a clone](#fine-tuning-a-clone-without-destroying-it)),
drifted **down** — eval 100.39 at iteration 5 against the clone's 115.05, with
per-iteration rollout scores oscillating 77–110. Best-checkpoint selection kept
the clone, so nothing was lost, but nothing was gained either.

Two untried levers, in the order they deserve trying:

1. **More worlds.** Training was 120 generated worlds resampled 24 at a time,
   which lets the policy drift onto them while the disjoint val set degrades.
   The generator is unbounded; a 600-world set was being built when this
   stopped. `_gen_cache` is keyed on `(count, seed)`, so pick the number once —
   going 120 → 600 regenerates all 600 rather than extending.
2. **A trust region.** Vanilla PPO with no KL guard lets each iteration move a
   near-optimal policy further than the advantage estimate justifies. Target-KL
   early stopping per iteration is the standard fix and was not tried.

The honest read is still the one in
[What this can and cannot win](#what-this-can-and-cannot-win): against a
referee the shipped law already scores 115.23 of 115.5 on, there is almost no
headroom for *any* method. That is a benchmark problem, not a PPO problem.

Training is **two stages** and the first is not optional — see
[Why cloning first](#why-cloning-first).

```bash
# 1. clone the shipped law (writes checkpoints/bc.pt + logs/bc_bc.json)
python -m dimos.navigation.motion.control.research.ml.bc --jobs 12

# 2. PPO from there (adaptive stop; logs/train_<tag>.json + checkpoints/<tag>.pt)
python -m dimos.navigation.motion.control.research.ml.train --jobs 12 \
    --init dimos/navigation/motion/control/research/ml/checkpoints/bc.pt

# score the trained net on the real battery, both tracks
python -m dimos.navigation.motion.control --score --controller ml
python -m dimos.navigation.motion.control --score --blind --controller ml

# against the shipped law on the same track — the A/B that means something
python -m dimos.navigation.motion.control --score --controller hinted

# watch one
python -m dimos.navigation.motion.control --view -s zigzag_room --controller ml

# held-out rules, never trained on
python -m dimos.navigation.motion.control --score --ood 20 --controller ml

# tests and types
python -m pytest dimos/navigation/motion/control/research/ml -q
python -m mypy dimos/navigation/motion/control
```

`ML_TC_CKPT=/path/to/net.pt` picks a checkpoint other than `checkpoints/base.pt`.

## Layout

```
obs.py        observation builder — the seam's inputs as a flat vector (SEARCH SURFACE)
policy.py     actor-critic net + the action -> twist envelope
env.py        the RL environment: referee episode, taped; reward shaping
bc.py         stage 1: behaviour-clone the shipped law
train.py      stage 2: PPO with an adaptive budget
candidate.py  the deterministic controller the referee loads (--controller ml)
logs/         committed training logs (one JSON per run)
checkpoints/  weights (gitignored — train before you score)
```

## The environment is an inversion, not a simulator

There is no simulator here and there must not be one. The plant is the matched
MuJoCo go2 under the real walking policy blob (`motion/simulation/`, physics
fitted to two real recordings), and the loop around it is the referee's own
`run_episode` — same mechanism chain, same 29 Hz zero-order-hold pose, same
5 Hz replan, same judge.

`run_episode` calls `controller.update()` inside its loop, so `env.Agent` is a
controller that answers with the net and writes down what it saw and did on the
way past. The episode runs to completion; only then is the tape priced. **No
byte of `referee/` changes**, so no lab's `referee.lock` moves and the numbers
stay comparable to every other candidate by construction.

## What the net reads

`obs.py` is the search surface — arc marks, scalars, frame and scales are all
candidate-internal and all fair game. Everything keys off `obs.FORMAT`, so a
changed builder invalidates old checkpoints loudly.

The inputs are the controller seam and nothing else: pose, path, tick, room
hint. Two deliberate choices:

- **No embodiment vector.** The body under the controller is always the go2 —
  the referee's sim has one robot, and a scenario's `emb` shapes the *plan*,
  not the plant. Conditioning on it would be conditioning on something that
  never moves. The planner-side lab does take one, because there the body is a
  real input.
- **Both tracks fill one slot.** `hinted` is handed live clearance; `blind`
  decodes the profile the planner stamped into the path (`control/profile.py`),
  which is what `laws/blind.py` does. One net serves both, and a track bit
  tells it whether the room number was measured or inferred.

Room is clipped to `MAX_ROOM`. That is load-bearing, not tidy: an
obstacle-free world has *infinite* clearance, and an inf reaching the net makes
every output NaN, the twist NaN, and the sim unstable. Past the governor's
cruise threshold more room changes no decision anyway.

Actions are normalised to `[-1, 1]` and denormalised against the
`ControllerConfig` envelope, planar pair clipped as a **vector** — a per-axis
clip would let a diagonal reach `max_speed * sqrt(2)`, which no law does and
the plant would not deliver. The envelope is `laws/hinted.py`'s `ENVELOPE`
(0.45/0.95), because that is a *plant* measurement: gait initiation is a
bifurcation near 0.28 commanded, and a net clipped at the referee's 0.5 default
could not reach the hinted track's 0.75 m/s cruise target at all.

## Why cloning first

PPO from scratch does not find this task, and the reason is the plant rather
than the algorithm. Two things conspire:

- **Gait initiation is a bifurcation**, not a ramp — around 0.28 commanded
  (`laws/hinted.py`'s `ENVELOPE` documents the open-loop sweep). A mean-zero
  policy with enough exploration noise to be safe sits *below* the walk
  threshold, so the body never moves far enough to see a progress gradient.
- **Standing still pays.** The judge scores `100*arrived + 10*precision`, and a
  robot that never entered the world has a spotless precision pillar. Doing
  nothing banks 10.1 points and risks none of the gate.

Measured, from scratch: 6 iterations, `arrived 0.00`, score flat at 10.4 — the
policy found that local optimum immediately and stayed. `W_TIME` now charges
for the clock so a timeout is strictly worse than an arrival, but that alone
does not manufacture the exploration needed to cross a bifurcation.

So the net starts *from the law*. `env.TeacherAgent` wraps any shipped
controller, runs it inside real episodes, and tapes the observation the net
would have seen against the action the law actually took — on-distribution by
construction, because it is the law's own states. `bc.py` fits the actor to
that by regression.

Cloning caps the net **at** the teacher; it buys the state distribution PPO
needs before its gradients mean anything. Beating the teacher is `train.py`'s
job, and the comparison is always the same battery against
`--controller hinted`.

Cloning is also **not** the whole answer. Measured: the clone scores 115.05 on
worlds from its own generator but **89.86 on the canonical battery** (curated 16
= 81.84, gen 40 = 93.07) against the law's 115.25. That is covariate shift —
BC matches the teacher's actions on the teacher's states, small errors compound,
and the body drifts into states no training data covers. It fails by *stalling*
(13 timeouts of 56), never by colliding. Closing that tail on-policy is what
PPO is for.

## Fine-tuning a clone without destroying it

Three things had to be true before PPO stopped wrecking a 115-point policy.
All three were measured here, not assumed, and all three are cheap to get
wrong:

**1. Split the trunks.** The actor and critic must not share a representation.
BC fits the actor only, so PPO's first iteration starts with a random critic
regressing returns two orders of magnitude larger than anything it emits;
through a shared trunk those value-loss gradients tear up exactly the features
the clone just learned. Measured: one iteration took the net to 8
disqualifications, the next to 18 of 24. `test_value_loss_cannot_reach_the_actor`
pins it.

**2. Scale the reward.** The weights are written in judge points so they can be
argued against the score, which leaves the critic regressing O(100) targets.
`REWARD_SCALE` divides once at the end, keeping every ratio as reasoned about
and handing the optimiser O(1). Value loss went from exploding (43 → 102) to
0.008.

**3. Scale exploration to the task, not to convention.** This one is the trap.
The clone tracks the teacher to ~0.02 m/s. A conventional PPO `log_std` of -1.6
is σ 0.2 normalised, which on this envelope is **0.19 m/s of random command per
tick** — ten times the signal it perturbs. Measured with the actor *frozen*, so
nothing but the sampling was acting: 7 of 24 episodes disqualified and the
rollout score fell from 115.05 to 38.73. **The noise alone destroyed the
policy, before a single gradient step.** `--init-log-std` defaults to -3.0
(σ ≈ 0.05, 0.047 m/s) — a couple of times the clone's own tracking error, small
enough that an episode survives being explored.

For the same reason `--ent-coef` defaults to **0**: `log_std` is a free
parameter, and an entropy bonus pays the policy to widen it, re-inflating the
noise `--init-log-std` exists to hold down. Entropy buys exploration in
sparse-reward tasks; here the reward is dense and the start is already good, so
it only buys collisions.

`--warmup-iters` then trains the critic alone for a few iterations, so the
value function is worth listening to before the policy is allowed to move.

## Reward is shaping; the judge is the objective

`judge.score_episode` is what selects checkpoints and what the logs report.
The per-tick reward exists only to make the terminal reachable, and it is sized
to keep the judge's **lexicographic ordering inside the shaping**: over a
typical episode, closing a 4 m route earns ~40 while riding the clearance floor
the whole way costs ~120. A policy cannot buy its way out of a precision
violation with pace — the property the judge has and a naive dense reward
loses. Collision or fall is a terminal `R_GATE`, mirroring the judge's gate.

## Seed hygiene

| range                        | use                                                |
|------------------------------|----------------------------------------------------|
| `scenarios.GEN_SEED = 0` ..  | the referee's generated battery — never trained on |
| `battery.OOD_SEED = 7700` .. | held-out rules — never trained on                  |
| the curated 16               | never generated at all                             |
| `15_000..`                   | validation (`env.VAL_SEED0`)                       |
| `20_000..`                   | training (`env.TRAIN_SEED0`)                       |

`test_ml.py` asserts the disjointness rather than trusting the comment.

## What this can and cannot win

**The referee is close to saturated.** Autoresearch took hinted to 115.23 of a
115.5 ceiling and blind to 110.77 of 111, with zero collisions across 228
reality-mode episodes ([`../auto/README.md`](../auto/README.md)). Against
*this* referee a perfectly trained net ties, and that is the honest expectation
to hold: the binding constraint is the benchmark, not the algorithm.

What reopens it is closing the gap between the referee's world and the robot's.
The referee samples analytic box surfaces on a 0.05 m grid — no dropout, no
range noise, no occlusion, no ego-motion smear, full 360° coverage including
the far side of every obstacle — and hands the follower a clean 29 Hz pose. The
real stack sees a partial, occluded `local_map` and a Point-LIO pose that
drifts. That regime — many weak, noisy, partially observed cues instead of a
few clean geometric ones — is where a learned law earns its keep and a
hand-shaped one does not. `--dr` already draws the fitted mechanisms per
episode; cloud realism and pose drift do not exist yet.

**The gait cost is real and should stay priced.** The shipped laws carry an
explicit calibration (`walk_gain`, `walk_slip`, measured by
`referee/probe_walk_slip.py`) that is a property of the policy blob, not of the
law — on a new gait it is *re-keyed*, two scalars. A net absorbs the gait into
its weights and has to be **retrained**. This candidate is trained against
`ml-trajectory-research/freewalk_mcf.bin`; on a different blob its number means
nothing until it is retrained.

## Shipping

Python inference is a research vehicle — it proves the pipeline and the
accuracy ceiling, not the deployment path. A shipped learned law is a rust port
under `control/rust/` held to `test_rust_parity.py`'s 1e-9 discipline, as the
hand-written laws are: ONNX + tract, single-threaded and deterministic, with
`obs.py` ported alongside. The observation was kept to the seam's own inputs
partly for this reason — there is no extra plumbing to port.
