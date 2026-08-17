# go2sim — seams and procedure

A measured Go2 plant, and the pipeline that measured it.

This document states where the package IS. How it got here — the retracted
premises, the day-by-day tables, the superseded presets — lives in git
history of this file, not in it.

---

## 0. Watch it

No environment variables and no checkouts: the scene is vendored (§10), so
these run from a fresh clone. Both open a MuJoCo viewer; both are the same
functions the scored runs use, with a viewer attached and paced to wall clock.

**Loop 2 — the real policy in the sim, against the recorded robot.** This is
the referee, and the closest thing to "does it look right":

```bash
python -m dimos.robot.unitree.go2.sim.sysid.ground \
    ~/recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap \
    data/ml-trajectory-research/freewalk_mcf.bin \
    --view --start 6 --seconds 40 --replicates 1
```

`--replicates 1` runs a single rollout and gets to the viewer sooner — the
report labels it "a draw, not a verdict" (§8); `--speed 0.25` slows it
down to watch footfalls. Add `--preset stock` to see what bare
menagerie does, which is the comparison every claim here rests on.

> **The numbers these commands print are NOT the verdict.** A short window
> with one replicate measures its floor from itself, so the floor is narrow
> and the SNRs are inflated — and a single draw of the SAME plant reads
> anywhere in loss 0.80–2.59, 5–8 of 11 (§8). The verdict needs the full
> span, the default `--replicates 8`, and the robot-repeat floor via
> `--noise-from <two repeat recordings>` (§7). Use these to LOOK; use
> §7's floor to judge.

**Loop 1 — open-loop plant replay, no policy.** Recorded joint targets driven
straight into the plant:

```bash
python -m dimos.robot.unitree.go2.sim.sysid.replay \
    ~/recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap \
    --view --no-reinit --speed 0.25
```

`--no-reinit` disables the multiple-shooting snap-back, which is the only way
to SEE what open loop actually costs — with re-init on it looks deceptively
good, because it is corrected every 0.4 s.

### What to trust with your eyes, and what not to

**Do not judge body tilt against the ghost.** The ghost is the recorded
*tracker* pose, and the tracker's ATTITUDE is a retired instrument (§6):
its flexing mount inflates roll by ~2.5x. The ghost will visibly rock more
than the sim, and that difference is the mount, not the physics.

**Trust the ghost's POSITION**, which is what it is good for: trajectory,
where it ends up, how it turns.

**What is actually still wrong** is an ~8% short stride at matched cadence
(§9), so expect the sim to fall gradually BEHIND the ghost rather than to
look wrong — no single step is visibly off, but over 40 s at ~0.5 m/s it
accumulates into metres of separation. Running with and without
`--preset stock` makes the difference obvious.

**Which recordings can be watched in loop 2:** only those driven by Ivan's
executor, e.g. `20260816-194142`. Recordings made with the Go2's native
runner, whose smoothed output Mode B does not model (§7), are floor and
loop-1 material.

---

## 1. The seam

A backend declares what it can **vary** and what it can **predict**. Both are
backend-specific: PhysX has no `foot_solref_time`, and may not expose a
site-mounted virtual IMU.

```python
class Backend(Protocol):
    def knobs(self)    -> Mapping[str, Knob]:   # what it can VARY
    def channels(self) -> frozenset[str]:       # what it can PREDICT
    def apply(self, values: Mapping[str, float]) -> None    # set the knobs
    def rollout(self, plan: RolloutPlan) -> Prediction      # run the physics

class ClosedLoopBackend(Backend, Protocol):     # declared, like channels()
    def session(self, pose, *, ghost, view, view_speed) -> LoopSession
```

**`apply`** writes knob values into the engine — the only way the fit changes
anything. It is where `armature = 0.006` becomes `dof_armature[LEG_DOFS]` in
MuJoCo, or whatever the equivalent is elsewhere; the caller never touches
engine state directly. Two rules make it safe to call in a loop: a knob left
out keeps the engine's own default rather than being reset to some notion of
zero, and applying the same values twice gives the same model. It is separate
from `rollout` because a fit sets knobs once and then rolls out many segments
against them.

`rollout` is the only one that does physics; the rest are declarations. It
takes a **`RolloutPlan`** — the complete instruction sheet for one replay,
decided before any physics runs — and returns a **`Prediction`**:

```python
RolloutPlan:                       Prediction:
  t0, duration    which stretch      q, dq, tau         joints
  commands        what the robot     imu_accel, gyro    trunk sensors
                  was actually       body_pos, rot      tracker quantities
                  sent: q, dq,       reinit_t/pos/rot   where each clip began
                  kp, kd, tau_ff
  reinit          measured states
                  to snap to, and
                  when
  base            FREE (walking) or
                  PINNED (hanging)
  base_track      if pinned, the
                  measured attitude
                  to follow
```

In one line: *here is what the robot was told to do and where to start from —
tell me what your simulator thinks happened.*

**"Decided before any physics" is the load-bearing part.** Two candidate plants
get an IDENTICAL plan — same segment, same clip boundaries, same snap times,
same commands — so any difference in the output is the physics and nothing
else. If a backend computed its own schedule, or one were drawn fresh per run,
you would be comparing schedules as much as plants. That is not hypothetical:
changing only the random seed moves fitted parameters by up to 8.8×, and a
10-second change in which window is scored swung a headline number 10×. Fixing
the plan upstream keeps that out of every comparison — and it is why the
schedule lives in `sysid.regimes`, so a future backend is handed the same
plans and the cross-simulator comparison means something.

Everything above the seam is simulator-agnostic: ingest, regimes, clips,
segments, scoring, identifiability, the fit. The backend never sees a regime, a
channel weight or a fit — it is handed a plan and returns a prediction.

**The closed loop (Mode B) crosses the same seam through a second, declared
capability.** A `RolloutPlan` is a complete instruction sheet decided before
any physics — which a policy IN the loop deliberately cannot be. So loop 2's
seam is a stepping primitive instead: `ClosedLoopBackend.session()` returns a
`LoopSession` (`state()` / `step(torques)` / `show_ghost` / `close`), and the
GENERIC driver (`sysid.ground.rollout_policy`) owns the policy, the
observation build, the command slew, the actuator chain and all four
default-off loop mechanisms. A second backend implements the five session
methods and inherits the entire referee. Backends are PICKLABLE by contract —
configuration only, no live engine state — which is what lets worker
processes receive the configured backend itself instead of rebuilding one.

**The recording format is the third seamed axis** (engine, robot wire
format, and — via `RobotSpec` in `anchors.py` — the robot's physical
constants): readers implement `RecordingReader` (`sysid/recording.py`),
everything downstream consumes `Streams`, and the Go2 DDS reader in
`ingest.py` is one implementation. All of this is held by `test_seam.py`,
not convention: every above-seam module is imported in a clean interpreter
(no engine may arrive), and an AST pass allows the engine's name only
inside a `main()` — the leak route that once put `mj_step` inside loop 2 is
closed by test.

**Fitted values do not transfer between backends.** The recordings, the
anchors (a weighed robot is 16.5 kg in any simulator), the regime labels and
the method do.

```
recordings ──► ingest ──► regimes ──► segments+clips ──► rollout ──► score
                                                            ▲          │
                                                         BACKEND       ▼
                                                                     knobs
```

### The knob spec

```python
@dataclass(frozen=True)
class Knob:
    lo: float
    hi: float
    log: bool = False   # the range is judged in THIS metric
    unit: str = ""
    why: str = ""       # where the range came from; required to ship

    def position(self, v) -> float:   # where v sits in [0,1], in its own metric
```

```python
"armature": Knob(0.001, 0.05, log=True, unit="kg*m^2",
                 why="seed spread 0.0015-0.0131 (8.8x); cross-regime up to 0.049")
```

Four fields, and three of them exist because of a specific mistake:

- **`log`** — a linear "is it at the bound?" test on a log-scaled range gave
  four false positives, one of which was used to argue that a fit had failed to
  converge. `position()` is a method on the type rather than arithmetic at call
  sites so that the metric cannot be forgotten again.
- **`why`** — a range without provenance is a guess wearing a number's
  clothes. Required for anything shipped, and it is what lets a later reader
  distinguish "measured" from "someone typed this".
- **`unit`** — channels are summed in the score, and unlabelled quantities in
  N·m and kg·m² invite exactly the unit-encoded weights §3 warns about.

A knob's ENTRY says what values are admissible; a knob's PROVENANCE (§3a) says
whether we are searching that range or pinning a known value inside it. The two
are separate: pinning `mass_kg` does not delete its range, it just stops the
search from using it.

---

## 2. Channels: what gets compared

`Prediction` carries only what the real robot also measures — if the robot
can't measure it, it isn't in the type.

| channel | from, on the robot        | rate              |
|---------|---------------------------|-------------------|
| `joint` | `motor_state.q`           | 500 Hz            |
| `dq`    | `motor_state.dq`          | 500 Hz            |
| `tau`   | `motor_state.tau_est`     | 500 Hz            |
| `accel` | `imu_state.accelerometer` | 500 Hz            |
| `gyro`  | `imu_state.gyroscope`     | 500 Hz            |
| `pos`   | tracker                   | ~200 Hz, optional |
| `rot`   | tracker, or IMU attitude  | ~200 Hz           |

**Scoring compares the intersection** of what the recording has, what the
backend predicts, and what the regime permits. No tracker → `pos`/`rot` are
absent and everything else proceeds. A suspended recording has a held trunk, so
`accel` is meaningless there and drops out. Adding a tracker adds a channel;
nothing else changes.

Measured so far, as knobs resolved out of 14 on hard-floor walking:
`accel` 11, `joint` 5, `gyro` 5, `tau` 4.

---

## 3. The score is a weight vector

```
score = Σ  w[channel, regime] · residual[channel, regime] / scale[channel]
```

`scale` makes terms dimensionless — m/s² and rad and N·m cannot be added
otherwise, and unnormalised weights would just encode unit choices. Each
channel's own residual RMS under the BASELINE plant, computed once on the
shared schedule and then FROZEN (`Objective.calibrate`): a scale that moved
with the candidate would make the search see a moving objective, exactly as an
unshared clip schedule would. The noise floor replaces it once a repeat
recording exists.

So "which channel do we score" and "how much does flight count" are the same
question: entries in one weight vector. That vector is a **hyperparameter of
the method**, not a property of the robot — which is what loop 2 selects.

Implemented in `sysid.score`: terms are masked by regime span (a suspended
span permits only `joint`/`dq`/`tau` — a held trunk makes every trunk-frame
signal an echo of the boundary condition), missing terms renormalise instead
of poisoning the total, and the total is the mean over segments of each
segment's weighted term sum — which is what gives the loss a measurable
sampling noise (§4a).

---

## 3a. Everything is a knob; only the provenance differs

There is no separate category of "anchor". `mass_kg`, the tracker lever arm and
the mount yaw are the same kind of object as `armature` — they differ only in
whether we happen to know the value.

The history makes the point: an early revision **fitted** the tracker mount yaw
(272.92°, the circular mean of travel direction under pure +vx) while the lever
arm was **measured** with a ruler. Same quantity, opposite provenance. And both
were once silently wrong — by 178.9° and 38.6 mm.

So: one table, one shape.

```json
{ "mass_kg":        { "pin": 16.500, "why": "kitchen scale" },
  "mount_yaw_deg":  { "search": [0, 360], "wrap": true },
  "tracker_lever":  { "pin": [0.0324, 0, 0.186], "why": "measured 2026-08-16" },
  "armature":       { "search": [0.001, 0.2], "log": true } }
```

`robot.json` is simply a source of pins. Prefer a pin when you can measure the
thing — a weighing found 1.3 kg that no fit ever did — but a pin is a claim
like any other, and a wrong pin held with confidence is worse than a searched
range.

---

## 4. Two loops

|         | loop 1 — identify           | loop 2 — ground                    |
|---------|-----------------------------|------------------------------------|
| runs    | open-loop replay, no policy | the real policy, closed loop       |
| against | recorded signals, per clip  | tracker position + IMU attitude    |
| speed   | thousands of rollouts       | a handful                     |
| decides | **the knobs**               | **loop 1's hyperparameters**  |

**Identify with 1, validate with 2, never fit the plant on 2.** A controller is
designed to hide plant error, so judging the plant through one means judging it
through the thing engineered to mask it. Loop 1 yields ~10⁵ constraints per
recording; loop 2 yields ~11 statistics. Enough to choose between three clip
ranges; nowhere near enough to fit thirteen parameters. This was tried: the
preset fitted on loop 2 is worse than no tuning at all under loop 1.

**And the converse: never promote on a loop-1 score alone.** A plant 20%
better on the open-loop objective was WORSE on the referee on every window
tried (§9). The open-loop objective is not a reliable proxy for closed-loop
fidelity; loop 2 adjudicates every promotion.

### Which signals belong to which loop — and why it must be a partition

Each loop can see something the other cannot, and the design intent is that
each scores **only** what is its own.

- **Loop 1's exclusive capability is joint-level, pointwise.** `q`, `dq` and
  `tau` across 12 joints — 36 signals at 500 Hz, compared sample by sample.
  Loop 2 can never access this: over a 40 s free rollout chaos destroys
  pointwise comparison, which is exactly why loop 2 compares distributions
  instead. Multiple shooting is what buys it — re-initialising every
  0.05–0.8 s keeps the residual a measure of dynamics rather than of
  divergence.
- **Loop 2's exclusive capability is emergent, body-level.** Where the robot
  actually went, how it followed commands, how it oscillated — quantities
  that only exist after seconds of closed-loop behaviour.

  Its intended headline is **windowed positional divergence of `base_link`**:
  re-initialise the closed loop from measured state every T seconds and ask how
  far the sim's base is from the robot's at the window's end. Unbounded position
  error is meaningless — nothing corrects the sim toward the robot, so it
  diverges eventually whatever the plant — but bounded by T it is the most
  directly interpretable number this project can produce: *metres, after
  seconds*, rather than a unitless loss. This is multiple shooting one storey
  up, and T is measurable rather than guessable: sweep it and take where plant
  discrimination peaks against the chaos floor (too short and divergence has no
  time to express; too long and chaos swamps it). Decompose it — **along-track**
  error is largely the known 8% stride deficit (§9), so **cross-track and
  heading** carry the new information, and are what a navigation stack cares
  about. NOT BUILT: `rollout_policy` currently never snaps, so this needs a
  re-init path on the closed loop. It is a first-class term beside the eleven
  statistics, not a replacement — a single scalar cannot see oscillation, gains
  or lags, and collapsing to one number would destroy the misspecification map
  below.

**The partition is what makes loop 2 a referee.** A referee that judges what
the fit optimised is not an independent check, and the pair collapses into one
loop with extra steps. So body-level channels (`pos`, `rot`) carry weight in
loop 2's verdict, and loop 1 earns its keep on the joints.

Today the objective violates this: loop 1 fits **`accel`** — a body-level
signal — at 0.5 floor / 0.5 flight, with `pos`, `rot`, `joint`, `dq`, `tau` and
`gyro` all at zero weight. `accel` was chosen on identifiability grounds (11
of 14 knobs resolved, against `joint`'s 5) and that reasoning is sound as far
as it goes, but resolution is not correctness.

**A hypothesis this partition would test, not an established finding:** the
anti-transfer above may be the overlap biting. Loop 1 fits body acceleration
over 0.4 s; loop 2 judges body motion over 40 s. Those are the same physical
quantity at two horizons, and a plant tuned to match one being worse at the
other is the shape you would expect from two horizons of one quantity fighting.
If so, fitting joints and judging body is not merely cleaner — it is the fix,
and it needs no new recordings to try.

### What loop 2 selects — the weight vector is a misspecification map

Loop 2's job is to choose loop 1's **judge weights**, and it is worth being
precise about what that means, because it is not tuning.

If the simulator were correct, the optimal weights would be a pure *efficiency*
question — weight by inverse noise variance and be done, since a correct model
can match every channel at once. There would be no tension to resolve.

The simulator is misspecified, so no parameter setting matches all channels
simultaneously. Under misspecification the weights decide **which discrepancies
are absorbed into the parameters and which are left in the residual**. Fitting
one channel hard distorts the parameters until that channel matches, and the
distortion reappears elsewhere.

So the fitted weight vector is a **map of where this simulator cannot be made
to fit** — chosen against what matters downstream. Two consequences:

- **Zero weight must never mean unreported.** A de-weighted channel's residual
  is part of the deliverable: *"`tau` was down-weighted to 0.1 and its residual
  sits at 3× the noise floor"* is a precise statement of where the model is
  wrong. That statement is the product, as much as the plant is.
- **It makes a falsifiable prediction.** Two misspecifications are already
  measured directly (§9): real propulsion-axis leg compliance, and MuJoCo's
  μ-insensitive tangential contact creep. If the selected weights de-weight
  exactly the channels those contaminate, the map independently confirms them.
  If they de-weight something else, there is a defect we have not found.

What keeps it anchored to the real robot — both mechanisms already exist and
must bind the weights too:

- **Knob ranges and measured pins are inviolable.** Mass is weighed, torsional
  friction is derived; no weight vector may move them. Without this, loop 2
  drives loop 1 wherever the statistics are flattered — the latency proxy
  (§9) is the worked example of loop 2 wanting something physically impossible.
- **Weights are selected on one recording and reported on another**, so a
  vector that only flatters its selection set is caught.

| hyperparameter           | today                                                     |
|--------------------------|-----------------------------------------------------------|
| channel + regime weights | one channel, `accel`; `w_flight` 0.5 (invented)           |
| clip length range        | U(0.05, 0.8) s, picked by eye                             |
| segment count / length   | stratified, seeded                                        |
| loss statistic           | mean (p50 measured better, never switched)                |

**Three of the axes `OuterPoint` exposes are vacuous on the objective as it
stands, and this is why the outer study has not been run.** With one weighted
channel, `normalised` is a single constant factor on the loss and cannot change
which plant wins or which the fit finds. `w_flight` splits weight against a
`flight` regime that does not exist in an all-floor walking recording — the fit
recordings are 100% `floor`, and flight lives only in the jumps recording,
which is held out. And `stratified` is a coverage question that loop 1's own
identifiability instrument answers far more cheaply than the referee can.

So the objective must be respecified *before* an outer search is worth running:
weight the joint-level channels, and bring a recording with `flight` into the
fit so the regime weights refer to something. Only then do the weight axes
become decisions rather than no-ops.

Implementation is **nested Optuna**: an outer study over loop-1's
hyperparameters, where evaluating one outer trial means running a whole inner
study and scoring its winner on loop 2.

```
outer study            trial = one hyperparameter set
  └── inner study      full loop-1 fit, k seeded restarts, median + spread
        └── rollouts   thousands
  └── score            loop 2: real policy, held-out recording
```

Three consequences worth designing around:

- **Outer trials are expensive** — each is a full fit, 1–2 h. Ten or twenty
  total, not hundreds. Use a sample-efficient sampler (TPE, or plain coordinate
  descent); CMA-ES is the wrong tool at that budget.
- **Three splits, not two.** Inner fits on the fit set, the outer study selects
  on a validation recording, and the final number is quoted on a third
  recording neither has touched. Selecting hyperparameters on the same
  recording you then report is overfitting one storey up, with the added
  insult that it looks rigorous.
- **Keep the outer dimension tiny.** Loop 2 yields ~11 statistics per
  recording. That supports two or three decisions. The rest stay at defensible
  defaults, each labelled *"chosen because X, never validated"*.

---

## 4a. How many restarts? Ask the data, don't type a number

One Optuna study is not a result. Four studies differing ONLY in seed agree on
the loss to within 3 points and disagree on the parameters by up to **8.8×** —
`armature` 0.0015 to 0.0131, `actuator_tau` 0.0009 to 0.0066. The knobs trade
against each other, so the fit locates a REGION and cannot locate a point in
it. Every draw beat the incumbent on held-out data, so the region is real even
though no point in it is identified.

So loop 1 restarts, and stops when it has the region rather than when a counter
runs out:

```
k = 3                                # minimum for leave-one-out to mean anything
loop:
    run study k with seed k
    harvest every trial within 1 SE of that study's best
        (a study makes ~90 trials and keeps 1; the near-optimal ones are
         free samples OF the region. The SE is PAIRED per segment — every
         candidate scores identical segments, so segment difficulty is
         common mode; the unpaired SE is ~16% of the loss and harvests the
         search's whole path instead of the region)
    pool across studies
    point  = per-parameter MEDIAN of the pool
    spread = 10th-90th percentiles
    drift  = max over left-one-study-out recomputations, over searched knobs,
             of |quantile change| / current spread
    stop when drift < 10% twice in a row
    k += 1
```

**Stop on the spread, not the median.** The median's precision does not matter
— every point in the region scores about the same, which is the finding. The
spread is what training consumes.

**Take the median, never the best draw.** Picking the best-scoring restart
spends the held-out set to make the choice and stops it being held out. When we
did take a median it scored −16.9% on held-out jumps, against −11.5% for the
draw that had been shipped by luck of the seed.

**Hitting the cap (~12) is a RESULT, not a failure.** It means the region is
wider than this data can pin — which is exactly what the package exists to say
out loud instead of papering over with a point estimate.

**Sample size matters for the SPREAD, not the point.** min/max over n draws
covers (n−1)/(n+1) of the distribution — 60% at n=4 — so a small-n range is too
NARROW, and under-randomising training is the dangerous direction to be wrong
in.

**Loop 2's verdict follows the same discipline** (§8) — replicated rollouts,
per-statistic median, the spread beside the point.

### Parallelism: pure functions fan out, the sampler stays sequential

A segment rollout is a pure function of (knob values, plan), so segments fan
out across worker processes (`sysid.rollouts`) and reassemble in order — this
speeds up every trial and every Jacobian, and serial vs parallel is
**bit-identical by construction and by test**. Restarts are independent and
run in batches sized to the core budget; overshooting the stopping rule is
accepted (extra samples are what the spread wants anyway). Trials within one
study are NEVER parallelised: CMA-ES updates its distribution from the history
it has seen, so parallel trials change what each trial sees and the study
stops being reproducible from its seed — and seeded reproducibility is what
the whole restart-and-median argument rests on.

```bash
python -m dimos.robot.unitree.go2.sim.sysid.fit REC.mcap --workers 20 \
    --held-out OTHER.mcap --out results/freewalk
```

---

## 5. Tuning from scratch

1. **Weigh the robot.** `robot.json` `mass_kg`. This anchors trunk mass, CoM
   and inertia. The model was 1.3 kg light and no amount of fitting found it.
2. **Record.** Walk it, several speeds, include hard maneuvers. Standing still
   contributes *nothing* — information concentrates in aggressive motion. Hang
   it and run dynamic moves for the joint-level knobs. Keep one recording
   entirely out of the fit.
3. **Declare** the two things no signal reveals: `suspended`, and the floor.
   Everything else — flight spans, contamination, command source, tracker
   presence — is detected.
4. **Identify before fitting.** Run the spectrum. It tells you which knobs the
   data resolves and, when few do, that the robot never did anything hard.
5. **Anchor what physics knows**; fit only what the spectrum says is visible.
6. **Fit k times, take the per-parameter median**, and ship the spread. One run
   is not a result: seeds agree on loss within 3 points and disagree on
   parameters by up to 8.8×. Stop when the spread stabilises, not the median.
7. **Validate on the held-out recording**, on loop 2, with `--view` to watch it
   against the recorded ghost.
8. **Ship the plant AND its ranges.** A point estimate alone is a claim this
   pipeline can prove it cannot make.

---

## 6. The referee's instruments

**Verify the net first.** Mode B is meaningless unless the net in the sim is
the one that produced the recording; `sysid.verify_net` checks by
teacher-forced replay against the recorded `policy/lowcmd`, with a DIFFERENT
net as the yardstick. `freewalk_mcf.bin` explains
`194142_policy-freewalk-hard` at 0.024 rad RMS (ratio 0.123 of the signal);
the v11 control net is 4-5× worse. Same gains (kp 40 / kd 1.0), obs 45×6.

**Position from the tracker, attitude from the IMU.** Loop 2's real side
(`sysid.real.real_summary` — pure recording processing, no engine import,
deliberately outside the MuJoCo-coupled `ground.py`) reads the position
family from the tracker and the attitude family from the IMU quaternion.
The split is not a preference, it is a measurement: the tracker's mount
FLEXES — against the raw gyro it invents roll rate at **2.47×** the true
value with correlation only **0.44** (a rigid mount correlates 0.95+), and
the invented rotation grows 34× with activity, so it fails exactly in the
regime being scored. No constant calibration transform can remove a
time-varying relationship (a full 3-DOF mount fit moved `roll_std` by
0.0000). The IMU side is trustworthy three ways: 0.0103 rad/s RMS at rest
(38× SNR against walking roll), the Go2's own attitude filter reproduces
the raw gyro at correlation 0.992–1.000, and accel reads 9.591 m/s² static
(a 2.2% scale error, irrelevant to rates). **Anyone "simplifying" the
referee back to tracker-everything silently reintroduces a ~2× attitude
error.** Every `Summary` carries an instrument-provenance `source` field
(`pos:tracker att:imu`, `sim`, …) printed on every table, so no claim
silently changes instrument; `attitude="tracker"` keeps the retired
instrument available for diagnosis. A tracker-less recording scores its
attitude statistics (position statistics are NaN and drop out of the SNR)
— the IMU needs no tracker.

**The statistic set: 11 scored, two retired.** The stride pair
(`stride_hz`, `stride_len`) comes from `sysid.gait` — engine-free by
design: foot positions from rigid FK on the joint angles (validated
bit-exact against MuJoCo), touchdown events from the gravity-aligned foot
height, one code path for recording and rollout (`lq` + tracker travel on
one side, `PolicyRun.q` + `run.pos` on the other). Two statistics are
`NOT_COMPARABLE` — printed on every table, never scored:

* `gait_hz` — it measures its own estimator, not the robot: the bob
  autocorrelation locks onto harmonics (it read 1.33 vs 1.67 while the legs
  of BOTH sides cycled at 1.93/1.96 Hz). The stride pair is the cadence
  claim.
* `height_mean` — its value is the room-frame floor calibration, an unknown
  of the recording setup, not of the robot.

The pattern deserves a name: a statistic is untrustworthy when perturbing
its estimator's nuisance parameters (a band edge, a peak margin, a frame
offset) moves it by more than the claim's floor. A standing guard that
nudges nuisance parameters and flags movers is proposed, not built.

---

## 7. Floors: what "close enough" is measured against

**The floor's source is part of the claim.** Two floors exist:

* **sim-perturb (chaos)** — replicate rollouts of the identical plant under
  ±3° initial-pose perturbation; the sim against itself. This is what
  `ground` uses when no repeat recordings are given (the `--replicates`
  rollouts double as the floor).
* **robot-repeat** — repeat recordings of the same walk, battery sag and
  motor temperature included: `ground --noise-from REC2.mcap REC3.mcap`.
  The grounded recording itself stays OUT of the floor, so the floor and
  the verdict are measured on different data. This is the floor the
  publishable sentence needs: *"the simulator differs from the robot by
  less than the robot differs from itself, on N of 11 statistics."*

The robot's own variability is 2–5× the chaos floor on the attitude family
and height (the command/lag family roughly ties) — the chaos floor is the
HARSHER yardstick, never the more lenient one. A 2-sample repeat floor is
if anything too narrow (§4a's small-n rule).

**Which recordings can be grounded, and which only floor.** Mode B models
Ivan's executor — raw un-smoothed actions on `policy/lowcmd`, measured
22.3 ms median tick — which is the loop every policy actually deploys
through. Recordings driven by the Go2's NATIVE policy runner are a second
execution path Mode B does not model: a true 20.0 ms tick, an
output-smoothing stage (teacher-forced `verify_net` explains them at ratio
only ~0.53 vs 0.123 on executor recordings; inverting a fitted EMA
α ≈ 0.3–0.5 halves the residual), and commands only on the 500 Hz
`rt/lowcmd` bus. Grounding one would confound plant error with
executor-model mismatch — they serve the real-side floor and loop 1 only
(the real side needs no net and no executor model). The 2026-08-17
afternoon trio (`153320`/`153558`/`154201`) is native-runner material.

**Floor repeats must also be command-envelope-comparable** to the grounded
recording, or the command-conditioned statistics (`speed_gain`,
`yaw_rate_gain`) compare different walks: the native-runner trio saturates
at vx ±1.5 against `194142`'s ±0.90, and its `speed_gain` reads 0.49–0.54
against 0.88.

**The current verdict floor** is built from two 2026-08-17 freewalk
recordings driven by Ivan's OWN executor (`195401` + `195715`), back to
back in one session, deliberately inside `194142`'s command envelope
(vx ≤ ±0.81 vs ±0.90, never saturating; their `speed_gain` reads
0.872/0.882 against `194142`'s 0.880, where the native-runner trio reads
0.49–0.54) — same executor, comparable envelope, which retires the
executor and envelope caveats a native-runner floor carries. The third
recording of that session (`195539`, the cleanest: `verify_net` ratio
0.077 vs 0.127/0.149) is held entirely out of every selection — reserved
as the verdict recording for a future same-session grounding.
Per-statistic 2-sample spreads, `pos:tracker att:imu`, full spans (the
scoring floor is `usable_floor`: clamped below by 5% of the real value
and a cross-recording floor, which binds on `speed_gain`):

| statistic       | pair spread |
|-----------------|-------------|
| `roll_std`      | 0.003       |
| `pitch_std`     | 0.005       |
| `tilt_p99`      | 0.018       |
| `height_std`    | 0.000       |
| `speed`         | 0.120       |
| `speed_gain`    | 0.010       |
| `yaw_rate_gain` | 0.164       |
| `speed_lag`     | 0.010       |
| `yaw_lag`       | 0.160       |
| `stride_hz`     | 0.217       |
| `stride_len`    | 0.038       |

One caveat rides this floor, stated rather than buried: the wide members
(`speed` 0.120, `yaw_rate_gain` 0.164, `yaw_lag` 0.160) are run-to-run
DRIVING differences, not robot noise — unsaturated free driving varies
more between runs than the native trio's envelope-clipped driving did —
so on those statistics this floor is lenient. The tight members — the
attitude family, `speed_gain`, the stride pair — are the load-bearing
ones, and they are tighter than the native-runner pair gave.

---

## 8. The verdict is a distribution, not a number

Loop 2's point estimate is a single chaotic rollout, and single rollouts do
not resolve plant differences. Variance components, measured on `194142`,
shipped plant, robot-repeat floor (the prior native-runner one — the
structure is the claim, not the third decimal), full span:

* **Chaos.** Replicate groundings of the IDENTICAL plant (±3° initial pose)
  span loss 0.77–1.68 (median 0.86, heavy right tail) and 5–9 of 11; the
  outliers are attitude-family excursions on rough patches.
* **Numerics.** A 1e-9..1e-3 relative knob perturbation redraws the loss
  from the chaos distribution with |Δloss| FLAT in the perturbation size —
  per-draw differences are unresolvable statistically, not numerically.
* **Window.** Median-of-6 verdicts on 20 s windows vary 1.35–1.86 by start
  position; duration 71→40→20 s reads 0.85→1.14→1.51. Both effects dwarf
  chaos: **losses compare ONLY on the identical window.**

**Minimum detectable difference** (bootstrap, 95%, same window and floor):
single draws **0.77** — the size of a verdict itself; median-of-4 **0.31**;
median-of-8 **0.16**; median-of-16 **0.07**. The match count stays coarse
at any n — **k wobbles ±1** whenever statistics sit near SNR 1; quote it
with its draw range, never alone.

So `ground --replicates` (default 8) rolls perturbed rollouts: the verdict
is the per-statistic MEDIAN (§4a's shape), the per-draw loss and k ranges
print on every table, and a single-rollout report labels itself "a draw,
not a verdict". `meta experiment`/`run_outer` ground replicated, log the
draw range, and declare losses closer than the MDD a tie. Eight verdict
rollouts add ~2 min to a 1–2 h outer trial; plant-scale effects (envelope
0.44, tuning ~6) clear the n=8 MDD, anything closer wants
`--replicates 16`. What no replication buys: comparisons across windows,
or more than two or three decisions per recording (§4).

**The headline** (194142, full span, robot-repeat floor from
`195401`+`195715`, `--replicates 8`): `measured` **loss 0.92, 7 of 11**
(draws 0.80–2.59, 5–8 of 11) — what fails is `roll_std` 1.6 and
`speed_gain` 1.5, with `height_std`/`speed_lag` on the 1.0 boundary;
`stock` **12.52, 4 of 11** (draws 11.56–12.99, 3–4 of 11). The
measured-vs-stock separation is ~70× the n=8 MDD. The envelope's loss
contribution (~0.44, measured under the prior native-runner floor) also
clears it, replicated; its match-count gain does not — the envelope's
attitude cost sits ON the robot floor, so k is not a claim the envelope
can carry.

---

## 9. The plant: what is closed, what is open

**What ships** (`ranges.py`): two built-in presets with distinct jobs.
`measured` is the plant — the weighed trunk, six fitted knobs, the measured
torque envelope, and the friction pair DERIVED from `FLOOR_MU = 0.90`
(DR_FLOOR's centre, the μ the fit pins), with torsional friction computed
from its source at import (`anchors.derive`, the same call `fit`'s default
plan pins from, so the plant and the fitting discipline agree by
construction). Every value names its provenance on the preset
(`Preset.provenance`); two structural tests hold the discipline (every
preset value inside the range that admits it, every derived value equal to
its own derivation). `stock` is bare vendored menagerie — kept not as
history but as the experimental control every comparative claim needs:
delete it and "the tuned plant matches the robot better than bare
menagerie" becomes unverifiable, including in the eventual transfer
experiment.

**The plant's provenance is deliberately hybrid — do not "fix" it by
refitting.** The knobs were fitted with the envelope OFF and are run with
the envelope ON, so the envelope's average effect is double-counted into
the viscous/inertial knobs. The envelope-consistent refit was run: it is
the better open-loop identification (−20.3% on loop 1, −2.7% on held-out
jumps) and it grounds WORSE on the referee on every window — the
**anti-transfer** result (§4's converse rule). The hybrid is kept because
the referee prefers it everywhere; the trade is stated on the preset
itself in `ranges.py`.

**The open problem is the speed family.** The sim delivers ~15–18% less
speed per unit command — understriding, not understepping: the gait cycles
at the robot's own rate to within 1.6% (`stride_hz`) and covers ~10% less
ground per cycle (`stride_len`). The policy commands the same steps, the
plant tracks them; the sim swings higher (0.126 vs 0.091 m) and spends a
smaller fraction of the cycle in ground dwell (0.47 vs 0.58). What is
known, so nobody re-opens it:

* **No admissible knob setting closes it.** Every inertia/dissipation
  deflation strides SHORTER; `frictionloss` at its range floor still
  leaves `speed_gain` at 1.5× the robot floor and buys the rest only by
  breaking attitude; `actuator_tau` at 16 ms is the same trade.
* **The measured torque envelope closes about half of it** — attitude-free,
  on both the fit recording and the held-out rubber span. It is a
  measurement (`sysid.drive`'s transfer function), zero free parameters,
  which is why promoting it into the default plant spent no data.
* **The residual ~8% is real and located.** On the robot: measured
  propulsion-axis series compliance — during dwell the real body advances
  FASTER than motor-side kinematics allows, the excess growing with
  horizontal foot force (+2.4…+8.7 (mm/s)/N, CIs excluding zero on 7 of 8
  leg×recording cells; ~8 mm/stance released as body motion the encoders
  never see). In the sim: 12–14 mm/stance of stance slip that is
  INSENSITIVE to μ across 0.6–1.6 — not friction-limited sliding but
  soft-contact tangential creep, a contact-model question, not a knob.
  Vertical compliance is clean on both sides (real excess ~10–20 µm/N,
  under a millimetre at stance loads). Instruments: `sysid.compliance`
  (within-stance-demeaned deflection-vs-load regressions, rigid-sim
  control, injected-spring recovery test).
* **Friction is exonerated at 3× the declared range.** μ 0.6 → 1.6 moves
  closed-loop speed 0.4%, non-monotone, inside noise — against an 18%
  deficit. Nobody needs to sweep it again.
* **The drive is measured first-order.** `sysid.drive` (demanded-vs-
  delivered transfer, zero free parameters, no simulator): equivalent lag
  2–9 ms, |H| exceeds 1 nowhere in band (no resonance), delivered/demanded
  gain 0.94–1.03 at low speed in every amplitude bin down to 0–0.5 N·m
  (no backlash or deadband). Don't go looking there again.

**The loop mechanisms stay default-off** (`action_latency`, `ObsNoise`,
`control_intervals`; the envelope rides the preset instead). Every loop leg
is measured (`sysid.loop`): command transport 1.34 ms, target→plant ≈ 0,
sensor noise 2–3× BELOW the training levels, executor tick 22.3 ms median
— the physical latency budget sums to under 6 ms. Yet an unbounded latency
search buys 10–16 ms, and the "demand" tracks each plant's maximum
survivable delay: it is a PROXY that manufactures oscillation, not a
transport measurement, and under the IMU referee the fitted latency+noise
configuration overshoots oscillation the sim was never missing. Pinning a
real latency wants a hardware timestamp echo, not a fit on the referee.

**A hardware note:** FR's kinematic gain reads 0.63–0.69 on three separate
recordings against 0.88–1.14 for the other legs — whatever it is (pad
wear, a lazy calf), it is on the robot, consistent, and worth a look.

---

## 10. The base model is pinned — vendored assets, exact engine

Every fitted knob is a DELTA on menagerie's `unitree_go2/scene.xml`, so the
base model is part of the plant's identity:

* **Vendored.** `data/go2_menagerie` (via `get_data`, the same LFS road as
  every other data blob) carries the `unitree_go2` subtree + `LICENSE`
  (menagerie's aggregate file; the `unitree_go2` section is BSD-3-Clause,
  Unitree Robotics — notice retained verbatim, as that license asks) from
  google-deepmind/mujoco_menagerie @
  `4c358ef` (2026-06-04), verified byte-identical per file against the git
  tree at vendoring time and pinned since by a tree hash
  (`model.MENAGERIE_TREE_SHA256`, held by `test_vendor.py`). Missing or
  altered assets are a FAILURE, not a skip — the assets ship with the
  repo, so unlike the recordings their absence is never an environment
  gap. `PROVENANCE.md` inside the archive carries the recipe.
* **Resolution.** Vendored by default; `MUJOCO_MENAGERIE` stays as the
  explicit developer override — off the pinned bytes, on your own head.
* **Engine.** `mujoco==3.10.0` exactly, everywhere the repo declares
  mujoco. A range is a claim that the contact solver doesn't matter, and
  §9 measured 12-14 mm of contact creep. Bumping is a deliberate act:
  re-run this package's acceptance suite.

A side effect worth having: `stock` MEANS something — bare vendored
menagerie at a known commit — so tuned-vs-stock is a comparison anyone can
reproduce, not "whatever your checkout had that day".

---

## 11. State

**Built:** the seam (knobs AND channels, `test_seam.py`-enforced), the
MuJoCo backend, plant/ranges/anchors with per-value provenance and
structural tests, ingest, regimes, segments, Mode A replay, identifiability,
the weight-vector score, the fit (pins/searches, seeded restarts, paired-SE
harvest, LOO-spread stopping, median + spread), segment-parallel rollouts
(bit-identical to serial), loop 2 (`sysid.ground`: closed-loop Mode B, the
11-statistic referee with instrument provenance, pluggable noise floors,
replicated verdicts with draw spreads), net-identity verification
(`sysid.verify_net`), the meta-search scaffolding (`sysid.meta`), `--view`
on both modes (ghost, `--speed`, `--no-reinit`; the viewer and the headless
run are the same function), the loop-2 identifiability probe
(`sysid.probe`), the four default-off loop mechanisms with measured values
(`sysid.loop`), the drive instrument (`sysid.drive`), envelope-consistent
fitting (`fit --envelope`, the envelope recorded on the preset and honoured
by every downstream path, `replay` included), the instrument split
(`sysid.real`), the stride instrument (`sysid.gait`), the
series-compliance instrument (`sysid.compliance`), the robot-repeat floor,
and the vendored, hash-pinned base model. Acceptance is bit-frozen against
the shipped plant.

**Not run yet:** the full outer study (10-20 trials × one inner fit each).

**Owed:** a same-session Mode B grounding — the held-out recording
(`195539`, Ivan's executor, untouched by any selection) is the natural
verdict recording for the floor built from its two session-mates,
closing the last floor-vs-verdict session gap.

**Honest caveat:** none of this is validated by a policy transferring to
hardware. The value on offer is the method and the provenance, not an
accuracy claim — and the sharpest illustration of why that distinction
matters is in this file's history: three careful investigations, all sound,
all aimed at an oscillation gap that was mostly a loose tracker mount. The
referee is only ever as good as the sensor behind it.
