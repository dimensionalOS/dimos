# go2sim — seams and procedure

A measured Go2 plant, and the pipeline that measured it.

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

The history makes the point: the frozen code **fitted** the tracker mount yaw
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

|         | loop 1 — identify           | loop 2 — ground               |
|---------|-----------------------------|-------------------------------|
| runs    | open-loop replay, no policy | the real policy, closed loop  |
| against | recorded signals, per clip  | tracker pose + run statistics |
| speed   | thousands of rollouts       | a handful                     |
| decides | **the knobs**               | **loop 1's hyperparameters**  |

**Identify with 1, validate with 2, never fit the plant on 2.** A controller is
designed to hide plant error, so judging the plant through one means judging it
through the thing engineered to mask it. Loop 1 yields ~10⁵ constraints per
recording; loop 2 yields ~11 statistics. Enough to choose between three clip
ranges; nowhere near enough to fit thirteen parameters. This was tried: the
preset fitted on loop 2 is worse than no tuning at all under loop 1.

### Meta-search: what loop 2 selects

| hyperparameter           | today                                                     |
|--------------------------|-----------------------------------------------------------|
| channel + regime weights | one channel at a time, `accel`; `w_flight` 0.5 (invented) |
| clip length range        | U(0.05, 0.8) s, picked by eye                             |
| segment count / length   | stratified, seeded                                        |
| loss statistic           | mean (p50 measured better, never switched)                |

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

## 6. State

**Built:** seam (knobs AND channels), MuJoCo backend, plant, ranges, anchors,
ingest, regimes, segments, Mode A replay, identifiability, the weight-vector
score, the fit (pins/searches, seeded restarts, paired-SE harvest, LOO-spread
stopping, median + spread), segment-parallel rollouts. Acceptance is
bit-identical to the frozen instrument, and parallel is bit-identical to
serial.

**Not built:** loop 2, meta-search, `--view` (a regression — the frozen tool
had it).

**Honest caveat:** none of this is validated by a policy transferring to
hardware. The value on offer is the method and the provenance, not an accuracy
claim.
