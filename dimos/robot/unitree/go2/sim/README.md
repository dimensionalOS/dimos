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

## 5a. Loop 2 in practice

**Verify the net first.** Mode B is meaningless unless the net in the sim is
the one that produced the recording; `sysid.verify_net` checks by
teacher-forced replay against the recorded `policy/lowcmd`, with a DIFFERENT
net as the yardstick. Measured: `freewalk_mcf.bin` explains
`194142_policy-freewalk-hard` at 0.024 rad RMS (ratio 0.123 of the signal)
and the freewalk span of `015155_policy-mixed` at 0.037 rad (0.181); the v11
control net is 4-5× worse on both. Same gains, obs 45×6.

**The floor's source is part of the claim.** Today `sim-perturb` (the sim
against itself under chaos); the better floor is `robot-repeat` — repeat
recordings of the same walk, battery sag and motor temperature included —
which `ground --noise-from REC2.mcap REC3.mcap` swaps in without a rewrite
(the grounded recording itself stays out of the floor), and which the
publishable sentence needs: *"the simulator differs from the robot by less
than the robot differs from itself, on N of 11 statistics."* Measured
2026-08-17 — §5f.

**First grounding result (2026-08-16).** *(Tracker-frame losses — pre-§5f
referee; §5f re-checks this ordering under the IMU referee.)* The two
disagreeing scorers were put to the referee: the real freewalk net
closed-loop in `measured`, `accel`, and a re-derived phase-4 fit point,
under one shared sim-perturb floor.

| grounding loss (RMS SNR)   | fit rec (hard floor) | held-out span (rubber) |
|----------------------------|----------------------|------------------------|
| `measured`                 | **7.50**             | **4.81**               |
| phase-4 fit (re-derived)   | 7.94                 | 5.38                   |
| `accel`                    | 8.02                 | 5.16                   |

The grounding prefers `measured` on BOTH recordings — the go2sim-style
scorer's answer, against the frozen scorer's 11-18% preference for `accel`.
Margins are ~7%, and 0 of 10 statistics sit within the chaos floor (the sim
is measurably different from the robot on every one — dominated by `yaw_lag`
on the fit recording, which no plant knob can close because the command
path's delay is outside the plant). The outer study (`sysid.meta outer`) is
seeded with both scorer styles; its final number is owed to a third recording
it never reads.

---

## 5b. The over-damped verdict, probed: it is a missing mechanism
> **See 5e — the premise below did not survive.** The ~2x oscillation
> deficit is mostly the VR tracker's flexing mount, not the simulator.
> The negative results in this section still stand; the gap they were
> aimed at does not. Every `roll_std`/`pitch_std`/`tilt_p99` and every
> grounding loss quoted below is a TRACKER-frame number (pre-5f referee);
> §5f gives the IMU-frame recomputation.

After the lag-axis fix the referee's verdict left one coherent story: every
body-oscillation statistic ~2x too small and the speed low — the sim walks
SMOOTHER and slower than the robot (`roll_std` 0.023 vs 0.044, `tilt_p99`
0.131 vs 0.249, on the verified freewalk net, hard floor).

**`sysid.probe` is loop 2's identifiability instrument**: push each knob to
the ENDS of its admissible range (endpoints, not derivative nudges — chaos
makes small-step derivatives noise, and the endpoints bound what any
admissible value could ever do), roll the real policy, and judge each
statistic's movement against the chaos floor and against the sim-real gap.

```bash
python -m dimos.robot.unitree.go2.sim.sysid.probe REC.mcap NET.bin \
    --preset measured --workers 18
```

**Finding 1 — no knob in the table closes the oscillation gap.** Across all
14 knobs at both range ends, the only ones that move `roll_std`/`tilt_p99`
materially do so by breaking the robot: `armature` at 0.05 or 16+ ms of
`actuator_tau` produce flailing (grounding loss 5.16 → 6.6-88). Everything
else moves the oscillation statistics by under ~35% of the gap, mostly
under the chaos floor. A fit over this table can NEVER close the verdict.

**Finding 2 — the missing mechanisms are in the loop, not the plant.** The
ideal closed loop lacks three things the real one has, now implemented
default-off in `rollout_policy` (every existing number reproduces
bit-for-bit):

* `action_latency` — the policy's target reaches the PD instantly; the
  real loop has obs transport + inference + command transport + board
  application. Delay eats phase margin, so LESS delay = LESS oscillation.
* `obs_noise` — the policy sees perfect state; the real one sees sensor
  noise, and the net was TRAINED with noise (`ObsNoise` defaults are the
  legged_gym training levels), so a noiseless loop is out-of-distribution
  smooth.
* the measured torque `envelope` (§plant) — the sim's actuators deliver
  full demand at swing speeds where the real drive delivers ~half.

Measured on the grounding, seed-replicated (3 seeds, shared base floor):
latency 10 ms + training-level noise takes the loss **5.16 → 2.79 ± 0.13**
on the fit recording — `roll_std` closes 33% of its gap, `pitch_std` ~100%,
`tilt_p99` 75%, `speed` 81%. The latency response is monotone and steep:
each oscillation statistic crosses its real value at 10-14 ms, and 20 ms
destabilises the gait entirely. On the rubber-floor held-out span the same
configs move every one of the five statistics toward the real robot (loss
5.49 → 4.31 at 12 ms), closing less of the gap — the remainder there is
plausibly the unmodelled 20 mm mat and leg flex.

**The envelope double-counts.** Alone it helps (loss 4.34); stacked on
latency+noise it overshoots (7.9). The `measured` knobs were fitted open
loop WITHOUT the envelope, so its average effect is already absorbed in
them — re-fit loop 1 with the envelope on before stacking it.

**The latency's value is plausible, not yet measured.** A sweep of
`verify_net --state-offset` puts the executor's software latency at ~0-2 ms
(obs built from the freshest state, inference fast) — but the ingest clock
rebase absorbs both one-way transports, so the network legs are invisible
to that instrument. Physical expectation for two WiFi legs + board cycle is
5-25 ms; the frozen instrument's closed-loop fit found 9 ms independently.
10 ms is therefore a NAMED MECHANISM with a plausible value, not a fitted
constant — the default stays 0, and pinning the number wants a measurement
(e.g. a hardware timestamp echo), not a fit on the referee.

Caveat for probe readers, since fixed: the `gait_hz` estimator was bimodal
under some probes — the "first local maximum" rule could lock onto a noise
ripple near the step harmonic (measured: a −0.010 ripple at 3.85 Hz chosen
over a +0.171 stride peak at 1.72 Hz). The estimator now yields to a peak
stronger by `GAIT_PEAK_MARGIN`; the fix fires NOWHERE on the canonical
default grounding (real side, base and all four floor rollouts identical
before and after), so every default-path number stands. Cells where the sim
genuinely doubles its bob can still read high, with the replicate spread
saying so.

---

## 5c. The mechanisms, measured — and the proxy verdict
> **See 5e — the premise below did not survive.** The ~2x oscillation
> deficit is mostly the VR tracker's flexing mount, not the simulator.
> The negative results in this section still stand; the gap they were
> aimed at does not. Every `roll_std`/`pitch_std`/`tilt_p99` and every
> grounding loss quoted below is a TRACKER-frame number (pre-5f referee);
> §5f gives the IMU-frame recomputation.

§5b earned its mechanisms a fitted value each; this section replaces the
fits with MEASUREMENTS (`sysid.loop`), and reports honestly how much of the
gap survives. Spoiler: most of it.

**The loop, measured leg by leg** (2026-08-16 freewalk session):

* **Command transport = 1.34 ms** (p10-p90 1.29-1.48): the recordings carry
  the same command on `policy/lowcmd` and `rt/lowcmd`; matching all 3539
  messages by exact 12-float payload times the leg directly
  (`sysid.loop.transport_leg`).
* **Target->plant leg ≈ 0**: shifting the recorded command timeline and
  replaying Mode A is a latency knob scored without any referee
  (`sysid.loop.command_shift_sweep`) — sharply resolved (44% depth over
  ±30 ms), optimum −5..+2 ms, +10 ms is 9-12% worse.
* **Sensor noise is 2-3x BELOW the training levels**: the >20 Hz residual
  (`sysid.loop.sensor_noise`) measures dq 0.29 rad/s RMS (×0.33 of
  `ObsNoise`'s ±1.5), gyro 0.073 (×0.63), q 0.0032 (×0.55), gravity ~0
  (×0.01 — the attitude filter smooths it clean).
* **The '50 Hz' executor actually runs at 44 Hz**: measured inter-command
  intervals (`sysid.loop.control_timing`) have median 22.2-22.3 ms on ALL
  THREE policy recordings, σ 3.5 ms, plus a dropout tail (71 intervals
  >30 ms on the freewalk file; 8% of intervals on the rubber sessions, max
  ~200 ms). Zero free parameters; replayed as a sequence by
  `rollout_policy(control_intervals=...)`.

Adding the leg bounds: 1.3 ms of command transport + a state side bounded by
the ~3.9 ms lowstate batch spacing + ~0-2 ms executor software =
`LATENCY_BAND_S` **(1.3, 6) ms**. The fitted 10-14 ms does not fit inside it.

**The measured-mechanism table** (`sysid.meta mechanisms`, 3 replicates,
shared base floor; fit recording, hard floor / held-out rubber freewalk
span):

| grounding loss              | fit rec        | held-out span  |
|-----------------------------|----------------|----------------|
| ideal loop                  | 5.24 ± .17     | 4.14 ± .23     |
| FITTED lat 10 ms + noise ×1 | **2.82 ± .18** | **3.54 ± .08** |
| latency at measured band    | 5.24-6.82      | 4.09-4.28      |
| noise = measured            | 5.28 ± .23     | 4.18 ± .06     |
| timing = measured           | 5.57 ± .23     | 4.81 ± .26     |
| timing+noise+lat 6 ms       | 4.74 ± .14     | 3.76 ± .60     |

Every mechanism at its measured value, alone or stacked, closes at most
**~10% of the loss** on either recording; the fitted point closes ~46%.
Two informative surprises: the measured sensor noise is too small to excite
any body motion at all, and the measured timing makes the sim SLOWER
(speed 0.483→0.439 vs real 0.571 on the fit recording) — the real robot
walks faster than the sim while ticking its policy slower, so the 44 Hz
clock deepens the speed deficit rather than explaining it, and neither
jitter nor dropouts are the missing oscillation mechanism.

**The proxy question, settled** (`sysid.meta latency` — `action_latency` as
a bounded loop-2 knob, selected on the fit recording, reported on the
held-out span; the select/report split is what makes tuning against the
grounding legitimate):

* With every measured mechanism ON, the unbounded search still wants
  **12 ms** (select loss 2.83; 16+ ms destabilises). Bounded to the measured
  band it tops out at 6 ms / 4.74. On the held-out span the preference
  TRANSFERS: 0 ms → 4.20, 6 ms → 3.76, 12 ms → 3.23.
* Bare (no other mechanisms) the landscape is noisy (10 ms: ±4.01) and the
  held-out gain of its 8 ms pick is marginal (4.14 → 4.02).

**Verdict: `action_latency` at 10-14 ms is a PROXY.** The loop cannot
physically contain it — every leg is measured or bounded, and their sum is
under 6 ms — yet the referee wants twice that, and the preference survives a
held-out split. So the delay stands in for a real missing mechanism that is
NOT a transport delay (candidates: actuator/drive dynamics beyond the
first-order `actuator_tau`, gear backlash, foot-contact compliance — things
that add phase lag without being a clock). The mechanisms stay default-off;
nothing here earned a default promotion, and the honest headline is that
~90% of the §5b gap survives measured parameterisation.

---

## 5d. The drive, measured directly — two suspects out, the proxy survives
> **See 5e — the premise below did not survive.** The ~2x oscillation
> deficit is mostly the VR tracker's flexing mount, not the simulator.
> The negative results in this section still stand; the gap they were
> aimed at does not. Every `roll_std`/`pitch_std`/`tilt_p99` and every
> grounding loss quoted below is a TRACKER-frame number (pre-5f referee);
> §5f gives the IMU-frame recomputation.

§5c left three candidates. `sysid.drive` interrogates the first two with
ZERO free parameters and no simulator in the loop: the recorded commands
plus the board's own `q`/`dq` fix the torque the PD law DEMANDED at every
500 Hz sample, and `tau_est` is what the drive DELIVERED — the transfer
between them IS the drive, measured on the real robot.

    python -m dimos.robot.unitree.go2.sim.sysid.drive REC.mcap

**Drive dynamics beyond first order: OUT.** Free second-order fits land at
35-60 Hz with damping ~0.8-1.1 — mimicking a lag, never a peak; |H| exceeds
1 nowhere in band, so the drive cannot inject energy at gait frequency. The
equivalent first-order lag is 2-9 ms (walking: hips 4.9 / thighs 7.4 / calf
1.8 ms), the same order as the fitted `actuator_tau`. The sensor is not
late either: ddq — which never touches `tau_est` — correlates best with the
demand AND with `tau_est` at ~0 ms on the suspended file.

**Backlash / deadband: OUT.** At |dq| < 3 rad/s the delivered/demanded gain
is 0.94-1.03 in EVERY amplitude bin down to 0-0.5 N·m; every low-gain cell
is |dq| ≥ 3, which is the torque envelope, already a named mechanism.

**What the drive DOES show is the envelope acting in-band** — thigh |H|
dips to ~0.83 over 3-16 Hz, exactly where swing passes 3 rad/s — and §5b
had already warned the `measured` knobs were fitted WITHOUT it, absorbing
that deficit into the viscous/inertial knobs (armature 0.029 = 6x the
accel-fit spread; Mode A shows the sim OVER-driving real tau 1.1-1.35x at
gait frequencies while walking slower and smoother). So §5b's prescription
was finally run: `fit --envelope central`, same objective and procedure as
the phase-4 refit, the envelope name recorded ON the preset so the plant
can never silently run bare (`results/fit6-freewalk-env`). The knobs
deflate exactly as the story predicts — armature 0.029 → 0.0048,
actuator_tau 5.25 → 1.2 ms, frictionloss 1.47 → 1.30 — the open-loop score
improves 20.3% over the `measured` baseline, and held-out jumps improve
2.7%. As an open-loop identification, the envelope-consistent plant is the
better-identified one.

**The bigger finding is the ANTI-TRANSFER.** The envelope-refit plant is
better on the loop-1 objective (−20.3%, held-out jumps −2.7%) and WORSE on
the referee everywhere: 6.16 vs 5.47 at 0 ms on the select window, 7.68 vs
6.37 on the held-out reference, best-case 4.13 vs 2.83. An open-loop
improvement that anti-transfers to closed loop is a result about the
METHOD, not about this plant: the `accel/floor` objective is not a reliable
proxy for closed-loop fidelity — the same disagreement the referee already
showed over fit5 (§5a), now reproduced under an envelope-consistent fit.
That bears on the two-loop design itself and is what the outer study has to
adjudicate.

**The decisive test answers NO — the demand does not shrink.** The §5c
select/report latency split, identical windows (select on the fit
recording, report on the rubber freewalk span t=68.3-83.2), measured
timing+noise on:

| plant                          | unbounded pick | select at pick | report 0 ms → pick |
|--------------------------------|----------------|----------------|--------------------|
| `measured` (no envelope)       | 12 ms          | 2.83 ± .24     | 6.37 → 6.20        |
| `fit6-env` (env central)       | **16 ms**      | 4.13 ± .12     | 7.68 → 6.34        |
| fit6 knobs, env OFF (control)  | **20 ms**      | 3.06 ± .21     | 8.77 → 6.60        |

The demand ROSE, and the control run turns the three rows into one
statement: the unbounded pick sits just under each plant's own stability
cliff. `measured` loses the gait past 12 ms (30.9 ± 33.2 at 16, 57.4 at
20); the envelope-refit is stable at 16 (± 0.12) and dies at 20; the
control is still stable AT 20 (± 0.21) — and each time the referee buys
delay right up to the edge. The "latency demand" is not a constant of the
loop; it is each plant's MAXIMUM SURVIVABLE DELAY, which is what finally
makes it unmistakable as a proxy: a plant that tolerates more delay needs
more delay to fake the same oscillation, i.e. it walks even more
over-damped — the opposite of what deflating
`armature`/`actuator_tau`/`leg_mass_scale` was supposed to buy. The
envelope itself is not the closed-loop culprit either way (confound
control, 0 ms: ON helps the rubber window 8.77 → 7.68, marginally hurts the
hard one 5.73 → 6.16); the regression vs `measured` sits mostly in the
refit knobs.

**Verdict: a confident negative.** The drive is first-order to within
measurement error, backlash is absent, the envelope-consistent refit is
implemented and grounded — and the ~12-16 ms latency demand survives all of
it. Of §5c's candidates, foot-contact compliance was already bounded by the
§5b knob-endpoint probe (all four contact knobs, under ~35% of the gap), so
the proxy now points OUTSIDE everything this plant parameterises. Nothing
here earns a default promotion: the mechanisms stay off, `measured` stays
the default plant, and the envelope-refit plant ships as a results artifact
with its spread.

**The surviving named candidate is a HYPOTHESIS, not a measurement:**
series compliance between the motor-side sensors and the body — leg-link /
belt flex. It is motivated by this instrument's blind spot (`q`, `dq`,
`tau_est` all live motor-side, so a compliance after the gear is
structurally invisible to `sysid.drive`), it adds exactly the loop phase
lag the referee keeps buying as latency, and no rigid-body knob expresses
it. It has a test that needs no new recording: motor-side `q` through rigid
kinematics predicts a trunk pose, the tracker measures the actual one, and
the discrepancy should GROW WITH LOAD — deflection against measured torque
is a series stiffness with zero free parameters. Not implemented here;
scoped separately.

---

## 5e. RETRACTION: the premise of 5b–5d was an instrument artifact

**The ~2x oscillation deficit that 5b, 5c and 5d were chasing is mostly the
VR tracker's mount, not the simulator.** Measured 2026-08-17. The three
investigations' *negative* results stand — they are about knobs, latency and
the drive, and none of them depended on the gap being real — but their shared
premise did not survive, and no conclusion should be drawn from the gap
itself.

Loop 2 takes `roll_std`/`pitch_std`/`tilt_p99` from the tracker
(`ground.real_summary` → `Streams.base_pose_room`). Against the **raw gyro** —
a direct body-rate measurement no attitude filter has touched, so it can
arbitrate between two fused estimates — the tracker is wrong:

| | raw gyro | tracker | ratio | corr |
|---|---|---|---|---|
| roll rate | 0.389 | 0.962 | 2.47x | **0.44** |
| pitch rate | 0.321 | 0.608 | 1.90x | 0.52 |
| yaw rate | 1.022 | 1.378 | 1.35x | 0.73 |

The correlation matters more than the ratio: a rigid mount correlates 0.95+.

**It is mount FLEX, not tracker noise.** Binned by how hard the robot is
moving (both sides low-passed at 15 Hz — an unfiltered comparison amplifies
the tracker's differentiation noise and misleadingly looks flat), the excess
rotation the tracker invents grows **34x** with activity: 0.033 rad/s in the
quietest bin, 1.133 in the loudest. Noise would be constant; flex is excited.
So the tracker is accurate when the robot is calm and fails exactly in the
regime being scored. Tightening the mount screws did not fix it.

**Calibration cannot rescue it.** A full 3-DOF mount rotation plus room-frame
tilt, fitted against the IMU, moved `roll_std` from 0.0415 to 0.0415 — a
*constant* transform cannot remove a *time-varying* relationship. Nor can the
translation offset: where the tracker sits does not affect which way it
points. (`TRACKER_Z = 0.207` is a documented guess against a true ~0.15 m,
but sweeping it 0 → 0.25 m moves `speed` by 0.0009 m/s — harmless, because
the velocity estimator smooths.)

**The gyro is trustworthy, three ways.** (a) It reads 0.0103 rad/s RMS with
the robot still — a **38x** SNR against the 0.389 rad/s walking roll rate.
(b) The Go2's own attitude filter, a separate output path, reproduces the raw
gyro at correlation **0.992–1.000**, ratio 0.96–1.00, so the quaternion is not
smoothing oscillation away. (c) Accel reads 9.591 m/s² static vs 9.81 — a 2.2%
scale error, irrelevant to rates but worth knowing if accelerations are ever
scored. The contrast that makes this intuitive: 0.0103 rad/s integrated for
60 s is ~0.6 rad of drift — useless for dead reckoning, ample for an AC
measurement in a band.

**By the gyro the simulator was already right about oscillation**: sim
`roll_std` 0.0200 vs IMU 0.0203, `pitch_std` 0.0180 vs 0.0206, `tilt_p99`
0.1240 vs 0.1322. Which also explains 5b cleanly — no knob closed the gap
because there was no physical gap to close, and the referee's latency demand
tracked each plant's stability cliff (5c/5d) because delay was the only way to
manufacture oscillation that was never missing.

**What survives:** the **speed deficit** (0.571 vs 0.484) is position-derived,
and mast flex at 0.15 m contributes millimetres — it is real and unexplained.
And 5d's **anti-transfer** result is untouched: it compares plants against each
other, not against the tracker's attitude.

**Consequence for loop 2:** split by what each instrument is actually good at —
**tracker for position** (trajectory, speed, height), **IMU for attitude**
(roll, pitch, tilt, yaw rate). Implemented in §5f: ``sysid.real.real_summary``
reads attitude from the IMU quaternion (which reproduces the raw gyro at corr
0.992–1.000, so quaternion-vs-integrated-gyro is a readability choice, not a
numerical one), and every ``Summary`` carries an instrument-provenance
``source`` field so no claim silently changes instrument. Every oscillation
number in 5b–5d is a TRACKER-frame number; §5f gives the old-vs-new pairs.

---

## 5f. The instrument split, built — and the first robot-repeat floor

Two things landed together on 2026-08-17: loop 2 now reads **position from
the tracker and attitude from the IMU** (`sysid.real.real_summary`), and the
three same-session freewalk recordings gave the project its first
**robot-repeat noise floor**. All numbers below are from the `measured`
plant on the verified freewalk net.

**The split as built.** The real side of loop 2 moved to `sysid.real` —
pure recording processing, no engine import, deliberately outside the
MuJoCo-coupled `ground.py`. `summarize` takes a separate attitude timeline,
so the two instrument families ride their own clocks; attitude comes from
the IMU quaternion (`Streams.lquat`, 500 Hz). Every `Summary` carries a
`source` field (`pos:tracker att:imu`, `sim`, …) that the grounding table
prints — no claim silently changes instrument. `attitude="tracker"` keeps
the retracted instrument for diagnosis and for reproducing pre-split
numbers. A tracker-less recording now scores its attitude statistics
(position statistics are NaN and drop out of the SNR) instead of raising —
the IMU needs no tracker. And `ground --noise-from` no longer folds the
grounded recording into the robot floor: the floor and the verdict must be
measured on different data.

**Old vs new referee, from IDENTICAL rollouts** (the sim side and its
chaos floor are computed once; only the real side's instrument changes).
Fit recording 194142, ideal loop, full span:

| statistic   | sim   | real (tracker) | real (IMU) | SNR trk | SNR imu |
|-------------|-------|----------------|------------|---------|---------|
| `roll_std`  | 0.023 | 0.044          | **0.020**  | 9.6     | 2.3     |
| `pitch_std` | 0.020 | 0.031          | **0.021**  | 6.8     | 0.7     |
| `tilt_p99`  | 0.131 | 0.249          | **0.133**  | 9.5     | 0.2     |
| `speed`     | 0.486 | 0.571          | 0.571      | 3.0     | 3.0     |
| loss        |       | **5.16**       | **2.06**   |         |         |

The 5.16 reproduces §5b's headline exactly; under the IMU referee it is
2.06. **§5e's prediction lands**: the oscillation gap closes to within 15%
(`roll_std` +15%, `pitch_std` −5%, `tilt_p99` −1.5% — the last two inside
even the chaos floor). What remains is the position family: `speed` (the
known deficit), `speed_gain`, `speed_lag`, `gait_hz`, `height_std` — all
tracker-derived, all untouched by the split. On the rubber held-out span
the same recomputation gives loss 7.31 → 6.75 (roll SNR 6.4 → 2.1, pitch
6.8 → 0.7, tilt 3.2 → 1.7), with the residual dominated by the
position-derived `gait_hz`/`height_std` on that 15 s window.

**The fitted mechanisms, re-judged — the flip is total.** Replicated under
§5c's own conventions (3 seeds, shared base floor; the tracker rows
reproduce §5c's table exactly):

| loss, fit recording  | tracker referee | IMU referee    |
|----------------------|-----------------|----------------|
| ideal loop           | 5.24 ± 0.17     | **2.15 ± 0.10** |
| FITTED lat 10 ms + noise ×1 | **2.82 ± 0.18** | 6.33 ± 0.27 |

The old referee's star mechanism is the new referee's worst config: it
OVERSHOOTS the oscillation that was never missing — `roll_std` 0.029 vs
real 0.020, `pitch_std` 0.030 vs 0.021, `tilt_p99` 0.214 vs 0.133 — and
what it still buys on the speed axis (`speed` 0.486 → 0.550 against real
0.571) cannot pay for that. §5b/§5c's oscillation-closure percentages were
tracker-frame artifacts end to end; the mechanisms stay default-off, now
for a POSITIVE reason rather than a bookkeeping one. (Under a floor
measured with the mechanisms applied — `ground()`'s per-config convention —
the config merely ties the ideal loop, 1.86 vs 2.06 on single rollouts: a
mechanism-matched floor is wider and forgives the overshoot. The shared
base floor is the comparable convention, as in §5c.)

**Plant orderings survive the referee change.** All five plants (`measured`,
`accel`, fit5, fit6-env, fit6-noenv-control), one shared floor per
recording, both referees: on the fit recording `measured` stays first under
both (5.16 → 2.06; the accel/fit5 midfield reshuffles within single-rollout
noise), and on the rubber ideal-loop window the order is IDENTICAL under
both (fit6-control < fit6-env < measured). So §5a's scorer verdict
(`measured` survives into the outer study's seed) and §5d's between-plant
comparisons stand — the instrument swap rescales the losses, it does not
reorder the plants. What flips is only what §5b/§5c built ON the absolute
oscillation gap.

**The 2026-08-17 session is usable for the floor, not for Mode B.** The
three recordings (`153320`/`153558`/`154201`, hard floor, tracker, back to
back) were driven by a NEW executor, and not just a renamed schema: no
`policy/lowcmd` at all (commands only on the 500 Hz `rt/lowcmd` bus — the
same logging regression as the stale `policy/state: sport` label), a true
20.0 ms tick where the 08-16 executor measured 22.3 ms, and an
output-smoothing stage: teacher-forced `verify_net` on deduplicated bus
ticks explains the recordings at ratio only 0.53 (vs 0.123 on 194142),
per-joint correlation 0.78–0.97 at sub-unity gain with ~1-tick lag, and
inverting a fitted EMA (α ≈ 0.3–0.5) halves the residual to 0.29 — same
net, filtered actions. (The freewalk net still beats the v11 control
decisively, 0.53 vs 0.74, gains kp 40 / kd 1.0 match — the `sport` label is
wrong, exactly as suspected.) Mode B simulates the 08-16 loop semantics, so
grounding an 08-17 recording would confound plant error with executor-model
mismatch; the REAL side needs no net and no executor model, so the floor
stands. The session difference is visible in the command-conditioned
statistics: `speed_gain` 0.49–0.54 on 08-17 (wider envelope, vx ±1.5,
saturating) vs 0.88 on 194142, `yaw_lag` 0.04–0.06 vs 0.25.

**The robot-repeat floor, measured** (start = 6 s, each file's full span,
`pos:tracker att:imu`):

| statistic       | 153320 | 153558 | 154201 | 3-way spread | chaos floor |
|-----------------|--------|--------|--------|--------------|-------------|
| `roll_std`      | 0.017  | 0.022  | 0.023  | **0.005**    | 0.001       |
| `pitch_std`     | 0.017  | 0.022  | 0.019  | **0.004**    | 0.001       |
| `tilt_p99`      | 0.118  | 0.141  | 0.141  | **0.023**    | 0.008       |
| `height_std`    | 0.007  | 0.008  | 0.008  | **0.002**    | 0.000       |
| `gait_hz`       | 1.020  | 1.282  | 1.250  | **0.262**    | 0.143       |
| `speed`         | 0.558  | 0.606  | 0.580  | **0.049**    | 0.029       |
| `speed_gain`    | 0.542  | 0.522  | 0.494  | **0.047**    | 0.044       |
| `yaw_rate_gain` | 0.801  | 0.826  | 0.834  | **0.033**    | 0.032       |
| `speed_lag`     | 0.120  | 0.110  | 0.110  | **0.010**    | 0.010       |
| `yaw_lag`       | 0.060  | 0.050  | 0.040  | **0.020**    | 0.013       |

The robot's own variability is 2–5x the sim-perturb chaos floor on the
attitude family and height (the command/lag family roughly ties) — so the
chaos floor was a HARSHER yardstick than the robot itself, never a more
lenient one. The verdict floor uses only
`153320`+`153558` (a 2-sample range — if anything too NARROW, §4a), keeping
`154201` entirely out of every selection.

**The claim, tested.** 194142, `measured` plant, ideal loop, robot-repeat
floor: *the simulator differs from the robot by less than the robot differs
from itself on **6 of 10** statistics* — loss 1.30, vs 2.06 on the chaos
floor.

| holds (SNR)                                              | fails (SNR)          |
|----------------------------------------------------------|----------------------|
| `yaw_lag` 0.0, `tilt_p99` 0.1, `pitch_std` 0.2,          | `gait_hz` 1.3x,      |
| `yaw_rate_gain` 0.2, `roll_std` 0.5, `height_std` 0.7    | `speed` 1.7x,        |
|                                                          | `speed_lag` 2.0x,    |
|                                                          | `speed_gain` 2.7x    |

Every failure is the speed axis — the one deficit §5e already said was real
and unexplained. Caveats, stated rather than buried: the floor is
CROSS-SESSION (08-17 repeats judging an 08-16 recording; a same-session
repeat of the verdict walk does not exist), and for the command-conditioned
gains the "same walk" premise is weak — the 08-17 envelope is wider, so
`speed_gain`'s floor may not transfer. The attitude half of the verdict is
robust to both caveats: those spreads are tight, session-stable, and the
sim sits inside them.

---

## 5g. The speed family, decomposed — understriding, and the envelope's half

§5f left one failure family: `speed`, `speed_gain`, `speed_lag`, `gait_hz` —
the sim delivers ~15-18% less speed per unit command. Measured 2026-08-17,
all on the `measured` plant and the verified freewalk net.

**The instrument** (`sysid.gait`): `speed = stride length x stride
frequency`, measured per leg — foot positions from rigid FK on the joint
angles (validated bit-exact against MuJoCo), touchdown events from the
gravity-aligned foot height, stride length as body planar travel between a
leg's consecutive touchdowns. Engine-free, one code path for both sides:
the recording contributes `lq` + tracker travel, the rollout contributes
the new `PolicyRun.q` + `run.pos`. First finding: **`gait_hz` does not
measure cadence** — it reads 1.33 (sim) vs 1.67 (real) while the legs of
BOTH cycle at 1.93/1.96 Hz — so its §5f "failure" was an artifact of the
bob autocorrelation, not a physical mismatch. The stride statistics replace
it as the cadence claim.

**The decomposition: understriding, not understepping** (194142, ideal
loop; robot-repeat floor from `153320`+`153558` as §5f):

| | sim | real | floor | SNR |
|---|---|---|---|---|
| `stride_hz` | 1.925 | 1.957 | 0.306 | **0.1** |
| `stride_len` | 0.299 | 0.332 | 0.030 | 1.1 |

The gait cycles at the robot's own rate to within 1.6% and covers 10% less
ground per cycle. Localising further, with the same instrument on both
sides: the policy COMMANDS the same steps (AP foot excursion of the
commanded joints 0.222 sim vs 0.216 real), the plant TRACKS them (achieved
0.215 vs 0.207), and the sim's feet are not sliding away the difference —
true world-frame slip is 12-14 mm per stance, ~4% of the stride. What does
differ: the sim swings 38% higher (0.126 vs 0.091 m) and keeps its feet in
ground dwell a smaller fraction of the cycle (0.47 vs 0.58) — the leg
motion budget matches, but less of it is spent propelling.

**Friction is exonerated, at 3x the declared range.** Sweeping
`foot_friction` 0.6 → 1.6 (deliberately past the knob's 1.0 ceiling,
torsional friction re-derived at each μ) moves closed-loop speed by
**0.4%**, non-monotone, inside noise — against an 18% deficit. Nobody
needs to sweep it again. The provenance inconsistency found on the way is
recorded on the knob: `measured` ships `foot_friction = 0.635`, BELOW
`DR_FLOOR`'s declared (0.8, 1.0) and the 0.90 the fit pins — inert for
every statistic (this sweep), left as shipped.

**The re-probe, against the corrected objective.** §5b's spectrum judged
knobs on oscillation statistics the tracker had corrupted; re-judged on
the speed family (now including the stride pair) under the IMU referee and
the robot-repeat floor, one rollout per candidate (chaos on these
statistics sits 4-15x below the robot floor — except `speed_lag`, whose
10 ms lag-grid quantum IS both floors, so its SNR moves in whole steps):

| candidate | speed | gain | lag | stride_len | spd RMS | att RMS | ≤floor |
|---|---|---|---|---|---|---|---|
| base `measured` | 0.486 | 0.759 | 0.080 | 0.299 | 1.78 | 0.36 | 7/11 |
| `frictionloss=0.3` | 0.521 | 0.805 | 0.080 | 0.318 | 1.28 | 0.65 | 8/11 |
| `frictionloss=0.1` (lo bound) | 0.527 | 0.813 | 0.080 | 0.326 | 1.21 | 0.80 | 8/11 |
| `actuator_tau=0.016` | 0.527 | 0.822 | 0.090 | 0.298 | 0.98 | **1.15** | 5/11 |
| **`envelope=central`** | 0.518 | 0.810 | 0.090 | 0.316 | **1.04** | 0.42 | 8/11 |
| `fl=0.3` + envelope | 0.536 | 0.829 | 0.090 | 0.325 | 0.80 | **0.96** | 7/11 |
| `armature=0.006` | 0.471 | 0.733 | 0.070 | 0.285 | 2.32 | 0.78 | 6/11 |
| `accel` preset | 0.484 | 0.755 | 0.080 | 0.324 | 1.77 | 0.62 | 7/11 |
| `fit6-env` preset | 0.487 | 0.759 | 0.070 | 0.305 | 2.03 | 0.64 | 7/11 |

(real: 0.571 / 0.880 / 0.100 / 0.332)

Three verdicts. **The sluggish-plant hypothesis is refuted**: every
inertia/dissipation deflation — `armature` to the accel-fit 0.006,
`leg_mass_scale` to 0.7, the accel and fit6 presets wholesale — makes the
speed family WORSE (lighter legs stride shorter, 0.268-0.287 m).
**`frictionloss` helps monotonically but cannot finish**: at its range
floor it still leaves `speed_gain` at 1.5x the robot floor, and stacked on
the envelope it buys the rest by breaking pitch/roll (1.6x / 1.2x) — a fix
that reintroduces an oscillation error is not a fix, and `actuator_tau` at
16 ms is the same trade (att 1.15, §5d's buy-delay proxy again). **The
measured torque envelope closes half the family attitude-free**: loss
1.30 → 0.87 on the §5f statistic set, every speed statistic moved toward
the robot, attitude family untouched.

**The held-out check transfers.** On the rubber freewalk span
(t=68.3-83.2, no part in selection), the same base plant shows the same
−18% speed deficit, and `envelope=central` closes 55% of it (0.365 →
0.409 vs real 0.445), 61% of `speed_gain`, `speed_lag` to exact, with
attitude unchanged. The §5d select/report discipline, passed.

**The referee, upgraded (statistic set v2).** The findings above changed
the instrument, so the change is recorded here rather than applied
silently. Scored set: the stride pair joins (from `sysid.gait`, filled by
`real_summary` and `sim_summary` through the same FK instrument), and
`gait_hz` retires to `NOT_COMPARABLE` — kept in every table, never scored.
It is the SECOND statistic retired for measuring its own instrument
(`height_mean` was the first: its value is the room-frame calibration, not
the robot), and the pattern deserves a name: a statistic is untrustworthy
when perturbing its estimator's nuisance parameters (a band edge, a peak
margin, a frame offset) moves it by more than the claim's floor. A cheap
standing guard would compute each statistic under nudged nuisance
parameters and flag any that move past the robot floor — a third floor,
beside chaos and repeatability; proposed, not built. Old and new, same
rollouts, robot-repeat floor, 194142:

| plant | v1 referee (10 scored, incl `gait_hz`) | v2 referee (11 scored, stride pair, no `gait_hz`) |
|---|---|---|
| `measured` | 6/10, loss 1.30 | 7/11, loss 1.23 |
| `measured-env` | 6/10, loss 0.87 | **8/11, loss 0.77** |

The v2 failures on `measured` are `speed_gain` 2.7, `speed_lag` 2.0,
`speed` 1.7, `stride_len` 1.1 — the same one family, now with the
instrument-artifact member removed and the true cadence shown passing
(`stride_hz` 0.1). On `measured-env` what remains is `speed_gain` 1.6,
`speed` 1.1, and `speed_lag` on the 1.0 boundary. `measured-env` is now a
built-in preset (the measured plant + its measured envelope, one claim);
**the default plant is unchanged** — promotion is pending with the owner.

**Verdict: no admissible knob setting closes the speed family; the torque
envelope — a measured mechanism with zero free parameters — closes about
half of it, on both recordings, without touching attitude.** It is the
first mechanism to earn a positive closed-loop case under the corrected
referee (§5f rejected latency+noise for overshooting; the envelope was
never the overshoot's source). It stays default-off here — promoting it
belongs with a plant whose preset carries `envelope="central"` so the
claim travels with the physics (§ranges `Preset.envelope`), a decision for
the project owner, not this probe. The remaining ~8% speed deficit under
the envelope is the honest residual: a missing mechanism, with §5d's
series-compliance hypothesis (leg-link/belt flex, motor-side sensors blind
to it) still the named candidate — now also consistent with the sim's
high-swing/low-dwell signature, which no rigid-body knob reproduced.

---

## 6. State

**Built:** seam (knobs AND channels), MuJoCo backend, plant, ranges, anchors,
ingest, regimes, segments, Mode A replay, identifiability, the weight-vector
score, the fit (pins/searches, seeded restarts, paired-SE harvest, LOO-spread
stopping, median + spread), segment-parallel rollouts, loop 2 (`sysid.ground`:
closed-loop Mode B, the 11 statistics, pluggable noise floors, SNR reports),
net-identity verification (`sysid.verify_net`), the meta-search scaffolding
(`sysid.meta`, including the measured-mechanism table and the select/report
latency search — §5c), `--view` on both modes (ghost, `--speed`,
`--no-reinit`; the viewer and the headless run are the same function),
loop 2's identifiability probe (`sysid.probe`), the four default-off loop
mechanisms (`action_latency`, `ObsNoise`, `control_intervals`, the torque
envelope — §5b/§5c), the loop-measurement instruments (`sysid.loop`:
transport leg, control timing, sensor noise, command-shift sweep — every
mechanism's value is measured, not fitted), the drive instrument
(`sysid.drive`: demanded-vs-delivered transfer function, model
discrimination, deadband detector, ddq torque timing — §5d), and
envelope-consistent fitting (`fit --envelope`, the envelope recorded on the
preset and honoured by every downstream path), the §5e instrument split
(`sysid.real`: tracker for position, IMU for attitude, provenance on every
`Summary`, tracker-less recordings score attitude-only — §5f), the
robot-repeat noise floor, measured (§5f: three 08-17 recordings, verdict
floor from two with the third held out), and the stride instrument
(`sysid.gait`: engine-free FK strides, one code path for recording and
rollout, `PolicyRun.q` feeding the sim side — §5g). Acceptance is
bit-identical to the frozen instrument, and parallel is bit-identical to
serial.

**Not run yet:** the full outer study (10-20 trials × one inner fit each).

**Owed:** Mode B still simulates the 08-16 executor's loop semantics — the
08-17 session runs a NEW executor (true 20 ms tick, EMA-like action
smoothing, `policy/lowcmd`+`policy/state` logging dropped — §5f), so those
recordings serve the real side only until `rollout_policy` learns the new
loop and `verify_net` can pass on it. And the verdict floor is
cross-session: a same-session repeat pair of a VERDICT walk (record the
repeats with the recording you will ground) would remove §5f's two floor
caveats at once.

**Honest caveat:** none of this is validated by a policy transferring to
hardware. The value on offer is the method and the provenance, not an accuracy
claim — and §5e is the sharpest illustration of why that distinction matters:
three careful investigations, all sound, all aimed at a gap that was mostly a
loose instrument. The referee is only ever as good as the sensor behind it.
