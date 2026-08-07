# Go2 sim-to-real: where we are

Goal: make MuJoCo behave like the real Go2 well enough to train against.

**Status: matched, base AND legs, on BOTH recordings.** The same HIMLoco
policies that ran on the robot run in sim, driven by the recorded commands,
scored against VR-tracker ground truth *and* the executor's own commanded
joint targets (`policy/lowcmd`). `FITTED_*` in `evaluate.py` is the **joint
two-recording fit** — 300 CMA-ES trials scored on himloco01 and v11 at once,
seeded with the himloco-only preset so it had to beat it:

```
armature 0.00712   damping 0.2850   frictionloss 0.3650
foot_friction 0.7860   foot_friction_torsional 0.00613
trunk_mass_scale 0.9412   trunk_inertia_scale 0.8601
trunk_com_x -0.00685   leg_mass_scale 1.616
command_delay 0.00898   actuator_tau 0.01510
```

Joint loss **3.44 → 1.52**, reproduced at 2.80 → 1.44 on fresh 6-seed floors.
Commanded front foot lift 0.071 vs 0.066 m real on himloco01 and 0.112 vs
0.115 on v11 — the base-only fit high-stepped at 0.217 and looked like
prancing. **Ivan has confirmed it in the viewer on both recordings.**

A hard lesson is baked in here: that earlier base-only fit reached "everything
sub-noise" while commanding 3x the real front foot lift. **A judge only
constrains what it can see.** (A `--ghost` bug that silently ran stock physics
under `--fitted` cost a round of false "almost falling" reports too.)

The **himloco-only** preset it replaced — com +4.4 cm forward, trunk 20%
heavier, legs 0.72x, delay 32 ms — read as physically coherent (that is where
the lidar and head sit) and was viewer-confirmed on himloco01. The joint fit
reverses that story: trunk near stock, legs 1.62x, delay 9 ms. **Two
recordings constrain these parameters where one did not**, and §D already
flagged this cluster as weakly identified — so the payload interpretation was
partly policy-style absorption, not measurement.

The judge now scores the **entire recording** (a 20 s window once certified a
config that was never scored past t=26), carries **tilt_p99** as a stability
tail (real: p99 8.6°, max 16.7°), and scores **seven leg statistics** from the
commanded joint streams (lift, thigh/calf spans per pair, command-space gait
frequency). A follow-up 300-trial full-run search found a point that trades a
little translation for gait/rotation/legs without dominating — both sit in
one flat basin — and exposed that the 4-seed noise floor varies run-to-run
(BLAS-order chaos), which caps single-recording resolution. Further precision
must come from the held-out v11 recording, not more trials here.

**Held-out validation (v11) — the mechanisms transfer, the texture is
partial.** The v11 net (46-obs, commandable body height as channel 45) ran
under the same judge on `unitree_v11_gait_height01` with the himloco01-fitted
config, untouched. On the clean-walking window the fitted physics beats stock
exactly where the fitted *mechanisms* live: speed 0.033 err vs 0.112 stock,
speed_gain 0.073 vs 0.221, yaw_rate_gain 0.002 vs 0.057, thigh_span_rear
0.006 vs 0.140 rad, tilt tail closer. Stock stays slightly better on the
mm/mrad oscillation texture (pitch_std 0.012 vs 0.017, roll_std, detrended
height_std) and the lag estimates — consistent with the flat-basin caveat:
those last-mile values absorbed himloco-specific style. The height channel is
verified end-to-end: commanded 0.31→0.10 m, the sim base drops 0.14 m where
the real tracker drops 0.11 m, and both follow the raise to 0.37 m.

v11-specific judge caveats: the policy is so strongly stabilizing that the
4-seed noise floor collapses (SNR/loss degenerate — hence `usable_floor`);
body-bob `gait_hz` reads the sway/height envelope, not the steps (real 1.0 Hz
vs 2.9 Hz in the joint commands — use `cmd_gait_hz`); `height_std` in the
height-play window is dominated by the commanded crouches. The last two are
declared per-recording in `search.INVALID_STATS` and dropped from the joint
objective. Real tilt_p99 is 0.32 rad (18°) — the robot genuinely nearly fell
during the 0.10 m crouch, and both configs underestimate that tail.

**What the joint fit bought** (`search.run_joint`), against the himloco-only
preset, as absolute error vs real: v11 `pitch_std` 0.020 → 0.004, `roll_std`
0.010 → 0.003, `speed_gain` 0.129 → 0.016; himloco `gait_hz` 0.146 → 0.026,
`thigh_span_rear` 0.118 → 0.019. Front lift improves on *both* recordings
(0.011 → 0.005, 0.006 → 0.003), so it does not re-open the prancing failure.
It gives up `thigh_span_front` (0.006 → 0.043 on himloco) and v11
`yaw_rate_gain` (0.001 → 0.073). That is the texture gap the held-out check
exposed, closed by the recording that exposed it.

---

## 1. What the recordings contain

`data/ml-trajectory-research/` — two runs with the networks that produced them.

| stream | rate | use |
|---|---|---|
| `control_log` | 48 Hz | the operator's command: `{"action":"walk","vx","vy","vyaw"}` |
| `vive_pose` | 253 Hz | ground-truth body pose (JSON: `p`, `q`, `t_host`) |
| `policy_state` | once | which policy — check it matches the `.bin` |

`control_log` also carries `{"action":"gait_height","gh"}` entries when the
operator moves the height slider against a net that listens (v11, obs 46 —
the raw height in metres rides as obs channel 45, the blob's own
ob_mean/ob_scale normalize it). The hardware clamps it to 0.1–0.4 m but does
*not* slew it, holds 0.31 m until first touch, and 45-channel policies never
see it — `walk.read_gait_height` + the `heights=` schedule mirror all of
that, so himloco scoring is unchanged.

**Joint-space data: commands yes, state no.** `policy/lowcmd` (~44 Hz) is the
executor's own log of the joint targets it sent — the real policy's output,
and the leg-space ground truth the `legs` objective scores against. What is
still missing is joint *state*: `rt/lowcmd` is zeroed, `lowstate.q` calf
angles are physical in 0.0% of rows, velocity and torque identically zero.
(An earlier pass read only the zeroed `rt/lowcmd` and concluded no joint data
existed at all.)

This also means the simulator's initial pose cannot be restored from a
recording — it always starts standing, so `--start 6` is needed to skip the
robot getting to its feet.

**Both streams share one epoch: the first walk command.** `t_host` and
`log_time` are the same clock (2.5 ms apart, no drift), but the vive stream
starts recording before the operator presses walk — 0.31 s earlier on
himloco01, 4.4 s on v11. Zeroing each stream at its own first message paired
every real pose with a command from its future, and a search then "fitted"
0.317 s of command delay, within 4 ms of the bookkeeping offset.

## 2. Calibration that is settled

**Target frame.** The official URDF root link is `base` (there is no
`base_link`), origin at the trunk's geometric centre, level with the hips.
menagerie inherits it one-to-one, so MuJoCo `qpos[0:3]` *is* that frame.

**Tracker mount**, fitted sim-free from the recording:

* quaternion is **wxyz**, frame is **z-up**
* tracker is mounted **inverted** — `R[2,2] = -0.997`
* robot forward is **+94°** in the tracker's xy plane — i.e. the mount is ~4°
  skewed from square, which is why this is fitted, not assumed. Two
  independent fits agree (93.6° circular mean; 94.0° by maximizing
  cos(velocity, command)).

**Not settled: the tracker's translation.** `--tracker-z 0.207` is a guess and
the in-plane offset is unmodelled. Its influence is now *contained* rather
than fixed: height is compared in sensor space and the orientation statistics
are immune to it (§3). A ruler measurement would end it.

## 3. How the judge works

**Distributional, because the gait is chaotic.** A 3° initial-pose
perturbation grows to 136 cm of position error in 12 s, non-monotonically; by
a 10 s horizon the sim-vs-real gap is smaller than the gap between two
identical simulators. Trajectory error carries no information about physics,
so `metrics.py` compares distributions — speed, gains, response lags, bob
amplitude and frequency, pitch/roll oscillation — and `evaluate.py` divides
each difference by its own noise floor from perturbed rollouts.

**Height in sensor space.** Inverting the guessed tracker offset on the real
data injected the guess into the ground truth: 11.4 mm of the "real" z std was
lever-arm swing against 5.6 mm of actual tracker bob. Instead the real side
keeps the raw tracker height and the sim mounts a *virtual tracker* with the
same guess — the guess distorts both sides identically and cancels.

**Orientation statistics.** `pitch_std` / `roll_std` (detrended, so a constant
mount or room-calibration tilt drops out) carry the gait's body oscillation
and owe nothing to the tracker translation. They are what pinned the sim
oscillating ~2× too fast and ~2.5× too hard before the physics fit — visible
by eye as the exaggerated leg lifts, and confirmed independently on z, pitch
and roll.

## 4. The three mechanisms

**Command slew — known constants, not a knob.** The robot rate-limits operator
commands per-axis before the policy sees them (go2web `policy.rs
ramp_velocity`): max 0.05 / 0.04 / 0.10 (vx/vy/vyaw) per 20 ms tick. The
recorded `control_log` carries the operator *target*; the policy on hardware
only ever saw the ramp. A yaw reversal ramps for 0.4 s where a speed nudge
ramps for a tenth of that — which is why the real yaw answers ~0.1 s later
than the real speed does, an axis-dependent lag no uniform delay could fit
(the pre-slew search traded translation 0.79 against rotation 3.03 and could
not have both). With the slew alone, default physics puts sim yaw_lag at
0.170 vs real 0.170.

**Actuator lag ≈ 15–30 ms.** A MuJoCo motor delivers requested torque on the
same step; a real BLDC through a gearbox does not. First-order lag on the
torque; every Pareto-optimal trial keeps it (searched from 0, so an ideal
actuator is free — none chose it). The joint fit puts it at 15 ms.

**Command delay ≈ 9–32 ms.** What genuinely remains of transport latency once
the epoch artifact is gone. The single-recording fit said 32 ms and the joint
fit 9 ms — it trades against actuator lag and trunk inertia, so treat the
sum of the three as the identified quantity, not any one of them.

**Corrected en route:** the menagerie feet are *not* frictionless — the
`condim="1"` default class only governs the calf capsules; the foot geoms
override with `priority="1" condim="6"` and a full friction cone. The open
question was the friction *values*; the search settled on much lower torsional
friction (0.003 vs the shipped 0.02).

## 5. How to run it

```bash
# score one configuration (~1.6 s)
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --eval

# watch it, with the recorded pose as a ghost box
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --view --ghost

# multi-objective search, Pareto front
python -m dimos.navigation.motion.simulation.search data/ml-trajectory-research/unitree_himloco01.mcap data/ml-trajectory-research/freewalk_mcf.bin --multi --trials 300

# the v11 gait-height run, same flow (crouches to 0.10 m around t=32)
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_v11_gait_height01.mcap --policy data/ml-trajectory-research/v11_final.bin --view --ghost --fitted --start 6

# joint fit across both recordings, starting from the current preset
python -m dimos.navigation.motion.simulation.search data/ml-trajectory-research/unitree_himloco01.mcap data/ml-trajectory-research/freewalk_mcf.bin --also data/ml-trajectory-research/unitree_v11_gait_height01.mcap data/ml-trajectory-research/v11_final.bin --seed-fitted --trials 300
```

---

# Next steps

## A. Validate on the held-out recording — DONE, then folded in

The v11 check did its job twice over. First as a *test*: the mechanism side
of the himloco-only fit (command response, leg geometry, stability tail)
transferred and beat stock, while the mm-level oscillation texture did not —
localizing the run-specific absorption to exactly the parameters §D flagged
as weakly identified. Then as *data*: the joint fit over both recordings
(§ status) closed that gap and is now the preset.

What v11 has NOT answered: both configs still underestimate its tilt tail
during the deep crouch (0.21 sim vs 0.32 rad real). No parameter in the
current space moved it, which is the signature of a **missing mechanism**
rather than a mis-set value — torque saturation, or contact behaviour in a
deeply crouched stance, are the candidates. That is the next thing to find,
and it wants a third recording or a crouch-window-only search to isolate.

## B. Measure the tracker translation

A ruler against the trunk centre ends the `--tracker-z 0.207` guess, unlocks
`height_mean` as a tenth statistic, and removes the last systematic the
sensor-space trick only cancels to first order. Ten minutes on the robot.

## C. Capture data that answers what these cannot

* **Low-level control enabled**, if the hardware supports it, so
  `lowcmd`/`lowstate` carry real joint data. That upgrades the whole method
  from distributional matching to short-horizon prediction error
  (re-initialize sim from real state, score 0.5 s predictions) — far better
  conditioned, chaos-free, and it can localize error to individual joints.
* **Deliberate pitch and roll**, to make the tracker lever arm observable.
* **Longer straight lines** — the current runs live in a 1.5 m box.

## D. Housekeeping

* The joint fit pulled every parameter well inside its bounds, which the
  single-recording fit did not — but the payload cluster (trunk mass/inertia/
  com_x vs leg_mass_scale) and the latency cluster (command_delay vs
  actuator_tau vs trunk inertia) still trade against each other. Quote the
  configuration, not the individual values, until a measurement pins one.
* The noise floor shrinks ~10–100× at the fitted physics (the matched sim is
  much less statistically chaotic than the default one), so a standalone
  `--eval` at the best config reports inflated/infinite SNR against its own
  floor. Compare absolute sim/real columns there, reuse a default-physics
  floor, or use `search.usable_floor`. Four-seed peak-to-peak is also a
  fragile spread estimator; the joint fit was re-checked at six.
* `speed_lag` is the weakest surviving statistic (sim 0.12 vs real 0.08 s,
  the one residual above 1 after the collapse in absolute terms).

## E. Now: train against it

The project goal, and A is green. The matched sim is the environment; the
fitted config is the domain-randomization centre, and the noise-floor
machinery doubles as a regression test that future sim changes stay matched.
Randomize *around* the fitted point along the traded clusters above — that
is where the identified uncertainty actually lives.

---

# Traps worth not repeating (leg edition)

* **A judge only constrains what it can see.** Nine base statistics all
  sub-noise, and the robot was visibly prancing — leg behavior was simply
  outside the objective. When a match looks perfect and the eye disagrees,
  believe the eye and find the missing statistic.
* **Check every topic before declaring data absent.** `policy/lowcmd` sat in
  the recording the whole time while FINDINGS said "no joint-space data" —
  the check had read the zeroed `rt/lowcmd` and stopped.
* **Compare command-to-command, not replay.** Open-loop replay of a closed
  loop's commands falls over in seconds and proves nothing; FK on commanded
  targets is stable and symmetric between sim and recording.
* **Emergent gait style is not one knob.** Contact stiffness, torque limits,
  com, leg mass, joint friction, dq smoothing — every single-parameter A/B on
  front lift was null or backwards, then the joint search found a
  configuration where five of those knobs *together* fix it.

# Traps worth not repeating

Each of these produced a plausible, finite, wrong number.

* **Put both streams on one epoch.** Zeroing each stream at its own first
  message turned "how long the tracker ran before the operator pressed walk"
  into 0.31 s of phantom command delay — which a search then confidently
  fitted, within 4 ms. Per-recording, and 4.4 s on the other run.
* **Never invert a guessed extrinsic onto the ground truth.** The lever-arm
  correction made the "real" height mostly an artifact of the guessed offset.
  Simulate the sensor instead of correcting the measurement.
* **A mechanism beats a knob.** The axis-dependent lag was unfittable by any
  of nine parameters, and was ten lines of the robot's own command shaping
  with published constants. When a search trades two objectives it should be
  able to satisfy, go read the executor.
* **Check what a fitted value is absorbing.** trunk_inertia ×3.4 and 8× spec
  frictionloss were the search compensating for the two artifacts above; they
  relaxed to ×1.5 and 3.7× once the artifacts died. Implausible fitted values
  are structural-error alarms, not measurements.
* **Filter by time, not samples; regression slopes, not means of ratios;
  autocorrelation, not FFT, for gait frequency; fit the lag before the gain.**
  Estimator bugs that each reported confident nonsense (a 3.9 m/s Go2, a
  backwards-turning simulator, a retraction that had to be un-retracted).
* **Never fit the mount against the simulator** — rollouts diverge in seconds,
  so the sim-vs-ghost score is flat and its argmin ~180° wrong. The mount is a
  property of the recording.
* **Check that a patch applied, and don't append a mutable array** — one
  silent edit failure and one aliased `vel_cmd` buffer each invalidated a
  round of numbers; both now have regression tests.
* **`|q| < 4 rad` is not a validity check.** Against real joint limits, 0.0%
  of recorded calf angles are physical.
