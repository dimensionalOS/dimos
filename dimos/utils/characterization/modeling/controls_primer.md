# Controls primer (for someone learning)

You don't need control theory to use this code, but you do need it to
understand what the numbers mean. Reading time: ~15 minutes. Concepts
are anchored to your actual data.

## 0. The 30-second version

You're trying to make the robot follow a path. To do that the
controller needs to send commands and predict how the robot will
respond. The "predict" part needs a model — a couple of numbers that
describe how the robot reacts to commands. **This whole package
exists to measure those numbers from data.**

Three numbers per channel: **K, τ, L**. Once you have them, standard
formulas turn them into controller gains. Without them, you're tuning
by trial-and-error — which is what issue #921 is asking us to stop
doing.

## 1. The plant

In control terms, the **plant** is the thing you're trying to control.
Not the controller, not the planner, not the path — just the physical
system that takes a command in and produces a response out.

For your Go2:

```
   Twist command  ──▶  WebRTC link  ──▶  onboard locomotion policy
                                                  │
                                                  ▼
                                            leg dynamics
                                                  │
                                                  ▼
                                            measured odom (body velocity)
```

That whole chain is "the plant" from the controller's perspective. You
don't get to look inside it; you only get to see what comes out when
you send something in. Modeling it means writing down a function that
predicts the output from the input — accurately enough that a
controller built on the prediction will work.

## 2. Why send step inputs?

A **step** is the simplest possible command: zero, then jump to some
value, then hold. Like flipping a light switch.

Why steps:

1. They excite the plant across all timescales at once. A step contains
   energy at every frequency, so the response reveals fast and slow
   dynamics simultaneously.
2. They're easy to interpret visually — anyone can look at a step
   response and see "it rose, it took some time to settle, here's the
   final value."
3. The math for fitting a model to a step response is well-understood
   and robust.

Other inputs (ramps, chirps, sinusoids) probe other things — saturation,
frequency response, nonlinearity. Rung 1 collected all of them, but
this Rung 2 modeling step only uses the steps.

## 3. What we learn from a step response — K, τ, L

Open one of your overlay plots:
[`session_20260425-135928/modeling/plots/e1_vx_+1.0__overlay.svg`](/home/mustafa/char_runs/session_20260425-135928/modeling/plots/e1_vx_+1.0__overlay.svg)

You'll see:
- A flat line at zero (pre-roll)
- A delay where nothing happens
- A rising curve toward some plateau
- A noisy plateau (steady-state)

That whole shape is fully described by **three numbers**:

### K — steady-state gain

What you commanded vs what you got, at steady state.

> command 1.0 m/s, robot eventually goes 0.92 m/s → **K = 0.92**

K=1 means the plant tracks perfectly. K<1 means it under-tracks (probably
saturating). K>1 means it over-rotates / over-shoots (your Go2's wz has
K≈2.2 — commanded angular rate gets amplified ~2×; this is a genuine
finding from your data).

K is the steady-state, *not* the peak. If the response overshoots and
then settles back, K is the settled value.

### τ — time constant ("tau")

How long the response takes to react. Specifically: after the deadtime,
τ is the time it takes the response to reach **63%** of its final value.

> τ = 0.28s on your vx → from the moment the response starts rising, it
> reaches 63% × 0.92 ≈ 0.58 m/s in 0.28s. After ~5τ ≈ 1.4s it's
> indistinguishable from steady-state.

Why 63%? It's `1 - 1/e`. The response is exponential, and τ is the
natural unit for exponential decay/rise. Smaller τ → faster plant.
Larger τ → sluggish plant.

This is the most physically intuitive parameter once you've stared at a
plot or two: τ is "how snappy is the robot."

### L — deadtime (also called "transport delay")

The flat region at the start, before any response begins. The cmd has
changed, but the robot hasn't moved yet.

> L = 0.06s on your vx → for 60ms after the cmd jumps from 0 to 1.0,
> nothing happens. Then the response starts rising.

L comes from real physical things: WebRTC packet round-trip, the
locomotion policy's internal latency, mechanical compliance. You can't
do anything about it — but you have to plan around it. Deadtime is the
single biggest enemy of fast control: a controller can't react to
something it hasn't seen yet, so the larger L is, the slower your
closed-loop has to be.

## 4. Putting it together — the model

The full model:

```
   y(t) = 0                                  for t < L
   y(t) = K · u_step · (1 - exp(-(t-L)/τ))   for t ≥ L
```

In English: zero for L seconds, then exponentially climb toward
`K · u_step` with time constant τ. That's it. Three parameters, simple
formula, captures most of what matters about a real plant.

This is called **First-Order Plus Deadtime (FOPDT)**. It's not the only
model, just the simplest useful one. It's wrong for some plants
(systems with lots of inertia want a second-order model) but it's right
for surprisingly many.

> If you've heard "transfer function" and `G(s) = K e^(-Ls) / (τs + 1)`
> — that's the same thing in Laplace-transform notation. You can ignore
> the s-domain entirely for now; it's just notation, not new physics.

## 5. Reading your numbers

From the cross-mode comparison:

```
default vx: K=0.92, τ=0.28s, L=0.06s
default wz: K=2.19, τ=0.26s, L=0.08s
rage    vx: K=0.92, τ=0.29s, L=0.06s   (essentially identical to default)
rage    wz: K=2.19, τ=0.26s, L=0.09s   (essentially identical to default)
```

Plain English:

- **vx tracks well** (K≈1) and **responds in about a third of a second**
  with negligible lag.
- **wz over-responds by ~2×** — commanding 1.0 rad/s yields ~2.2 rad/s
  in the body frame. This is a real plant property; either the
  yaw-rate odom estimate is mis-scaled or the Go2 firmware interprets
  cmd in different units. Worth investigating before tuning.
- **Default and rage are the same plant.** Rage changes the locomotion
  policy's gait/top-speed limits, but the closed-loop dynamics seen by
  the controller are identical. You only need one controller.

## 6. The complications you'll see in the report

### Direction asymmetry

> "vx: direction_asymmetric=True"

The plant responds differently going forward vs backward. Quadrupeds
often have this: the gait that produces forward motion isn't the
mirror-image of the reverse gait. If asymmetry is large, your
controller may need separate gains for forward vs reverse, or
direction-aware feedforward.

### Gain scheduling

> "linear_in_amplitude={K: False, ...}"

K (or τ, or L) depends on how big the cmd is. Your vx K varies from
~0.95 at small cmds to ~1.4 at 1.0 m/s to ~0.65 at 1.5 m/s — it's not
constant. The plant is **nonlinear** in amplitude.

Two ways to handle this:
1. Stay in the linear regime (clamp cmd ≤ 1.0 m/s on your robot).
2. Use a **gain schedule** — different controller gains at different
   amplitudes. The `gain_schedule` block in the JSON gives you the slope
   and intercept.

### Saturation

The K=0.65 at 1.5 m/s isn't really K dropping — it's the robot
physically maxing out. You commanded 1.5 but it can only do ~1.0.
Mathematically the FOPDT fit dilutes K to absorb the missing motion.
Practically it means **the plant has a hard speed limit** and the
controller needs to know about it (anti-windup, command clipping).

### Rise vs fall (decel) asymmetry

> "rise vs fall: τ differs (0.29 vs 0.18)"

The robot stops faster than it starts. This is direction-of-time
asymmetry. A symmetric controller will under-actuate stops and
over-shoot starts. Either: ignore it for v1, or add direction-aware
feedforward in v2.

## 7. From K/τ/L to a working controller

For a PI controller (the simplest useful kind) on a FOPDT plant, the
**lambda tuning** rule gives you:

```
λ = max(τ, L)            # closed-loop time constant you choose
Kp = τ / (K · (λ + L))   # proportional gain
Ki = Kp / τ              # integral gain
```

You pick λ. Conservative: λ = 2τ (slow but rock-solid stable).
Aggressive: λ = τ (fast but easier to push into oscillation). Plug in
your numbers — you'll get specific Kp, Ki to test on the robot.

That's the payoff. The whole pipeline — collect → validate → fit FOPDT
→ tune — turns "guess and check" into a calculation.

## 8. What's next (the rest of the project)

- **Rung 2 Session 2 (validation)**: take the K/τ/L we just fitted, simulate
  what the model predicts the robot will do at amplitudes / directions
  we *didn't* train on, compare to the held-out reverse-direction runs.
  If the model predicts within noise, FOPDT is sufficient. If not,
  upgrade to second-order or add nonlinear terms.
- **Rung 2 Session 3 (controller design)**: pick λ, compute Kp/Ki, build
  a closed-loop simulator using the model, verify stability margins,
  ship the gains.
- **Rung 3 (closed-loop benchmark)**: run path-following with the new
  controller at 2-3× current speeds. The whole point of the project.

## 9. Glossary (one sentence each)

- **Plant**: the physical system being controlled.
- **Open-loop**: send command, see response, no feedback. (What we did.)
- **Closed-loop**: controller compares measured to desired, adjusts the
  command in real time. (What Rung 3 builds.)
- **Step input**: input that jumps from 0 to a constant value at t=0.
- **Steady-state**: where the response settles after transients die out.
- **K (gain)**: steady-state output divided by input.
- **τ (time constant)**: how long until the response reaches 63% of
  steady-state.
- **L (deadtime)**: pure delay before any response begins.
- **FOPDT**: First-Order Plus Deadtime — the simplest useful model.
- **Saturation**: the plant has a hard limit beyond which more cmd
  produces no more output.
- **Gain schedule**: different controller gains at different operating
  points (e.g. low-speed vs high-speed).
- **Lambda tuning**: a family of formulas that turns K/τ/L into PI
  controller gains, parameterized by your desired closed-loop speed λ.
- **Bode plot**: frequency-response plot. Shows how the plant attenuates
  / shifts each input frequency. Useful for stability margins; not
  needed for FOPDT-tuned PI controllers.
- **Rise time / settle time / overshoot**: alternative ways to
  characterize a step response. The harness already computes them per
  run; they're more intuitive but contain less information than K/τ/L.
