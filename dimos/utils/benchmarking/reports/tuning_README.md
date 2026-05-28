# Twist-base controller tuning — measure → derive → validate (HARDWARE)

Two CLI tools that turn one real measurement of a velocity-commanded
mobile base into a single versioned config artifact with every parameter
needed to tune its path controller, then validate it on the real robot.
**Robot-agnostic**: everything robot-specific lives in a
`RobotPlantProfile` (`--robot`, default `go2`). Adding a robot = one
profile entry (see *Adding a robot* below); the two commands are
otherwise identical.

```
characterization --robot R --mode hw  ──▶  R_config_hw_*.json (robot-valid)
benchmark --robot R --mode hw --config …  ──▶  same file + section 5
                                          "for tolerance X cm, run Y m/s"
```

Both tools run the baseline path follower **inside a real
`ControlCoordinator`** in this process, driving the existing
`transport_lcm` twist-base adapter. The operator brings up whichever
connection module owns the robot side of the LCM topics in another
terminal — `unitree-go2-webrtc-keyboard-teleop` for hw, the new
`coordinator-sim-fopdt` (an in-process FOPDT plant exposed on the same
`/{robot_id}/cmd_vel|odom`) for sim. **The two modes are architecturally
identical**: same coordinator, same adapter, same task; only the robot
on the other side differs.

**This is a hardware deliverable.** Sim exists only as a plumbing
self-test / pre-check and is explicitly stamped not-robot-valid — never
tune from it.

## Why these numbers (settled findings, not re-derived)

A velocity-commanded base is FOPDT per axis. At a given speed the
tracking error is the plant floor `(τ+L)·v`; no reactive control law
beats it. So the recommended controller is hardcoded to the production
baseline P-controller, and the only real levers — feedforward gain
(`1/K`) and a curvature velocity profile — are *derived from the measured
plant*, not hand-tuned. (The embedded evidence string cites the Go2
result; a different robot's headroom is TBD until characterized.)

## Prerequisites (real robot)

1. The host that reaches the robot (for the Go2 profile:
   **`dimensional-gpu-0`**).
2. Terminal 1: `dimos run <profile.blueprint>` — for `--robot go2` that
   is `unitree-go2-webrtc-keyboard-teleop`, which brings up the Go2
   connection (publishes the odom topic, consumes the cmd topic) **and**
   a keyboard teleop for repositioning, run **publish-only-when-active**
   (silent while idle, so it does not flood the cmd topic / fight the
   tool). A different robot needs an equivalent bring-up blueprint that
   speaks Twist on the profile's cmd topic + `PoseStamped` odom.
3. Terminal 2: strip nix from the linker path or `.venv` numpy breaks
   (`GLIBC_2.38`):
   ```
   export LD_LIBRARY_PATH="$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' \
       | grep -v /nix/store | paste -sd:)"
   ```
4. Repositioning: the robot is **stopped** at every prompt. Reposition
   (Go2: keyboard teleop WASD/QE, then **release all keys** so it goes
   silent), then press ENTER. The tool then owns the cmd topic for that
   run. Do not hold teleop keys while a run is going.
5. Operator-tunable timings (defaults come from the profile):
   `--step-s` (time safety cap), `--max-dist` (real-space bound — each
   step ends at whichever of distance/time comes first; `wz` spins in
   place so it ends on time), `--pre-roll-s`, `--odom-warmup`.

## Tool 1 — `characterization`

For Go2 HW the preferred entrypoint is the all-in-one blueprint
(one terminal, autoconnects GO2Connection + ControlCoordinator + pygame
keyboard teleop + the Characterizer module + a per-session telemetry
recorder):

```
dimos run unitree-go2-characterization
```

All operator input goes through the pygame teleop window. Movement keys
(WASD/QE) are unchanged; the SI loop adds three gate keys that the
operator presses while the pygame window has focus:

- **Enter** — advance to the next step
- **K** — skip the current amplitude
- **Backspace** — quit (no artifact written)

Mechanism: `KeyboardTeleop.gate: Out[Int8]` publishes those events on
`/characterizer/gate`, `Characterizer.gate: In[Int8]` consumes them.
Other blueprints that bundle `KeyboardTeleop` are unaffected (the new
port is just unwired). Configuration fields map 1:1 to CLI flags, so
``dimos run unitree-go2-characterization --module.characterizer.surface
grass`` is equivalent to ``--surface grass`` on the CLI.

### Telemetry recording (blueprint only)

The blueprint composes a `CharacterizationRecorder` that captures four
streams into a per-session SQLite DB so post-process tools can re-fit
or dissect spikes without re-running on hardware:

- `cmd_vel` (commanded `/cmd_vel`)
- `joint_state` (`/coordinator/joint_state` — x/y/yaw)
- `odom` (raw `/go2/odom` from GO2Connection)
- `gate` (operator advance/skip/quit events)

The DB lands next to the JSON+PNG artifact:
``<repo>/data/characterization/<robot_id>/<robot_id>_recording_<date>_<sha>.db``.
Read back with:

```python
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState

store = SqliteStore(path="<the .db file>")
store.start()
for obs in store.stream("joint_state", JointState).iterate_ts():
    ts, msg = obs.ts, obs.data
    # re-fit, plot, etc.
```

### Noise rejection + fit procedure (legged-base data)

Raw Go2 odom (~15-19 Hz on default tick) carries a ~2 Hz body bob from
the trot gait plus single-sample spikes from leg impacts. The per-step
pipeline before `fit_fopdt`:

1. **Buffer raw pose** during pre-roll + step (``_JointStatePoseStream``).
   The pre-roll samples are zero-commanded → robot at rest → their
   std becomes our `noise_std` estimate for the fit (see step 4).
2. **Savitzky-Golay on position, then central difference**.
   ``reconstruct_body_velocities`` applies ``scipy.signal.savgol_filter``
   to (x, y, yaw) (default window=11 ≈ 2 trot cycles, order=2), then
   ``np.gradient`` to recover (vx, vy, dyaw). Smooth-then-differentiate
   keeps gait-frequency noise out of the velocity signal —
   differentiate-then-smooth (the obvious wrong order) can't recover
   what the differentiation step destroyed. Knobs: ``savgol_window``
   / ``savgol_order``; set window=0 to disable.
3. **Hampel filter** on reconstructed velocity. Catches residual
   single-sample spikes that savgol smooths over but doesn't replace.
   Window=11, n_sigma=3.0; set window=0 to disable.
4. **`fit_fopdt` with three guards** to keep the deadtime estimate
   honest at our sample rate:
   - ``min_deadtime``: floor L at the data's median sample interval
     (you can't physically resolve deadtime finer than odom rate).
   - ``noise_std``: from step 1; used for weighted least squares and
     for the data-driven L detection.
   - ``two_stage=True``: detect L as the first time the response
     crosses ``5σ`` above ``noise_std``, pin it, then fit only (K, τ).
     Removes the joint-fit's L/τ correlation that lets the optimizer
     trade off a tiny L for a slightly inflated τ. Auto-disabled if
     ``noise_std`` can't be estimated. Toggle via
     ``CharacterizerConfig.two_stage_fit``.

The PNG plot dots the raw post-reconstruction velocity under the
filtered+fit lines when Hampel replaced any points, and annotates how
many. Note that in two-stage mode the reported L corresponds to "when
the response crossed 5σ above the rest-noise floor" — not a free
optimization parameter — so cross-channel L comparisons are
interpretable.

The two-terminal CLI flow still works and is required for
``--mode self-test`` (no robot needed) and for the end-to-end sim
variant via ``coordinator-sim-fopdt``:

```
uv run python -m dimos.utils.benchmarking.characterization \
    --robot go2 --mode hw --surface concrete --gait-mode default
```

Per excited channel (`profile.excited_channels`; Go2 = vx, wz — it does
not strafe in the default gait) × a few amplitudes:

1. Robot **stopped**; prompt `ENTER=run s=skip q=quit`. Reposition, ENTER.
2. Pre-roll zeros (settle), then a velocity step (`--step-s`) at the
   profile tick rate, recording commanded vs body-frame velocity
   differentiated from the odom topic. Ends at `--max-dist` or `--step-s`.
3. `safe_stop`, fit FOPDT.

Drift is bounded to one step (operator gate before each). Safety: clamp
to the profile envelope, stale-odom abort, distance + time caps,
zero-Twist on exit / Ctrl-C / `q`.

Artifacts land at `<repo>/data/characterization/<robot_id>/<robot_id>_config_<…>.{json,png}`
by default (overridable via `--out` / `CharacterizerConfig.out`).

**Primary output is a graph** — `<robot_id>_config_<…>.png`, one column
per channel overlaying every step's *measured* velocity (solid) with its
*fitted FOPDT* step response (dashed), annotated K/τ/L/r² per amplitude
— this is what you read to judge whether the model matches the real
robot. The `.json` alongside is the machine handoff the benchmark
consumes (sections 1–4 + 6; section 5 pending; `valid_for_tuning=true`).
Channels not excited (e.g. vy on a non-strafing robot) are placeholdered
= vx and flagged in the caveats.

`--mode self-test` (no robot, no blueprint, no coord): steps the
profile's in-process FOPDT sim plant and recovers it. Proves the
measure→fit→derive code runs; artifact stamped `valid_for_tuning=false`.
The pytest/CI path — **not a tuning artifact**.

For a `--mode hw` *style* run against the sim plant (full coordinator +
transport_lcm path, no robot): bring up `coordinator-sim-fopdt` in
terminal 1, then run the benchmark/characterization with `--mode sim`
in terminal 2. Same architectural shape as hw — only the LCM peer
differs.

## Tool 2 — `benchmark`

For Go2 HW the preferred entrypoint is the all-in-one blueprint
(one terminal, autoconnects GO2Connection + ControlCoordinator + pygame
keyboard teleop + Benchmarker + a per-session telemetry recorder):

```
dimos run unitree-go2-benchmark --module.benchmarker.config <artifact>
```

Operator UX matches the characterization blueprint: WASD/QE in the
pygame window to reposition/aim the robot between runs; **Enter** to
advance, **K** to skip, **Backspace** to quit. Comparison arms map 1:1
to CLI flags via `--module.benchmarker.<field>` overrides; recordings
land at `<repo>/data/benchmark/<robot_id>/<robot_id>_benchmark_<date>_<sha>.db`
(tag="benchmark" so they don't collide with characterization recordings
in the sibling `data/characterization/` dir).

The two-terminal CLI flow still works and is required for `--mode sim`
(end-to-end pre-check against `coordinator-sim-fopdt`):

```
uv run python -m dimos.utils.benchmarking.benchmark \
    --robot go2 --config reports/go2_config_hw_concrete_<date>_<sha>.json \
    --mode hw --speeds 0.3,0.5,0.7,0.9,1.0 --tolerances 5,10,15
```

**By default it runs the BARE stock baseline P-controller — no
feedforward, no velocity profile.** That is the point: this measures the
**plant's physical tracking limit** with the existing production
controller, the number you compare everything against and check against
the `(τ+L)·v` floor. Path set is fixed (`straight_line`, `single_corner`
2 m/90°, `square` 2 m, `circle` R1.0). For each (path, speed): operator
gate, the path is **anchored to the robot's current pose**, then tracked
closed-loop at the profile tick rate off real odom; CTE scored from the
real trajectory. The **bare** run writes section 5 (operating-point map
+ tolerance→max-safe-speed inversion) back into the artifact — the
canonical physical-limit map. Same safety as Tool 1.

Optional comparison arms (off by default), each measured *against* the
bare physical limit, written to standalone `_<arm>_` files that never
clobber section 5:

- `--ff` — apply the artifact's derived feedforward.
- `--profile` — apply the artifact's derived curvature velocity profile.
- `--ff --profile` — both (the fully-derived config).
- `--rg --e-max <m>` — apply the **reference governor's** per-waypoint
  speed profile (geometric + saturation + lateral + precision
  constraints, then forward/backward accel passes). `--e-max` (default
  0.05 m) is the corridor half-width fed into the precision constraint
  `v ≤ e_max / max(τ_vx+L_vx, τ_wz+L_wz)`. The follower applies the
  resulting list[float] cap via `start_path(velocity_profile=…)` —
  the same 8-waypoint lookahead the static profile uses so braking
  starts before each corner.

The RG arm imports `solve_profile()` directly from the reference
governor module — there is no RG Module instance in this blueprint. The
math runs in-process inside the Benchmarker, and the resolved profile
crosses to the follower as a plain list of floats. RG-as-Module (live
`e_max` stream → reactive recompute) is a B3 concern.

`--mode hw` **refuses a non-robot-valid config when any comparison arm
(`--ff` / `--profile` / `--rg`) is set** (sim-derived gains/plant are
meaningless on the real robot). The bare physical-limit run accepts
any config.

`--mode sim`: optional fast pre-check. Same baseline + coordinator +
`transport_lcm` path as hw, but the LCM peer is the
`coordinator-sim-fopdt` blueprint (FOPDT plant + odom integrator)
instead of the real Go2 bring-up. Loudly labelled a pre-check; the map
is not a real-robot result.

## Reading the artifact

| Section | Field | Meaning |
|---|---|---|
| 1 | `provenance` | robot/surface/mode/date/sha, `sim_or_hw` |
| 1 | `valid_for_tuning` | **false ⇒ do not tune from this** (self-test) |
| 1 | `plant` | fitted FOPDT `{K,τ,L}` per axis |
| 2 | `feedforward` | `1/K` per axis + clamps |
| 3 | `velocity_profile` | curvature speed profile |
| 4 | `recommended_controller` | baseline + plant-floor evidence |
| 5 | `operating_point_map` | per (path,speed) CTE + tolerance→speed (null until Tool 2 bare run) |
| 6 | `caveats` | validity scope; self-test artifacts lead with a loud DO-NOT-TUNE banner |

## Adding a robot

Append one `RobotPlantProfile` to `ROBOT_PLANT_PROFILES` in
`dimos/utils/benchmarking/plant.py`: its `robot_id` (= LCM topic prefix
= `transport_lcm` adapter `hardware_id`), the hw `blueprint` and the
sim `sim_blueprint` the operator runs in the other terminal, saturation
envelope (`vx_max`, `wz_max`), `tick_rate_hz`, `excited_channels`
(omit `vy` if it doesn't strafe), `si_amplitudes`, and a `sim_plant`
(`TwistBasePlantParams`) used as the self-test ground truth and by
`FopdtPlantConnection` when the sim blueprint is composed. Then the
identical two commands with `--robot <id>`. For a brand-new sim plant
shape (different topic prefix), add a tiny blueprint mirroring
`coordinator_sim_fopdt` in `dimos/control/blueprints/mobile.py`.

## When to re-run

Re-run Tool 1 (then Tool 2) on any plant change: different surface
(friction → K/τ), gait mode, firmware/locomotion change. The `caveats`
state exactly what the artifact is valid for.

## Tests

```
uv run pytest dimos/utils/benchmarking/test_tuning.py -q
```

Pure DERIVE (1/K per axis, wz-ceiling margin + envelope clamp, accel
formulas, hardcoded baseline + evidence), `valid_for_tuning` (true only
for hw; self-test false + leading DO-NOT-TUNE caveat; survives
round-trip), artifact round-trip + schema rejection, tolerance→max-speed
inversion. HW loops require a robot — covered by the manual prerequisites
above, not pytest.

## Not here (by design)

The MPC/RPP/Lyapunov bake-off, command smoothers, sweeps, and plotting
R&D were the evidence for "baseline + FF + curvature profile"; they are
the appendix, archived off-repo, not the product.

## Reference governor (precision → per-waypoint v)

The artifact is the *static* tuning. The reference governor is the
*runtime* layer that consumes a corridor half-width `e_max` (metres)
and produces a per-waypoint velocity profile. It is intentionally
**open-loop, model-based** — no measured-CTE feedback, no new
controller, no toppra; just one new closed-form constraint composed
with the existing curvature MVC + accel passes.

Data flow:

```
            ┌──────────────────────────────────────────────┐
nav path ──▶│  ReferenceGovernor (Module)                  │
            │  • Loads TuningConfig artifact (plant + caps)│
e_max ─────▶│  • Composes 4 constraints (one is NEW):      │
            │      v ≤ v_max               (geometric)     │
            │      v ≤ ω_max / κ           (saturation)    │
            │      v ≤ √(a_lat / κ)        (lateral)       │
            │      v ≤ e_max / max(τ+L per channel)        │
            │           ── precision; NEW                 │
            │  • Forward/backward accel passes (reused)    │
            │  • Atomic-snapshot read API                  │
            └──────────────────────────────────────────────┘
                              │
                              ▼  installed as external_profile_cap
            ┌──────────────────────────────────────────────┐
            │  BaselinePathFollowerTask                    │
            │  (control law unchanged; only cap source     │
            │   swaps from VelocityProfileConfig→governor) │
            └──────────────────────────────────────────────┘
```

### The precision constraint

The straight-line CTE floor of a velocity-commanded FOPDT base is
`(τ_vx + L_vx) · v` (issue #921 characterization, see
`project_go2_plant_and_diagnostic`). On *curved* segments the dominant
lag is the wz channel rather than vx — the heading-tracking lag of
`τ_wz + L_wz` produces a comparable CTE term. Empirically (sim runs
on `smooth_corner`, `slalom`, `figure_eight`), using only the vx
channel under-predicts CTE on curved paths by ~2×. The shipped
constraint takes the worse channel:

    v  ≤  e_max / max(τ_vx + L_vx, τ_wz + L_wz)

For the vendored Go2 plant this gives `max(0.46, 0.65) = 0.65 s`, so
e_max=5 cm caps v at 0.077 m/s (was 0.109 m/s under the vx-only
formula). This is the *only* new math. The other three caps already
lived in `velocity_profiler.py` (geometric/lateral) or in the artifact
(saturation envelope). The constraint set composes with `min()` per
waypoint; the existing accel passes run on top.

The remaining residual on continuously-curving paths (~30% above the
predicted floor) reflects closed-loop heading-chase dynamics that no
single-segment FOPDT model captures. Closing the loop on measured CTE
is the next layer — see the demo's `cte_max` print for the residual on
each path.

### Where it lives

| Path | What |
|---|---|
| `dimos/navigation/reference_governor/reference_governor.py` | Module + 4 constraint classes + `solve_profile` |
| `dimos/control/tasks/baseline_path_follower_task.py` | Added `external_profile_cap` injection seam (control law unchanged) |
| `examples/go2_reference_governor_demo.py` | End-to-end demo (in-process FOPDT sim) |

### Demo

```
# Static e_max (5 cm corridor on the canonical 90° corner)
uv run python examples/go2_reference_governor_demo.py --path single_corner --e-max 0.05

# Time-varying e_max (loose → tight every 2s) — exercises the
# atomic-snapshot recompute path on a hot stream
uv run python examples/go2_reference_governor_demo.py --path circle --mode square-wave \
    --e-max-high 0.10 --e-max-low 0.02 --period 4.0
```

A `*.png` plot is written to `/tmp/reference_governor_demo.png` by
default: reference path + executed trajectory, plus an e_max(t) vs
commanded `|vx|(t)` overlay in square-wave mode.

### Open-loop assumption — what the calibration plot would tell you

The model predicts `cte ≈ max(τ_vx+L_vx, τ_wz+L_wz) · v_binding`. The
demo prints actual `cte_max` from `score_run` so you can compare
against the predicted floor for each `e_max`. On the vendored Go2
plant (max τ+L = 0.65 s), `--e-max 0.05` predicts ~5 cm CTE; sim runs
on `single_corner` measure ~8 cm and on `smooth_corner` measure ~7 cm.
The ~30–50% residual is the heading-chase term the open-loop model
doesn't see.

For a systematic precision-vs-speed sweep, lean on the existing
benchmark harness:

```
uv run python -m dimos.utils.benchmarking.benchmark \
    --robot go2 --config <artifact> --mode sim --profile
```

(the `--profile` arm exercises the static curvature profiler — directly
analogous to the governor in the `e_max → ∞` limit). For the
governor's precision axis specifically, drive the demo script across a
grid of `--e-max` values and read off the per-run `cte_max` it prints.

### Scope honesty

The constraint set is validated against the same path battery used by
the static tuning (`straight_line`, `single_corner`, `square`, `circle`,
`smooth_corner` etc.). For geometries with materially different local
curvature distributions (cusps, near-zero-radius corners no arc-radius
blend smooths), the characterization → derive → governor chain must be
re-checked. The governor is open-loop by design: it does *not* see the
live CTE, so a model gap manifests as silently wider-than-predicted
tracking error. The demo's `cte_max` print is your sanity check.

### Closed-loop α-feedback variant — NEGATIVE result (shipped, opt-in, default OFF)

A closed-loop variant is wired and shipped behind the `--closed-loop`
flag on the demo (and the `closed_loop=True` config field on
`ReferenceGovernor`). It observes per-tick measured CTE via
`scoring.nearest_segment`, runs a PI law on `(cte - e_max)` with
anti-windup + EMA filter, and applies a multiplicative α ∈ [α_min,
1.0] scaling to the open-loop profile output. The architecture and
unit tests pass cleanly. **The empirical result on the canonical path
battery, however, does not show convergence:**

| Path | OL (option A) | CL default | Final α | Bare follower |
|---|---|---|---|---|
| `single_corner` | 8.8 cm | **8.8 cm** | 1.000 | 10.3 cm |
| `smooth_corner` | 9.9 cm | **10.0 cm** | 1.000 | 7.7 cm |
| `slalom` | 9.8 cm | **10.4 cm** | 0.722 | 5.0 cm |
| `figure_eight` | 11.9 cm | **12.1 cm** | 0.852 | 5.2 cm |

Two failure modes:

1. **Spike-CTE paths** (`single_corner`, `smooth_corner`): α stays at
   1.0 because the high-CTE excursion at the corner is too brief — the
   EMA filter dampens it before the PI integrator can wind up. Could
   be addressed with more aggressive gains (`kp_alpha ≈ 20`,
   `cte_ema_alpha ≈ 0.7`), but the corner is anyway a transient
   feature.
2. **Sustained-CTE paths** (`slalom`, `figure_eight`): α DOES drop
   (0.72–0.85, the loop is actuating) — but `cte_max` gets slightly
   *worse*, not better. Slowing further does not help because the
   bottleneck on these paths is **wz authority**, and the cap's
   geometry-preserving scaling reduces `wz` proportionally with `vx`.
   The controller loses turning authority at exactly the moment it
   needs more. The closed-loop is doing the algebraically correct
   thing for the wrong physics model.

The fix is structural, not a gain tune. Three options, all larger
than the V1 closed-loop scope:

- **Decouple α** into `α_vx` and `α_wz`. The cap stops preserving
  turn radius when wz is the bottleneck. Simplest concept; adds a
  second feedback channel.
- **Per-waypoint α gating** — apply α only at waypoints where
  precision binds (not saturation/lateral). Requires the solver to
  track binding constraints.
- **Adapt e_max** — when wz is the binding bottleneck, widen the
  corridor instead of slowing the robot. Concedes the precision
  promise but stays inside the open-loop framework.

The V1 closed-loop is left in place (opt-in, default off) so the
machinery exists for the structural follow-up. The demo prints a
"converged?" line and an α(t) overlay to make this state visible.

### Non-goals

- No toppra dependency. The constraint-generator architecture is
  future-ready for a toppra swap but does not pull it in.
- No new control law in the follower.
- The shipped closed-loop variant is wired but does not converge on
  the canonical battery — see "Closed-loop α-feedback variant" above.
