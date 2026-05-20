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

`--mode hw` only **refuses a non-robot-valid config when `--ff`/`--profile`
is set** (sim-derived gains are meaningless on the real robot). The bare
physical-limit run accepts any config.

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
