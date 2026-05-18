# Go2 controller tuning — measure → derive → validate (HARDWARE)

Two CLI tools that turn one real measurement of your Go2 into a single
versioned config artifact with every parameter you need to tune the base
controller, then validate it on the real robot.

```
go2_characterization --mode hw  ──▶  go2_config_hw_*.json   (robot-valid)
go2_benchmark --mode hw --config …  ──▶  same file + section 5
                                          "for tolerance X cm, run Y m/s"
```

**This is a hardware deliverable.** Sim exists only as a plumbing
self-test / pre-check and is explicitly stamped not-robot-valid — never
tune from it.

## Why these numbers (settled findings, not re-derived)

Go2 base = FOPDT per axis. At a given speed the tracking error is the
plant floor `(τ+L)·v`; no reactive control law beats it. So the
recommended controller is hardcoded to the production baseline
P-controller, and the only real levers — feedforward gain (`1/K`) and a
curvature velocity profile — are *derived from the measured plant*, not
hand-tuned.

## Prerequisites (real robot)

1. Run on **`dimensional-gpu-0`** (the only host that reaches the Go2).
2. Terminal 1: `dimos run unitree-go2-webrtc-keyboard-teleop`
   — brings up the Go2 connection (publishes `/go2/odom`, consumes
   `/cmd_vel`) **and** the keyboard teleop for repositioning. This
   blueprint runs the teleop **publish-only-when-active**: it stays
   silent while no movement key is held (one zero Twist on release,
   then nothing), so it does not flood `/cmd_vel` and coexists with the
   tool. (Plain `KeyboardTeleop` defaults to streaming every loop —
   only this blueprint enables the silent mode.)
3. Terminal 2: strip nix from the linker path or `.venv` numpy breaks
   (`GLIBC_2.38`), then run the tool:
   ```
   export LD_LIBRARY_PATH="$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' \
       | grep -v /nix/store | paste -sd:)"
   ```
4. Repositioning: the robot is **stopped** at every prompt. Reposition
   with the keyboard teleop (WASD move/turn, QE strafe), then **release
   all keys** — the teleop goes silent — and press ENTER. The tool then
   owns `/cmd_vel` for that run. Do not hold keys while a run is going.
5. Timings are operator-tunable when the robot needs more time to reach
   the commanded speed or the connection is slow to come up:
   `--step-s` (default 8 s, time safety cap), `--max-dist` (default 6 m,
   the real-space bound — each step ends at whichever of distance/time
   comes first; `wz` spins in place so it ends on time), `--pre-roll-s`
   (1 s), `--odom-warmup` (default 10 s).

## Tool 1 — `go2_characterization`

```
uv run python -m dimos.utils.benchmarking.go2_characterization \
    --mode hw --surface concrete --gait-mode default
```

Per channel (vx, vy, wz — vy is real, the Go2 strafes) × a few
amplitudes:

1. Robot is **stopped**; prompt: `reposition robot … ENTER=run s=skip
   q=quit`. Reposition with the keyboard teleop, release keys, ENTER.
2. Pre-roll zeros (settle), then a velocity step for `--step-s` (default
   8 s) at 10 Hz — long enough for the real Go2 to ramp to and hold the
   commanded speed — recording commanded vs body-frame velocity
   differentiated from `/go2/odom`.
3. `safe_stop`, fit FOPDT.

Drift is bounded to one step (operator gate before each). Safety
throughout: velocity clamp (`VX_MAX=1.0`, `WZ_MAX=1.5`), stale-odom
abort, timeout, zero-Twist on exit and on Ctrl-C.

**Primary output is a graph** — `go2_config_<…>.png`, one column per
channel (vx, wz) overlaying every step's *measured* velocity (solid)
with its *fitted FOPDT* step response (dashed), annotated with
K/τ/L/r² per amplitude. This is what you read to judge whether the
model matches the real robot. The `.json` is written alongside only as
the machine handoff the benchmark consumes (sections 1–4 + 6; section 5
pending; `valid_for_tuning=true`). vx has 3 amplitudes, wz 3; vy is the
vx placeholder (not strafe-capable in this gait).

`--mode self-test` (no robot): steps an in-process FOPDT plant seeded
with the vendored ground truth and recovers it. Proves the
measure→fit→derive code runs; artifact stamped
`valid_for_tuning=false`. This is the pytest/CI path — **not a tuning
artifact**.

## Tool 2 — `go2_benchmark`

```
uv run python -m dimos.utils.benchmarking.go2_benchmark \
    --config reports/go2_config_hw_concrete_<date>_<sha>.json \
    --mode hw --speeds 0.3,0.5,0.7,0.9,1.0 --tolerances 5,10,15
```

**By default it runs the BARE stock baseline P-controller — no
feedforward, no velocity profile.** That is the point: this run measures
the **plant's physical tracking limit** with the existing production
controller, the number you compare everything against and check against
the `(τ+L)·v` floor from characterization. Path set is fixed
(`straight_line`, `single_corner` 2 m/90°, `square` 2 m, `circle` R1.0).
For each (path, speed): operator gate (reposition+aim, ENTER), the path
is **anchored to the robot's current pose** (so it need not be placed
precisely), then tracked closed-loop at 10 Hz off real odom; CTE scored
from the real trajectory. The **bare** run writes section 5
(operating-point map + tolerance→max-safe-speed inversion) back into the
artifact — that is the canonical physical-limit map. Same safety as Tool 1.

Optional comparison arms (off by default), each measured *against* the
bare physical limit, written to standalone `_<arm>_` files that never
clobber section 5:

- `--ff` — apply the artifact's derived feedforward.
- `--profile` — apply the artifact's derived curvature velocity profile.
- `--ff --profile` — both (the fully-derived config).

`--mode hw` only **refuses a non-robot-valid config when `--ff`/`--profile`
is set** (sim-derived gains are meaningless on the real robot). The bare
physical-limit run accepts any config (it doesn't use the derived
params).

`--mode sim`: optional fast pre-check against the FOPDT sim plant. Loudly
labelled a pre-check; the map is not a real-robot result. Useful to
sanity-check wiring before committing the robot.

## Reading the artifact

| Section | Field | Meaning |
|---|---|---|
| 1 | `provenance` | robot/surface/mode/date/sha, `sim_or_hw` |
| 1 | `valid_for_tuning` | **false ⇒ do not tune from this** (self-test) |
| 1 | `plant` | fitted FOPDT `{K,τ,L}` per axis |
| 2 | `feedforward` | `1/K` per axis + clamps |
| 3 | `velocity_profile` | curvature speed profile |
| 4 | `recommended_controller` | baseline + plant-floor evidence |
| 5 | `operating_point_map` | per (path,speed) CTE + tolerance→speed (null until Tool 2) |
| 6 | `caveats` | validity scope; self-test artifacts lead with a loud DO-NOT-TUNE banner |

## When to re-run

Re-run Tool 1 (then Tool 2) on any plant change: different surface
(friction → K/τ), gait mode (e.g. `rage`), firmware/locomotion change.
The `caveats` state exactly what the artifact is valid for.

## Tests

```
uv run pytest dimos/utils/benchmarking/test_go2_tuning.py -q
```

Pure DERIVE (1/K incl. real vy, wz-ceiling margin + envelope clamp,
accel formulas, hardcoded baseline + evidence), `valid_for_tuning`
(true only for hw; self-test false + leading DO-NOT-TUNE caveat;
survives round-trip), artifact round-trip + schema rejection, and the
tolerance→max-speed inversion. HW loops require a robot — covered by the
manual prerequisites above, not pytest.

## Not here (by design)

The MPC/RPP/Lyapunov bake-off, command smoothers, sweeps, and plotting
R&D were the evidence for "baseline + FF + curvature profile"; they are
the appendix, archived off-repo, not the product.
