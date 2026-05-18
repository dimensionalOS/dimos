# Go2 controller tuning — measure → derive → validate

Two CLI tools that turn one real measurement of your Go2 into a single
versioned config artifact with every parameter you need to tune the base
controller — no guesswork.

```
go2_characterization  ──(measure + derive)──▶  go2_config_*.json
                                                      │
go2_benchmark  ──(validate across speed)──▶  same file, section 5 added
                                                      │
                                              "for tolerance X cm,
                                               run at speed Y m/s"
```

## Why these numbers (the settled findings — not re-derived here)

The Go2 base is **FOPDT per axis** (first-order + dead-time). At a given
speed the path-tracking error is the **plant floor `(τ + L)·v`** — it
scales with speed, and no reactive control law has meaningful headroom
over it (validated controller bake-off). So:

- The recommended controller is **hardcoded to the production baseline
  P-controller**. It is not re-selected; the evidence string is embedded
  in the artifact.
- The only effective levers are **feedforward gain** (so commanded ==
  achieved velocity) and **speed vs path curvature** (the velocity
  profile). Both are *derived from the fitted plant*, not hand-tuned.

## Tool 1 — `go2_characterization`

```
uv run python -m dimos.utils.benchmarking.go2_characterization \
    --mode sim --surface mujoco            # sim self-test
uv run python -m dimos.utils.benchmarking.go2_characterization \
    --mode hw  --surface concrete --gait-mode default   # real robot
```

Runs a space-cheap system-ID sequence (per-channel velocity steps for
**vx, vy, wz** — vy is real, the Go2 base strafes — no long paths), fits
FOPDT per axis, runs the DERIVE step, and writes the artifact (sections
1–4 + 6; section 5 left empty for tool 2). Prints the artifact path.

**One harness, plant swapped by `--mode`.** Identical SI loop and FOPDT
fitter both modes; only *where the robot behaves* differs:

- **sim**: the plant is the in-process FOPDT `Go2PlantSim` seeded with
  the vendored `GO2_PLANT_FITTED` ground truth. A self-test — it recovers
  the injected model (printed "recovered vs injected" table should match;
  small dead-time `L` is mildly under-fit — expected, not load-bearing).
  Validates the whole pipeline without a robot.
- **hw**: the plant is the real Go2 over LCM (publish `/cmd_vel`,
  differentiate `/go2/odom` to body-frame velocity). Same loop, same
  `fit_fopdt`. Wired; not run by CI/sim. No separate data-acquisition or
  statistics pipeline — point estimates per channel (mean over swept
  amplitudes), not research-grade confidence intervals.

## Tool 2 — `go2_benchmark`

```
uv run python -m dimos.utils.benchmarking.go2_benchmark \
    --config reports/go2_config_sim_mujoco_<date>_<sha>.json \
    --speeds 0.3,0.5,0.7,0.9,1.0 --tolerances 5,10,15
```

Loads the artifact, runs the **hardcoded baseline** with the derived
feedforward + velocity profile across the speed ladder on a fixed,
real-space-constrained path set (`straight_line`, `single_corner` 2 m/90°,
`square` 2 m, `circle` R1.0). Scores each (path, speed), builds the
operating-point map + the tolerance→max-safe-speed inversion, and writes
**section 5 back into the same artifact**. Also emits a standalone
`go2_benchmark_<sha>.json` and one `cte_max`-vs-speed plot.

Final line, e.g.: *"For tolerance 15 cm, run at speed 0.30 m/s with this
profile (binding path: circle)."* The binding path is the slowest path —
its limit gates the fleet.

## Reading the artifact

| Section | Field | Meaning |
|---|---|---|
| 1 | `provenance` | robot / surface / mode / date / git sha / sim·hw — the validity scope |
| 1 | `plant` | fitted FOPDT `{K, τ, L}` per axis |
| 2 | `feedforward` | `K_axis = 1/K_plant` + output clamps — give to `FeedforwardGainConfig` |
| 3 | `velocity_profile` | curvature speed profile — give to `VelocityProfileConfig` |
| 4 | `recommended_controller` | `baseline`, `k_angular`, and the plant-floor evidence |
| 5 | `operating_point_map` | per (path,speed) `cte_max/cte_rms/arrived` + tolerance→speed table (`null` until tool 2) |
| 6 | `caveats` | surface/mode/sim-vs-hw validity limits |

`schema_version` is enforced on load; an artifact from an incompatible
schema is rejected rather than silently mis-read.

## When to re-run

Re-run **tool 1** (then tool 2) whenever the plant changes:

- different **surface** (carpet vs concrete vs ice) — friction changes K/τ,
- different **gait mode** (e.g. `rage`),
- a firmware/locomotion change,
- moving from **sim numbers to a real robot** (`--mode hw`).

The artifact's `caveats` state exactly what it is valid for. A sim
artifact validates the pipeline, not absolute on-robot numbers.

## What is *not* here (by design)

The MPC / RPP / Lyapunov / pure-pursuit bake-off, command smoothers,
oscillation & speed sweeps, and plotting R&D were the *evidence* that
justified "baseline + feedforward + curvature profile" — they are the
appendix, not the product, and were deliberately left out of this
deliverable. `velocity_profile` stays default-off for any non-blueprint
caller.

## Tests

```
uv run pytest dimos/utils/benchmarking/test_go2_tuning.py -q
```

Covers the pure DERIVE step (1/K per axis incl. real vy, wz-ceiling
margin + envelope clamp, accel formulas, hardcoded baseline + evidence,
provenance-driven caveats), artifact JSON round-trip + schema-version
rejection, and the tolerance→max-speed inversion (binding = min across
paths, not-arrived excluded, infeasible → none).
