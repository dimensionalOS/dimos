# Nav-3D Autonomous Improvement Protocol

You are improving the 3D navigation stack: the MLS planner and the voxel ray
mapper. The evaluator in this package is your loss function. It replays real
robot recordings and scores planning on the maps the robot would have had at
the time. Higher is better. The score only counts if every gate below holds.

## Boundary

Editable:

- `dimos/navigation/nav_3d/mls_planner/` (Python and Rust)
- `dimos/mapping/ray_tracing/` (Python and Rust)
- `results.tsv` at the repo root (your journal)

Everything else is off limits, in particular `dimos/navigation/nav_3d/evaluator/`
(including `cases/` and this file). The gate parameters in `EvalConfig` are
physical measurements of the Unitree Go2 and its demonstrated capabilities,
not tuning knobs. Algorithm parameters are the constructor defaults of
`MLSPlanner` and `VoxelRayMapper`; tune them by editing those defaults.

The human reviewer verifies the boundary mechanically against the commit
the session branched from (BASE = the parent branch, e.g.
andrew/feat/nav-evaluator):

    git diff --name-only $(git merge-base BASE HEAD) \
      | grep -vE '^(dimos/navigation/nav_3d/mls_planner/|dimos/mapping/ray_tracing/|results\.tsv)'

Any output fails the whole session.

## Setup

Work on a fresh branch. Confirm the recordings exist (`data/*.db`), then run
the suite once and save the report as your first kept baseline:

    python -m dimos.navigation.nav_3d.evaluator run --json data/reports/kept.json

## Iteration cycle

1. Form one hypothesis and make one focused change.
2. Rebuild whichever Rust module you touched, always in release:

       uv run maturin develop --uv --release -m dimos/navigation/nav_3d/mls_planner/rust/Cargo.toml
       uv run maturin develop --uv --release -m dimos/mapping/ray_tracing/rust/Cargo.toml

   If you touched the mapper (code or defaults), also wipe the replay caches:

       rm -rf data/.final

   A mapper change makes the next run rebuild the maps (~5 minutes). Planner
   changes skip that, so planner experiments are much cheaper.
3. Commit, then evaluate:

       python -m dimos.navigation.nav_3d.evaluator run --json data/reports/candidate.json
       python -m dimos.navigation.nav_3d.evaluator diff data/reports/kept.json data/reports/candidate.json

4. Keep or discard:
   - Keep iff the score improved AND `diff` exits 0 (no case regressed). A
     BROKE line means a start/goal pair the stack used to handle now fails;
     a higher average does not excuse it. Perf budget lines on this parallel
     run are advisory only; wall-clock under parallel contention runs about
     20 percent hot.
   - Keep: pass the confirmation check below, then
     `cp data/reports/candidate.json data/reports/kept.json` and keep the
     commit.
   - Discard: `git reset --hard HEAD^`. If you discarded a mapper change,
     wipe `data/.final` again so caches match the reverted code.
5. Confirmation check, required before every keep. Rerun serially and
   require bit-identical results and in-budget timings:

       python -m dimos.navigation.nav_3d.evaluator run --workers 1 --json data/reports/serial.json
       python -m dimos.navigation.nav_3d.evaluator diff data/reports/candidate.json data/reports/serial.json --exact

   The serial run is slower but its timings are the binding perf gate:
   uncontended wall-clock is what the robot experiences. A nonzero exit
   means the change is non-reproducible or over budget. Fix it or discard.
6. Append one row to `results.tsv` and continue:

       commit_hash<TAB>score<TAB>final_score<TAB>plan_p95_ms<TAB>fixed<TAB>broke<TAB>keep|discard|crash<TAB>one-line description

## Rules

- Crashes are experiments too: fix trivial bugs, abandon flawed ideas, always
  journal the row.
- Never edit the evaluator, the case manifests, or the gate parameters.
- Never weaken determinism to gain score.
- The unit tests of the modules you edit must pass:

      python -m pytest dimos/navigation/nav_3d/mls_planner/ dimos/mapping/ray_tracing/ -q

- Do not add dependencies.
- Run continuously without asking for approval until interrupted.
