# Autoresearch

An autoresearch loop points [evo](https://github.com/evo-hq/evo) at one
function and lets it run experiments against a benchmark: subagents form a
hypothesis in an isolated worktree, edit the target, run the benchmark, and
the orchestrator extends whichever branch scored best. Gates decide what a
score is allowed to mean.

The live loop optimizes `PointCloud2.agent_encode()` — the lossy text a robot's
lidar frame becomes before an LLM ever sees it — against the pointcloud VQA
suites in `dimos/evals/suites/`. The first run took the geometry suite from
0.136 to 0.96 over 13 experiments ([#3415](https://github.com/dimensionalOS/dimos/pull/3415)).
This page is the wiring for the next one.

> Everything the loop needs lives in this directory and is deleted once a run
> has landed its winner — it is scaffolding, not shipped code. The only traces
> outside it are the slice tag in `generate.py`, the `split.assign()` call in
> the three sliced suites, and the `train` tag in the four hand-authored ones.
> See [README.md](README.md).

## Where the encoder stands

Measured on `gpt-5.6-luna`, 2026-08-18, before the slices below existed:

| Suite | Family | Sighted | Blind | Chance |
|---|---|---|---|---|
| `go2_pointcloud` | geometry (7 families) | ~0.96 | — | — |
| `go2_pointcloud_clearance` | `clearance` | 0.88 | 0.38 | 0.50 |
| `go2_pointcloud_route` | `route` | 0.61 | 0.38 | 0.33 |
| `go2_pointcloud_glass` | `crossing` | 0.45 | — | 0.50 |

Crossing is *below* chance: the encoding shows a robot-width hole in the
body-height boxes and the model walks through the glass. Route's third answer,
`unknown`, is a route that exists only if unmeasured space happens to be clear.

Both failures are the same hole. `agent_encode()` emits where returns are and
nothing else — it cannot distinguish space the sensor swept and found empty
from space the sensor never reached, and glass returns nothing at the pane
*and* nothing behind it. That shadow is the whole evidence. Free space, not
more obstacle precision, is the axis the next run has to find.

And the four hand-authored families added 2026-08-19, same model, same day:

| Suite | Family | Sighted | Blind | Always-negative | Sighted, positive half |
|---|---|---|---|---|---|
| `go2_pointcloud_doorway` | `doorway` | 0.50 | 0.50 | 0.50 | **0.67** |
| `go2_pointcloud_rooms` | `rooms` | 0.00 | 0.00 | 0.50 | 0.00 |
| `go2_pointcloud_floor_level` | `floorlevel` | 0.50 | 0.50 | 0.50 | 0.00 |
| `go2_pointcloud_stairs` | `stairs` | 0.33 | 0.50 | 0.50 | 0.00 |

**Read the last column, not the first.** Each of these suites is three positive
rows and three negative ones, and `matched_set` scores a correct "none" as 1.0,
so answering "none" six times out of six banks 0.50 having seen nothing. The
family mean cannot tell a better encoder from a more timid model. The positive
half can: it is 0.00 blind across all twelve positive rows, because no model
guesses a coordinate. `tool_evo_bench` prints it as `(positive N)` next to any
family whose two means differ, and carries it in the result payload as
`families_positive`.

Three readings worth having before the next run:

- `doorway` is the one family where the cloud demonstrably helps — 0.67 sighted
  on the positive half against 0.00 blind. It hands the gain straight back by
  inventing doorways on two of three negatives (0.33 against a blind 1.00).
  Real signal, cancelled by false positives.
- `floorlevel` sighted is bit-identical to blind: `level` six times. The
  encoding contributes nothing there at all.
- `stairs` scores *below* its own always-negative floor, like `crossing` does.
  It hallucinated a staircase at `4.21,-0.30,0.60` on a frame with no steps.

`floorlevel` and `stairs` deliberately share all six frames and ask different
questions of them. The gap between them was meant to separate "carries
elevation" from "carries steps" — but both score 0.00 on every positive frame,
so today it measures only which phrasing induces more hallucination. That
ablation becomes readable the moment either one lifts off zero.

## Slices

Rows sampled seconds apart off the same wall at the same bearing with the same
answer are one question asked twice. `dimos/evals/temp/split.py` groups rows by the
scene moment they came from — one glass pane, or one 30 s block of one
recording — and sends a whole group to one slice:

```bash
python -m dimos.evals.temp.split      # the table
```

| Family | train | holdout | spare |
|---|---|---|---|
| `clearance` | 23 | 10 | 3 |
| `route` | 25 | 11 | 0 |
| `crossing` | 10 | 4 | 8 |

The four hand-authored families are **not sliced and have no holdout.** Six
rows each is too few to hold a group-disjoint holdout, and `split.py` groups by
30 s blocks, which would collapse most of them. They are tagged `train` where
they are built, so the bench sees them; nothing verifies that a gain on them
left the frames it was found on. Treat a doorway or stairs win as a hypothesis
until more frames are labelled — the same standing caveat as `crossing`, only
stronger.

`train` is what the optimizer sees. `holdout` is scored only at gate time and
shares no group with train, so a gain that does not survive it was memorization.
`spare` rows are near-duplicates of a kept row: still runnable
(`dimos evals run … --tags spare`), never paid for in the loop.

The geometry suite is not sliced. It is the frozen regression set — tagged
`frozen`, gated, never optimized.

Two things to know before trusting a crossing number: its holdout is four
cases, and its eleven barrier rows come from three glass surfaces. That is a
tripwire, not a measurement. Label more panes before believing a crossing win.

## The harness

`dimos/evals/temp/tool_evo_bench.py` speaks evo's inline instrumentation contract —
reads `EVO_RESULT_PATH`, `EVO_TRACES_DIR`, `EVO_EXPERIMENT_ID`; writes
`{"score", "tasks", "families", …}` and one trace per case; exits 0 whenever
the measurement completed, because an encoder that scores 0.0 is a dead branch
and not a broken harness.

```bash
python -m dimos.evals.temp.tool_evo_bench                  # the benchmark: train + frozen
python -m dimos.evals.temp.tool_evo_bench --dry-run        # wiring only, no model calls
python -m dimos.evals.temp.tool_evo_bench --limit 6        # smoke, one case per family
python -m dimos.evals.temp.tool_evo_bench --slice holdout --gate --min-score 0.62
```

The score is the mean of the *scored* families' means — `clearance`, `route`,
`crossing`, `doorway`, `rooms`, `floorlevel`, `stairs` — family-weighted so 40
solved geometry rows cannot outvote 22 glass rows. The frozen suite rides along
in the same run without moving the score, so the regression gate reads it back
for free. Every run also drops `.evo_bench/<slice>.json` (gitignored) for the
gates and for you.

## Gates

| Gate | Phase | Command | Catches |
|---|---|---|---|
| `static` | pre | `python -m dimos.evals.temp.tool_evo_gate static` | the benchmark being edited; the encoder importing the answer or reaching the filesystem |
| `budget` | pre | `python -m dimos.evals.temp.tool_evo_gate budget` | winning by emitting more — 6 kB and 80 ms per frame, against a seed of 3.3 kB and 14 ms |
| `floors` | post | `python -m dimos.evals.temp.tool_evo_gate floors` | any frozen family sagging more than 0.05 below baseline |
| holdout | post | `python -m dimos.evals.temp.tool_evo_bench --slice holdout --gate --min-score <floor>` | gains that do not leave the groups they were found on |

Pre-gates decide from the worktree alone, so a candidate that rewrites the
questions or bloats the encoding fails before a model call is paid for.

Route's ground truth is our own planner and clearance's is the eval generator,
so `static` bans `dimos.navigation`, `dimos.mapping`, `dimos.evals` and
`dimos.perception` from the encoder outright — importing the answer would score
1.0 and ship nothing.

The manifest cannot defend itself: an optimizer that edits `temp/evo_frozen.json`
and the gate together passes. The per-experiment diff and the review before
landing are what catch that. The gate exists to make drift loud.

The blind ablation is *not* a per-experiment gate. Blind runs withhold the
observations, so no encoder can move them; run it once when questions change,
not every experiment.

## Setting up a run

```bash
evo install claude-code

# 1. baseline the encoder you are starting from, then freeze what it measured
python -m dimos.evals.temp.tool_evo_bench --write-floors        # -> dimos/evals/temp/evo_floors.json
python -m dimos.evals.temp.tool_evo_bench --slice holdout       # note the score; it becomes --min-score
python -m dimos.evals.temp.tool_evo_gate freeze                 # -> dimos/evals/temp/evo_frozen.json
git add dimos/evals/temp/evo_floors.json dimos/evals/temp/evo_frozen.json && git commit

# 2. wire the workspace
evo init --name pointcloud-encode --host claude-code \
  --target dimos/msgs/sensor_msgs/PointCloud2.py \
  --benchmark "uv run python -m dimos.evals.temp.tool_evo_bench" \
  --metric max --per-exp-timeout 3600 \
  --gate "uv run python -m dimos.evals.temp.tool_evo_gate static && uv run python -m dimos.evals.temp.tool_evo_gate budget"

# 3. register the post gates on the baseline node, then check they inherit
evo gate add exp_0000 --phase post --name floors \
  --command "uv run python -m dimos.evals.temp.tool_evo_gate floors"
evo gate add exp_0000 --phase post --name holdout \
  --command "uv run python -m dimos.evals.temp.tool_evo_bench --slice holdout --gate --min-score <baseline>"
evo gate list exp_0000
evo run exp_0000 --check     # validates wiring without consuming retry budget

# 4. drive it
/evo:discover   # seeded with what the encoder cannot express — free vs unmeasured space
/evo:optimize subagents=3
```

Re-run `tool_evo_gate freeze` after any deliberate change to a suite, scorer,
or the harness itself, or `static` will fail every experiment.

## What a round costs

One experiment is 122 paid calls for the benchmark (58 sliced train + 24
hand-authored + 40 frozen) plus 25 for the holdout gate — roughly 25 minutes of
wall clock. `floors`, `static`
and `budget` are free. Three subagents on a ten-round run is on the order of a
hundred benchmark runs; size the round before starting it, not after.

## Landing a winner

Evo commits every experiment on its own branch with the hypothesis as the
message. Port the winner the way the planner lab was ported: one squashed
commit of the target file, the `exp_NNNN` messages kept in the PR body as the
lineage, and the baseline-vs-winner table per family — train, holdout and
frozen, all three.
