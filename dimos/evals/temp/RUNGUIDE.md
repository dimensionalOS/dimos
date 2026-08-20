# Runguide — driving the `agent_encode` autoresearch loop

Operating instructions: start a run, watch it, steer it, stop it, pick it back
up. What the loop *is* and what it is trying to fix live in
[autoresearch.md](autoresearch.md); this page is the buttons.

Verified against `evo-hq-cli 0.8.0` and the installed Claude Code plugin
(`~/.claude/plugins/cache/evo-hq-evo/evo/0.8.0`). Where this file and
`evo <cmd> --help` disagree, the CLI wins.

> Not covered by the `static` gate — that hashes `*.py` under `dimos/evals/`
> and `*.py`/`*.json` under `suites/`. Editing this file mid-run is safe and
> needs no re-freeze.

---

## The short version

```
/evo:discover      # one-time: set up the workspace (asks what to optimize)
/evo:optimize      # run the loop
/evo:report        # read-only: what happened
/evo:ship          # land the winner as a PR
```

Everything below is detail behind those four.

---

## 1. Before the first experiment

Five checks. Skip one and you find out twenty minutes into a paid round.

**1. Datasets resolve.** The bench needs five recordings: `go2_short`,
`go2_china_office`, `go2_agentic_20260819`, `go2_stairs_20260819`,
`go2_teleop_20260819`. Confirm free of charge:

```bash
uv run python -m dimos.evals.temp.tool_evo_bench --dry-run
```

Expect `122 cases | 0 errors` and 14 families. A missing dataset exits 2 with
`BENCH INFRA FAILURE` naming it.

**2. Record the floors.** A real, paid bench run (~122 calls).

```bash
uv run python -m dimos.evals.temp.tool_evo_bench --write-floors
```

**3. Note the holdout score.** It becomes `--min-score` on the holdout gate.

```bash
uv run python -m dimos.evals.temp.tool_evo_bench --slice holdout
```

**4. Freeze the manifest — last, after every other edit.**

```bash
uv run python -m dimos.evals.temp.tool_evo_gate freeze
uv run python -m dimos.evals.temp.tool_evo_gate static   # must pass
git add dimos/evals/temp/evo_floors.json dimos/evals/temp/evo_frozen.json && git commit
```

Any later change to a suite, scorer, or harness file means re-running `freeze`,
or `static` fails every experiment.

**5. Blind ablation, once.** Only when the *questions* changed — no encoder can
move a blind score, so this is not a per-experiment gate.

```bash
uv run python -m dimos.evals.temp.tool_evo_bench --blind
```

---

## 2. Set up the workspace

### The normal path

```
/evo:discover optimize PointCloud2.agent_encode() against the pointcloud VQA suites
```

`discover` asks what to optimize, the benchmark command, and the metric
direction — seed the answer in the invocation to skip the questions. It
initializes the workspace and starts the dashboard, printing the URL in chat.

Our benchmark already exists in the repo, so **gates are opt-in** — `discover`
only auto-attaches a held-out-slice floor when it builds a benchmark from
scratch. Add ours by hand (section 3).

### The explicit path

Equivalent, and what [autoresearch.md](autoresearch.md) documents. Use it if
you would rather state the wiring than answer questions:

```bash
evo init --name pointcloud-encode --host claude-code \
  --target dimos/msgs/sensor_msgs/PointCloud2.py \
  --benchmark "python -m dimos.evals.temp.tool_evo_bench" \
  --metric max --per-exp-timeout 3600
```

### Two settings this repo actually needs

Experiment worktrees are fresh git checkouts. **They have no `.venv` and no
`.env`.** Both must come through evo rather than being baked into the benchmark
string:

```bash
evo config runtime set --prefix "uv run"    # prepends benchmark + gate commands
evo env inherit-shell on                    # or: evo env load .env --all
```

`--prefix` is why the `--benchmark` above says `python -m ...` and not `uv run
python -m ...`. Env values resolve fresh on each `evo run`; the config stores
key names, not secret values. The bench needs a working model credential or
every case errors to 0.0 and the branch looks dead rather than broken.

---

## 3. Gates

Pre-gates decide from the worktree alone, so a candidate that rewrites the
questions or bloats the encoding fails before a model call is paid for.

```bash
evo gate add exp_0000 --phase pre --name static \
  --command "python -m dimos.evals.temp.tool_evo_gate static"
evo gate add exp_0000 --phase pre --name budget \
  --command "python -m dimos.evals.temp.tool_evo_gate budget"
evo gate add exp_0000 --phase post --name floors \
  --command "python -m dimos.evals.temp.tool_evo_gate floors"
evo gate add exp_0000 --phase post --name holdout \
  --command "python -m dimos.evals.temp.tool_evo_bench --slice holdout --gate --min-score <step-3-number>"
evo gate list exp_0000
```

Two draws of the same seed differed by 0.02 on the holdout score and by up
to 0.2 on a ten-row family, so set `--min-score` below the seed's draw (round
3: 0.50 against 0.55 / 0.54) and keep `floors` to the frozen families only —
`--write-floors` records every family, including the ones under optimization.

Gates are node-scoped and **inherit to every descendant**, so registering them
on the baseline covers the whole tree. Pass/fail is exit-code only — a command
that prints a bad score and exits 0 passes.

Validate the wiring. Both run the post gates too, and `--check` runs the
benchmark itself — a full paid bench plus the holdout gate, ~218 calls:

```bash
evo run exp_0000 --check
evo gate check exp_0000       # runs all gates regardless of phase, mutates nothing
```

---

## 4. Run

```
/evo:optimize
```

Round width, per-branch depth, and the stall limit are the three knobs:

```
/evo:optimize subagents=3 budget=2 stall=5
```

The loop stops on its own after **stall** consecutive rounds with no
improvement (default 5), or when you interrupt it.

**Two behaviors are on by default.** The skill arms both at startup:

- **autonomous** — the stop-nudge. At every turn boundary the orchestrator is
  re-prompted to keep driving until the stall limit. Without it the loop does
  not continue across turn boundaries.
- **subagents-only** — a deny-gate nudging orchestrator edits toward subagents.

You do not need a flag to change either. **Say it in plain language** and the
skill resolves it at startup: "review each round before continuing", "one round
then stop", "check in with me" → autonomous off. "you can edit directly" →
subagents-only off. A run with a behavior off announces it in the opening
message.

### Cost

One experiment is **122 paid calls** for the benchmark plus **25** for the
holdout gate — roughly 25 minutes. `static`, `budget` and `floors` are free.
Three subagents over ten rounds is on the order of a hundred benchmark runs.
Agree that number before starting, not after.

---

## 5. Watch progress

| Question | How |
|---|---|
| What happened / what improved? | `/evo:report` |
| Is the score climbing? | `evo report --watch 10` |
| One-liner: metric, best, counts | `evo status` |
| Bounded digest of current state | `evo scratchpad` |
| Full tree | `evo tree` |
| Everything, in a browser | the dashboard |

`/evo:report` is a read-only skill: the scatter plot in chat, best and frontier
candidates, what landed overnight. It never runs benchmarks or gates.

**The dashboard starts itself** with `/evo:discover` (or `evo init`) and prints
its URL. It is supervised — respawned on crash with backoff. If it is not up:

```bash
cat .evo/dashboard.port     # actual bound port; 8080 may have been taken
cat .evo/dashboard.dead     # written only when the supervisor gave up
tail .evo/dashboard.log     # the traceback
```

### Drilling into one experiment

```bash
evo show exp_0007            # status, score, attempts, gates, lineage — one JSON blob
evo diff exp_0007            # what it changed, vs parent
evo diff exp_0007 exp_0004   # ...or against another node
evo traces exp_0007          # every case
evo traces exp_0007 go2_agentic_20260819_doorway_t400   # one case
evo path exp_0007            # root-to-node chain
evo annotations --exp exp_0007
```

Each case trace carries its score, the expected value, and the model's actual
answer truncated to 120 chars. First place to look when a family moves.

For scanning *across* experiments, read the attempt artifacts directly —
`outcome.json` and `traces/task_*.json` under
`.evo/run_*/experiments/<exp>/attempts/<NNN>/`. Bulk reads beat N subprocesses;
`evo show` is for one node.

### Decisions waiting on you

```bash
evo awaiting                 # evaluated, not yet committed or discarded
evo frontier                 # nodes ranked by the configured strategy
evo discards --like "voxel"  # what was thrown away, searchable
```

### Outcome vocabulary

| Outcome | Meaning |
|---|---|
| `COMMITTED` | score improved and gates passed — node kept |
| `EVALUATED` | ran, but score regressed or a gate failed — retry or discard |
| `FAILED` | infra/runtime crash — **does not consume retry budget** |

### Reading the score

Every run also writes `.evo_bench/<slice>.json` with `score`, per-case `tasks`,
`families`, and `families_positive`.

**For `doorway`, `rooms`, `floorlevel` and `stairs`, read the positive-half
mean.** Each is three positive rows and three negative ones, and a model
answering "none" six times scores 0.50 having seen nothing. The bench prints it
as `(positive N)` beside any family whose two means differ. A family mean that
rises while its positive mean sits still means the encoder made the model more
timid, not more capable.

Those four also have **no holdout**. Nothing verifies a gain on them left the
frames it was found on. Treat one as a hypothesis.

---

## 6. Steer a run without stopping it

```bash
evo direct "stop widening; go deeper on the free-space channel"
evo direct exp_0007 "the budget gate is the constraint, not the score"
evo direct "..." --wait          # block until a session acks
```

Agents receive it as a banner and treat it as a new user turn. This is the
right tool for "not that direction" — it does not cost you the round.

---

## 7. Pause, stop, resume

Nothing is lost by stopping: the graph, scores, traces and annotations are all
on disk.

### Stop one experiment

```bash
evo abort exp_0007              # SIGTERM the driver AND its subprocess tree
evo abort exp_0007 --force      # SIGKILL, no grace period
```

`abort` does **not** mutate graph state. Park the node afterward so the loop
knows what happened:

```bash
evo discard exp_0007 --reason "..." --failure-class hypothesis
```

`--failure-class` is `build` (the artifact step broke → fix and retry), `eval`
(artifact fine, scoring wrong → retest without rebuilding), or `hypothesis`
(ran clean, did not help → branch elsewhere). It routes reuse vs. branch, so it
is worth setting honestly.

### Pause the loop

Best: ask for it up front — "one round then stop", "check in with me each
round". Mid-run, disarm the nudge:

```bash
evo autonomous off
```

The orchestrator then stops naturally at the next turn boundary: finishes the
round, reports, stops.

### Halt everything

```bash
evo exit-optimize-mode
```

Clears optimize-mode and both opt-in flags, **discards any `active`
experiments**, reports orphan `evo run` PIDs, and prints the remaining halt
steps. It cannot reach the host runtime, so **you must still stop running
subagents yourself** (TaskStop on Claude Code).

That active-experiment discard is destructive. If you only want to pause, use
`evo autonomous off`.

### Resume

```bash
evo run exp_0007          # re-runs; a dead PID from the prior attempt is reclaimed
```

To resume the *search* rather than one node, re-invoke `/evo:optimize`.

`evo run` refuses a second attempt while another has a live driver PID —
concurrent attempts multiply spend by N. When you know the prior driver is gone
but the state was not reclaimed (recycled PID):

```bash
evo run exp_0007 --force
```

### What stopping costs

The in-flight benchmark's model calls. A 122-call bench is ~25 minutes; abort
halfway and you re-pay it. To stop at a clean boundary instead:

```bash
evo wait --for experiments --count 1 --timeout 2h
```

Exit 0 on match, 124 on timeout.

---

## 8. Housekeeping

```bash
evo prune exp_0007 --exhausted        # close a branch, keep its result valid
evo prune exp_0007 --invalid --yes    # exclude it and its descendants from best/frontier
evo restore exp_0007                  # undo a prune or discard
evo gc                                # reclaim worktree disk from finished nodes
evo note "..."                        # workspace-level observation
evo notes --limit 20
evo config show                       # redacted config dump
evo config set per-exp-timeout 3600
evo config set frontier-strategy epsilon_greedy
```

Frontier strategies: `argmax`, `top_k`, `epsilon_greedy`, `softmax`,
`pareto_per_task`. Also settable in the dashboard's Frontier tab.

Never hand-edit `.evo/*.json` — advisory locks exist and the dashboard may be
writing concurrently.

---

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Every experiment fails `static` | a suite/scorer/harness file changed since the freeze | `tool_evo_gate freeze`, commit the manifest |
| Every case errors, score 0.0 | worktree has no model credential | `evo env inherit-shell on` or `evo env load .env --all` |
| Benchmark cannot find the interpreter | worktree has no `.venv` | `evo config runtime set --prefix "uv run"` |
| Bench exits 2, `BENCH INFRA FAILURE` | dataset missing or a suite will not import | read the message, it names it — this is not a scoring event |
| `floors` fails: "families missing from the run" | run did not include the frozen suite | check `--slice`; `bench` is train + frozen |
| Score jumps, no capability gain | the family mean rose on the negative half | compare `families_positive` in `.evo_bench/bench.json` |
| `budget` gate fails | encoder got bigger or slower than 6 kB p95 / 80 ms | `tool_evo_gate budget --report` prints every sampled frame |
| Experiment hangs past its timeout | wedged driver | `evo abort <exp>`, then `evo run <exp>` |
| `evo run` refuses: concurrent attempt | stale PID | `evo run <exp> --force` |
| Loop stalls after one round | autonomous not armed, or `default-orchestrator` stuck on `workflow` | `evo autonomous on`; `evo config set default-orchestrator prose` |
| Dashboard gone | supervisor gave up | `cat .evo/dashboard.dead`, `tail .evo/dashboard.log` |

---

## 10. Land the winner

```
/evo:ship
```

Distills the best-scoring experiment down to the minimal diff, opens a PR when
the repo has a remote, and attaches a mergeability report.

Whatever route you take, the PR needs four things:

1. One squashed commit of `PointCloud2.py` alone.
2. The `exp_NNNN` hypotheses in the PR body as the lineage.
3. A baseline-vs-winner table **per family** — train, holdout and frozen — with
   the positive-half column beside the family mean for the four hand-authored
   families.
4. A plain statement of which families have no holdout backing their gain.

Then delete `dimos/evals/temp/`. What comes out with it: the slice tag in
`generate.py`, the `split.assign(...)` calls in the three sliced suites, and the
`train` tag in the four hand-authored ones. After that the suites run every row,
as they did before.
