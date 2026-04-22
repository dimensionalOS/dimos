# go2_cpu autoresearch

Karpathy-style autonomous research loop adapted for DimOS: minimize the CPU
footprint of a Go2 basic-blueprint replay run. Modeled on
[karpathy/autoresearch](https://github.com/karpathy/autoresearch).

## Setup

This directory is tracked on the `ruthwik/autoresearch` branch, so each
experiment is a real git commit. Keep via commit, discard via
`git reset --hard HEAD~1` (Karpathy's original pattern).

1. **Confirm branch**: you should be on `ruthwik/autoresearch` or a sub-branch
   like `ruthwik/autoresearch/expN`. Never run the loop on `dev` or `main`.
2. **Environment**: use `test-venv`, not `.venv`. See
   [GUIDELINES.md](GUIDELINES.md) for details. All `dimos` invocations must
   resolve to `test-venv/bin/dimos`.
3. **Read the in-scope files** (the directory is small):
   - `eval.py` — runs the replay, samples the process tree with psutil,
     parses `user+sys`, appends a record to `results.jsonl`. **Do not
     modify.** This is the ground-truth metric harness.
   - `optimizations.py` — the one file you edit. Returns
     `{cli_args, env, startup_code}`, which `eval.py` splices into the replay
     subprocess.
   - `GUIDELINES.md` — environment + DimOS hooks reference.
4. **Verify the replay dataset exists**: `ls data/unitree_go2_bigoffice/`
   from the dimos repo root must show `lidar/`, `odom/`, `video/`. If not,
   the data lives at `data/.lfs/unitree_go2_bigoffice.tar.gz` — tell the
   human to extract it.
5. **Record a clean baseline**: confirm `optimizations.py::apply()` is the
   baseline you want (empty `{}` for a true zero-optimization baseline, or
   the current state if continuing a prior session). Commit it:
   `git commit -am "baseline"`.
6. **Confirm and go.**

## Experimentation

Each experiment runs the full Go2 replay blueprint **once to EOF** (fixed
work, not fixed time — see `--exit-on-eof` in `GUIDELINES.md`). Launch as:

```bash
bash run.sh                      # full run
bash run.sh --skip-profile       # skip py-spy profiling step
```

**What you CAN do:**
- Edit `optimizations.py` — the primary knob file. The `apply()` function
  returns a dict with:
  - `cli_args` — extra CLI flags inserted before the `run` subcommand
    (e.g. `--n-workers=2`, `--log-level=WARNING`).
  - `env` — extra env vars for the subprocess (e.g. `OMP_NUM_THREADS=1`).
  - `startup_code` — Python code written to `_patches/sitecustomize.py` and
    injected via `PYTHONPATH`. Runs before `dimos` imports — monkey-patch
    module-level constants, thread-pool sizes, poll timeouts, etc.
- Make **surgical DimOS source edits** gated behind env vars so that
  `optimizations.py` can toggle them. Source edits must be:
  - Small (< 20 lines per file)
  - Gated behind `os.environ.get("DIMOS_*")` checks (no-op when unset)
  - Limited to the `unitree-go2-basic` replay path
  - Non-breaking for live robot / rerun / other blueprints

**Existing source-level gates (already wired):**
- `DIMOS_SKIP_WEBSOCKET_VIS=1` — skip WebsocketVisModule in blueprint
- `DIMOS_SKIP_CLOCK_SYNC=1` — skip ClockSyncConfigurator in blueprint
- `DIMOS_LAZY_ASYNCIO=1` — lazy asyncio loop creation in ModuleBase
- `DIMOS_REPLAY_PREFETCH=N` — prefetch N pickle files in background thread

**What you CANNOT do:**
- Modify `eval.py` — it's the ground-truth harness.
- Install new packages or add dependencies.
- Make source edits that aren't gated behind env vars.
- Weaken the metric (skip the run, fake the timing output).
- Commit to `main` or `dev`. Stay on `ruthwik/autoresearch` (or a
  sub-branch). Never force-push.

**Goals (multi-score, tracked independently):**
- **Primary:** minimize `SCORE = user + sys` CPU seconds
- **TTFM:** minimize time to first message (startup latency)
- **PEAK_MEMORY_MB:** minimize peak RSS across process tree
- **CTX_SWITCHES:** minimize context switches (thread/process overhead)

All over one full replay pass, without breaking the LCM pipeline.

## Known harness limitations

Work around them, don't exploit them:

1. **Validation is a stub.** `validation: PASS` is printed on any clean
   exit. There is no real LCM-message-count or payload diff against a
   baseline recording yet. **You can silently break the pipeline and the
   harness will not notice.** Always sanity-check: threads, CPU%, and I/O
   shouldn't collapse to zero; a "PASS" with zero I/O means the pipeline
   is dead.
2. **Single run, no variance gate.** One sample per candidate. A <5% delta
   is likely noise. Aim for ≥10% improvements, or run the same config 3×
   manually and compare the median.
3. **Profile is a stub.** `run_profile()` writes a placeholder unless
   `py-spy` is on PATH. For real profiling:
   `py-spy record -o profile.svg -d 60 -- test-venv/bin/dimos --replay \
     --viewer=none --exit-on-eof --replay-dir=unitree_go2_bigoffice \
     run unitree-go2-basic`

## Output

Each `eval.py` run prints a summary to stdout and appends one JSON line to
`results.jsonl`:

```
==================================================
SCORE: 45.23
TTFM: 8.50
VALIDATION: PASS
REAL: 12.34
USER: 30.10
SYS: 15.13
PEAK_CPU_PCT: 450.0
PEAK_MEMORY_MB: 2600.0
AVG_THREADS: 351.9
IO_READ_MB: 527.6
IO_WRITE_MB: 12.3
CTX_SWITCHES: 150000
==================================================
```

`results.jsonl` is **tracked** — commit it with each experiment. One line
per run; never overwrite past lines.

Quick leaderboard queries:

```bash
jq -c '{commit, score, status, desc}' results.jsonl | tail -20
jq -c 'select(.status=="keep") | {score, desc}' results.jsonl
```

## The experiment loop

**LOOP FOREVER:**

1. Look at state: `git log --oneline -20` and
   `jq -c '{commit,score,status,desc}' results.jsonl | tail -20` to recall
   what's been tried.
2. Form one hypothesis. Prefer hypotheses grounded in a profile
   (`profile.svg` / `profile_output.txt`) or known hot paths (see list
   below). Avoid random hyperparameter jiggling.
3. Edit `optimizations.py` to implement the hypothesis.
4. Run: `bash run.sh --skip-profile`.
5. Grep the score from stdout or read the tail of `results.jsonl`.
6. If the run crashed or hung, inspect the log tail. If obviously fixable
   (typo, missing import), fix and re-run. Otherwise log `status=crash`
   and move on.
7. If `SCORE < best_score`:
   - Sanity-check secondaries (`AVG_THREADS`, `PEAK_MEMORY_MB`,
     `IO_READ_MB`) haven't collapsed to zero — a broken pipeline can still
     "PASS".
   - Verify `TTFM` and `PEAK_MEMORY_MB` haven't regressed significantly.
   - If sane: `git commit -am "<one-line hypothesis> score=<X>s ttfm=<Y>s"`,
     mark the record `status=keep`.
8. If `SCORE >= best_score`: revert the edit
   (`git checkout -- optimizations.py` if not yet committed, or
   `git reset --hard HEAD~1` if committed), mark `status=discard`.
9. Go to 1.

**Never** `git reset --hard` past the baseline commit. **Never** force-push.

**Timeout per experiment:** A full EOF replay is ~4 minutes wall. Harness
kills at 5 minutes (`DEFAULT_TIMEOUT = 300s`). If a single run exceeds
that, it's treated as a hang/crash.

**NEVER STOP.** Once the loop has begun, do not pause to ask "should I keep
going?". The human is asleep or away and expects you to iterate
indefinitely until manually stopped. If you run out of ideas, re-read the
hot-paths list below, re-read `dimos/protocol/service/lcmservice.py` and
`dimos/protocol/rpc/pubsubrpc.py`, look at
`dimos/robot/unitree/go2/connection.py` for daemon threads, and combine
previous near-misses.

## Known hot paths

**Monkey-patchable (knobs 1-6 in optimizations.py):**
- `lcmservice._lcm_loop` — 50ms polling timeout, runs per module instance
- `ModuleBase.__init__` — creates 50-worker thread pool per module
- `connection.publish_camera_info` — 1 Hz busy-wait loop in daemon thread
- BLAS/OpenMP thread counts — numpy/torch spawn per-core threads
- Log verbosity — INFO-level formatter CPU + stdout volume

**Source-gated (knobs 7-10, env-var controlled):**
- WebsocketVisModule — full web server (Uvicorn + SocketIO) with zero clients
- ClockSyncConfigurator — NTP sync during replay (no real robot)
- Per-module asyncio event loop — each module spawns its own loop thread
- LegacyPickleStore sequential I/O — one pickle file at a time, no prefetch
- LCM serialization/deserialization overhead

## Starting point

Re-baseline with a clean `optimizations.py` (empty `apply()` returning
`{}`) as your first action on any new session. Prior run history is not
comparable across harness changes.

## Results so far (2026-04-14)

Baseline (empty knobs) vs optimized:

| Metric | Baseline | Optimized | Change |
|---|---|---|---|
| SCORE | 24.11s | 2.04s | **-91.6%** |
| REAL | 302.39s | 11.06s | **-96.3%** |
| RSS | 1399 MB | 880 MB | **-37.1%** |
| Threads | 237 | 35 | **-85.2%** |
| CTX switches | 13587 | 367 | **-97.3%** |

Top wins by impact:
1. `DIMOS_SKIP_SENSOR_PUBLISH` — skip LCM encode+broadcast of unused lidar/video (-48.7%)
2. `DIMOS_REPLAY_SPEED=1000` — compress wall time, removes idle poll overhead (-45.6%)
3. `DIMOS_REPLAY_ONLY` — skip WebRTC/aiortc/sounddevice imports (-9.7%, -95MB RSS)
4. `DIMOS_SKIP_TF` — skip 300Hz TF transform publishing during replay (-6.6%)
5. BLAS pinning (OMP/MKL/OPENBLAS=1) — stop libs from spawning per-core threads (-57% threads)
6. Lazy open3d import — avoids sklearn import chain (import 2.9s→1.5s)
7. Lazy rerun import in Image — rerun spawns 16 threads at import
