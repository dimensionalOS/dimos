# T16 — Per-run directories (counter-named, one system for logs + debug + health)

Status: **done** (2026-07-21). Owner: session.
Motivating incident: successive `go2_nav` runs appended into one shared
`logs/debug.db` — replaying the same recording writes identical data-time ts
into the same streams, silently entangling runs (found 2026-07-21 within an
hour of T15 landing).

## What already exists (reuse, don't fork)

- `logging_config.set_run_log_dir(dir)` — routes main.jsonl, migrates live
  FileHandlers, exports `DIMOS_RUN_LOG_DIR` (forkserver workers inherit).
- `dimos run` CLI already mints `LOG_DIR/<run_id>` via `generate_run_id()` +
  sets `DIMOS_RUN_ID` for watchdog/stale-run cleanup (`robot/cli/dimos.py:345`).
- T15 debug.db resolution already prefers `$DIMOS_RUN_LOG_DIR/debug.db`.

The gap: direct entrypoints (`python -m dimos.pure.modules.go2_nav`, example
mains) never mint — they fall back to flat `logs/` + one shared `debug.db`.

## Design

**One minting function** (`dimos/utils/rundir.py` or into logging_config):

```python
def mint_run_dir(label: str) -> Path:
    """Claim LOG_DIR/NNN_label (next counter), set_run_log_dir it, repoint latest."""
```

- **Naming: `NNN_label`** — zero-padded counter + short label:
  `logs/001_go2-nav-pure/`, `002_unitree-go2/`. Counter = max existing NNN
  prefix + 1; claim via `mkdir(exist_ok=False)` retry loop (concurrent-safe).
  Counter gives Ivan "run 47" ergonomics; label answers "which was that";
  wall time lives on the files inside.
- **`logs/latest` symlink** repointed atomically at mint (`ln -sfn`
  semantics) — `dimos pure debug logs/latest`, `pm.debug_latest()` reads it
  (fallback: scan for highest NNN).
- **Contents of a run dir**: `main.jsonl` + per-worker jsonl (existing
  mechanism), `debug.db` (T15 decisions/rows/state + T9 health), future
  run-scoped artifacts (`.rrd` saves, checkpoints).
- **`GlobalConfig.run_dir: str | None = None`** — explicit override
  (absolute, or relative to LOG_DIR); None → auto-mint. Overriding to an
  existing dir intentionally resumes/appends into it.

**Who mints:**
- `dimos run` — switch `generate_run_id()` naming to the counter scheme
  (keep `DIMOS_RUN_ID` tagging as-is).
- Entrypoint mains (`go2_nav.main`, example runners) — mint at startup,
  BOTH `--pure` and deploy modes.
- Library calls (`m.over()` in tests/REPL) — NEVER mint implicitly.
- **debugrec fallback change**: when capture is requested and resolution
  would land on a SHARED location (no run dir anywhere), mint instead of
  appending — the "no silent cross-run mixing" invariant. Tests keep
  isolating via `Debug(db=tmp)` / cleared env (T9-wave fixture).

**Retention** (follow-up, not this wave): `dimos runs list` / `dimos runs
prune --keep N`; counters make both trivial.

## QUESTIONS

QUESTION 1: dir name — `NNN_label` or bare `NNN`?
DEFAULT: `NNN_label` (label = blueprint/entrypoint slug).
OPTIONS: a) NNN_label (default); b) bare NNN — purest counter, `meta.json` inside carries the label; c) NNN_label_YYYYMMDD — redundant with file mtimes, noisy.

QUESTION 2: counter scope — global across all labels (001 unitree, 002 pure-eval, ...) or per-label?
DEFAULT: global (one ordered timeline of runs, `latest` is unambiguous).
OPTIONS: a) global (default); b) per-label — nicer per-workflow numbering, but two "003"s exist and `latest` needs a tiebreak.

QUESTION 3: does `--pure` eval mint by default, or only when debug/health capture is on?
DEFAULT: always mint in entrypoint mains (a run dir with one main.jsonl is cheap; consistency beats sparing empty dirs).
OPTIONS: a) always (default); b) only when capturing — fewer dirs, but logs stay flat for non-captured runs and the layout forks.

## Triage resolutions (Ivan, 2026-07-21): ALL THREE AT DEFAULT

Q1 `NNN_label`; Q2 global counter; Q3 entrypoint mains always mint.
Implementation is GO.

## Implementation notes (2026-07-21)

- **`dimos/utils/rundir.py`** — `mint_run_dir(label)` + `slugify(label)`.
  Counter = one past the highest `^\d{3,}_` prefix under `LOG_DIR`; claimed via
  a `mkdir(exist_ok=False)` retry loop (a losing racer increments and retries),
  zero-padded 3 (rolls to 4+ digits naturally past 999). `set_run_log_dir(dir)`
  routes `main.jsonl` + exports `DIMOS_RUN_LOG_DIR`; `LOG_DIR/latest` is
  repointed atomically (`os.symlink` to a `.latest.<pid>.tmp` then
  `os.replace` — no dangle window). The symlink target is relative when the run
  dir is a direct child of `LOG_DIR`. `GlobalConfig.run_dir` override:
  absolute used verbatim, relative resolved under `LOG_DIR`, existing dir
  appended into (`mkdir(parents=True, exist_ok=True)`).
- **`GlobalConfig.run_dir: str | None = None`** added.
- **Minting sites**: `dimos run` (`robot/cli/dimos.py`) mints the counter DIR
  via `mint_run_dir(blueprint_name)`; `DIMOS_RUN_ID` stays an independent opaque
  tag from `generate_run_id()` (see coupling note below). `go2_nav.main()` mints
  at startup with label `go2-nav-pure` / `go2-nav`, BOTH modes, before build.
- **debugrec fallback** (`_resolve_db`): the old final branch landed on a shared
  `logs/debug.db`; it now mints a run dir labelled `debug` — the
  no-silent-cross-run-mixing invariant. `Debug(db=...)` and `$DIMOS_RUN_LOG_DIR`
  are unchanged and mint nothing. `latest()` now resolves `LOG_DIR/latest`
  first, then the highest `NNN_*` dir with a debug.db, then legacy flat-mtime.
  `pm.debug_latest` re-exports it.
- **generate_run_id / watchdog coupling**: `DIMOS_RUN_ID` is only ever an opaque
  tag matched by `kill_run_processes` / the watchdog against descendants'
  environ — its format is never parsed (the only format assertions live in
  `test_daemon` testing `generate_run_id` directly). So the run id is kept
  independent of the dir name: the dir gets the counter, the id keeps its
  timestamp format. No watchdog change needed.
- **Tests**: `dimos/utils/test_rundir.py` (21) — counter increment/global,
  slugify, concurrent claim (8 threads → distinct dirs), latest atomic repoint,
  run_dir override (abs/rel/append), rollover past 999, debugrec mint-fallback +
  explicit-db/env untouched, `latest()` symlink-first then highest-NNN.
  Isolated by monkeypatching `rundir.LOG_DIR` + resetting logging_config
  globals/env (never writes the real `logs/`).
