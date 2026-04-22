# Autoresearch Guidelines

## Scope

Autoresearch targets **current dev**
env-var names, `pip install -e .` between refs, version sniffing). That file
benchmarks across history; the autoresearch harness optimizes current HEAD.

## Environment

- **Always use `test-venv`, never `.venv`** for running, installing, or
  profiling anything under `myprojects/autoresearch/`. Applies to every shell
  command (`uv run`, `pip install`, `python`, `py-spy`, etc.) and every
  subprocess spawned by the harness.
- Activate explicitly: `source test-venv/bin/activate`, or invoke directly:
  `test-venv/bin/python …`.
- Harness subprocesses (e.g. `eval.py` → `dimos …`) must resolve to
  `test-venv` — set `VIRTUAL_ENV=test-venv` / prepend `test-venv/bin` to
  `PATH` in the env passed to `subprocess.Popen`. Don't rely on caller's
  ambient activation.

## Current DimOS hooks for autoresearch

State as of this doc — update if DimOS adds/renames flags.

- **`--exit-on-eof`** ([dimos/core/global_config.py](dimos/core/global_config.py)):
  replay streams run once and signal shutdown at EOF instead of looping.
  Wired via `GlobalConfig.exit_on_eof` → `ReplayConnection(..., exit_on_eof=...)`
  → `stream(loop=False)`. `GO2Connection.start()` subscribes to all three
  replay streams' `on_completed` and, when all complete, sends `SIGINT` to
  `DIMOS_MAIN_PID` to break `coordinator.loop()`. **Use this — it's what
  makes benches fixed-work instead of timeout-killed.**

- **`--viewer=none`**: the rerun/foxglove workaround in `perf_bench.py`
  (forcing via `VIEWER` / `VIEWER_BACKEND` env vars) is **no longer needed**.
  The CLI flag works correctly — CLI applies `cli_config_overrides` to
  `global_config` *before* importing blueprints, so the top-level
  `if global_config.viewer == …` branch in `unitree_go2_basic.py` sees the
  override. `RerunBridgeModule` is not deployed when `viewer=none`.

- **`DIMOS_MAIN_PID`** env var: set by `dimos run` CLI to its own PID. Worker
  processes use this to signal the coordinator for graceful shutdown (survives
  forkserver/multi-level fork layers where `getppid()` would be wrong).

## Canonical benchmark invocation

```bash
test-venv/bin/dimos --replay --viewer=none --exit-on-eof \
    --replay-dir=unitree_go2_bigoffice \
    run unitree-go2-basic
```

Target the **basic** blueprint (`unitree-go2-basic`), not the smart one
(`unitree-go2`). The smart blueprint adds VoxelGridMapper, CostMapper,
ReplanningAStarPlanner, WavefrontFrontierExplorer, PatrollingModule — each
adds CPU noise unrelated to the replay pipeline we're optimizing, and
PatrollingModule also has a graceful-stop bug that inflates shutdown wall
time by 5s. Basic is the minimum that exercises the LCM / RPC / replay
stack.

This runs one full replay pass and exits cleanly with `user+sys` reflecting
CPU-per-fixed-workload rather than CPU-per-wall-second.

## Known rough edges (as of current dev)

- `stop_timer` attribute error on `ReplayConnection.stop()` — patched via
  a no-op `stop()` override at
  [dimos/robot/unitree/go2/connection.py:142](dimos/robot/unitree/go2/connection.py#L142).
  If the error reappears in a run log, that patch was reverted.

## What's still missing for a real autoresearch harness

In priority order:

1. Pipeline recorder + `baseline_record.json` diff → real validation
   (currently stubbed `PASS`).
2. `py-spy record` integration (replace `run_profile()` placeholder).
4. Promote monkeypatched knobs (`lcm_loop_timeout_ms`,
   `rpc_thread_pool_max_workers`, `camera_info_publish_hz`) to
   `GlobalConfig` fields so optimizations become declarative config instead
   of `sitecustomize.py` injection.

`--exit-on-eof` (#1 prerequisite for any of the above being meaningful) is
now done.
