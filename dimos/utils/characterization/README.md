# Twist Characterization Harness

Measures how a mobile base's body velocity tracks a commanded Twist. Built
for the Go2 (WebRTC + mujoco), adapter-agnostic underneath. Not registered
as installed CLI; invoke with `python -m`. Promote to `[project.scripts]`
after team review.

## Run a session

One coordinator boot, many recipes, pauses between each. Keyboard teleop
is a sibling module — WASD/QE drives during pauses, releases yield to
the next recipe.

```bash
python -m dimos.utils.characterization.scripts.run_session \
    --recipes "dimos.utils.characterization.experiments:e1_vx_pos_1p0:3" \
    --surface carpet --notes "first E1 pilot" \
    --out-dir ~/char_runs --randomize --rng-seed 1
```

Flags: `--simulation`, `--backend mock`, `--auto`, `--no-teleop`, `--rage`, `-v`.

## Run layout

```
session_YYYYMMDD-HHMMSS/
    session.json                 # plan + run completion state
    recording.db                 # ONE memory2 DB; sliced per run via ts_window_wall
    000_step_vx_1.0_r1of3/
        run.json                 # run_id, recipe, clock_anchor, bms, ts_window_wall
        cmd_monotonic.jsonl      # one line per commanded sample
```

`run.json.clock_anchor` ties monotonic ↔ wall clocks. Don't compare
`tx_mono` to `obs.ts` directly.

## Recipe

```python
from dimos.utils.characterization.recipes import TestRecipe, step
my_step = TestRecipe(name="step_vx_0.5", test_type="step", duration_s=3.0,
                    signal_fn=step(amplitude=0.5, channel="vx"))
```

Helpers in `recipes.py`: `step`, `ramp`, `chirp`, `constant`, `composite`.
`signal_fn: (t) -> (vx, vy, wz)`.

## Pipeline

```bash
SESSION=~/char_runs/session_<ts>
python -m dimos.utils.characterization.scripts.process_session validate    $SESSION
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done
python -m dimos.utils.characterization.scripts.process_session aggregate   $SESSION
python -m dimos.utils.characterization.scripts.process_session deadtime    $SESSION   # E8
python -m dimos.utils.characterization.scripts.process_session coupling    $SESSION   # E7
python -m dimos.utils.characterization.scripts.process_session envelope    $SESSIONS... --mode default --out envelope_default.md
python -m dimos.utils.characterization.scripts.process_session compare-modes --default <s>... --rage <s>... --out compare.md
```

Each step writes derived artifacts; nothing destructive. Per-step rationale
in `processing/<name>.py` docstrings.

## Velocity reconstruction

Go2 odom is world-frame `(x, y, yaw)`. Pipeline: Savitzky-Golay → central
difference → rotate by `-yaw`. Use **raw** for edge timing (rise time,
deadtime, FOPDT); **cleaned** only for plots and aggregate means.

## Gotchas

- Coordinator does NOT control; velocity task is pass-through.
- Mujoco odom ~50 Hz, real Go2 ~10 Hz — absolute timings don't transfer sim↔real, shapes do.
- BMS is real-robot only.
- `TransportTwistAdapter.read_velocities()` echoes last commanded.
