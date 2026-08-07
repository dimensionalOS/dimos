# referee: the motion-planner benchmark

Standalone referee for the dimos local planner: ground-truth 2D worlds, an
SE(2) gold oracle, and a judge that scores candidate plans against exact truth.
The candidate it was built around is the Rust crate one level up
(`planner/rust/`, pyo3 module `dimos_motion2_target`).

The package depends on **numpy + scipy + pydantic only** (rerun optionally,
for `--view`). All internal imports are relative, so copying this directory
anywhere yields a working referee — that is the basis of the lab export
([`../research/auto/export/`](../research/auto/export/)), which packages the
benchmark for evo autoresearch loops. Keep both properties when editing here:
they are what makes a lab possible.

## Quick start (in-repo)

```bash
# build the latest candidate planner and score the full battery
uv run python -m dimos.navigation.motion.planner --build --score --gen 40

# visual check in rerun (writes sim2d.rrd; --spawn opens a live viewer)
uv run python -m dimos.navigation.motion.planner --view -s gen028

# machine-readable per-world results
uv run python -m dimos.navigation.motion.planner --json --gen 40 | jq .summary

# parallel battery: N worker processes, one pinned core each
uv run python -m dimos.navigation.motion.planner --score --gen 40 --jobs 8

# reference planners
uv run python -m dimos.navigation.motion.planner --score --planner gold
uv run python -m dimos.navigation.motion.planner --score --planner target-py

# tests: the gold oracle must survive its own judge
uv run pytest dimos/navigation/motion/planner/referee -q
# optional behavioral tests of the rust crate (not a harness gate)
cargo test --release --no-default-features --manifest-path \
    dimos/navigation/motion/planner/rust/Cargo.toml
```

Always enter through `python -m dimos.navigation.motion.planner` (the
package `__main__`): it pins the BLAS thread pools before numpy loads, which
the CPU-time speed measurement needs to be stable (measured: identical-code
score spread 0.334 → 0.0069).

## Scoring

`score_world` = `gate * (100*gold + 10*consistency + 1*speed)`, max 111 per
world; the battery score is the mean. Priorities are lexicographic by
magnitude: speed never buys back deviation, nothing buys back a non-vetoed
collision (gate 0). Speed is `min(1, 20ms / avoid_ms)` measured in
`time.process_time()` — CPU time, so a multi-threaded candidate is charged
all of its threads and gains nothing.

## Timing model

- Serial (`--jobs 1`, default): the process pins itself to one core.
- Parallel (`--jobs N`): each worker process pins itself to a distinct core
  and measures its own CPU time, so per-world timings stay valid; workers
  share L3/memory bandwidth, so absolute speed carries a small conservative
  bias vs a serial run. Gold and consistency are deterministic and exact
  under either mode. The parent warms the world/gold caches serially first;
  workers run cache-read-only (`AUTORESEARCH_CACHE_RO`).

## Caches

`.se2_cache.pkl` (gold maneuvers, keyed on world+query) and
`.gen_cache.pkl` (generated worlds, keyed on generator source) live next to
`scenarios.py` by default; `AUTORESEARCH_CACHE_DIR` redirects both — the
export tool uses this to share one cache dir across lab worktrees. A cold
`--gen 40` run recomputes gold maneuvers for several minutes; keep the
caches warm.

## Provenance

`geometry.py` is vendored from `dimos/navigation/motion/obstacle.py` @
`d7c1b7c88` (the referee epoch scores are pinned to), and `types.py`
replicates the used slice of `dimos.msgs` op-for-op. Both were verified
bit-exact against the source battery (56 worlds, gold + target planners).
If the robot-side scoring math in `obstacle.py` changes, re-vendor
deliberately and re-baseline.
