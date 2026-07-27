# New Simple Nav Smoothing Performance Port

## Baseline And Scope

- Baseline branch: `melolong/new_simple_nav_merged_main`
- Baseline commit: `18a6fb823e695f366e4d021a81b1d5e0a7b7522e`
- Integration branch: `codex/new-simple-nav-smoothing-performance`
- VM worktree: `/home/markus/work/dimos_new_simple_nav_perf`

This port transfers the path-smoothing latency work without importing the
simulation branch's physical candidate validator, MuJoCo modules, or controller
changes. Public smoothing configuration, backtracking order, collision sample
spacing, cost threshold, and published path behavior remain unchanged.

## Implemented Optimizations

1. Candidate and raw-resampled geometry remain NumPy XY arrays until the final
   path is selected, so rejected candidates do not allocate `Path/PoseStamped`
   messages.
2. Internal collision and cost scans calculate grid indices directly instead
   of constructing `Vector3` values through `OccupancyGrid.world_to_grid` for
   every sample.
3. The sequential smoothing loop evaluates local triples with scalar values,
   avoiding per-point candidate arrays and `np.vstack` allocations.
4. GlobalPlanner records raw validation, reference cost, smoothing loop,
   candidate evaluation, final message, publication, and LocalPlanner handoff
   timing in one `Path smoothing performance.` log record.
5. `scripts/m20_path_smoothing_benchmark.py` provides a deterministic,
   non-LFS 2/5/10/20/40 m benchmark.

Runtime timing is disabled by default. Enable it only while debugging:

```yaml
replanningastarplanner:
  path_smoothing_performance_logging_enabled: true
```

When enabled, GlobalPlanner emits one structured `Path smoothing performance.`
record per successful constrained plan. When disabled, it passes `timing=None`,
so phase timing collection and JSON serialization are skipped rather than only
hiding the log line. Restart DimOS after changing the YAML profile.

## Offline Results

The same fixed-seed fixture was run before and after the port. Baseline used
3 warmups and 15 measured repetitions; the optimized formal run used 5 warmups
and 30 measured repetitions.

| Path length | Raw points | Baseline P50 | Optimized P50 | P50 reduction | Optimized P95 |
|---:|---:|---:|---:|---:|---:|
| 2 m | 38 | 30.4 ms | 4.5 ms | 85.0% | 6.0 ms |
| 5 m | 93 | 76.4 ms | 10.8 ms | 85.8% | 11.2 ms |
| 10 m | 186 | 174.3 ms | 23.9 ms | 86.3% | 24.3 ms |
| 20 m | 372 | 363.6 ms | 50.7 ms | 86.1% | 51.6 ms |
| 40 m | 743 | 737.3 ms | 101.3 ms | 86.3% | 103.5 ms |

For all five lengths, final point counts, XY coordinates, and orientation
quaternions matched the baseline exactly; maximum absolute difference was
`0.0`.

## Verification

Passed:

- 19 focused constrained-smoothing tests;
- randomized direct-grid cost equivalence across positive and negative map
  origins;
- 250 randomized local-triple equivalence cases;
- exact array/message resampling and one-final-message checks;
- 27 available smoothing and replanning-A* tests;
- 32 targeted tests after adding the default-off runtime logging switch and
  M20 YAML/config propagation coverage;
- Ruff lint, Ruff format, and `git diff --check`.

Eight historical tests could not start because this lightweight checkout has
no credentials for private LFS fixtures `occupancy_simple.npy` and
`three_paths.npy`. The same limitation existed before the port; no test reached
an assertion and failed.

`dimos run m20-simple-nav --help` currently exits in Pydantic while resolving a
default factory that requires `validated_data`. The same command fails
identically on a detached, unmodified `18a6fb82` worktree, so this is a known
baseline CLI-help issue rather than a smoothing-port regression.

Reproduce the benchmark from this worktree:

```bash
cd /home/markus/work/dimos_new_simple_nav_perf
PYTHONPATH=$PWD /home/markus/work/dimos_m20/.venv/bin/python \
  scripts/m20_path_smoothing_benchmark.py \
  --warmups 5 \
  --repetitions 30 \
  --output-dir /tmp/new-simple-nav-smoothing-benchmark
```
