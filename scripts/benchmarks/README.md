# Rust vs Python mapper benchmark (issue #2820)

Benchmarks for the Rust rewrite of the go2 mapping stack: `VoxelGridMapper`
(`dimos/mapping/voxels.py`) and `CostMapper` (`dimos/mapping/costmapper.py`)
vs their Rust ports (`dimos/mapping/rust`, wrapped by
`dimos/mapping/rust_mappers.py`).

Both stacks consume the identical sensor input: the 461 lidar frames recorded
in `data/go2_short.db` (60 s of a real Go2 run at 7.7 Hz, ~23k points/frame),
via the same replay machinery `dimos --replay run unitree-go2` uses. Blueprint
configs everywhere: `voxel_size=0.05`, `carve_columns=True`, `emit_every=5`,
`height_cost` with defaults.

Correctness first: the Rust ports are golden-tested against the Python
implementations on this same recording (`dimos/mapping/rust/tests/golden.rs`)
— exact voxel-key equality and bit-identical costmaps at 5 checkpoints.

## Pieces

| file | what it does |
|---|---|
| `baseline_mappers.py` | Python per-stage timings, frames pulled straight from the db |
| `dump_golden.py` | dumps frames + Python outputs for golden tests and the Rust bench |
| `bench_mappers` (rust bin) | Rust per-stage timings on the identical frames |
| `e2e_bench.py` | runs a full blueprint on the replay; measures message rates, stamp-matched per-hop latency, process-tree CPU/RSS |
| `compare.py` | merges the CSVs/JSONs into the final report |

## Reproduce

```bash
# offline compute benchmark
uv run python scripts/benchmarks/baseline_mappers.py scripts/benchmarks/baseline_python.csv
uv run python scripts/benchmarks/dump_golden.py
(cd dimos/mapping/rust && cargo run --release --bin bench_mappers \
    ../../../scripts/benchmarks/baseline_rust.csv)

# end-to-end (one at a time; each takes ~2 min)
uv run python scripts/benchmarks/e2e_bench.py unitree-go2 scripts/benchmarks/e2e_python.json
uv run python scripts/benchmarks/e2e_bench.py unitree-go2-rust-mapping scripts/benchmarks/e2e_rust.json

# report
uv run python scripts/benchmarks/compare.py
```

## Results (2026-07-18, Apple-silicon macOS, CPU-only, zenoh transport)

Offline per-frame compute (identical 461 replayed frames):

| stage | Python mean (ms) | p95 | Rust mean (ms) | p95 | speedup |
|---|---|---|---|---|---|
| add_frame | 14.88 | 24.54 | 0.73 | 1.05 | **20.4x** |
| emit | 5.80 | 10.99 | 0.29 | 0.47 | **19.8x** |
| costmap | 3.19 | 4.79 | 2.19 | 3.28 | **1.5x** |

Whole-run offline throughput: Python 56 frames/s, Rust 815 frames/s (14.5x).
Growth over the 60 s run (first->last quarter): Python add_frame 8.9->22.0 ms;
Rust 0.49->0.94 ms (same O(map) shape, ~23x lower constant).

End-to-end on the live replay (stamp-matched hop latency, includes decode +
compute + encode + zenoh):

| metric | Python stack | Rust stack | improvement |
|---|---|---|---|
| lidar->map mean / p95 (ms) | 31.3 / 58.1 | 3.2 / 5.1 | **10x / 11x** |
| map->cost mean / p95 (ms) | 9.1 / 12.2 | 5.1 / 8.6 | **1.8x / 1.4x** |
| map->cost worst (ms) | 124.6 (numba JIT, first msg) | 11.2 | 11x |
| whole-pipeline CPU mean | 774% | 727% | ~0.5 core |
| whole-pipeline RSS max (MB) | 6944 | 6106 | ~840 MB |

(91 vs 92 emissions: the rust run's subscriber attached a frame later; not a
drop. Whole-pipeline CPU/RSS include the unchanged planner/vis/replay modules,
so the mapper share of the delta is understated.)

## Methodology notes / caveats

- **CPU-only comparison.** Open3D falls back to CPU on machines without CUDA
  (including the benchmark machine, an Apple-silicon Mac). On a CUDA robot the
  Python voxel mapper could use GPU; the Rust port is CPU-only by design.
- **Same algorithm, naive-first.** The Rust `carve_columns` deliberately keeps
  the Python implementation's O(map size)-per-frame full scan so the language
  comparison is apples-to-apples. An XY-indexed carve (output-identical,
  asymptotically better) is a known follow-up.
- **Python costmap is already native.** `height_cost_occupancy` runs numba +
  scipy C kernels, so the Rust win there is modest compared to the pure-Python
  voxel path. The first Python costmap call pays ~180 ms of numba JIT
  compilation (excluded from stats; Rust pays this at build time).
- **Per-stage timings exclude transport**; the e2e run includes decode +
  compute + encode + Zenoh transport via stamp-matched arrival times observed
  by an external subscriber.
- **Growth matters more than means**: all stages scale with map size, and this
  recording is only 60 s. First-quarter -> last-quarter growth is reported for
  that reason.
