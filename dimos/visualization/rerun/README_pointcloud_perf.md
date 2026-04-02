# Rerun Visualization Performance

## Problem

The Rerun bridge was slow, especially for global_map (100-200ms at 400k voxels) and global_costmap (~30ms). This limited visualization updates and caused backpressure.

## Optimizations

### 1. PointCloud2 — LUT colormap + class_ids + remove dedup

**Before:** `to_rerun()` called matplotlib on all 400k points every frame (~140ms) + `np.unique` dedup (~500ms, redundant since VoxelGridMapper already deduplicates).

**After:**
- Build 256-entry turbo LUT once from matplotlib (cached via `lru_cache`)
- Use Rerun `AnnotationContext` + `class_ids` — pass uint16 class IDs instead of 400k×3 color array, viewer resolves colors
- Added `points_f32()` — reads float32 direct from tensor, bypasses legacy float64 conversion
- Removed `np.unique` dedup in boxes mode

### 2. OccupancyGrid — LUT for costmap texture

**Before:** `_generate_rgba_texture()` called matplotlib per frame with per-cell masking and blending.

**After:** Pre-built 102-entry RGBA LUT (grid values -1..100), cached per (colormap, opacity, background) config. Texture generation is one numpy index: `lut[grid + 1]`.

### 3. Bridge cleanup

- Removed rate-limiting, drop-at-latency, unused imports
- Kept CSV latency logging for benchmarking
- Register `AnnotationContext` on startup for class_ids coloring

## Benchmark Runs

### Run 1 — Baseline (2026-04-01_13-56-39)
- CSV: `data/rerun_logs/2026-04-01_13-56-39_rerun_latency.csv`
- Graphs: `data/rerun_graphs/2026-04-01_13-56-39_rerun_latency/`
- Code: original `to_rerun()` with matplotlib + `np.unique`

| Entity | Avg convert_ms | Notes |
|---|---|---|
| global_map | ~35ms, up to 200ms at 400k pts | matplotlib + np.unique |
| global_costmap | ~25ms | matplotlib per frame |
| color_image | ~5ms | passthrough |
| lidar | ~5ms | small frames |

### Run 2 — LUT colormap fix (2026-04-02_11-33-33)
- CSV: `data/rerun_logs/2026-04-02_11-33-33_rerun_latency.csv`
- Graphs: `data/rerun_graphs/2026-04-02_11-33-33_rerun_latency/`
- Code: LUT colormap + removed dedup for PointCloud2

| Entity | Avg convert_ms | Notes |
|---|---|---|
| global_map | **~5ms** | LUT + no dedup |
| global_costmap | ~25ms | not yet optimized |
| color_image | ~5ms | unchanged |

### Run 3 — class_ids + OccupancyGrid LUT (2026-04-02_11-53-35)
- CSV: `data/rerun_logs/2026-04-02_11-53-35_rerun_latency.csv`
- Graphs: `data/rerun_graphs/2026-04-02_11-53-35_rerun_latency/`
- Code: class_ids for PointCloud2, LUT for OccupancyGrid, points_f32()

| Entity | Avg convert_ms | Notes |
|---|---|---|
| global_map | **~5ms** | class_ids + points_f32 |
| global_costmap | **~5ms** | LUT texture |
| navigation_costmap | ~20ms | LUT texture + Mesh3D serialization |
| color_image | ~5ms | unchanged |

### Comparison boxplot
- `data/rerun_graphs/comparison/boxplot_comparison.png`
- Generated with: `python scripts/plot_rerun_latency.py <csv1> <csv2> <csv3>`

## Standalone Benchmark

```bash
pytest dimos/visualization/rerun/test_rerun_pointcloud_speed.py -s
```

Replays lidar frames through VoxelGridMapper → class_ids → Boxes3D → Rerun, with per-frame timing. Compares LUT colors vs class_ids approach.

Results at 402k points:
- LUT colors: avg 7.9ms, max 26.5ms
- class_ids: avg 2.5ms, max 8.6ms (3.2x faster)

## Files Changed

- `dimos/msgs/sensor_msgs/PointCloud2.py` — `_get_colormap_lut()`, `_colormap_class_ids()`, `register_colormap_annotation()`, `points_f32()`, updated `to_rerun()`
- `dimos/msgs/nav_msgs/OccupancyGrid.py` — `_build_occupancy_lut()`, replaced `_generate_rgba_texture()` with LUT indexing
- `dimos/visualization/rerun/bridge.py` — cleaned up `_on_message`, register AnnotationContext on start, CSV logging
- `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py` — no functional changes

## Known Issues

- **CUDA OOM**: 8GB GPU shared between Rerun viewer rendering and VoxelGridMapper CUDA hashmap. At ~300k+ voxels, GPU runs out of memory. Fix: larger GPU, or run VoxelGridMapper on CPU.
- **Rerun backpressure**: viewer can't ingest all streams at full rate. `rr.log()` blocks when internal channel is full. The `receive and decode` step in Rerun is ~99ms — viewer-side bottleneck.
- **Rerun memory_limit**: `25%` controls RAM only, not VRAM. Viewer VRAM usage is uncontrolled.
- **navigation_costmap ~20ms**: mostly `rr.Mesh3D` serializing the RGBA texture — Rerun overhead, not optimizable from Python.

## How to Generate Graphs

```bash
# Single CSV — per-run graphs
python scripts/plot_rerun_latency.py data/rerun_logs/<timestamp>_rerun_latency.csv

# Multiple CSVs — comparison boxplot
python scripts/plot_rerun_latency.py <csv1> <csv2> <csv3>
```
