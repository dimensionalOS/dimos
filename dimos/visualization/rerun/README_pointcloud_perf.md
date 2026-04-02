# PointCloud2 → Rerun Performance

## Problem

The production path for visualizing the global map in Rerun (`PointCloud2.to_rerun(mode="boxes")`) was taking 100-200ms at 400k points, limiting updates to ~5-10Hz. The bottleneck was in `to_rerun()`, not `rr.log()`.

## Root Causes

1. **matplotlib colormap on every frame** (`PointCloud2.py:650`): `_get_matplotlib_cmap(colormap)` + `cmap(z_norm)` called on all 400k points every update — ~140ms per call.
2. **Redundant `np.unique` dedup** (`PointCloud2.py:665`): `to_rerun()` snaps points to a voxel grid and deduplicates with `np.unique(axis=0)` — ~500ms. This is unnecessary because `VoxelGridMapper` already outputs deduplicated voxel centers via its internal hashmap.

## Solution: LUT-based colormap

Build a 256-entry turbo color lookup table once (from matplotlib, ~2ms one-time cost at import). Per-frame coloring becomes a single numpy index operation:

```python
# One-time LUT build (cached via lru_cache)
cmap = _get_matplotlib_cmap(name)
t = np.linspace(0, 1, 256)
LUT = (cmap(t)[:, :3] * 255).astype(np.uint8)

# Per-frame: quantize Z → index into LUT
z = points[:, 2]
idx = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint8)
colors = LUT[idx]
```

Skip `np.unique` entirely for VoxelGridMapper output — it's already deduplicated.

## How We Tested

### 1. Standalone script (isolated benchmark)

Loaded the cached global map pickle (`unitree_go2_bigoffice_map.pickle`, 402k points) and benchmarked different strategies for sending it to Rerun:

```bash
pytest dimos/visualization/rerun/test_rerun_pointcloud_speed.py -s
```

Compared matplotlib vs LUT colormap, with/without `np.unique`, boxes vs points. Found:
- matplotlib colormap: ~140ms per call
- LUT colormap: ~20ms per call
- `np.unique` on 400k points: ~500ms (unnecessary since VoxelGridMapper deduplicates)
- `rr.Boxes3D()` + `rr.log()`: ~15ms (Rerun overhead, not optimizable)

### 2. Applied to production code

Changed `PointCloud2.to_rerun()` to use `_apply_colormap_lut()` instead of calling matplotlib per-frame, and removed the `np.unique` dedup from boxes mode. Also cleaned up `bridge.py` — removed rate-limiting, drop logic, and CSV latency logging from `_on_message`.

### 3. Replay test (full pipeline validation)

The standalone test also replays lidar frames through `VoxelGridMapper` → LUT colormap → `rr.Boxes3D` → `rr.log()`, frame by frame, to measure real-world throughput as the map grows:

```
frame 0:    33,586 pts →  3.4 ms
frame 500: 173,433 pts →  4.0 ms
frame 1000: 255,381 pts → 5.9 ms
frame 2250: 402,295 pts → 9.8 ms
```

Now running `dimos --replay` to validate the full stack end-to-end and compare latency graphs with the old ones.

## Results (402k points)

| Metric | Before (production `to_rerun()`) | After (LUT, no dedup) |
|---|---|---|
| Avg convert+log | ~50ms | **6.1ms** |
| Max convert+log | ~200ms | **19.9ms** |
| Theoretical max Hz | ~5-10 Hz | **50 Hz** |

~20x faster.

## Changes Made

- `dimos/msgs/sensor_msgs/PointCloud2.py` — added `_get_colormap_lut()`, `_apply_colormap_lut()`, updated `to_rerun()` to use LUT, removed `np.unique` dedup in boxes mode
- `dimos/visualization/rerun/bridge.py` — removed rate-limiting, drop-at-latency, CSV logging, unused imports from `_on_message`

## Notes

- Rerun's gRPC proxy has a 1GB memory limit. Logging every frame at full rate will hit it.
- `rr.Boxes3D` does not support native scalar→colormap coloring, so we must compute colors in Python.
- The visual override in `unitree_go2_basic.py` calls `grid.to_rerun(voxel_size=0.1, mode="boxes")` — no change needed there, it benefits automatically.
