# Rerun Visualization Speed: 10 Implementation Plans

All benchmarks: 400k voxels, warmed up, measured on dimos-52.

Current production: **~660ms/update**. Fix1 branch: **~23ms/update**.

---

## Idea 1: class_ids Instead of Per-Point Colors

**Effort: 1 line change | Latency: 7.3ms → 1.4ms | Speedup: 5x**

### Problem
`to_rerun()` builds a 400k×3 uint8 color array via `_apply_colormap_lut()` (4.5ms), then `rr.Boxes3D(colors=...)` serializes it (2.6ms). Total: 7.3ms.

### Solution
Rerun's `AnnotationContext` lets you pre-register a palette of colors by class ID. Instead of a 400k×3 color array, you pass a 400k uint16 class_id array. Rerun resolves colors on the viewer side.

### What to Change

**File: `dimos/visualization/rerun/bridge.py`**

In `start()`, after `rr.init()`, register the turbo colormap:

```python
# After rr.init/spawn, before subscribing:
from dimos.msgs.sensor_msgs.PointCloud2 import _get_colormap_lut
lut = _get_colormap_lut("turbo")
rr.log("/", rr.AnnotationContext([
    rr.datatypes.ClassDescription(
        info=rr.datatypes.AnnotationInfo(id=i, color=lut[i].tolist())
    ) for i in range(256)
]), static=True)
```

**File: `dimos/msgs/sensor_msgs/PointCloud2.py`**

In `to_rerun()`, replace the colormap block:

```python
# BEFORE (fix1):
point_colors = None
if colormap is not None:
    point_colors = _apply_colormap_lut(points, colormap)
elif colors is not None:
    point_colors = colors
...
return rr.Boxes3D(centers=points, half_sizes=[h]*3, colors=point_colors, fill_mode="solid")

# AFTER:
class_ids = None
point_colors = None
if colormap is not None:
    z = points[:, 2]
    class_ids = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint16)
elif colors is not None:
    point_colors = colors
...
return rr.Boxes3D(
    centers=points, half_sizes=[h]*3,
    colors=point_colors, class_ids=class_ids,
    fill_mode="solid",
)
```

### Benchmark
```
LUT + Boxes3D(colors):     avg=7.30ms
class_ids + Boxes3D:       avg=1.16ms  (float32)
class_ids + Boxes3D:       avg=3.15ms  (float64, from LCM decode)
```

### Caveats
- `AnnotationContext` must be logged before any entity that uses class_ids
- 256 color buckets means slight color quantization (imperceptible for turbo on Z height)
- If a custom `colors=` list is passed explicitly, class_ids is skipped (backwards compatible)

---

## Idea 2: static=True for Global Map

**Effort: 1 line change | Latency: same SDK | Viewer memory: -47 MB/sec**

### Problem
Every `rr.log("world/global_map", ...)` appends a new entry to Rerun's timeline. At 7.4 Hz with 400k points (6.4 MB each), the viewer accumulates 47 MB/sec of map history. After 5 minutes: 14 GB of map snapshots in the chunk store. This forces aggressive GC which slows ingestion.

### Solution
The global map is always a complete snapshot (not incremental). `static=True` tells Rerun to replace the previous data instead of appending.

### What to Change

**File: `dimos/visualization/rerun/bridge.py`**

In `_on_message()`, check if the entity should be static:

```python
# Option A: Add static_entities config (already declared but unused):
is_static = entity_path in self.config.static_entities
...
rr.log(entity_path, cast("Archetype", rerun_data), static=is_static)

# Then in the blueprint config:
static_entities=["world/global_map", "world/navigation_costmap"]
```

**File: `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py`**

Add to rerun_config:
```python
rerun_config = {
    ...
    "static_entities": ["world/global_map", "world/navigation_costmap"],
}
```

### Benchmark
```
rr.log(entity, arch):              0.02ms
rr.log(entity, arch, static=True): 0.02ms  (same SDK cost)
Viewer memory savings:             47 MB/sec → 0 MB/sec for map
```

### Caveats
- Static entities don't appear on the timeline scrubber (can't scrub back to see old map states)
- For the global map this is fine since it's cumulative anyway
- Costmap is also a full snapshot each time, so also safe to make static

---

## Idea 3: float32 Throughout

**Effort: small | Latency: 3.15ms → 1.16ms for class_ids path | Speedup: 2.7x**

### Problem
Data flows as float32 through most of the pipeline (Open3D tensors, LCM encoding), but `as_numpy()` converts through legacy Open3D (`Vector3dVector` = float64). This doubles memory and slows every downstream numpy operation.

### Solution
Add a fast path that returns float32 directly from the tensor, bypassing legacy.

### What to Change

**File: `dimos/msgs/sensor_msgs/PointCloud2.py`**

Add method:
```python
def points_f32(self) -> np.ndarray:
    """Get positions as float32 numpy array (zero-copy from tensor when possible)."""
    self._ensure_tensor_initialized()
    if "positions" in self._pcd_tensor.point:
        arr = self._pcd_tensor.point["positions"].numpy()
        return arr.astype(np.float32) if arr.dtype != np.float32 else arr
    return np.zeros((0, 3), dtype=np.float32)
```

In `to_rerun()`, use the fast path:
```python
# BEFORE:
points, _ = self.as_numpy()

# AFTER:
points = self.points_f32()
```

### Benchmark
```
as_numpy() (float64) → class_ids Boxes3D:  3.15ms
points_f32()         → class_ids Boxes3D:  1.16ms
```

### Caveats
- `as_numpy()` still exists for backwards compatibility (returns float64 + colors)
- `points_f32()` doesn't return colors; for viz, colors come from class_ids anyway
- Zero-copy only works if tensor is already on CPU; CUDA tensors require a copy

---

## Idea 4: JPEG Compress Images in Bridge

**Effort: small | gRPC bandwidth: 87 MB/s → 17 MB/s | Speedup: N/A (bandwidth)**

### Problem
Raw RGB 1920×1080 at 14 Hz = 6.2 MB/frame = 87 MB/s through the shared gRPC pipe. This is 93% of total bandwidth. Even though `rr.log()` is async, the gRPC proxy backs up, delaying ALL topics equally.

### Solution
JPEG-encode images before sending to Rerun. Use `rr.EncodedImage` which the viewer decodes on the GPU.

### What to Change

**File: `dimos/msgs/sensor_msgs/Image.py`**

Add a method:
```python
def to_rerun_jpeg(self, quality: int = 75) -> Archetype:
    """Convert to Rerun EncodedImage (JPEG compressed)."""
    import cv2
    import rerun as rr
    data = self.to_rgb().data
    _, buf = cv2.imencode('.jpg', data, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return rr.EncodedImage(contents=buf.tobytes(), media_type='image/jpeg')
```

**File: `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py`**

Change the visual override for the image (or add a new one):
```python
def _convert_color_image(img: Any) -> Any:
    return img.to_rerun_jpeg(quality=75)

rerun_config = {
    ...
    "visual_override": {
        ...
        "world/color_image": _convert_color_image,  # add this
    },
}
```

### Benchmark
```
Raw RGB to_rerun + log:    2.4ms, 6.2 MB/frame
JPEG encode + log:         9.6ms, 1.2 MB/frame  (80% less bandwidth)
JPEG q=50:                 7.7ms, 0.85 MB/frame (86% less)
```

### Caveats
- JPEG encoding adds ~10ms CPU per frame (trade CPU for bandwidth)
- Slight quality loss (imperceptible at q=75 for camera feed)
- Requires `cv2` (already a dependency)
- If bandwidth isn't the bottleneck, this trades CPU for bandwidth (net slower on fast networks)

---

## Idea 5: Async Bridge (Queue + Worker Thread)

**Effort: medium | Callback blocking: 237ms/sec → 0ms/sec**

### Problem
`_on_message()` runs synchronously on the LCM callback thread. Every `to_rerun()` + `rr.log()` call blocks all other message processing. At current rates:
- global_map: 7.4 Hz × 1.4ms = 10ms/sec blocked
- image: 14 Hz × 2.4ms = 34ms/sec blocked
- costmap: 7.4 Hz × 5ms = 37ms/sec blocked
- Total: ~80ms/sec of blocking (with class_ids fix), up to 260ms/sec without

This means odom/tf messages can be delayed by however long `to_rerun()` takes on the message ahead of them in the callback queue.

### Solution
Make `_on_message` non-blocking. Queue messages, process in a dedicated viz thread.

### What to Change

**File: `dimos/visualization/rerun/bridge.py`**

```python
import queue
import threading

class RerunBridgeModule(Module[Config]):
    ...

    def start(self) -> None:
        ...
        # Add viz worker
        self._viz_queue: queue.Queue[tuple[Any, Any]] = queue.Queue(maxsize=100)
        self._viz_thread = threading.Thread(target=self._viz_worker, daemon=True)
        self._viz_thread.start()

    def _on_message(self, msg: Any, topic: Any) -> None:
        """Non-blocking: just enqueue for the viz worker."""
        try:
            self._viz_queue.put_nowait((msg, topic))
        except queue.Full:
            pass  # drop oldest viz frame rather than block callback

    def _viz_worker(self) -> None:
        """Dedicated thread for to_rerun + rr.log."""
        import rerun as rr
        while True:
            msg, topic = self._viz_queue.get()
            entity_path = self._get_entity_path(topic)
            rerun_data = self._visual_override_for_entity_path(entity_path)(msg)
            if not rerun_data:
                continue
            if is_rerun_multi(rerun_data):
                for path, archetype in rerun_data:
                    rr.log(path, archetype)
            else:
                is_static = entity_path in self.config.static_entities
                rr.log(entity_path, cast("Archetype", rerun_data), static=is_static)

    def stop(self) -> None:
        self._viz_queue.put(None)  # sentinel
        self._viz_thread.join(timeout=2.0)
        super().stop()
```

### Benchmark
```
Callback thread load (current):  ~237ms/sec (23.7%)
Callback thread load (async):    ~0ms/sec (just queue.put_nowait)
Viz worker thread load:          ~237ms/sec (on its own thread)
Odom/TF jitter:                  ±0ms (was ±1-5ms)
```

### Caveats
- `queue.Full` silently drops frames (acceptable for viz, not for control)
- `maxsize=100` prevents unbounded memory growth
- Messages may be processed slightly out of order between topics (acceptable for viz)
- `to_rerun()` on some message types may not be thread-safe (check Image, PointCloud2)

---

## Idea 6: Direct Rerun Log from VoxelGridMapper

**Effort: medium | Latency: 23ms → 1.4ms | Speedup: 16x**

### Problem
The mapper has data as an Open3D tensor. The viz path does:
```
tensor → legacy pcd (5ms) → PointCloud2 → LCM encode (6ms) → UDP → decode (2ms) → as_numpy (2ms, float64) → to_rerun (7ms) → rr.log
```
That's 22ms of overhead before `to_rerun()` even starts.

### Solution
Log directly to Rerun from the mapper. `tensor.numpy()` is zero-copy, float32.

### What to Change

**File: `dimos/mapping/voxels.py`**

Add an optional Rerun output:
```python
class Config(ModuleConfig):
    ...
    rerun_entity: str | None = None  # e.g. "world/global_map"
    rerun_fill_mode: str = "solid"

class VoxelGridMapper(Module[Config]):
    ...

    def publish_global_map(self) -> None:
        pc = self.get_global_pointcloud2()
        self.global_map.publish(pc)  # keep LCM path for other consumers

        if self.config.rerun_entity:
            self._log_to_rerun()

    def _log_to_rerun(self) -> None:
        import rerun as rr

        voxel_coords, _ = self.vbg.voxel_coordinates_and_flattened_indices()
        pts = (voxel_coords + (self.config.voxel_size * 0.5)).numpy()  # zero-copy f32

        if len(pts) == 0:
            return

        half = self.config.voxel_size / 2
        z = pts[:, 2]
        class_ids = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint16)

        rr.log(
            self.config.rerun_entity,
            rr.Boxes3D(
                centers=pts,
                half_sizes=[half, half, half],
                class_ids=class_ids,
                fill_mode=self.config.rerun_fill_mode,
            ),
            static=True,
        )
```

**File: `dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py`**

```python
VoxelGridMapper.blueprint(voxel_size=0.05, rerun_entity="world/global_map")
```

**File: `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py`**

Remove or skip the `"world/global_map"` visual_override (mapper handles it now):
```python
"visual_override": {
    "world/camera_info": _convert_camera_info,
    # "world/global_map": _convert_global_map,  # handled by mapper directly
    "world/navigation_costmap": _convert_navigation_costmap,
},
```

### Benchmark
```
LCM path (fix1):          23ms total
Direct from mapper:        1.4ms total (class_ids + Boxes3D + log)
```

### Caveats
- Mapper now has a soft dependency on `rerun` (import inside method, lazy)
- If no viewer is running, `rr.log` is a no-op (Rerun handles this gracefully)
- LCM path still exists for other consumers (navigation, recording, etc.)
- AnnotationContext must be registered before mapper starts logging (bridge handles this in Idea 1)
- If mapper runs on CUDA, `tensor.numpy()` requires a device transfer (~0.5ms for 400k points)

---

## Idea 7: Incremental Map Updates (Deltas Only)

**Effort: medium-large | Latency: 1.4ms → ~0.01ms | Speedup: 100x**

### Problem
Each lidar frame adds ~100-500 new voxels to a 400k-voxel map. But we re-log all 400k voxels every frame. That's 6.4 MB of redundant data per update.

### Solution
Track which voxels changed (added/removed) and only log the delta.

### What to Change

**File: `dimos/mapping/voxels.py`**

Track previous hashmap state:
```python
class VoxelGridMapper(Module[Config]):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        ...
        self._prev_active_count: int = 0
        self._prev_keys_hash: int = 0

    def _log_to_rerun(self) -> None:
        import rerun as rr

        voxel_coords, _ = self.vbg.voxel_coordinates_and_flattened_indices()
        pts = (voxel_coords + (self.config.voxel_size * 0.5)).numpy()

        current_count = len(pts)

        # Skip if map hasn't changed
        if current_count == self._prev_active_count:
            # Could also hash a sample of keys for more precision
            return

        self._prev_active_count = current_count

        # For now: full re-log with static=True (cheapest correct approach)
        # Future: compute actual delta from hashmap
        half = self.config.voxel_size / 2
        z = pts[:, 2]
        class_ids = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint16)
        rr.log(self.config.rerun_entity, rr.Boxes3D(
            centers=pts, half_sizes=[half]*3, class_ids=class_ids, fill_mode="solid"
        ), static=True)
```

For true delta-only logging (more complex):
```python
    def _log_to_rerun_incremental(self) -> None:
        import rerun as rr

        active_indices = self._voxel_hashmap.active_buf_indices()
        current_keys = self._voxel_hashmap.key_tensor()[active_indices]

        # Compare with previous frame's keys
        # This requires maintaining a set/hash of previous keys
        # Could use a generation counter per voxel instead

        new_keys = ... # keys in current but not in previous
        removed_keys = ... # keys in previous but not in current

        if len(new_keys) > 0:
            new_pts = (new_keys.float() * self.config.voxel_size + self.config.voxel_size/2).numpy()
            # Log new voxels to a sub-entity that accumulates
            rr.log(f"{self.config.rerun_entity}/batch_{self._batch_id}",
                   rr.Boxes3D(centers=new_pts, ...))
            self._batch_id += 1

        if len(removed_keys) > 0:
            # Clear removed sub-entities
            ...
```

### Benchmark
```
Full re-log (400k pts):    1.4ms
Skip-if-unchanged:         0ms (most frames)
True delta (500 pts):      ~0.01ms
```

### Caveats
- Rerun's entity model doesn't natively support "append to existing Boxes3D" — each `rr.log` replaces
- Sub-entity approach (batch_0, batch_1, ...) works but creates many entities over time
- `static=True` with full re-log when changed + skip when unchanged is the pragmatic middle ground
- True incremental requires maintaining a diff of the hashmap, which adds complexity
- Column carving (removing old voxels when new ones arrive in same X,Y) makes pure append impossible

---

## Idea 8: GPU Shader Coloring in Viewer Fork

**Effort: large (Rust/WGSL) | Latency: eliminates color computation | Speedup: marginal over class_ids**

### Problem
Even with class_ids (Idea 1), we still compute 400k uint16 values in Python (1.1ms). And the viewer still does a lookup per instance to resolve the class_id to a color.

### Solution
Pass raw Z values as a custom component. Write a WGSL shader in the viewer that maps Z → turbo color directly on the GPU.

### What to Change

**File: `dimos-viewer/dimos/src/` (new Rust module)**

1. Define a custom `ScalarPerInstance` component type
2. Register a custom visualizer for Boxes3D that reads the scalar component
3. Write a WGSL fragment shader:
```wgsl
fn turbo_colormap(t: f32) -> vec3<f32> {
    // Attempt inlining of the turbo lookup table (34 coefficients)
    let r = /* polynomial approximation */;
    let g = /* polynomial approximation */;
    let b = /* polynomial approximation */;
    return vec3(r, g, b);
}

@fragment
fn fs_main(@location(0) z_normalized: f32) -> @location(0) vec4<f32> {
    let color = turbo_colormap(z_normalized);
    return vec4(color, 1.0);
}
```

**File: `dimos/msgs/sensor_msgs/PointCloud2.py`**

In `to_rerun()`, pass Z as a scalar:
```python
# Log Z values as a separate component, let GPU color it
z = points[:, 2]
z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
rr.log(entity, rr.Boxes3D(centers=points, half_sizes=[h]*3, fill_mode="solid"))
rr.log(entity, rr.Scalar(z_norm))  # custom component
```

### Benchmark
```
class_ids (Python):         1.1ms for color computation
GPU shader (Rust/WGSL):     0ms Python, ~0.01ms GPU
Boxes3D without any color:  0.09ms (!)
```

### Caveats
- Requires modifying the dimos-viewer Rust codebase
- Custom Rerun archetypes/components need to be registered
- Turbo colormap polynomial approximation exists (Google published coefficients)
- Major effort for marginal gain over class_ids (~1ms saved)
- Useful foundation for future custom visualizations (heatmaps, scalar fields, etc.)

---

## Idea 9: SharedMemory Transport for Viz

**Effort: large | Latency: eliminates serialization | Speedup: same as Idea 6**

### Problem
LCM serialization (encode: 6ms, decode: 2ms) exists because the mapper and bridge may run in different processes. If they're in the same process, LCM is pure overhead. If they're in different processes, LCM adds serialization + UDP + deserialization.

### Solution
Use `multiprocessing.shared_memory` for zero-copy data sharing between mapper and bridge.

### What to Change

**New file: `dimos/core/transport/shm_pointcloud.py`**

```python
import multiprocessing.shared_memory as shm
import numpy as np

class ShmPointCloudWriter:
    """Write pointcloud to shared memory (mapper side)."""
    def __init__(self, name: str, max_points: int = 500_000):
        # Header: [count (int32), timestamp (float64)] = 12 bytes
        # Data: max_points * 3 * 4 bytes (float32)
        self._max = max_points
        size = 12 + max_points * 12
        self._shm = shm.SharedMemory(name=name, create=True, size=size)
        self._header = np.ndarray((1,), dtype=[('count', np.int32), ('ts', np.float64)],
                                   buffer=self._shm.buf[:12])
        self._data = np.ndarray((max_points, 3), dtype=np.float32,
                                 buffer=self._shm.buf[12:])

    def write(self, points: np.ndarray, ts: float) -> None:
        n = min(len(points), self._max)
        self._data[:n] = points[:n]
        self._header[0] = (n, ts)

class ShmPointCloudReader:
    """Read pointcloud from shared memory (bridge side)."""
    def __init__(self, name: str, max_points: int = 500_000):
        self._shm = shm.SharedMemory(name=name, create=False)
        self._header = np.ndarray((1,), dtype=[('count', np.int32), ('ts', np.float64)],
                                   buffer=self._shm.buf[:12])
        self._data = np.ndarray((max_points, 3), dtype=np.float32,
                                 buffer=self._shm.buf[12:])

    def read(self) -> tuple[np.ndarray, float]:
        n = int(self._header[0]['count'])
        ts = float(self._header[0]['ts'])
        return self._data[:n].copy(), ts  # copy to avoid race
```

### Benchmark
```
LCM encode:       6ms
LCM decode:       2ms
SharedMemory:     ~0.1ms (memcpy of 400k * 12 bytes = 4.8 MB)
```

### Caveats
- Only works within the same machine (not across network)
- Need to handle process lifecycle (create before read, cleanup on exit)
- Race condition on concurrent read/write (need a lock or double-buffering)
- More complex than Idea 6 (direct logging) for similar benefit
- Useful if mapper and bridge MUST run in separate processes

---

## Idea 10: Rust-Native Voxel Mapper in Viewer

**Effort: massive | Latency: <1ms end-to-end | Speedup: 35x+**

### Problem
The entire pipeline exists because data must travel from Python (mapper) through serialization to Rust (viewer). Every conversion, serialization, and deserialization step adds latency.

### Solution
Move the voxel mapping into the Rust viewer. Raw lidar arrives via gRPC, the viewer maintains its own voxel grid, and renders directly. Zero Python in the viz path.

### Architecture

```
Current:
  LiDAR → Python VoxelGridMapper → PointCloud2 → LCM → Bridge → to_rerun → gRPC → Viewer → render
  ~35ms

Proposed:
  LiDAR → gRPC (raw points) → dimos-viewer (Rust voxel mapper) → GPU render
  <1ms
```

### What to Change

**File: `dimos-viewer/dimos/src/mapping/` (new Rust module)**

1. Implement a hashmap-based voxel grid in Rust:
```rust
use std::collections::HashMap;

struct VoxelGrid {
    voxels: HashMap<[i32; 3], VoxelData>,
    voxel_size: f32,
}

impl VoxelGrid {
    fn insert_points(&mut self, points: &[[f32; 3]]) {
        for p in points {
            let key = [
                (p[0] / self.voxel_size).floor() as i32,
                (p[1] / self.voxel_size).floor() as i32,
                (p[2] / self.voxel_size).floor() as i32,
            ];
            self.voxels.entry(key).or_insert(VoxelData::default());
        }
    }
}
```

2. Add a custom gRPC endpoint for raw lidar ingestion:
```rust
// Accept raw float32 point arrays, no Arrow/PointCloud2 overhead
service DimosViewer {
    rpc IngestLidar(LidarFrame) returns (Empty);
}
message LidarFrame {
    bytes points = 1;  // raw float32[N*3]
    double timestamp = 2;
}
```

3. Add instanced cube rendering:
```rust
// Use wgpu instanced draw calls instead of Rerun's Boxes3D
// Each voxel = 1 instance with a position + Z-based color from shader
fn render_voxels(&self, encoder: &mut wgpu::CommandEncoder) {
    // Bind cube vertex buffer (8 vertices, shared)
    // Bind instance buffer (400k positions)
    // Draw instanced: 36 indices * 400k instances
    render_pass.draw_indexed(0..36, 0, 0..self.voxels.len() as u32);
}
```

**File: `dimos/mapping/voxels.py`**

Add a gRPC output option:
```python
# Instead of (or alongside) LCM publishing:
if self.config.viewer_grpc_url:
    raw_bytes = pts.numpy().tobytes()  # zero-copy
    grpc_stub.IngestLidar(LidarFrame(points=raw_bytes, timestamp=ts))
```

### Benchmark
```
Python pipeline:    35ms per map update
Rust-native:        <1ms (hashmap insert + instanced draw, no serialization)
GPU instanced draw: 400k cubes at 60fps with <1ms render time
```

### Caveats
- Massive engineering effort (Rust voxel grid, gRPC endpoint, custom renderer)
- Tight coupling between viewer and DimOS data model
- Would need to replicate Open3D's column carving logic in Rust
- Maintenance burden: two voxel mappers (Python for logic, Rust for viz)
- Could be justified if visualization performance is a core product requirement
- Alternative: use Rerun's planned "custom visualizer" API when it ships

---

## Combined Implementation Roadmap

### Phase 1 (1-2 hours): Ideas 1 + 2 + 3
- class_ids coloring
- static=True for global_map
- float32 fast path

**Result: 23ms → 1.4ms per map update (16x faster)**

### Phase 2 (half day): Ideas 4 + 5
- JPEG images in bridge
- Async bridge worker

**Result: zero callback blocking + 80% less gRPC bandwidth**

### Phase 3 (1 day): Idea 6
- Direct Rerun logging from VoxelGridMapper
- Skip entire LCM round-trip for viz

**Result: 1.4ms with zero serialization overhead**

### Phase 4 (2-3 days): Idea 7
- Skip-if-unchanged (simple version)
- True incremental deltas (complex version)

**Result: most frames skip logging entirely (0ms)**

### Phase 5 (1-2 weeks): Ideas 8 + 10
- GPU shader coloring
- Rust-native voxel mapper

**Result: <1ms end-to-end, zero Python in viz path**
