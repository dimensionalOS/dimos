# Viewer Backends

Dimos supports three visualization backends: `rerun` (default), `foxglove`, and `none`.

## Quick Start

Choose your viewer via the CLI:

```bash
# Rerun native viewer (default) - dimos-viewer with built-in teleop + click-to-navigate
dimos run unitree-go2

# Explicitly select the viewer backend:
dimos --viewer rerun run unitree-go2
dimos --viewer foxglove run unitree-go2
dimos --viewer none run unitree-go2
```

Control how the Rerun viewer opens with `--rerun-open` and `--rerun-web`:

```bash
# Open native desktop viewer (default)
dimos --rerun-open native run unitree-go2

# Open web viewer in browser
dimos --rerun-open web run unitree-go2

# Open both native and web
dimos --rerun-open both run unitree-go2

# No viewer (headless) — data still accessible via gRPC
dimos --rerun-open none run unitree-go2

# Serve the web viewer without auto-opening a browser
dimos --rerun-web --rerun-open native run unitree-go2
```

## Viewer Modes Explained

### Rerun Native (`rerun`, `--rerun-open native`) — Default

**What you get:**
- [dimos-viewer](https://github.com/dimensionalOS/dimos-viewer), a custom Dimensional fork of Rerun with built-in keyboard teleop and click-to-navigate
- Native desktop application (opens automatically)
- Better performance with larger maps/higher resolution
- No browser or web server required

---

### Rerun Web (`rerun`, `--rerun-open web`)

**What you get:**
- Browser-based dashboard at http://localhost:7779
- Rerun 3D viewer + command center sidebar in one page
- Teleop controls and goal setting via the web UI
- Works headless (no display required)

---

### Foxglove (`foxglove`)

**What you get:**
- Foxglove bridge on ws://localhost:8765
- No Rerun (saves resources)
- Better performance with larger maps/higher resolution
- Open layout: `assets/foxglove_dashboards/old/foxglove_unitree_lcm_dashboard.json`

---

## Rendering with Custom Blueprints

To enable visualization in your own blueprint, use `vis_module`:

```python
from dimos.core.global_config import global_config
from dimos.visualization.vis_module import vis_module
from dimos.hardware.sensors.camera.module import CameraModule

camera_demo = autoconnect(
    CameraModule.blueprint(),
    vis_module(viewer_backend=global_config.viewer),
)

if __name__ == "__main__":
    camera_demo.build().loop()
```

Every LCM stream, such as `color_image` (output by CameraModule), that uses a data type (like `Image`) that has a `.to_rerun` method will get rendered (`rr.log`) using the LCM topic as the rerun entity path. In other words: to render something, simply log it to a stream and it will automatically be available in rerun.

## Performance Tuning

### Symptom: Slow Map Updates

If you notice:
- Robot appears to "walk across empty space"
- Costmap updates lag behind the robot
- Visualization stutters or freezes

This happens on lower-end hardware (NUC, older laptops) with large maps.

### Increase Voxel Size

Edit [`dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py`](/dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py):

```python
# Before (high detail, slower on large maps)
voxel_mapper(voxel_size=0.05),  # 5cm voxels

# After (lower detail, 8x faster)
voxel_mapper(voxel_size=0.1),   # 10cm voxels
```

**Trade-off:**
- Larger voxels = fewer voxels = faster updates
- But slightly less detail in the map

---

## How to use Rerun on `dev` (and the TF/entity nuances)

Rerun on `dev` is **module-driven**: modules decide what to log, and `Blueprint.build()` sets up the shared viewer + default layout.
