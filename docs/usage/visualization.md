# Viewer Backends

Dimos supports three visualization backends: Rerun (web or native) and Foxglove.

## Quick Start

Choose your viewer via the CLI (preferred):

```bash
# Rerun native viewer (default) - dimos-viewer with built-in teleop + click-to-navigate
dimos run unitree-go2

# Explicitly select the viewer mode:
dimos --viewer rerun run unitree-go2
dimos --viewer rerun-web run unitree-go2
dimos --viewer foxglove run unitree-go2
```

Alternative (environment variable):

```bash
# Rerun native viewer (default) - dimos-viewer with built-in teleop + click-to-navigate
VIEWER=rerun dimos run unitree-go2

# Rerun web viewer - browser dashboard + teleop at http://localhost:7779
VIEWER=rerun-web dimos run unitree-go2

# Foxglove - Use Foxglove Studio instead of Rerun
VIEWER=foxglove dimos run unitree-go2
```

## Viewer Modes Explained

### Rerun Native (`rerun`) — Default

**What you get:**
- [dimos-viewer](https://github.com/dimensionalOS/dimos-viewer), a custom Dimensional fork of Rerun with built-in keyboard teleop and click-to-navigate
- Native desktop application (opens automatically)
- Better performance with larger maps/higher resolution
- No browser or web server required

---

### Rerun Web (`rerun-web`)

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

To enable rerun within your own blueprint simply include `RerunBridgeModule`:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

camera_demo = autoconnect(
    CameraModule.blueprint(),
    RerunBridgeModule.blueprint(
        viewer_mode="none",  # native (desktop), web (browser), none (headless)
    ),
)

```

Run the stack locally (this blocks until you stop the process):

```python skip
from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    ModuleCoordinator.build(camera_demo).loop()
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

```python skip
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
