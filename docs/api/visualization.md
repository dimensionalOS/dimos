# Viewer Backends

Dimos supports three visualization backends: Rerun (web or native) and Foxglove.

## Quick Start

Choose your viewer backend via the CLI (preferred):

```bash
# Rerun native viewer (default) - native Rerun window + teleop panel at http://localhost:7779
dimos run unitree-go2

# Explicitly select the viewer backend:
dimos --viewer-backend rerun run unitree-go2
dimos --viewer-backend rerun-web run unitree-go2
dimos --viewer-backend foxglove run unitree-go2
```

Alternative (environment variable):

```bash
VIEWER_BACKEND=rerun dimos run unitree-go2

# Rerun web viewer - Full dashboard in browser
VIEWER_BACKEND=rerun-web dimos run unitree-go2

# Foxglove - Use Foxglove Studio instead of Rerun
VIEWER_BACKEND=foxglove dimos run unitree-go2
```

## Viewer Modes Explained

### Rerun Web (`rerun-web`)

**What you get:**
- Full dashboard at http://localhost:7779
- Rerun 3D viewer + command center sidebar in one page
- Works in browser, no display required (headless-friendly)

---

### Rerun Native (`rerun`)

**What you get:**
- Native Rerun application (separate window opens automatically)
- Command center at http://localhost:7779
- Better performance with larger maps/higher resolution

---

### Foxglove (`foxglove`)

**What you get:**
- Foxglove bridge on ws://localhost:8765
- No Rerun (saves resources)
- Better performance with larger maps/higher resolution
- Open layout: `assets/foxglove_dashboards/old/foxglove_unitree_lcm_dashboard.json`

---

## Performance Tuning

### Symptom: Slow Map Updates

If you notice:
- Robot appears to "walk across empty space"
- Costmap updates lag behind the robot
- Visualization stutters or freezes

This happens on lower-end hardware (NUC, older laptops) with large maps.

### Increase Voxel Size

Edit [`dimos/robot/unitree_webrtc/unitree_go2_blueprints.py`](/dimos/robot/unitree_webrtc/unitree_go2_blueprints.py) line 82:

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
