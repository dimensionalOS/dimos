# DimOS Operator Console (Babylon viewer)

Browser-based 3D viewer for any dimos system: scene mesh and/or gaussian
splat, the robot (FK from `joint_state` + `odom`), pointclouds, nav path,
camera frames, teleop (`/cmd_vel`) and click-to-navigate — in one page,
with no client install.

This is the viewer-only build of the simulation scene viewer that has been
driving the sim work (and has been teleoperated across the internet). It
**renders, it never owns state**: every display topic and every
browser-authored message flows through the LCM<->WebSocket bridge
([`dimos/web/lcm_bridge`](../lcm_bridge/README.md)), so the browser is just
another peer on the bus. The browser-physics / entity-authority half of
the original module stays in the simulation tree.

## Usage

```python
from dimos.web.viewer.module import BabylonViewerModule

autoconnect(
    my_robot_blueprint,
    BabylonViewerModule.blueprint(
        mjcf_path=...,    # optional: robot meshes + server-side FK
        assets=...,       # optional: {filename: bytes} for MJCF meshes
        scene_path=...,   # optional: visual scene .glb/.gltf
        splat_path=...,   # optional: gaussian splat .ply
    ),
)
```

Open `http://<host>:8091/`. Everything is optional — with no arguments you
get pointclouds, path, camera and teleop over a grid floor.

What the page shows, and where it comes from:

| Layer            | Source                                        |
| ---------------- | --------------------------------------------- |
| robot meshes     | `/robot.json` (parsed from the MJCF once)     |
| robot pose       | server-side FK -> binary frames on `/ws`      |
| scene / splat    | `/assets/…` (content-hash versioned)          |
| pointclouds      | `/global_map` etc. via `/lcm-ws`, web worker  |
| camera           | JPEG `Image` packets via `/lcm-ws`            |
| nav path, goals  | `/nav_path`, `point_goal` via `/lcm-ws`       |
| teleop           | browser publishes `/cmd_vel` via `/lcm-ws`    |

`mujoco` (the `sim` extra) is only imported when `mjcf_path` is given —
the robot layer parses the same MJCF the simulator steps, so sim users
get a pixel-exact mirror for free. A URDF/pinocchio loader shared with
the rerun viewer is the natural follow-up for mujoco-free robot display.

## Try it against the G1 sim

```bash
dimos --transport lcm --simulation mujoco --scene-package dimos_office \
    run unitree-g1-groot-wbc
```

The console comes up at `:8091` next to the sim: the G1 mirrored from
live coordinator state inside the office scene. Enable Drive + WASD
walks it; the Lidar layer streams `/global_map`. `--transport lcm`
matters: display topics and teleop flow through the LCM bridge, which
does not yet tap the Zenoh bus (see the lcm_bridge README).

## Notes

- `DIMOS_BABYLON_OPEN=0` disables the auto-opened tab.
- `DIMOS_VIEWER_GLOBAL_MAP_HZ` (default 1.0) rate-caps the voxel map on
  the websocket leg only; bus consumers are unaffected.
- The HUD's respawn / arm-slider / policy buttons are wired to the
  simulation tree's authority module; in this build they are inert.
