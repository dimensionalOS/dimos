# VR World (live)

Live "Rerun in VR" for a single robot. Subscribes to a running robot's
`odom` / `lidar` / `color_image` streams, accumulates a voxel map on the fly,
and renders the robot + growing map in a Quest 3 from a god view. The **same
headset drives the robot** via the left thumbstick (`cmd_vel` Twist).

Live counterpart to [`memory_world`](../../teleop/memory_world/) (which replays a
recorded SQLite store). See [PLAN.md](PLAN.md) for the design.

## Running

Launch the blueprint directly (the Go2 must be reachable on the same
machine/network):

```bash
dimos run vr-world-go2
```

Override module config with the standard flag syntax:

```bash
# if the robot already publishes lidar in the world/odom frame
dimos run vr-world-go2 -o vrworldmodule.lidar_world_frame=true
# finer voxels / different port
dimos run vr-world-go2 -o vrworldmodule.voxel_size=0.05 -o vrworldmodule.server_port=8443
```

Then on the Quest 3 browser open `https://<host-ip>:8443/vr_world`, accept the
cert, tap **Connect**, enter VR.

## Controls

- **Left stick** — drive the robot (up/down = forward/back, left/right = turn)
- **Right stick X** — yaw your god-view
- **Right trigger** — teleport your viewpoint
- **Both grips** (or both hands pinch) — move apart/together to scale the world
- **Right B** — toggle voxel cubes / points
- **Left Y** — reset to the dollhouse overview

## How it works

Server ([module.py](module.py)) subclasses `QuestTeleopModule` for the web
server, then:
- `In[PoseStamped] odom` → push robot pose to headset (~15 Hz)
- `In[PointCloud2] lidar` → transform to world frame by latest pose, fold into
  an Open3D `VoxelGrid`; resend the full map on a throttle (~1.5 Hz)
- `In[Image] color_image` → JPEG, push to a head-locked HUD panel (~3 Hz)
- `Out[Twist] cmd_vel` ← left-stick drive commands from the headset

All map/camera/pose pushes are throttled (single robot, conservative bandwidth).
The voxel map is **resent in full** on each throttle tick (simplest correct
approach); delta encoding is a future optimization (see PLAN.md).

## Status

Experimental. Built and import/compose-verified, but **not yet tested against a
live robot** — the lidar→world-frame assumption (`lidar_world_frame`) and the
end-to-end LCM wiring need a real Go2 to validate.
