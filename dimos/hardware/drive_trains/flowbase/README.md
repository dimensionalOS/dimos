# FlowBase

Holonomic base control over Portal RPC. Coordinator side lives here; server side runs on the i2rt base.

## 1. Start the server (on the i2rt base)

SSH into the Pi (`i2rt@172.6.2.20`):

```bash
cd ~/i2rt/i2rt/flow_base
python flow_base_controller_modified.py
```

`flow_base_controller_modified.py` exposes a Portal RPC server on `172.6.2.20:11323` with `set_target_velocity` and `get_odometry`. A `No joystick/gamepad connected` line is fine — RPC works without one.

## 2. Verify reachability (on your machine)

```bash
nc -vz 172.6.2.20 11323
```

## 3. Launch

Four blueprints, pick one:

```bash
# FastLio2 SLAM + nav stack + click-to-drive in Rerun (requires Livox MID-360)
LIDAR_HOST_IP=192.168.1.5 LIDAR_IP=192.168.1.189 dimos run coordinator-flowbase-nav
# Click anywhere on the floor in the Rerun 3D viewer → robot navigates there.

# Coordinator + Rerun + WebSocket dashboard joystick (on-screen teleop, no LIDAR)
dimos run coordinator-flowbase-vis
# Then open http://localhost:7779/ to drive with the on-screen joystick.

# Coordinator + WASD pygame teleop (keyboard, no Rerun)
dimos run coordinator-flowbase-keyboard-teleop

# Coordinator only (no teleop, no viz — drive /cmd_vel from another source)
dimos run coordinator-flowbase
```

All four use the `flowbase` adapter against `172.6.2.20:11323` and publish on LCM `/cmd_vel` + `/coordinator/joint_state`.

- **`coordinator-flowbase-nav`** composes `FastLio2` SLAM + `create_nav_stack(planner="simple")` + `ControlCoordinator(flowbase)` + Rerun. Rerun's click events feed `SimplePlanner.goal` directly — no `MovementManager`. The blueprint's defaults assume MID-360 at `192.168.1.189`, this machine at `192.168.1.5`, sensor mounted 20cm forward / 20cm right / 10cm up. Override via the env vars above.
- **`coordinator-flowbase-vis`** opens a Rerun viewer and the WebSocket dashboard at `http://localhost:7779/`. The dashboard's on-screen joystick is wired to `/cmd_vel`. (`webbrowser.open_new_tab` should pop the URL automatically.)
- **`coordinator-flowbase-keyboard-teleop`** opens a small pygame window — **focus that window** to drive. Controls: W/S forward-back · Q/E strafe · A/D turn · Shift boost · Ctrl slow · Space stop · ESC quit.

> **Sanity check before the nav blueprint**: run `dimos run mid360-fastlio` first to verify the LIDAR locks in. If FastLio2 can't see the sensor, nav won't start.

## Notes

- Frame convention: FlowBase uses inverted Y/yaw. The adapter negates `vy` and `wz` before sending — commands in/odometry out are standard ROS frame.
- Address override: edit the `address=` default in `_flowbase_twist_base()` in `dimos/control/blueprints/mobile.py`.
- LIDAR mount: edit `_flowbase_mid360_mount` in `dimos/control/blueprints/mobile.py` to refine the sensor pose (or set a tilted quaternion if the MID-360 isn't level).
- `local_planner` reuses `G1_LOCAL_PLANNER_PRECOMPUTED_PATHS` for now. If local planning is unstable, generate FlowBase-specific paths as a follow-up.
