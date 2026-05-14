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

Three blueprints, pick one:

```bash
# Coordinator + Rerun + WebSocket dashboard joystick (on-screen teleop)
dimos run coordinator-flowbase-vis
# Then open http://localhost:7779/ to drive with the on-screen joystick.

# Coordinator + WASD pygame teleop (keyboard, no Rerun)
dimos run coordinator-flowbase-keyboard-teleop

# Coordinator only (no teleop, no viz — drive /cmd_vel from another source)
dimos run coordinator-flowbase
```

All three use the `flowbase` adapter against `172.6.2.20:11323` and publish on LCM `/cmd_vel` + `/coordinator/joint_state`.

- **`coordinator-flowbase-vis`** opens a Rerun viewer and the WebSocket dashboard at `http://localhost:7779/`. The dashboard's on-screen joystick is wired to `/cmd_vel`. (`webbrowser.open_new_tab` should pop the URL automatically.)
- **`coordinator-flowbase-keyboard-teleop`** opens a small pygame window — **focus that window** to drive. Controls: W/S forward-back · Q/E strafe · A/D turn · Shift boost · Ctrl slow · Space stop · ESC quit.

> **Note**: FlowBase has no perception sensor wired up on `main` — only wheel odometry over Portal RPC. The team's "click in Rerun to drive" pattern (`unitree-g1-nav-onboard`, `unitree-go2`) requires LiDAR/camera + planner; it's not viable here without first adding a perception module. The dashboard joystick (above) is the closest on-screen-control equivalent.

## Notes

- Frame convention: FlowBase uses inverted Y/yaw. The adapter negates `vy` and `wz` before sending — commands in/odometry out are standard ROS frame.
- Address override: edit the `address=` default in `_flowbase_twist_base()` in `dimos/control/blueprints/mobile.py`.
