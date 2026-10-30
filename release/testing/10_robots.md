# 10. Other robots

Go2 hardware checks are in [hardware](09_hardware.md). This checklist covers the G1, several Go2s at once, the Go2 over DDS and the FlowBase. Simulation and replay checks need no robot. Hardware checks name what they need.

## 10.1 G1 agent in simulation

```bash
uv run dimos --simulation run unitree-g1-agentic-sim
# In another terminal, once the MuJoCo and Rerun windows are up:
uv run dimos mcp list-tools
uv run dimos mcp call move --arg x=0.3 --arg duration=3.0
uv run dimos agent-send "walk forward one meter"
```

Expected result: MuJoCo shows the G1 standing in the office scene. The tool list includes `move`, `execute_arm_command` and `execute_mode_command`. The `move` call walks the robot forward, and the agent walks it forward again and confirms through the speakers.

## 10.2 G1 whole-body control in MuJoCo

The first run pulls the GR00T model and the office scene. The arm teleop needs a Quest headset on the same network.

```bash
uv run dimos --simulation mujoco --scene-package office run unitree-g1-groot-wbc
uv run dimos --simulation mujoco --scene-package office run unitree-g1-teleop
```

* GR00T: once the costmap appears in Rerun, click a point on the floor a few meters away. The G1 walks there.
* Teleop: open https://<host-ip>:8443/teleop in the Quest browser, accept the certificate and press Connect. Hold both grips and move the controllers.

Expected result: the G1 walks to the clicked point, and its arms follow the controllers in Viser at http://<host-ip>:8095.

## 10.3 G1 navigation and arm teleop on hardware

Needs a G1 in sport mode with dimos installed on it, its Mid-360 and RealSense, and a Quest headset, set up per the [G1 guide](../../docs/platforms/humanoid/g1/index.md).

```bash
# On the G1:
uv run dimos run unitree-g1-nav-simple --rerunbridgemodule.host 0.0.0.0 --rerunwebsocketserver.host 0.0.0.0
# On the laptop:
uv run dimos-viewer --connect rerun+http://<g1-ip>:9877/proxy --ws-url ws://<g1-ip>:3030/ws
# Stop the nav blueprint, then on the G1:
uv run dimos run unitree-g1-teleop --network-interface eth0
# In a second SSH session on the G1:
uv run dimos hardware g1 activate   # answer y
uv run dimos hardware g1 ready
uv run dimos hardware g1 disable    # after the headset check
```

* Nav: click a point on the floor in the viewer. The robot walks there.
* Teleop: open https://<g1-ip>:8443/teleop in the Quest browser, accept the certificate, hold both grips and move your hands. Both arms follow until you release.

Expected result: the G1 reaches the clicked point, `ready` moves both arms to the ready pose, the arms follow the headset, and `disable` disarms the robot.

## 10.4 Multiple Go2s

No robot is needed. In simulation the IP count sets the number of robots, and the simulated motion does not use those robot addresses. Zenoh may still warn that it cannot connect to the listed IPs.

```bash
ROBOT_IPS=10.0.0.1,10.0.0.2 uv run dimos --simulation run unitree-go2-multi-teleop
# In another terminal:
uv run dimos topic echo /robot0/odom
```

* Two MuJoCo windows open, with teleop windows titled `robot0/keyboardteleop` and `robot1/keyboardteleop`.
* Focus one teleop window and hold W. Only that robot walks. Repeat with the other window.

Expected result: each teleop window drives only its own robot, and the echo prints poses from robot0 only.

## 10.5 Go2 over DDS on the Jetson

Needs a Go2 whose Jetson has a dimos checkout and nix, a Mid-360 on the robot, and a laptop that reaches the Jetson on port 7447.

```bash
# On the Jetson:
uv run dimos run go2-nav --mid360.lidar-ip <lidar-ip>
# On the laptop:
uv run dimos --zenoh-connect tcp/<robot-ip>:7447 run go2-nav-viewer
# No hardware: play a recorded Go2 DDS session.
uv run dimos mem rerun go2_dds_stairs.mcap --seconds 60
```

* The viewer shows the camera, the odometry and the surface map. Click a point in the 3D view. The robot drives there.

Expected result: the robot reaches the clicked goal, and the recording plays its point cloud, odometry and camera in Rerun.

## 10.6 FlowBase holonomic base

Needs the lab FlowBase powered on, with its controller reachable from the laptop at `172.6.2.20:11323`. There is no mock or simulated FlowBase.

```bash
uv run dimos run coordinator-flowbase-keyboard-teleop
```

* The log prints `Connected to FlowBase at 172.6.2.20:11323` and a "Keyboard Teleop" window opens.
* With the window focused, W and S drive forward and back, Q and E strafe left and right, and A and D turn left and right. Space stops the base.

Expected result: the base moves in the direction of each key, and it stops when the keys are released or Space is pressed.
