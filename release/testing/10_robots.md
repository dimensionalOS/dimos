# 10. Other robots

Go2 hardware checks are in [hardware](09_hardware.md). This checklist covers the G1, several Go2s at once, the Go2 over DDS and the FlowBase. Simulation and replay checks need no robot. Hardware checks name what they need.

## 10.1 G1 stock DDS control

Needs a G1 prepared per the [G1 guide](../../docs/platforms/humanoid/g1/index.md)
and the Unitree DDS SDK on the control host.

```bash
uv run dimos run unitree-g1 --network-interface eth0
```

Expected result: the stock controller connects, and `dimos shell` exposes its
locomotion RPCs. Stop this stack before testing GR00T.

## 10.2 G1 GR00T control

```bash
uv run dimos run unitree-g1-groot-wbc --network-interface eth0
# In a second terminal on the same host:
uv run dimos hardware g1 status
uv run dimos hardware g1 activate
uv run dimos hardware g1 status
uv run dimos hardware g1 disable
```

Expected result: the stack starts unarmed with policy output in dry-run.
Activation runs the pose ramp and requires confirmation before enabling policy
output. Disable cancels arm motion, restores dry-run and disarms the controller.

## 10.3 G1 simulation boundary

G1 simulation is pending sim2 integration. Both G1 blueprints must reject
`--simulation` and `--replay` before opening hardware connections.

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
