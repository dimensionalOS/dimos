# R1Pro Quest VR teleop

## Fake and real entrypoints

```bash
# No vendor/ROS/hardware startup.
dimos run r1pro-vr-teleop --g.listen_host 0.0.0.0 --g.viewer none

# REAL HARDWARE: boots vendor HDAS/mobiman and the chassis gatekeeper.
# Physical E-stop and a clear workspace are required.
# Source ROS Humble and the robot's vendor overlay first.
dimos run r1pro-vr-teleop-real --g.listen_host 0.0.0.0 --g.viewer none
```

Viser is served on HTTP port 8095; Quest uses
`https://<robot-LAN-IP>:8443/teleop` (self-signed certificate). Use the
robot's Tailscale IP for remote Viser access. The real entrypoint uses
`tracking_speed=0.2` rad/s and disables wrist image subscriptions. Vendor
startup may still launch camera processes.

Both entrypoints share inputs, coordinator tasks, the manipulation module,
Home target and IK tuning. Starting real does **not** execute Home.

## Waist and starting pose

- Default IK: both arms plus top pitch `torso_joint3` and yaw `torso_joint4`.
  Lower waist joints 1/2 remain at measured feedback during teleop.
- Pitch posture cost is 5 times the arm/yaw cost, with a 0.15 rad/s pitch
  command limit. Yaw retains the original cost and common 0.5 rad/s limit.
  These are solver preferences and velocity limits, not an angular envelope.
- `R1PRO_HOME_POSE` is the shared forward-reaching target. Fake starts there.
  To move hardware toward it, select **only left_arm and right_arm** in the
  manipulation panel, select Home, Plan, inspect Preview, then Execute.
  A full torso Home can move lower waist joints even though teleop excludes
  them. Planning must succeed; FK/limit checks alone do not establish safety.

## Stop is not motor disable

Stop the DimOS launcher first and verify its workers/ports have exited.
Then stop the vendor controllers and sensor/discovery sessions started for
this experiment; do not kill unrelated desktop/navigation processes.

Neither killing a process, coordinator software E-stop nor a closed WebXR
page proves motor torque is disabled. Do not guess ROS control codes or send
raw CAN commands to disable motors. Use the exact robot's approved hardware
shutdown/disable procedure, ensuring arms and payloads cannot fall or trap
anyone. Retain physical E-stop access throughout. An accepted gripper target
is not guaranteed cancelled merely by stopping its publisher.

Vendor warnings observed during the on-robot experiment included missing
Torso key protection and physical controller mode 0 (external chassis control
may be rejected). They were not bypassed. The operator reported the experiment
working; this is not a claim that every safety/fault scenario was tested.
