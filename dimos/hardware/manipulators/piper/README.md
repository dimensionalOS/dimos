# Piper Driver

Driver for the Piper 6-DOF manipulator with CAN bus communication.

## Supported Features

✅ **Joint Control**
- Position control
- Velocity control (integration-based)
- Joint state feedback at 100Hz

✅ **System Control**
- Enable/disable motors
- Emergency stop
- Error recovery

✅ **Gripper Control**
- Position and force control
- Gripper state feedback

## Cartesian Control Limitation

⚠️ **Cartesian control is currently NOT available for the Piper arm.**

### Why?
The Piper SDK doesn't expose an inverse kinematics (IK) solver that can be called without moving the robot. While the robot can execute Cartesian commands internally, we cannot:
- Pre-compute joint trajectories for Cartesian paths
- Validate if a pose is reachable without trying to move there
- Plan complex Cartesian trajectories offline

### Future Solution
We will implement a universal IK solver that sits outside the driver layer and works with all arms (XArm, Piper, and future robots), regardless of whether they expose internal IK.

### Current Workaround
Use joint-space control for now. If you need Cartesian planning, consider using external IK libraries like ikpy or robotics-toolbox-python with the Piper's URDF file.
