# DimOS Robotics Context

DimOS describes robots, actuators, control surfaces, and manipulation planning using precise robotics terminology. This glossary records stable domain language only, not implementation details.

## Language

**Planning group**:
A named subset of a robot model's joints and frames that can be selected as a planning unit.
_Avoid_: move group, joint group

**Composite planning group**:
A planning group that represents coordinated motion across multiple selected planning groups.
_Avoid_: group combination, combined groups, multi-group plan

**Composite RoboPlan model**:
A RoboPlan-facing robot model that represents multiple registered robot models as one planning scene.
_Avoid_: combined URDF, merged robot scene

**Planning world**:
The authoritative belief state for manipulation planning, including robot state and scene state used by planners.
_Avoid_: planner context, backend instance

**Coordinated simulation clock**:
A simulation benchmark clock policy where simulator time advances in lockstep with the DimOS control coordinator clock.
_Avoid_: autonomous simulator loop, write-triggered stepping, settle step

**Runtime sidecar**:
A separate process or environment that owns a benchmark simulator backend while DimOS owns orchestration, control integration, skills, and artifacts.
_Avoid_: plugin, embedded simulator

**Depth observation**:
A depth image interpreted with its camera calibration and the pose of its camera frame at the image timestamp.
_Avoid_: depth point cloud, raw 3D points

**Grasp target**:
The intended physical object or bounded scene region that grasp generation should produce grasp candidates for.
_Avoid_: correct object, target object, grasp object

**Object id**:
A stable identifier for a registered perceived object, used when a robot command must refer to one non-ambiguous physical object.
_Avoid_: object-ish argument, object name when identity matters

**Registered object**:
A perceived object that has been assigned an Object id and has enough spatial metadata to be used as a Grasp target.
_Avoid_: detection when referring to a persistent object reference

**Target-masked TSDF**:
A grasp-generation workspace representation where observations outside the selected Grasp target are suppressed with a deliberate cushion so the target remains intact.
_Avoid_: censored TSDF, object-only scene

**Target bounds**:
A world-frame axis-aligned bounding region used as a rough attention area for a Grasp target before grasp generation.
_Avoid_: perfect object geometry, grasp geometry

**SHM runtime data plane**:
A shared-memory command/state channel between a simulated hardware adapter and a simulator runtime, used when high-rate control must cross process boundaries without RPC.
_Avoid_: public simulator API, benchmark control plane, module object sharing

**Motor state projection**:
The hardware-facing subset of simulator state that resembles what a raw robot driver exposes: actuator positions, velocities, efforts, commands, enable state, and errors.
_Avoid_: task observation, scene observation, evaluator state

**Whole-body motor surface**:
A hardware control surface that treats a robot as an ordered set of motors with per-motor state and commands, independent of whether the robot is a manipulator, mobile base, or humanoid.
_Avoid_: manipulator-only adapter, end-effector API, task action API

**Damiao-based Robot**:
A robot whose joints are actuated by one or more Damiao motors, possibly spread across multiple CAN buses and physical limbs.
_Avoid_: Damiao arm when the robot may contain multiple motor groups

**Damiao Joint Group**:
An ordered set of Damiao-driven joints that forms a meaningful physical group such as an arm, torso, or other controllable body section.
_Avoid_: Arm when the group is not necessarily an arm

**Damiao Bus**:
A named communication channel used by a Damiao-based Robot to reach one or more Damiao motors.
_Avoid_: Treating a bus as owned by a single joint group when multiple groups may share a channel

**OpenArm**:
An OpenArm robot configuration built from Damiao motors, with OpenArm-specific joints, side naming, limits, and robot description.
_Avoid_: Damiao robot when referring to OpenArm-specific geometry or naming
