# DimOS Robotics Context

DimOS describes robots, actuators, and control surfaces using precise robotics terminology. This glossary records domain language only, not implementation details.

## Language

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
