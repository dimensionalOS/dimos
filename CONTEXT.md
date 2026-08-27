# Teleoperation

This context defines the operator-tracking language used when human motion guides a robot through dimOS.

## Language

**Full-body SONIC teleoperation**:
An operating mode in which tracked motion of the operator's legs, pelvis, torso, arms, wrists, and head guides the G1 through SONIC. Articulated fingers are excluded.
_Avoid_: Full-body tracking, three-point teleoperation

**WebXR body snapshot**:
A set of named operator-joint poses observed together in one WebXR reference space.
_Avoid_: SMPL frame, SONIC pose

**Native-equivalent SONIC pose**:
An operator pose whose meaning matches the native SONIC PICO input, independently of the transport used to deliver it.
_Avoid_: Partial SONIC pose, policy-minimal pose

**SONIC retargeting**:
The dimOS-owned interpretation of a WebXR body snapshot as a native-equivalent SONIC pose.
_Avoid_: Packet packing, vendor conversion

**Motion Tracker calibration**:
The PICO-managed process that establishes the operator skeleton and tracker relationship before a WebXR session.
_Avoid_: Teleoperation alignment

**Teleoperation alignment**:
The per-engagement association between a complete operator skeleton and the G1 heading. It does not reshape or rescale the operator.
_Avoid_: Calibration, body normalization

**Full-body engagement**:
The interval while the operator holds the WebXR deadman control and fresh, complete body poses guide SONIC. Ending engagement returns SONIC to planner control without disarming the policy.
_Avoid_: Policy arming, robot activation

**Complete operator pose**:
A WebXR body snapshot containing every joint required to produce one native-equivalent SONIC pose. Partial snapshots do not advance the robot target.
_Avoid_: Best-effort pose, partial target

**Operator motion source**:
The engaged operator pose as SONIC's sole whole-body motion reference. Planner locomotion resumes after full-body engagement ends.
_Avoid_: Hybrid locomotion, simultaneous planner control

**Live PICO simulation test**:
An acceptance session in which a physical PICO and Motion Trackers guide the MuJoCo G1 through the production WebXR path.
_Avoid_: Synthetic replay, converter smoke test
