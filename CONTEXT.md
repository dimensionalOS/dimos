# Grasping

The grasping context describes how object observations become grasp candidates and, after robot-specific validation, executable grasps.

## Language

**Object Point Cloud**:
A segmented point cloud containing only the target object, expressed in metres in a declared coordinate frame.
_Avoid_: Scene point cloud, target geometry

**Gripper Model**:
A generator-facing representation of gripper morphology, independent of the robot carrying it.
_Avoid_: Gripper profile, robot gripper

**End-Effector Profile**:
A deployment configuration for the installed tool that links a Gripper Model to robot-specific TCP calibration, physical limits, and actuation semantics. The active profile is fixed while the grasping stack is running.
_Avoid_: Gripper config, gripper name

**Grasp Candidate**:
A robot TCP target pose and generator-local ranking score that has not yet been proven executable by robot planning.
_Avoid_: Grasp pose, valid grasp

**Grasp Candidate Array**:
An ordered result containing Grasp Candidates expressed in the coordinate frame and timestamp of one Object Point Cloud. For the current DimOS `PointCloud2`, this means preserving `frame_id` and `ts` exactly; it does not imply preservation of a richer input `Header` object.
_Avoid_: Pose array, grasp result

**Grasp Envelope Glyph**:
A single abstract visualization of one Grasp Candidate, derived from the Gripper Model's open and half-open sweep volumes and oriented at the candidate TCP. It does not depict a physical end effector or imply execution validity.
_Avoid_: Gripper mesh, hand model

**Executable Grasp**:
A Grasp Candidate transformed for an End-Effector Profile and validated against robot kinematics, collision constraints, and approach feasibility.
_Avoid_: Candidate, generated grasp

## GraspGenX integration constraints

GraspGenX is a direct in-process adapter in the shared DimOS environment. Its optional
extra participates in the universal lock; the accepted trade-off is shared torch/CUDA
and MuJoCo regression risk, not dependency or base-environment independence. The
checkpoint and sweep-volume gripper are deployment inputs, and the adapter performs no
runtime clone or network download. The YCB fixture is packaged for offline use with
wheel/sdist provenance metadata.
