# Return robot TCP candidates from a lean grasp-proposal interface

Status: accepted

`GraspGenSpec` will expose `propose_grasps(object_pointcloud) -> GraspCandidateArray`. The input is one segmented Object Point Cloud containing only the target object. Each returned Grasp Candidate contains a robot TCP target pose expressed in the input cloud's frame and a generator-local score used only to rank candidates from that call. Candidates are ordered by descending score; an empty array means inference completed but found no candidates, while invalid input and backend failures raise exceptions.

The adapter owns generator-specific concerns. For GraspGenX, deployment configuration supplies one fixed sweep-volume Gripper Model, model checkpoint, optional grasp-frame-to-TCP calibration that defaults to identity, and the maximum candidate count. Other inference parameters retain tested adapter defaults. The adapter converts GraspGenX poses into robot TCP poses before returning them.

Scene geometry, collision checking, inverse kinematics, approach feasibility, gripper actuation, runtime tool switching, raw depth segmentation, and batching remain outside this interface. They belong to perception, planning, or execution stages.

## Considered options

- The existing `PoseArray` was rejected because it discards candidate scores.
- Returning poses in GraspGenX's model frame was rejected because it leaks backend conventions and forces every consumer to repeat TCP calibration.
- Passing scene geometry and GraspGenX tuning controls per request was rejected because it mixes proposal generation with planning and backend configuration.
- A status envelope was rejected in favor of empty-array and exception semantics to keep the successful result small.

## Consequences

Every backend adapter must return the same TCP-frame candidate semantics, even when its native pose convention differs. Planning remains authoritative for deciding whether a Grasp Candidate is executable.
