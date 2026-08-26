# ADR 0001: Derive planning models from the canonical robot description

Status: Accepted

## Context

A robot can need several planning scopes without having several independent
descriptions. The G1 whole-body controller uses all 29 actuated joints, Quest
teleoperation controls 14 arm joints, and collision-aware manipulation needs
the pelvis, live waist, torso, and both arms but not the legs.

Copying or hand-editing an upper-body URDF would create another source of
truth. It could silently drift from the hardware description, joint limits,
meshes, or link transforms. Loading the full floating-base model also gives
manipulation backends a root and collision scope that do not match stationary
upper-body planning.

## Decision

`RobotModel` provides two lazy, immutable structural transformations:

- `with_subtree_rooted_at(link)` selects an existing link and all descendants.
- `without_joint_subtrees(*joints)` removes each named joint and its descendant
  link branch.

These operations record intent. They parse and validate the source only when a
backend calls `load()`, preserving lazy asset checkout. They run before joint
fixing, limit overrides, and fixed-frame additions.

Materialization also resolves relative mesh and texture references against the
source description's directory. This keeps an in-memory derived model usable
after a backend composes it without the original URDF working directory.

This is structural selection, not kinematic rerooting. The API never reverses
joints or recomputes transforms. It rejects unknown roots, unknown removal
joints, removals outside the selected subtree, cycles, and duplicate requests.

The G1 upper-body view starts from the canonical G1 URDF, selects the subtree
rooted at `pelvis`, and removes the branches beginning at
`left_hip_pitch_joint` and `right_hip_pitch_joint`. It retains the three waist
joints as observed state. Only the two seven-joint arm groups are eligible for
planning.

## Consequences

- Whole-body control, Quest IK, and manipulation share one robot description.
- Waist motion is represented in upper-body kinematics and collision checks
  without giving the manipulation planner ownership of the waist.
- Leg collision geometry is intentionally absent. G1 manipulation planning is
  valid only while the robot is stationary on clear, level ground.
- A future use case that needs a true kinematic reroot must introduce a
  different operation with explicit transform semantics.
