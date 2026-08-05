## Why

The grasp pipeline currently treats independently collision-free IK targets as sufficient evidence that a candidate is feasible. A candidate can therefore pass selection even when no connected motion exists from the current robot state through pre-grasp, grasp, and retreat.

## What Changes

- Make RoboPlan honor the planner contract's explicit start state instead of requiring it to match the live scene state.
- Validate an optional shared safety lift before evaluating candidate-specific motion.
- Accept a grasp candidate only when connected plans succeed from the shared start through pre-grasp, grasp, and retreat, with each segment starting at the preceding segment's endpoint.
- Keep validation motion-free and replan every physical execution segment from fresh measured state.
- Preserve stage-level candidate rejection reasons while logging detailed IK and planner outcomes.
- Add deterministic unit and backend-contract coverage for connected planning behavior.

## Capabilities

### New Capabilities

- `connected-grasp-sequence-validation`: Defines explicit-start planning, shared preparation, connected candidate validation, execution replanning, failure reporting, and MVP collision-model boundaries.

### Modified Capabilities

None.

## Impact

- Affects RoboPlan planner adaptation, manipulation planning helpers, and `PickAndPlaceModule` candidate selection.
- Extends the pending grasp-pipeline behavior without changing the public `pick` signature or adding configuration.
- Uses the existing planning timeout and planning-scene update behavior.
- Adds no new runtime dependency.
