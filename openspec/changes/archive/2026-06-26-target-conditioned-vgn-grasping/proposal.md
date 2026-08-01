## Why

The current VGN demo and TSDF grasp generator match upstream VGN semantics: they generate grasps for whatever geometry is present in the reconstructed workspace. For robot instructions such as “grasp the cup,” the system needs a non-ambiguous object-targeted path so VGN proposes grasps for the selected registered object rather than any nearby surface or clutter.

## What Changes

- Add a small typed `RegisteredObject` contract for cross-module object metadata: object id, name, center, size, frame id, and timestamp.
- Extend object scene registration with a typed lookup by stable object id.
- Extend the user-facing grasping orchestration API with object-id-based target selection.
- Extend TSDF grasp generation with target-bounds-conditioned generation while keeping object ids out of the grasp generation spec.
- Add target-masked TSDF preprocessing inside the VGN grasp generator using world-frame axis-aligned target bounds plus cushion.
- Update the VGN MuJoCo demo flow to move/initialize the wrist camera so the object is visible, resolve a registered object deterministically, visualize the selected target bounds, and generate target-conditioned grasps for that object.

## Capabilities

### New Capabilities
- `registered-object-targeting`: Cross-module registered object metadata lookup and object-id-based grasp target resolution.
- `target-conditioned-tsdf-grasp-generation`: Generate TSDF-native grasp candidates from a latest scene TSDF constrained by target bounds.
- `target-conditioned-vgn-demo`: Exercise object-id-targeted VGN grasp generation in the opt-in MuJoCo demo.

### Modified Capabilities
<!-- None. This change builds as an additive layer over the workspace-level VGN capability. -->

## Impact

- Affected contracts: `ObjectSceneRegistrationSpec`, `TSDFGraspGenSpec`, new `RegisteredObject` message contract.
- Affected modules: `ObjectSceneRegistrationModule`, `GraspingModule`, `VGNGraspGenModule`, and the opt-in `vgn-mujoco-grasp-demo` blueprint or demo helper flow.
- Existing pointcloud grasping and workspace-level `generate_grasps_from_tsdf(tsdf)` behavior remain available.
- No new external dependency is expected.
