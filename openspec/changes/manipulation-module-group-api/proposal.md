## Why

Once engines and algorithms are group-aware, the public manipulation module must expose those capabilities safely. This PR is the reviewer-facing API migration and should stay separate from the Viser UI rewrite.

## What Changes

- Add or migrate `ManipulationModule` APIs for group-aware joint targets, pose targets, previews, IK, and robot info.
- Preserve robot-scoped compatibility wrappers with explicit behavior when there is no unique pose-targetable group.
- Update coordinator client and example client call sites.
- Add module/unit/e2e tests for public behavior.

## Capabilities

### New Capabilities
- `manipulation-module-group-api`: Public manipulation APIs support planning-group IDs and safe compatibility wrappers.

### Modified Capabilities
- `manipulation-module-planning`: Robot-scoped pose wrappers no longer assume an arbitrary or hidden end-effector.

## Impact

- Base branch: PR 3 `group-aware-ik-rrt`.
- Reference implementation: `cc/spec/movegroup`, including Greptile follow-up behavior for robot-scoped wrappers.
- Primary files: `dimos/manipulation/manipulation_module.py`, coordinator/example clients, manipulation module tests, and planning-group e2e test.
- Out of scope: Viser UI and control/task changes.
