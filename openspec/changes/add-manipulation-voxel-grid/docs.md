## User-Facing Docs

- Add `docs/usage/xarm_voxel_planning_viser_demo.md` as a walkthrough for the xArm MuJoCo/Viser voxel-planning simulation.
- Explain the perception-to-planning flow: wrist-camera depth becomes a complete, pre-filtered Planning Collision Snapshot in the Planning World Frame, with the initial 0.05 m Snapshot Resolution and robot-body occupancy removed before submission.
- Document how to discover and run the simulation blueprint with `dimos list` and `dimos run`, install the manipulation dependencies, connect to the Viser view, and observe the mapped occupancy alongside the accepted-backend planning scene.
- Explain the unified obstacle lifecycle from PR #3108: backend mutation happens first; the first snapshot adds one obstacle, later snapshots atomically update its complete geometry under the same native ID, and an empty snapshot removes it. Describe RoboPlan's off-lock replacement-scene construction and atomic publication, preservation of the active scene on replacement failure, Viser add-or-replace handling, frame mismatches, RoboPlan octree support, Drake's explicit OCTREE rejection, and visualization failures as non-authoritative synchronization faults.
- Distinguish collision registration (which retains every occupied voxel) from the independent Viser display cap, describe encoded native-ID scene paths under `/manipulation/obstacles/<encoded-native-id>`, and use the terms defined in root `CONTEXT.md` rather than “planning voxel map” or “persistent world model.”
- Add a link to the guide from `docs/usage/index.md` and from `docs/capabilities/manipulation/index.md`. Keep links rooted in the `docs/usage` and `docs/capabilities/manipulation` documentation trees; do not add or reference Frontier README paths.

## Contributor Docs

No contributor documentation is required. The change adds a user-facing simulation walkthrough and does not introduce a contributor workflow, extension protocol, dependency-maintenance procedure, or backend-development contract beyond the existing `WorldSpec` obstacle lifecycle described by ADR 0001.

## Coding-Agent Docs

No coding-agent documentation is required. This feature does not change agent behavior, skills, MCP tools, repository operating instructions, or coding-agent workflows; its new blueprint and visualization behavior are adequately described by the user guide and existing architecture documentation.

## Doc Validation

- Run `python -m dimos.utils.docs.doclinks docs/` (the command used by the repository’s `doclinks` pre-commit hook) after adding the guide and both index links.
- Run `bin/run-doc-codeblocks --ci docs/usage/xarm_voxel_planning_viser_demo.md` to execute Python, shell, and Node code blocks in the new guide using the repository’s documentation wrapper. Keep runnable examples compatible with this validation mode.
- If the guide includes Mermaid or Pikchr diagrams, run `bin/gen-diagrams` as prescribed by `docs/development/writing_docs.md`; otherwise no diagram-generation step is needed.

## No Docs Needed

Not applicable because user-facing documentation is required for the runnable xArm voxel-planning Viser simulation and its collision-snapshot behavior.
