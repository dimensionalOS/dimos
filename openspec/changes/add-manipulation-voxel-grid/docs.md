## User-facing documentation

- Maintain `docs/usage/xarm_voxel_planning_viser_demo.md` as the runnable
  architecture and manual verification guide.
- Link the guide from the usage and manipulation indexes.
- Explain capture-aligned pose registration, model-derived self exclusion,
  eventual clear-mask semantics, stable typed OCTREE obstacle reconciliation,
  RoboPlan-only support, and current non-goals.

## Validation

- Run `python -m dimos.cli.doclinks docs/`.
- Run `bin/run-doc-codeblocks --ci docs/usage/xarm_voxel_planning_viser_demo.md`.
