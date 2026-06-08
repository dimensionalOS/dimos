## User-Facing Docs

- Update `docs/capabilities/manipulation/readme.md` to describe manipulation planning as backend-selectable rather than Drake-only, while keeping Drake/RRT as the default path.
- Add a planning backend section to the manipulation capability docs covering:
  - startup-time backend selection and the lack of runtime switching in this change;
  - supported backends: default Drake and optional MPlib;
  - the fact that execution still routes through existing `ControlCoordinator` trajectory tasks;
  - planning to joints, planning to poses, preview behavior, and backend capability diagnostics;
  - perception obstacle behavior: current object detections become box obstacles by default, optional convex-hull mesh obstacles remain derived from object pointclouds, and raw MPlib pointcloud collision is an explicit backend option rather than the current default.
- Update or add manipulation planning docs under `docs/capabilities/manipulation/` or `docs/usage/` for MPlib configuration:
  - required URDF/SRDF/package-path/move-group fields;
  - MPlib link and joint ordering requirements;
  - end-effector link, collision resolution, pointcloud policy, and attached-object capability notes;
  - clear troubleshooting for missing MPlib bindings, missing robot asset fields, mismatched joint names, unsupported geometry, skipped pointcloud layers, and unavailable native MPlib APIs.
- Update blueprint documentation only if implementation adds or renames user-runnable MPlib demo blueprints. If no new blueprint is added, document backend selection on existing manipulation/xArm planner blueprints instead.
- Update README-level featured commands only if a new stable MPlib blueprint becomes a recommended user entry point. Otherwise keep README unchanged.

## Contributor Docs

- Update `docs/development/` only if implementation changes contributor workflow, dependency setup, or blueprint-generation expectations.
- If MPlib is added to the manipulation extra with platform-specific install caveats, document contributor verification steps for optional manipulation dependencies in an appropriate development or installation doc.
- If new generated blueprint exports are added, ensure existing generated-registry guidance remains accurate and reference `pytest dimos/robot/test_all_blueprints_generation.py` where relevant.

## Coding-Agent Docs

- Update `docs/coding-agents/` if backend implementation introduces agent-facing conventions for manipulation planning, such as:
  - prefer `planning_backend` startup config over runtime backend switches;
  - avoid direct `WorldSpec`/Drake world access in new code;
  - use backend capability diagnostics instead of assuming Drake feature parity;
  - preserve current object-to-obstacle perception behavior when adding MPlib raw pointcloud support.
- Update `AGENTS.md` only if the new backend architecture creates a repo-wide coding rule that agents must follow outside the manipulation planning area. Otherwise no `AGENTS.md` change is needed.

## Doc Validation

- Run `md-babel-py run docs/capabilities/manipulation/readme.md` if runnable command/code examples in that file are changed.
- Run `md-babel-py run <new-or-updated-doc>` for any new manipulation backend guide containing Python or shell examples intended to be executable.
- Run the repository doc link checker if available for changed docs; use the project-standard `doclinks` command referenced by `docs/coding-agents/docs/doclinks.md` if present in the active environment.
- Run `bin/gen-diagrams` only if implementation docs add or modify generated diagram sources that require it. Mermaid-only diagrams do not require generation per `docs/development/writing_docs.md`.
- If user-runnable blueprints are documented, validate the listed blueprint names with `dimos list` or the generated blueprint registry test during implementation verification.

## No Docs Needed

Documentation changes are needed because this change affects user-visible backend selection, optional MPlib configuration, capability diagnostics, and manipulation planning behavior. This section is intentionally not used as a no-op justification.
