## User-Facing Docs

- Update `docs/usage/visualization.md` or add a manipulation-specific subsection that distinguishes:
  - global Rerun stream visualization selected by `GlobalConfig.viewer` / `vis_module()`;
  - manipulation planning visualization selected by `ManipulationModuleConfig.visualization_backend`.
- Document supported manipulation visualization backends:
  - `meshcat`: existing default when `enable_viz=True` and no explicit backend is selected;
  - `viser`: Viser manipulation visualizer with URL, robot scene, preview ghost, path animation, and optional panel UI;
  - `none`: no manipulation visualization, with visualization calls treated as no-ops.
- Document optional Viser installation, for example `uv sync --extra manipulation-viser`.
- Document that Viser panel execution is opt-in through an explicit safety gate and does not bypass existing manipulation planning/execution checks.
- If examples exist for xArm manipulation demos, add a short example showing how to select Viser in a manipulation blueprint or module config.

## Contributor Docs

- Update contributor-facing manipulation visualization notes if present under `docs/development/`.
- If no dedicated contributor doc exists, include implementation guidance in the user-facing visualization doc or a new capability note:
  - Viser runs in-process for this change.
  - Rendering must use small local copies from existing `ManipulationModule`/`WorldMonitor` accessors rather than direct `WorldSpec` access from GUI callbacks.
  - IK/planning from the panel must go through the in-process adapter and operation worker.
  - Viser dependencies must remain optional and lazily imported.

## Coding-Agent Docs

- No `AGENTS.md` update is required unless implementation adds new recurring coding-agent guidance.
- If a new coding-agent note is added, it should emphasize the distinction between global visualization (`dimos/visualization/vis_module.py`) and manipulation planning visualization (`VisualizationSpec`).

## Doc Validation

- Run documentation link checks if available in this repository, such as `doclinks`.
- If adding executable Python snippets, run the relevant markdown execution command for the changed document, for example `md-babel-py run docs/usage/visualization.md` if supported by the repo.
- Run formatting/lint checks included in the normal test workflow if documentation tooling is wired into CI.

## No Docs Needed

Documentation changes are needed because this introduces a user-selectable manipulation visualization backend and an optional dependency extra. The docs must prevent users from confusing Viser manipulation visualization with the existing global Rerun viewer.
