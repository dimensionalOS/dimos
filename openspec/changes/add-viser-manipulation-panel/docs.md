## User-Facing Docs

- Add a capability guide under `docs/capabilities/manipulation/`, or extend the existing manipulation capability guide if one is already the canonical entry point, with:
  - How to install the optional Viser panel dependencies.
  - How to launch a manipulation stack and then open the Viser manipulation panel.
  - The phase 1/2 operator workflow: select robot, choose a target preset, refine with the end-effector target control or joint sliders, inspect feasibility, plan, preview, execute, cancel, and reset.
  - Visual semantics: solid current robot, translucent target ghost, draggable end-effector target gizmo, planned path preview, and red infeasible target state.
  - Safety notes for hardware: execution requires a fresh plan and confirmation; cancel remains available; the panel does not bypass manipulation/coordinator safety checks.
  - Scope notes: no perception pick/place, rich scene editing, camera/point-cloud overlays, or full RViz parity in this phase.
- Add launch examples to the manipulation docs for the first supported surface, such as:
  - Start a stack: `dimos run xarm7-planner-coordinator` or the supported mock/sim equivalent.
  - Start/open the panel: use the final companion entrypoint or blueprint name chosen by implementation.
- If a named runnable blueprint is added, update the appropriate blueprint listing docs and regenerate any generated blueprint registry references required by existing docs.

## Contributor Docs

- Add contributor notes to manipulation development docs only if the implementation introduces new extension points, such as named target presets on robot configs or preview RPC response shapes.
- If new optional dependency groups are added, update the contributor/development docs that describe dependency extras and local validation commands.
- No broad `docs/development/` process documentation is required for the first panel unless implementation changes blueprint generation, dependency conventions, or documentation build procedures.

## Coding-Agent Docs

- Update `docs/coding-agents/` or `AGENTS.md` only if implementation establishes a new recurring pattern for Viser-based DimOS modules.
- If no reusable pattern is introduced beyond this feature, no coding-agent docs are required.
- If added, the guidance should emphasize:
  - Keep Viser UI callbacks non-blocking.
  - Route robot actions through `ManipulationModule` RPCs.
  - Do not read private manipulation state from panel code.
  - Keep optional visualization dependencies out of core manipulation installs.

## Doc Validation

- Run documentation link validation used by the repo after user-facing docs are changed, for example `doclinks` if available in the development environment.
- Run `md-babel-py run <doc>` for any changed markdown document that contains executable Python snippets.
- Run the relevant docs build or preview command if manipulation docs include generated diagrams or cross-linked pages.
- If no executable snippets are added, no `md-babel-py` command is needed for the new/changed panel docs.

## No Docs Needed

Documentation changes are needed because this change adds a new operator-facing workflow and launch surface. Users need to understand how to open the panel, distinguish current and target visuals, use target presets and synchronized controls, and safely plan/execute on simulation or hardware.
