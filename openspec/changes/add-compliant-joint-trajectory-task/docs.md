## User-Facing Docs

- Add or update a user-facing control/manipulation guide under `docs/usage/` or `docs/capabilities/` explaining compliant joint trajectory execution:
  - rigid trajectory vs compliant trajectory behavior;
  - supported command mode for v1: position-servo output;
  - required feedback: joint positions; optional feedback: velocities and configured effort;
  - safety clamps and saturation behavior;
  - recommended use cases such as gentle contact, guarded motion, and manipulation near uncertain obstacles.
- If a simulation blueprint or example is added, document how to run the MuJoCo smoke/contact verification scenario.

## Contributor Docs

- Update control architecture contributor docs if they exist, or add a short development note in `docs/development/` covering:
  - coordinator-facing tasks still emit final commands;
  - composed tasks are internal linear pipelines, not coordinator-level task graphs;
  - torque impedance remains future work until the adapter/hardware path supports torque output.

## Coding-Agent Docs

- Update `docs/coding-agents/` only if implementation introduces new coding-agent workflow guidance for control tasks.
- No `AGENTS.md` update is required unless new safety rules or generated-file requirements become broadly applicable to coding agents.

## Doc Validation

- Run link/documentation validation commands used by the repository for any changed docs, such as `doclinks` if available.
- For executable Markdown snippets, run `md-babel-py run <doc>` where applicable.
- If diagrams are added or regenerated, run `bin/gen-diagrams` if applicable.

## No Docs Needed

Documentation is needed because the change introduces user/developer-visible control behavior and safety semantics. Even if the first implementation is only selectable by blueprint/configuration, users need to understand that v1 is position-servo compliance rather than torque impedance.
