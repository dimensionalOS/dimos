## User-Facing Docs

- Update `docs/capabilities/manipulation/index.md` with the path-to-trajectory lifecycle and the fact that preview and execution use the same materialized trajectory.
- Update `docs/capabilities/manipulation/adding_a_custom_arm.md` to require valid URDF velocity and extended acceleration limits when RoboPlan TOPP-RA is selected.
- Update `dimos/manipulation/planning/README.md` with:
  - startup backend selection and configuration examples;
  - `simple_trapezoid` versus `roboplan_toppra`;
  - the `RoboPlanWorld` compatibility requirement;
  - supported RoboPlan fitting modes and bounded deviation;
  - no cross-backend fallback;
  - URDF limit ownership and explicit missing-limit failures;
  - Viser next-plan speed behavior and its non-retroactive boundary.

## Contributor Docs

- No new standalone contributor guide is required.
- If implementation reveals a non-obvious RoboPlan packaging or URDF 1.2 limit convention, add a focused note under `docs/development/` rather than expanding user-facing architecture prose.
- Keep the architecture decisions under `docs/development/adr/` and ensure the OpenSpec design remains consistent with them.

## Coding-Agent Docs

- Update `AGENTS.md` only if trajectory-parametrizer extension guidance becomes a stable coding-agent workflow. If updated, document:
  - the geometric-path versus timed-trajectory boundary;
  - startup-only backend selection;
  - RoboPlan URDF limit ownership;
  - the prohibition on silent cross-backend fallback.
- No coding-agent doc update is required merely for private class or file names.

## Doc Validation

- Run `doclinks` for changed Markdown links.
- Run `md-babel-py run dimos/manipulation/planning/README.md` if executable Python or shell examples are added or modified.
- Run `md-babel-py run docs/capabilities/manipulation/adding_a_custom_arm.md` if executable examples are changed.
- Run `bin/gen-diagrams` only if a checked-in generated diagram source is introduced or changed.
- Run the repository's documentation build/check command applicable to changed capability pages.

## No Docs Needed

Not applicable. Backend selection and URDF motion-limit requirements affect robot configuration and failure behavior, so user-facing documentation is required.
