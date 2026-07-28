## User-Facing Docs

Update `docs/capabilities/manipulation/index.md` to document the typed `planner`
configuration, the RoboPlan `linear_cartesian` subconfiguration, and the temporary
`planner_name` compatibility field. Clearly state that linear Cartesian planning is an
internal planner capability in this change and is not yet a new
`ManipulationModule`/RPC/skill API.

Do not add caller-facing linear-motion examples until the public facade is designed.

## Contributor Docs

No new contributor-process document is needed. The internal contract, target semantics,
backend mapping, and collision-validation rules belong in code documentation, tests,
and this change's design rather than a new process guide.

## Coding-Agent Docs

No changes to `docs/coding-agents/` or `AGENTS.md` are needed. This feature does not
change repository-wide coding-agent procedures.

## Doc Validation

- Run `uv run doclinks docs/capabilities/manipulation/index.md`.
- Run `uv run md-babel-py run docs/capabilities/manipulation/index.md` if executable
  code blocks are added or changed.

No diagram generation is expected.

## No Docs Needed

Not applicable: the typed planner configuration is developer/user-visible and requires
an update to the manipulation capability documentation.
