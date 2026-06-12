## User-Facing Docs

- Update manipulation planning usage documentation to explain explicit backend selection:
  - default Drake world backend,
  - optional RoboPlan world backend,
  - valid planner combinations (`rrt_connect` with Drake or RoboPlan world; `roboplan` only with RoboPlan world),
  - normal module option override examples for `dimos run`.
- Document that RoboPlan is an optional development dependency installed from Git for now, and that package publication/pinning is a separate follow-up.
- Document safety behavior: unsupported planning-critical robot, joint, or obstacle features fail before planning rather than being ignored.

## Contributor Docs

- Update contributor/development docs for manipulation planning backend extension points if such a page exists; otherwise add a short section to the most relevant manipulation planning docs.
- Include guidance that optional backend imports must be lazy and error messages should explain the required extra/dependencies.
- Include the initial acceptance test boundary: RoboPlan world must work with the generic RRT planner before native RoboPlan planning is considered complete.

## Coding-Agent Docs

- Update `AGENTS.md` or `docs/coding-agents/` only if implementation adds new conventions that coding agents need to remember, such as the explicit `world_backend`/`planner_name` combination policy or RoboPlan optional-extra install notes.
- If no coding-agent-specific conventions are introduced beyond user/developer docs, no coding-agent doc update is required.

## Doc Validation

- Run the repository's documentation link/format validation if available.
- If docs contain executable Python snippets, run `md-babel-py run <doc>` for the changed files.
- Run targeted docs checks listed in `docs/development/writing_docs.md` if that page specifies project commands.

## No Docs Needed

Documentation is needed because this change introduces a new optional backend, new configuration behavior, new dependency instructions, and safety-relevant unsupported-feature behavior.
