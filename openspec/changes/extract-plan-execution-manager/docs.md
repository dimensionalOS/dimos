## User-Facing Docs

- Add `docs/capabilities/manipulation/plan_execution.md` to explain generated-plan
  atomicity, successful and fresh plan requirements, execution acceptance,
  replacement behavior, cancellation outcomes, and the single-writer convention.
- Update `docs/capabilities/manipulation/planning_groups.md` to link to the planned
  execution guide and remove examples that imply a multi-robot plan can be filtered
  at execution time.

## Contributor Docs

None. The change introduces no contributor workflow, build, test-category, or
release-process change. Implementation details belong in the capability guide and
code documentation.

## Coding-Agent Docs

None. `AGENTS.md` and `docs/coding-agents/` need no new repository-wide rule. The
capability guide will give agents the same execution contract as other developers.

## Doc Validation

Run:

```bash
bin/doclinks docs/capabilities/manipulation/plan_execution.md
bin/doclinks docs/capabilities/manipulation/planning_groups.md
bin/run-doc-codeblocks docs/capabilities/manipulation/plan_execution.md
bin/run-doc-codeblocks docs/capabilities/manipulation/planning_groups.md
```

Run `bin/gen-diagrams` only if the implementation adds a generated diagram rather
than plain Mermaid.

## No Docs Needed

Not applicable. Whole-plan execution and execution acceptance are user-visible
behavior and require capability documentation.
