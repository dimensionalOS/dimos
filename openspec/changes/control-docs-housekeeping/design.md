## Context

These files do not obviously belong to the core planning-group review. Some may be genuine integration requirements; others may be incidental edits from the large branch.

## Goals / Non-Goals

**Goals:**
- Separate orthogonal changes from the planning-group PR stack.
- Keep generated files and docs synchronized only when required.
- Provide a clear report of accidental or questionable files.

**Non-Goals:**
- Do not change planning-group semantics.
- Do not include Viser UI or planner algorithm changes.
- Do not include files just because they exist on the reference branch.

## Decisions

- Treat this change as a triage/extraction PR, not a mandatory feature PR.
- Include control/task changes only if validation shows they are required or independently useful.
- Keep generated registry changes next to the PR that introduces the corresponding blueprint, unless generation must happen at the end of the stack.

## Risks / Trade-offs

- If control changes depend on module API changes, this PR should stack after PR 4.
- If docs describe earlier PR behavior, those docs should move into the earlier PRs instead of this housekeeping PR.
