# Feature context: <name>

<!-- Copy to openspec/changes/<slug>/context.md in the feature worktree.
     Fill every section (one line is fine); delete this comment.
     Commit as: spec: context for <slug>. For big features, follow up with
     /opsx-propose to generate the full proposal/design/tasks on top. -->

- **Branch:** ruthwik/<type>/<description>
- **Worktree:** ~/Documents/worktrees/<task>
- **Board card:** <id> (create with: board add <project> "<title>")
- **Linear:** DIM-<n> (if any)

## Goal

What this change accomplishes, in one or two sentences.

## Why / background

The context an implementer can't infer from the code: what prompted this,
prior attempts, related PRs or discussions.

## Scope

- In:
- Out (non-goals):

## Acceptance criteria

- [ ] Observable behavior 1
- [ ] Tests: <which test files prove it>
- [ ] `./bin/pytest-fast`, `uv run mypy`, `pre-commit run --all-files` clean

## Affected areas

Modules/dirs expected to change (e.g. dimos/teleop/, dimos/control/).
Flag anything CODEOWNERS-sensitive (/dimos/mapping/, /dimos/perception/).

## How to test

The runnable one-liner for the PR template (e.g. `dimos run unitree-go2-<x>`).

## Risks / unknowns

Anything that might bite: concurrency, hardware-only paths, LFS data needs,
packaging (pyproject) edges.
