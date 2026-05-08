# /ship - Run the DimOS PR Workflow

Usage: `/ship <one sentence request>`

Follow these steps in order. If any step fails, stop and report the current state
before deciding what to do next.

## 1. Prepare

- Read `AGENTS.md` and `.cursor/rules/00-workflow.mdc`.
- Run `git status --short`, `git branch --show-current`, and `git remote -v`.
- If the working tree has unrelated changes, leave them untouched and stage only
  files created for this task.

## 2. Branch

- Fetch from the fork remote: `git fetch origin`.
- Use the current `origin` tracking branch as the base and merge target unless
  the human names another branch. If no clear `origin` tracking branch exists,
  stop and ask which fork branch to use.
- Create a small task branch from `origin/<base-branch>`.
- Use DimOS branch prefixes: `feat/`, `fix/`, `refactor/`, `docs/`, `test/`,
  `chore/`, or `perf/`.

## 3. Plan

- Convert the request into a short todo list.
- Keep the implementation to the smallest necessary file set.
- If the task spans many modules or has meaningful trade-offs, pause and propose
  a split before coding.

## 4. Implement

- Follow existing DimOS module, blueprint, transport, and agent skill patterns.
- Do not edit generated files such as `dimos/robot/all_blueprints.py` by hand.
- Do not include secrets, local caches, or unrelated data changes.

## 5. Verify

Run:

```bash
bash scripts/verify.sh
```

If the full verify is blocked or too expensive, run the narrowest meaningful
checks and report exactly what was not covered.

## 6. Commit

- Review `git diff --stat` and `git diff`.
- Stage only task-related files.
- Commit with a concise conventional message, such as `fix: ...` or `chore: ...`.
- Do not use `--no-verify`.

## 7. Push And Merge

- Push with upstream tracking: `git push -u origin <branch>`.
- Create a PR targeting the selected fork base branch, or merge back into that
  fork branch using the repository's normal review flow.
- Fill the PR template with Summary, Test plan, Risk, Related, and the CLA
  checkbox when applicable.
- If auto-merge is enabled and checks are required, use squash auto-merge.

## 8. Finish

Report:

- Changed files and why.
- Verification command and PASS/FAIL.
- PR URL if created.
- Remaining human setup, such as branch protection, GitHub Actions, or Codex
  review configuration.
