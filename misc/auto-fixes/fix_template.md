You are fixing the issues recorded in `issues.ignore.md` in this working tree. The branch under
review is $$BRANCH$$; you are on a fresh branch `$$BRANCH$$-autofixes` whose tip matches that
branch. A pull request back into $$BRANCH$$ will be opened from your commits.

Read `issues.ignore.md` and fix every issue it lists, following the repo's existing conventions.

## Commits
- Make as MANY small commits as necessary -- one logical fix (or one tightly-related group) per
  commit. The reviewer will `git rebase -i` and drop any commit they disagree with, so every commit
  must stand alone and be independently revertible.
- When the SAME fix applies in several places (e.g. one rename across files), put all those edits in
  ONE commit.
- Use the repo's conventional prefixes: `fix:`, `refactor:`, `chore:`. Concise subject; short body
  when useful.
- Do NOT add a co-author trailer, a "Generated with Claude Code" line, or a robot emoji. Commits are
  authored solely by the repo's default git user; do not set GIT_AUTHOR_*/GIT_COMMITTER_*.
- Do NOT commit `issues.ignore.md` (it is git-ignored) or unrelated lock-file churn.

## Verify before committing each fix
- Run the tests RELEVANT to the code you changed (target specific files or `-k`):
  `uv run pytest <paths> -k <name> -m 'not (tool or self_hosted or mujoco or self_hosted_large)'`
- Run `uv run mypy` and ensure you introduce no new type errors.
- Only commit a fix once its relevant tests and mypy pass. If a fix can't be made to pass, skip it
  (note why in your summary) rather than committing broken code.

## Final quality gate
- Before finishing, run the full pre-commit suite the same way CI does:
  `pre-commit run --all-files` (use `uvx pre-commit run --all-files` if pre-commit is not on PATH).
  Fold any auto-formatting/lint fixes into the relevant commit (amend) or a final `style:` commit.
  Keep the PR focused -- revert any sweeping changes pre-commit makes to files unrelated to your
  fixes.

## Scope
- Only change code to address the recorded issues; no unrelated refactors.
- If `issues.ignore.md` is empty or lists nothing actionable, make no commits and stop.
- If something is too complicated or too controversial to fix, don't do it. The
  idea behind this is to automate quick wins. If something is hard, it should be
  left to human supervision.

When done, summarize what you changed and which issues you skipped and why.
