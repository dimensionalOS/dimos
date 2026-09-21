You are fixing the issues recorded in `issues.ignore.md` in this working tree. The branch under
review is $$BRANCH$$; you are on a fresh branch `$$BRANCH$$-autofixes` whose tip matches that
branch. A pull request back into $$BRANCH$$ will be opened from your commits.

Read `issues.ignore.md` and fix every issue it lists, following the repo's existing conventions.

## Commits
- Make as MANY small commits as necessary -- one logical fix (or one tightly-related group) per
  commit. Each commit must stand alone and be independently revertible, so the reviewer can drop any
  single one they disagree with.
- When the SAME fix applies in several places (e.g. one rename across files), put all those edits in
  ONE commit.
- Use the repo's conventional prefixes: `fix:`, `refactor:`, `chore:`. Concise subject; short body
  when useful.
- ONLY add new commits with `git add` + `git commit`. Do NOT run `git rebase`, `git merge`,
  `git cherry-pick`, `git stash`, `git commit --amend`, or anything else that rewrites history or can
  leave a conflict. When you finish, the working tree MUST be clean (`git status` shows nothing to
  commit) -- no uncommitted or unmerged changes left behind.
- Do NOT add a co-author trailer, a "Generated with Claude Code" line, or a robot emoji. Commits are
  authored solely by the repo's default git user; do not set GIT_AUTHOR_*/GIT_COMMITTER_*.
- Do NOT commit `issues.ignore.md` (it is git-ignored) or unrelated lock-file churn.

## Verify before committing each fix
- Run the tests RELEVANT to the code you changed (target specific files or `-k`):
  `uv run pytest <paths> -k <name> -m 'not (self_hosted or mujoco or self_hosted_large)'`
- Run `uv run mypy` and ensure you introduce no new type errors.
- Only commit a fix once its relevant tests and mypy pass. If a fix can't be made to pass, skip it
  (note why in your summary) rather than committing broken code.

## Final quality gate
- Before finishing, run the full pre-commit suite the same way CI does:
  `pre-commit run --all-files` (use `uvx pre-commit run --all-files` if pre-commit is not on PATH).
  Commit any auto-formatting/lint fixes it makes as a final `style:` commit (do NOT amend). Keep the
  PR focused -- revert any sweeping changes pre-commit makes to files unrelated to your fixes.

## Scope
- Only change code to address the recorded issues; no unrelated refactors.
- If `issues.ignore.md` is empty or lists nothing actionable, make no commits and stop.
- If something is too complicated or too controversial to fix, don't do it. The
  idea behind this is to automate quick wins. If something is hard, it should be
  left to human supervision.

When done, summarize what you changed and which issues you skipped and why.
