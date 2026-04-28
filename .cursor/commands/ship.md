# /ship — one-liner request → branch → verify → PR → auto-merge

> Usage: in Cursor chat, type `/ship <one-liner>`. Examples:
>
> ```
> /ship add a --no-window flag to dimos/utils/view_map.py for headless selftest
> /ship clamp the xArm6 joint velocities in dimos/robot/xarm/skill_container.py
> /ship document the McpServer port override in docs/usage/mcp.md
> ```
>
> Treat the steps below as a strict script. **If any step fails, stop and
> report — do not silently retry, force, or skip steps to keep the flow
> moving.**

---

## Steps (run in order)

### 1. Preflight

- Confirm we are in `/Users/zhang/project/dimos` (or the user's checkout
  of `dimensionalOS/dimos`).
- Read `AGENTS.md` and `.cursor/rules/00-workflow.mdc` into context.
- Run `git status` and `git branch --show-current`:
  - On `dev` with a clean tree → step 2.
  - On `dev` with uncommitted changes → ask the user: stash, commit
    elsewhere, or branch off and bring them along?
  - On any other branch → ask whether to keep iterating on it or to
    cut a fresh branch off `origin/dev`.

### 2. Cut a fresh branch off `origin/dev`

```bash
git fetch origin
git checkout -b <type>/<topic> origin/dev
```

`<type>` ∈ {`feat`, `fix`, `chore`, `docs`, `test`, `refactor`, `perf`}.
`<topic>` is kebab-case and ≤ 50 chars.

### 3. Plan the minimum change

- Convert the one-liner into 3–5 todos via `TodoWrite`.
- Constrain the diff to the **smallest set of files** that satisfy the
  request. **Do not** opportunistically refactor neighboring code.
- If the request actually needs to touch ≥ 5 files **or** crosses module
  boundaries (e.g. agents + robot + transport), **stop and ask the user
  whether to split into multiple PRs**.

### 4. Implement

- Walk the todos in order, marking each one `completed` as you go.
- Python: imports must succeed, mypy must remain strict-clean.
- yaml: must round-trip parse.
- Touching `@skill`: docstring stays, every param keeps a type
  annotation, return type stays `str`. Never stack `@rpc` + `@skill`.
- Touching a blueprint: run
  `pytest dimos/robot/test_all_blueprints_generation.py` and commit the
  regenerated `dimos/robot/all_blueprints.py` in the same PR.

### 5. Verify locally (mandatory)

```bash
bash scripts/verify.sh
```

- PASS → step 6.
- FAIL → debug loop: read traceback / mypy / pytest failure → minimal
  fix → re-run verify. **Never** comment out a failing test or relax
  mypy to make this pass.
- If the failure is clearly pre-existing repo debt (not introduced by
  this change), **stop and ask the user** whether to fix it in this PR
  or open a separate issue.

### 6. Stage the diff

```bash
git diff --stat
git add <only files this change touches>
```

**Do not** `git add -A`; the workspace may have unrelated dirty files
from another agent / window. If you see something you did not write,
stop and ask.

### 7. Commit

Conventional title:

```
<type>: <one-liner>

<optional 1–3 line body explaining the WHY, not the WHAT>
```

Forbidden: `--amend` on a pushed commit, `--no-verify` to skip pre-commit.

### 8. Push

```bash
git push -u origin <branch>
```

- First push **must** include `-u` to set upstream.
- On failure (network / permission), stop and report. **Never** retry
  with `--force`.

### 9. Open the PR (REST API)

`gh pr create` immediately after `git push` sometimes hits a GraphQL
indexing race. Use REST:

```bash
gh api repos/dimensionalOS/dimos/pulls \
  -X POST \
  -f base=dev \
  -f head=<branch> \
  -f title="<type>: <one-liner>" \
  -F body=@/tmp/pr-body-<branch>.md
```

PR body must follow `.github/pull_request_template.md`:

- **Description**: 1–3 bullets, what + why.
- **How to Test**: the exact `bash scripts/verify.sh` invocation, plus
  any blueprint / hardware command if relevant.
- **CLA**: keep the existing checkbox; tick only after reading.
- **Closes DIM-XXX**: linked Linear issue if any.

### 10. Enable auto-merge (squash)

```bash
gh pr merge --auto --squash <pr-url-or-number>
```

If `gh` reports "auto-merge not enabled on this repository", proceed to
step 11 with a human-todo entry.

### 11. Final report

Output exactly this format:

```
## Done

### Changes
- <file 1>: <one line>
- <file 2>: <one line>

### Verification
- bash scripts/verify.sh: PASS / FAIL (paste the last 5 lines on FAIL)

### PR
- <url> (auto-merge: enabled / not enabled)

### Codex review
- <pending / posted reaction / wrote review> (skip if Codex disabled)

### Human todo (if any)
- [ ] <e.g. enable auto-merge in repo settings>
- [ ] <e.g. run hardware regression on Go2 dock>
- [ ] <e.g. tag a release>
```

---

## Failure / abort rules

- Any failed step: stop, surface state, do not silently retry.
- If the user types `/cancel` or asks to abort, **leave the branch in
  place** so they can decide what to do with it. Do not delete.
- Forbidden recoveries:
  - skipping `verify.sh`,
  - removing tests / assertions to "make it pass",
  - editing files outside the agreed scope just because they are dirty,
  - `git push --force` to a reachable branch,
  - committing to `dev` or `main` directly,
  - clicking GitHub web settings on the user's behalf.
