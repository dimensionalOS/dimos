# One-run worker: dynamic robot-relationship video

Execute exactly one earliest PENDING micro-spec from:

- `.hermes/automation/dynamic-relation-video/CHECKLIST.md`
- `.hermes/automation/dynamic-relation-video/PROGRESS.md`

## Hard boundaries

- Work only in `/Users/tian/dimos-worktrees/dynamic-robot-relation-video` on branch `feat/dynamic-robot-relation-video`.
- Never edit `/Users/tian/dimos-worktrees/replayable-video-relation-evals` or PR #2989.
- Do not broaden scope into question generation, bundles, scoring, replay, identity stitching, online streaming, 3D geometry, or an importance model.
- One invocation = one micro-spec = at most one coherent implementation commit. Step 04 may produce evidence without a commit if no fix is needed, but it must atomically advance the progress ledger and commit that ledger update.
- Strict RED-GREEN-REFACTOR: write one behavior test, run and observe the expected failure, implement minimally, rerun focused and regression gates.
- Use explicit `git add` paths; never `git add .`, reset, clean, stash, amend, rebase, force-push, or modify unrelated files.
- Generated video, extracted frames, models, logs, and `.artifacts/` remain untracked.

## Preflight

1. Read both control files completely.
2. Verify branch, worktree, status, local head, and fork head.
3. If another worker has dirty owned files, preserve them and continue the same earliest step; do not discard work.
4. If dirty files are unrelated or unsafe, set `AUTOMATION_STATUS: BLOCKED` with exact evidence, commit only the ledger if safe, push, and stop.
5. If status is `COMPLETE` or `BLOCKED`, stop without changes.

## Per-step delivery

1. Select only `CURRENT_STEP`, which must be the earliest PENDING checklist row.
2. Follow that micro-spec exactly and keep APIs minimal.
3. Run the focused tests, Ruff for touched Python, and mypy for touched source.
4. Run one synchronous read-only review using a separate foreground Hermes process if available; otherwise perform an explicit self-review and record that limitation.
5. Atomically update all progress views: header, table row, evidence log, and next step.
6. Verify no generated media, model weights, logs, secrets, or machine-local links are staged.
7. Commit explicit files with the checklist subject, push to `fork feat/dynamic-robot-relation-video`, and verify local/fork SHA equality.
8. Exit after one delivered micro-spec.

## Final step

On Step 05, run the entire package suite plus Ruff and mypy, inspect the final diff, mark all progress views `COMPLETE`, push, verify SHA, and stop. Do not open a PR from the worker; the parent session will prepare a clean follow-up after #2989 merges.
