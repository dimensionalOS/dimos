# dimos — conventions for implementing agents

Distilled must-knows. Canonical long-form docs (read when in doubt): root
`AGENTS.md`, `docs/coding-agents/style.md`, `docs/coding-agents/code-quality-rules.md`,
`docs/coding-agents/testing.md`, `CONTRIBUTING.md`. This sheet only lists the
rules that break CI or reviews when missed, plus ruthwik's personal workflow.

## Setup & commands

| What | Command |
|---|---|
| New worktree (provisions venv + direnv) | `bin/worktree new --dir ../worktrees/<task> <branch> ruthwik/main` |
| Install / sync deps | `uv sync --all-groups` (uv ≥ 0.9.17; python 3.10–3.12, dev on 3.12) |
| Fast test suite | `./bin/pytest-fast` |
| One test file / one test | `uv run pytest dimos/path/test_x.py -k test_name` |
| Type check (strict) | `uv run mypy` |
| Lint + format + hooks | `pre-commit run --all-files` |
| Full CI parity | `bin/ci-check` (avoid `bin/pr-check` — it diffs against stale `dev`) |
| Regenerate blueprint registry | `pytest dimos/robot/test_all_blueprints_generation.py` |

## Hard rules — enforced by pytest suite `dimos/codebase_checks/` (CI fails)

- No `__all__` anywhere under `dimos/`. For re-export-only imports use `# noqa: F401`.
- No `__init__.py` (except root `dimos/__init__.py`) and no `__init__` re-exports —
  import symbols from the module that defines them.
- No comment banners / section dividers (`# ====`, `# region`). If a file needs
  sections, split it into a directory of focused files ("split module pattern").
- No `_ = ...` assignments (tuple unpacking `a, _ = f()` is fine).
- No `logging.getLogger` — use `logger = setup_logger()` (structlog, key/value pairs).
- Never edit `dimos/robot/all_blueprints.py` by hand — it is generated (see table above).
- Test files must not call `__new__` to bypass `__init__`.

## Do

- Type-annotate everything; `mypy --strict` must pass (test files are excluded from mypy).
- Imports at top of file (inline only for circular deps); `requests` not `urllib`;
  `Any` not `object` for JSON; `NDArray[np.uint8]` not bare `np.ndarray`.
- Config via `GlobalConfig` / module `config:` models — no hardcoded ports, IPs, or
  URLs; listen on `global_config.listen_host`.
- Concurrency: `threading.Event` not bool flags; lock reads too; register/dispose
  every `.subscribe()`; `start()`/`stop()` symmetry; `DEFAULT_THREAD_JOIN_TIMEOUT`
  for joins; remember `@rpc` methods run on other threads.
- Keep docstrings lean — no over-documentation, no stale comments, types in
  annotations not docstrings. (Recent history has whole PRs just trimming these.)
- Blueprints: set only what differs from defaults; no lambdas (unpicklable);
  no work at import time; `_` prefix for helper blueprints, `demo_` for demos.

## Don't

- Don't add a top-level `tests/` dir — tests live next to code
  (`dimos/core/pubsub.py` → `dimos/core/test_pubsub.py`).
- Don't let tests skip on missing deps — they should fail (`pytest-error-for-skips`).
  No fixed sleeps; suite has per-session thread-leak detection and 600 s timeouts.
- Don't add files > 75 KB (`largefiles` hook) or write state/output into the repo
  (use `STATE_DIR`/XDG or `.ignore.*` naming).
- Don't touch `pyproject.toml`/`MANIFEST.in` casually — packaging globs are fragile
  and changes trigger `release-build-check`; `uv.lock` must stay in sync
  (`uv lock` after `pyproject.toml` edits).
- Don't write illustrative code blocks in `docs/**.md` — CI *executes* them
  (`md-babel`); doc file links are validated by `doclinks`.
- Don't put scripts in `bin/` — use `[project.scripts]`.

## Tests

- pytest 8.3.5, asyncio auto-mode, parallel (`--dist=loadfile`). Default run
  excludes markers `self_hosted`, `mujoco`, `self_hosted_large`, `web_browser`.
- Dev-only scratch tests: `tool_*.py` (never collected); manual scripts: `demo_*`.
- Hermetic tests: `mocker`/`monkeypatch`, AAA ordering (see
  `.agents/skills/python-unit-tests/SKILL.md`).

## Git & PRs

- Branch names: `ruthwik/<type>/<description>`, type ∈ feat fix chore refactor docs
  (enforced by `bin/pr-name-check`).
- Conventional commits: `type(scope): subject`. The commit-msg hook silently
  truncates anything from `Generated with` / `Co-Authored-By` onward.
- PRs target `main` (unstable dev branch; merge queue). Never push `main`.
- Batch commits, push once — every push burns ~1 h of self-hosted CI.
- Fill out **every** section of the PR template (Problem, Solution, How to Test
  with a runnable one-liner, AI assistance, CLA) — stripped templates get closed.
- PR gates: lint (mypy + all pre-commit hooks), rust, md-babel, docs-validate,
  web (deno), tests matrix (py 3.10–3.14), self-hosted tests.

## LFS & data

- **Always `GIT_LFS_SKIP_SMUDGE=1` on every git operation.** A full LFS fetch
  pulls multi-GB objects and some 404 on the server.
- `data/*` is gitignored except `data/.lfs/*.tar.gz`; code fetches via
  `get_data("name")`. To add data: put it in `data/`, run `./bin/lfs_push`,
  commit the produced tarball.

## Personal workflow (ruthwik/main baseline — not company policy)

- All worktrees branch off `ruthwik/main` (= origin/main + `spec:` commits
  carrying `openspec/`). Feature context: copy
  `openspec/templates/feature-context.md` → `openspec/changes/<slug>/context.md`,
  fill it in, commit as `spec: ...`.
- Every commit touching `openspec/` is prefixed `spec:`. **openspec content must
  never reach origin/main** — before a PR:
  `git rebase --onto origin/main ruthwik/main <branch>` (then drop any remaining
  `spec:` commits).
- Track status on Agent Board: `board move <id> in_progress` when starting,
  `in_review` when implemented. Only the human moves cards to `done`.

## Known noise

- `warning: unable to access '_help/.gitignore': Too many levels of symbolic
  links` on every git command is harmless — `_help/` is local symlink clutter.
- A license header is auto-injected into every `.py`/`.rs` by pre-commit.
