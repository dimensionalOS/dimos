## 1. Triage candidate files

- [ ] 1.1 Compare each candidate file against the reference branch and decide whether it is required, independent, generated, docs-only, or accidental.
- [ ] 1.2 Extract only required or intentionally independent files.
- [ ] 1.3 Move docs back into earlier PRs when they explain earlier PR behavior.

## 2. Control/generated/docs extraction

- [ ] 2.1 Extract control/task changes if they are intentional.
- [ ] 2.2 Extract generated blueprint registry changes only when their source blueprint is included.
- [ ] 2.3 Extract `pyproject.toml` and `AGENTS.md` only if justified; otherwise report as excluded.

## 3. Validation

- [ ] 3.1 Run `uv run pytest dimos/control/test_control.py dimos/e2e_tests/test_control_coordinator.py dimos/robot/test_all_blueprints.py -q` for included files.
- [ ] 3.2 Run any docs or generated-registry checks required by included changes.
- [ ] 3.3 Return a file-by-file inclusion/exclusion summary.
