# For Agents

├── worktrees.md (creating provisioned worktrees with `bin/worktree`)
├── style.md (code style guidelines for dimos)
├── testing.md (docs about writing tests)
├── ../development/openspec.md (OpenSpec behavior-spec workflow)
├── docs (these are docs about writing docs)
│   ├── codeblocks.md
│   ├── doclinks.md
│   └── index.md
└── index.md

## Manipulation backend notes

- Use the active backend protocols in `dimos/manipulation/planning/backends/base.py` for backend-neutral manipulation work.
- Keep optional RoboPlan imports inside `dimos/manipulation/planning/backends/roboplan/`; default Drake-backed stacks must not import RoboPlan.
- Set RoboPlan RRT, IK, and TOPPRA options explicitly in adapter code instead of relying on upstream binding defaults.
- Do not edit `dimos/robot/all_blueprints.py` manually. Regenerate it with `pytest dimos/robot/test_all_blueprints_generation.py` only if new RoboPlan blueprints are exported.
