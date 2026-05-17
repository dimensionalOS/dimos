# Project Change Log

This file tracks project-level changes made in this fork/branch. It is not a
release changelog; it records architecture, code, testing, and upstream-sync
work so future contributors can understand why the fork differs from upstream.

Entries are listed in reverse chronological order.

## 2026-05-17

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented the first Go2 Layer 3 ExpertRouter and documented the
  Layer 3 v1-v4 roadmap.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/expert_router.py`
  - `dimos/robot/unitree/go2/blueprints/layers/test_expert_router.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain.py`
  - `dimos/robot/unitree/go2/blueprints/layers/README.md`
- Validation:
  - Ran `python -m py_compile` for the new ExpertRouter, test, and Layer 3
    blueprint files.
  - Ran `git diff --check`.
  - Attempted the targeted pytest file; local Windows environment still fails
    during repo `conftest.py` import because the Unix-only `resource` module is
    unavailable.
- Open items:
  - Feed route results into the LLM prompt or prompt policy so the agent calls
    `route_task` and `get_context` consistently.
  - Implement `SkillOutcomePredictor` after a shared skill-outcome store exists.

## 2026-05-17

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented the first Go2 Layer 3 ContextProvider and wired it into
  the internal Agent Brain layer.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/context_provider.py`
  - `dimos/perception/temporal_memory_spec.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain.py`
- Validation:
  - Ran `python -m py_compile` for the new ContextProvider, spec, test, and
    Layer 3 blueprint files.
  - Attempted the targeted pytest file; local Windows environment still fails
    during repo `conftest.py` import because the Unix-only `resource` module is
    unavailable.
- Open items:
  - Add a shared skill-outcome store so ContextProvider can report recent
    Layer 5 outcomes.
  - Decide whether to promote the Go2-specific provider into a robot-agnostic
    `dimos.agents` module after the API stabilizes.

## 2026-05-17

- Branch: `refactor/go2-architecture-layers`
- Summary: Added the project change-log convention and documented the intended
  ContextProvider scope for the Go2 architecture-layer migration.
- Files/modules:
  - `PROJECT_CHANGELOG.md`
  - `AGENTS.md`
  - `dimos/robot/unitree/go2/blueprints/layers/README.md`
- Validation:
  - Documentation-only change; no runtime tests required.
- Open items:
  - Implement `ContextProvider` as a Layer 3 module exposed through MCP.
  - Define the first stable context payload returned by `get_context(task)`.

## 2026-05-17

- Branch: `refactor/go2-architecture-layers`
- Summary: Split the Go2 blueprint composition into architecture-layer
  boundaries for layers 3 through 6 without moving underlying implementation
  modules or changing public blueprint names.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/`
  - `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_spatial.py`
  - `dimos/robot/unitree/go2/blueprints/agentic/`
- Validation:
  - Ran Python compile checks for the new/changed Go2 blueprint files.
  - Attempted blueprint-registry tests; local Windows dependency/platform
    blockers prevented a full local run.
- Open items:
  - Fill Layer 3 with ContextProvider, ExpertRouter, outcome prediction, and
    causal world-model behavior.
  - Gradually convert key skills to structured `SkillResult` returns.
