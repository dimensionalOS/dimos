# Project Change Log

This file tracks project-level changes made in this fork/branch. It is not a
release changelog; it records architecture, code, testing, and upstream-sync
work so future contributors can understand why the fork differs from upstream.

Entries are listed in reverse chronological order.

## 2026-05-19

- Branch: `refactor/go2-architecture-layers`
- Summary: Expanded the Layer 3 design document with a required
  function-level implementation-note standard and detailed notes for the
  current ContextProvider, ExpertRouter, SkillOutcomeStore, and
  SkillOutcomePredictor entry points.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Documentation-only change; runtime tests were not required.
- Open items:
  - Keep adding implementation notes whenever Layer 3 storage, data shape,
    decision logic, or public MCP/RPC behavior changes.

## 2026-05-19

- Branch: `refactor/go2-architecture-layers`
- Summary: Renamed the Go2 layer stage document to `GOAL_1.md` and added a
  Layer 3 implementation design document for readable change rationale.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
- Validation:
  - Ran `python -m py_compile` for the reorganized layer package entrypoints.
  - Ran `git diff --check`.
- Open items:
  - Keep `DESIGN.md` updated whenever Layer 3 data flow or module boundaries
    change.

## 2026-05-19

- Branch: `refactor/go2-architecture-layers`
- Summary: Reorganized the Go2 layers package so each architecture layer has
  its own directory, while preserving stable layer-level import paths.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/`
- Validation:
  - Ran `python -m py_compile` for the reorganized layer packages and focused
    Layer 3 tests.
  - Ran `git diff --check`.
- Open items:
  - Decide later whether layer directories should grow separate README files
    once Layer 4/5/6 contain more than blueprint composition.

## 2026-05-19

- Branch: `refactor/go2-architecture-layers`
- Summary: Added the first Layer 3 skill-outcome loop for Go2: a recent outcome
  store, a rule-based preflight predictor, and ContextProvider integration.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/`
- Validation:
  - Ran `python -m py_compile` for Layer 3 provider/router/outcome modules,
    blueprint wiring, and focused tests.
  - Ran `git diff --check`.
  - Attempted the focused pytest files; local Windows environment still fails
    during repo `conftest.py` import because the Unix-only `resource` module is
    unavailable.
- Open items:
  - Decide whether `McpClient` prompt policy should explicitly require
    `record_skill_outcome` after important tool calls.
  - Add automatic outcome capture later if the MCP server/tool wrapper gains a
    safe cross-module event sink.

## 2026-05-17

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented the first Go2 Layer 3 ExpertRouter and documented the
  Layer 3 v1-v4 roadmap.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
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
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/`
  - `dimos/perception/temporal_memory_spec.py`
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
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
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
