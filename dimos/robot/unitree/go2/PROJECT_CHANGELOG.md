# Project Change Log

This file tracks project-level changes made in this fork/branch. It is not a
release changelog; it records architecture, code, testing, and upstream-sync
work so future contributors can understand why the fork differs from upstream.

Entries are listed in reverse chronological order.

## 2026-05-20

- Branch: `refactor/go2-architecture-layers`
- Summary: Started Go2 Layer 6 construction by adding an observational
  robot-body/local-policy state facade, connecting it into the internal Go2
  robot-body layer, and exposing the Layer 6 snapshot through Layer 4 and Layer
  3 context.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/robot_body_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/robot_body_spec.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/test_robot_body_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/structured_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile` for the new Layer 6 spec/state/test files,
    the Layer 6 package entrypoint, the changed Layer 4 world-state files, and
    the changed Layer 3 ContextProvider files.
  - Ran `git diff --check`.
  - Copied the working tree to `~/dimos-layer-test` in WSL and ran the focused
    Layer 3 + Layer 4 + Layer 5 + Layer 6 test suite with a minimal pytest
    dependency set: `30 passed, 1 warning`.
- Open items:
  - Add real connection heartbeat/health if `GO2Connection` exposes it.
  - Add command-stream tracking after confirming it will not interfere with
    movement-manager fan-out.

## 2026-05-20

- Branch: `refactor/go2-architecture-layers`
- Summary: Started Go2 Layer 5 construction by adding a static skill-interface
  contract registry, connecting it into the full Go2 skill-interface layer, and
  exposing the contract summary to Layer 3 `ContextProvider`. The Layer 5
  package entrypoint now lazily builds blueprint variables so lightweight spec
  imports do not pull in the full skill dependency stack.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_registry.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_spec.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/test_skill_interface_registry.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile` for the new Layer 5 spec/registry/test files,
    the Layer 5 package entrypoint, and the changed Layer 3 ContextProvider
    files.
  - Copied the working tree to `~/dimos-layer-test` in WSL and ran the focused
    Layer 3 + Layer 4 + Layer 5 test suite with a minimal pytest dependency
    set: `27 passed, 1 warning`.
- Open items:
  - Compare the static Layer 5 contracts against the MCP server tool list in a
    later version so renamed or missing skills are detected automatically.
  - Add more precise constraints for Unitree sport commands and perception
    callback payloads.

## 2026-05-20

- Branch: `refactor/go2-architecture-layers`
- Summary: Started Go2 Layer 4 construction by adding RPC-only
  semantic-temporal memory and structured world-state facades, then connected
  Layer 3 `ContextProvider` to prefer the Layer 4 world snapshot when present.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/semantic_temporal_map.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/structured_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/world_state_spec.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_causal_world_model.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_expert_router.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_skill_outcomes.py`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile` for the new Layer 4 modules, focused tests, and
    changed Layer 3 ContextProvider files.
  - Ran `git diff --check`.
  - Ran static Layer 4 connectivity assertions confirming the path:
    Go2 blueprint -> `_go2_spatial_world_state` -> `_Go2StructuredWorldState`
    -> `_Go2SemanticTemporalMap` -> existing spatial/temporal memory, and
    Layer 3 `ContextProvider` -> `WorldStateSpec.get_world_snapshot(...)`.
  - Installed WSL user-level `uv` and WSL system build dependencies needed for
    Python packages that compile extensions.
  - Copied the working tree to `~/dimos-layer-test` in WSL and ran the focused
    Layer 3 + Layer 4 test suite with a minimal pytest dependency set:
    `21 passed, 1 warning`.
- Open items:
  - Add safety and connection status to `robot_state`.
  - Decide whether stable world snapshots should be persisted in a later Layer
    4 version.

## 2026-05-19

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented the first Go2 Layer 3 `CausalWorldModel` for V3,
  including in-memory causal transitions, repeated failure-pattern summaries,
  ContextProvider integration, Predictor risk escalation, prompt policy updates,
  and focused tests.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/causal_world_model.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_causal_world_model.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_outcome_predictor.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/prompt_policy.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_prompt_policy.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile` for the changed MCP client, Layer 3 modules,
    prompt policy, and focused tests.
  - Ran `git diff --check`.
  - Attempted targeted pytest with `.venv\\Scripts\\python.exe -m pytest`;
    local Windows environment still fails during repo `conftest.py` import
    because the Unix-only `resource` module is unavailable.
- Open items:
  - Decide later whether causal transitions should be persisted to JSONL,
    SQLite, TemporalMemory, or another durable event store.

## 2026-05-19

- Branch: `refactor/go2-architecture-layers`
- Summary: Completed the remaining Go2 Layer 3 V2 behavior by adding a default
  Layer 3 decision prompt policy and automatic `McpClient` outcome recording
  for non-internal MCP tool calls.
- Files/modules:
  - `dimos/agents/mcp/mcp_client.py`
  - `dimos/agents/mcp/test_mcp_client_unit.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/prompt_policy.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_prompt_policy.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_outcome_store.py`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile` for the changed MCP client, tests, prompt
    policy, Layer 3 blueprint entrypoint, and outcome store.
  - Ran `git diff --check`.
  - Attempted targeted pytest with `.venv\\Scripts\\python.exe -m pytest`;
    local Windows environment still fails during repo `conftest.py` import
    because the Unix-only `resource` module is unavailable.
  - Attempted `uv run pytest`; local Windows dependency resolution is blocked
    because `dimos-viewer==0.30.0a6.dev99` has no Windows wheel.
- Open items:
  - V3 should add `CausalWorldModel` for before/action/result/after causal
    transition records.

## 2026-05-19

- Branch: `refactor/go2-architecture-layers`
- Summary: Narrowed the Go2 Layer 3 roadmap to stop at V3 for the current
  architecture stage and deferred robot-agnostic Layer 3 extraction until
  Layers 4, 5, and 6 are clearer.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Documentation-only change; runtime tests were not required.
- Open items:
  - Finish V2 remaining prompt/outcome-recording behavior, then implement the
    V3 `CausalWorldModel` before revisiting robot-agnostic extraction.

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
