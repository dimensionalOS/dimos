# Project Change Log

This file tracks project-level changes made in this fork/branch. It is not a
release changelog; it records architecture, code, testing, and upstream-sync
work so future contributors can understand why the fork differs from upstream.

Entries are listed in reverse chronological order.

## 2026-07-05

- Branch: `refactor/go2-architecture-layers`
- Summary: Added explicit interface input/output references to the English and
  Chinese Go2 architecture review guides. The new quick reference enumerates
  Layer 4/5/6 Specs, Layer 3 MCP skills, ledger/proposal schemas, world-model
  contracts, MCP runtime surfaces, and dashboard update interfaces with their
  expected inputs and returned outputs.
- Files/modules:
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.md`
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.zh.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Documentation-only change.
  - Ran `git diff --check` on the changed review/changelog files.
- Open items:
  - Keep the interface quick reference synchronized with any future signature
    or schema changes.

## 2026-07-05

- Branch: `refactor/go2-architecture-layers`
- Summary: Normalized every local Go2 architecture/self-evolution review-guide
  section to the same implementation-review depth as the causal world model
  section. Added per-feature component breakdowns, inputs, data flows,
  decision ownership, state/write boundaries, failure modes, and safety/review
  boundaries in both English and Chinese.
- Files/modules:
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.md`
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.zh.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Documentation-only change.
  - Ran `git diff --check` on the changed review/changelog files.
- Open items:
  - During code review, keep the review guide aligned with any implementation
    changes requested by reviewers.

## 2026-07-05

- Branch: `refactor/go2-architecture-layers`
- Summary: Expanded the English and Chinese Go2 architecture review guides with
  finer implementation granularity, including entry points, inputs, decision
  ownership, state/write boundaries, failure modes, and a detailed explanation
  of the causal world model's online linear model, hand-authored SCM, and
  observational estimator.
- Files/modules:
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.md`
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.zh.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Documentation-only change.
  - Ran `git diff --check` on the changed review/changelog files.
- Open items:
  - Keep these review guides in sync if the implementation changes after code
    review.

## 2026-07-05

- Branch: `refactor/go2-architecture-layers`
- Summary: Synchronized the branch with upstream `main` at `6e813a72`, resolved
  conflicts against the Go2 layered architecture, and added a detailed review
  guide in English and Chinese for the full Go2 agent architecture/self-evolution
  feature set.
- Files/modules:
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.md`
  - `docs/reviews/2026-07-05-go2-agent-architecture-review.zh.md`
  - `dimos/agents/mcp/mcp_server.py`
  - `dimos/agents/mcp/test_mcp_server.py`
  - `dimos/perception/spatial_perception.py`
  - `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py`
  - `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_temporal_memory.py`
  - `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_spatial.py`
  - `dimos/robot/unitree/go2/test_connection.py`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Installed missing upstream dependency `eclipse-zenoh==1.9.0` into the local
    `.venv`.
  - Ran focused MCP server tests:
    `10 passed, 1 warning`.
  - Ran focused Go2 connection tests:
    `5 passed`.
  - Ran full focused Go2 Layer 3 agent-brain suite:
    `62 passed`.
  - Ran focused Go2 Layer 4 world-state suite:
    `3 passed, 1 warning`.
  - Ran focused spatial-memory config tests:
    `2 passed`.
  - Ran ModuleCoordinator suite under LCM:
    `35 passed`.
  - Ran MCP client/tool-stream suite under LCM on port 9991:
    `44 passed, 1 warning`.
  - Ran blueprint registry generation check:
    `1 passed`.
  - Ran `git diff --cached --check -- ':(exclude)*.patch'`.
- Open items:
  - Upstream now defaults macOS transport to Zenoh; some tests expose native
    pyo3 thread lifetime issues under that backend. The same suites pass with
    `DIMOS_TRANSPORT=lcm`.
  - MCP tests must avoid port `9990` on this machine while a live `dimos`
    process is running there.
  - Upstream `.patch` files contain whitespace that trips full
    `git diff --check`; excluded patch files during validation to avoid
    altering patch semantics.

## 2026-07-04

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented Go2 agent self-evolution plan Tasks 5-6 by extracting
  context evidence selection into a pure policy helper and adding a
  proposal-only skill-interface generator. The generator suppresses proposals
  for missing arguments/context and duplicate existing contracts, and writes
  only human-reviewable `dimos.skill_proposal.v1` proposals through the ledger.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_evidence.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_proposal.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_ledger.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/prompt_policy.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_evidence.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_skill_proposal.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran focused context-evidence policy test:
    `4 passed in 0.03s`.
  - Ran focused ContextProvider regression test:
    `4 passed in 0.67s`.
  - Ran focused skill-proposal test:
    `4 passed in 0.52s`.
  - Ran full focused Layer 3 agent-brain suite:
    `62 passed in 6.64s`.
  - Ran focused Layer 4 world-state suite:
    `3 passed, 1 warning in 6.13s`.
  - Ran Ruff check/format-check on Layer 3 agent-brain files and
    `git diff --check`.
- Open items:
  - Continue evaluating these policies against replay/simulation data before
    promoting them to robot-agnostic agent modules.

## 2026-07-04

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented Go2 agent self-evolution plan Tasks 2-4 by adding a
  Git-backed evolution ledger, a deterministic task feasibility preflight, and
  context evidence feedback recording. These modules add a low-volume
  reviewable control plane without replacing SpatialMemory, TemporalMemory,
  SkillOutcomeStore, CausalWorldModel, or Layer 5 skill contracts. Also added
  explicit world-model output contracts and review-only proposal validation for
  ledger proposal files.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_event.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_ledger.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_proposal.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/task_feasibility.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_feedback.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/world_model_contract.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/causal_world_model.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/prompt_policy.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_evolution_ledger.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_evolution_proposal.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_world_model_contract.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_task_feasibility.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_feedback.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran focused ledger test:
    `4 passed in 0.76s`.
  - Ran focused task-feasibility test:
    `5 passed in 0.66s`.
  - Ran focused context-feedback test:
    `5 passed in 0.68s`.
  - Ran focused world-model/proposal contract tests:
    `8 passed in 0.33s`.
  - Ran focused causal-world-model test:
    `15 passed in 2.34s`.
  - Ran full focused Layer 3 agent-brain suite:
    `54 passed in 6.59s`.
  - Ran focused Layer 4 world-state suite:
    `3 passed, 1 warning in 7.77s`.
  - Ran Ruff check/format-check on Layer 3 agent-brain files and
    `git diff --check`.
- Open items:
  - Continue with Task 5, extracting context-evidence policy into a pure helper.
  - Continue with Task 6, proposal-only skill interface evolution.

## 2026-07-04

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented the first Go2 agent self-evolution plan task by adding a
  read-only `memory_backend_status()` MCP skill. The status report shows which
  context, RAG, temporal, outcome, causal, and skill-interface backends are
  wired and whether small read probes succeed, without introducing another
  memory store.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/memory_backend_status.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_memory_backend_status.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran focused TDD test for `memory_backend_status()`:
    `3 passed in 0.43s`.
  - Ran full focused Layer 3 agent-brain suite:
    `31 passed in 4.14s`.
  - Ran focused Layer 4 world-state suite:
    `3 passed, 1 warning in 9.20s`.
  - Ran Ruff check/format-check on touched Python files and `git diff --check`.
- Open items:
  - Continue with the Git-backed evolution ledger task.

## 2026-07-04

- Branch: `refactor/go2-architecture-layers`
- Summary: Added a six-step engineering implementation plan for Go2 agent
  self-evolution, ordered to avoid redundant memory, skill, and decision
  systems while building on the existing Layer 3/4/5 architecture.
- Files/modules:
  - `docs/plans/2026-07-04-agent-self-evolution-implementation-plan.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Documentation-only planning change; no runtime tests required.
- Open items:
  - Execute the plan task-by-task with TDD, starting with
    `memory_backend_status()`.

## 2026-07-04

- Branch: `refactor/go2-architecture-layers`
- Summary: Added `context_evidence.v1` to the Go2 Layer 3 ContextProvider so
  each compact context response records which task, runtime, robot, RAG,
  temporal, skill, causal, and world-model evidence was selected and why. Also
  made the Layer 4 package entrypoint lazy so Layer 3 imports no longer pull in
  the heavy visual-memory stack when only specs/submodules are needed.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/__init__.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/DESIGN.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran focused ContextProvider pytest with default plugins:
    `4 passed in 0.61s`.
  - Verified direct `ContextProvider` import no longer imports the heavy Layer
    4 spatial-memory stack.
- Open items:
  - Persist selected context evidence and task outcomes to a Git-backed
    evolution ledger in a later change.
  - Replace coarse deterministic evidence labels with evaluated scoring once
    replay/simulation metrics are available.

## 2026-05-28

- Branch: `refactor/go2-architecture-layers`
- Summary: Made the security patrol tracker initialize lazily so the Go2
  agentic blueprint can deploy in CPU-only MuJoCo/WSL environments when
  security patrol is not used. Calling `start_security_patrol` still reports
  the EdgeTAM CUDA requirement if no compatible GPU is available.
- Files/modules:
  - `dimos/experimental/security_demo/security_module.py`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile dimos/experimental/security_demo/security_module.py`.
  - Instantiated `SecurityModule(camera_info=CameraInfo())` in UbuntuDev using
    the existing WSL dependency environment; construction completed on CPU.
- Open items:
  - Add a CPU-capable tracker fallback if security patrol needs to run without
    CUDA.

## 2026-05-22

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented the first Go2 Layer 5 V2 consistency check by comparing
  static skill contracts against MCP tool names from the full Go2 agentic
  blueprint, and filled previously missing contracts for MCP-exposed Go2 skills.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_registry.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_spec.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/test_skill_interface_registry.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile` for the changed Layer 5 spec, registry, and
    focused test file.
  - Ran focused Layer 5 pytest in WSL Ubuntu against the current Windows
    working tree using the existing WSL dependency environment:
    `9 passed, 1 warning`.
- Open items:
  - Add more precise argument constraints for Unitree sport commands and
    perception callback payloads.

## 2026-05-22

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented Go2 Layer 4 V3 by making the world-snapshot storage
  policy explicit and adding a full Go2 agentic blueprint static test for the
  Layer 3 -> Layer 4 -> Layer 6 RPC provider path.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/world_state_spec.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/structured_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
  - `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md`
- Validation:
  - Ran `python -m py_compile` for the changed Layer 4 spec,
    implementation, and focused test file.
  - Ran focused Layer 4 pytest in WSL Ubuntu against the current Windows
    working tree using the existing WSL dependency environment:
    `3 passed, 1 warning`.
- Open items:
  - If snapshot retention becomes necessary, revisit whether to persist
    normalized snapshots to `memory2` SQLite, JSONL, or TemporalMemory.

## 2026-05-22

- Branch: `refactor/go2-architecture-layers`
- Summary: Implemented Go2 Layer 4 V2 memory summaries by adding explicit
  semantic-temporal evidence entries and normalized named object/location
  summaries to the structured world snapshot.
- Files/modules:
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/semantic_temporal_map.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/structured_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py`
  - `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/DESIGN.md`
  - `dimos/robot/unitree/go2/blueprints/layers/GOAL_1.md`
- Validation:
  - Ran `python -m py_compile` for the changed Layer 4 implementation and
    focused test file.
  - Ran `git diff --check`.
  - Ran focused Layer 4 pytest in WSL Ubuntu against the current Windows
    working tree using the existing WSL dependency environment:
    `3 passed, 1 warning`.
  - Attempted focused pytest with system Python; blocked because `pytest` is
    not installed.
  - Attempted focused pytest with `uv run`; blocked by local Windows
    environment creation/cache errors before tests could run. Removed the
    partial `.venv` left by that failed attempt.
- Open items:
  - Add full Go2 agentic blueprint tests for the Layer 3 -> Layer 4 RPC path.

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
