# Agent Self-Evolution Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add DimOS agent self-evolution in six incremental, testable steps without creating a redundant parallel memory or skill system.

**Architecture:** Keep RAG and existing memory backends as the source of high-volume observations. Add a thin Layer 3 evolution/control plane that reads Layer 4 memory state, Layer 5 skill contracts, `context_evidence.v1`, skill outcomes, and causal transitions, then records low-volume decisions and feedback to a Git-backed ledger. The LLM may propose feasibility decisions and skill improvements, but rules, schemas, tests, replay/sim validation, and human review remain the acceptance gates.

**Tech Stack:** Python modules, DimOS `Module`/`@skill`/`Spec` APIs, JSON/YAML event files, Git CLI through safe subprocess wrappers, existing Layer 3/4/5 Go2 blueprints, pytest, ruff.

---

## Engineering Guardrails

- Do not replace SpatialMemory, TemporalMemory, SkillOutcomeStore, CausalWorldModel, or SkillInterfaceRegistry.
- Do not store images, embeddings, ChromaDB data, SQLite rows, or large logs in Git.
- Do not let the LLM directly modify executable robot skills or merge generated changes.
- Add one narrow module per responsibility; avoid a generic "self evolution manager" until duplication is proven.
- Keep every new MCP skill read-only or proposal-only unless it is explicitly recording audit data.
- Prefer Go2-scoped Layer 3 modules first. Promote to robot-agnostic `dimos.agents` only after the contracts stabilize.

## Recommended Order

1. `memory_backend_status()` for observability.
2. Git-backed evolution ledger schema and writer.
3. `evaluate_task_feasibility(...)` before risky planning.
4. Context outcome feedback recording.
5. Context evidence policy extraction and thresholding.
6. Skill evolution proposal generation.

This order avoids redundancy: status tells us what is actually wired; the ledger gives every later step one audit sink; feasibility and feedback define what should be learned; evidence policy uses feedback rather than guesswork; skill proposal waits until there is repeated evidence that current skills are insufficient.

---

### Task 1: Memory Backend Status

**Purpose:** Make runtime memory state explicit before adding any evolution logic.

**Files:**
- Create: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/memory_backend_status.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/DESIGN.md`
- Test: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_memory_backend_status.py`

**Interface:**

```python
@skill
def memory_backend_status(self) -> SkillResult:
    """Report which DimOS memory backends are wired and how much data they expose."""
```

**Implementation Notes:**
- Inject optional `WorldStateSpec`, `SpatialMemorySpec`, `TemporalMemorySpec`, `SkillOutcomeStoreSpec`, `CausalWorldModelSpec`, and `SkillInterfaceSpec`.
- Return a JSON-shaped `SkillResult` metadata payload with:
  - `spatial_memory`: wired, query_available, match_count_probe, backend_hint
  - `temporal_memory`: wired, entity_count, graph_stats_available
  - `skill_outcomes`: wired, recent_count
  - `causal_world_model`: wired, transition_count, sample_count, persistence path if exposed
  - `skill_interface`: wired, skill_count
  - `warnings`: dependency errors without failing the whole call
- Do not inspect Chroma internals directly in V1 unless the module exposes a stable RPC. A probe query with `limit=1` is safer than coupling to Chroma file layout.

**TDD Steps:**
1. Write tests with stubs for wired and missing backends.
2. Verify the tests fail because `memory_backend_status` does not exist.
3. Implement the module and blueprint wiring.
4. Run:
   `./.venv/bin/python -m pytest dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_memory_backend_status.py -q`
5. Run Layer 3 blueprint/context focused tests.

**Redundancy Check:** This is not another memory store. It is a status/readiness view over existing stores.

---

### Task 2: Git-Backed Evolution Ledger

**Purpose:** Add one durable, reviewable audit sink for low-volume evolution events.

**Files:**
- Create: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_ledger.py`
- Create: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_event.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
- Modify: `dimos/core/global_config.py` only if a reusable config field is needed; otherwise use an env var first.
- Test: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_evolution_ledger.py`

**Interface:**

```python
@skill
def record_evolution_event(
    self,
    event_type: str,
    task: str = "",
    payload_json: str = "{}",
    commit: bool = False,
) -> SkillResult:
    """Record a low-volume agent evolution event to the local ledger."""
```

**Storage Layout:**

```text
.dimos/evolution/
  events/YYYY/MM/DD/<timestamp>-<event_type>.json
  proposals/
  README.md
```

**Event Shape:**

```json
{
  "schema": "dimos.evolution_event.v1",
  "timestamp": 0.0,
  "event_type": "context_feedback",
  "task": "...",
  "run_id": "...",
  "payload": {},
  "git": {
    "commit_requested": false,
    "commit_sha": ""
  }
}
```

**Implementation Notes:**
- Default path: repo root `.dimos/evolution`; allow override with `DIMOS_EVOLUTION_LEDGER_DIR`.
- Use structured JSON and atomic file writes.
- If `commit=True`, only commit files under the configured ledger dir. Never stage unrelated repo changes.
- Use non-interactive Git commands and return the commit SHA.
- If the ledger directory is not inside a Git worktree, record the file but return a warning instead of failing.

**TDD Steps:**
1. Test JSON event creation in a temp repo.
2. Test invalid `payload_json` fails with `INVALID_INPUT`.
3. Test `commit=True` stages only the event file.
4. Verify red tests before implementation.
5. Implement minimal file writer and optional commit path.

**Redundancy Check:** The ledger stores decisions and summaries, not raw memory. Chroma/SQLite remain the retrieval systems.

---

### Task 3: Prompt Feasibility Evaluator

**Purpose:** Make "can this prompt be done here, now, with current memory and skills?" an explicit preflight tool.

**Files:**
- Create: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/task_feasibility.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/prompt_policy.py`
- Test: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_task_feasibility.py`

**Interface:**

```python
@skill
def evaluate_task_feasibility(
    self,
    task: str,
    context_json: str = "{}",
) -> SkillResult:
    """Evaluate whether the current task is feasible before selecting physical skills."""
```

**Return Metadata:**

```json
{
  "feasible": "yes|no|uncertain",
  "missing_context": [],
  "required_skills": [],
  "available_skills": [],
  "safety_risks": [],
  "recommended_next_action": "proceed|get_context|ask_clarifying_question|refuse|stop",
  "clarifying_question": "",
  "evidence_sources": []
}
```

**Implementation Notes:**
- V1 should be deterministic/rule-based. Do not add an LLM call inside the tool yet.
- Consume `context_evidence`, runtime mode, robot state, and skill contracts.
- Use Layer 5 contracts for required context and motion-sensitive flags.
- Hardware mode and missing robot state should push risky movement tasks to `uncertain` or `no`.
- Record an optional `task_feasibility` event to the ledger only after Task 2 exists.

**TDD Steps:**
1. Test a speech task is feasible with `speak`.
2. Test movement task with missing robot state is `uncertain` and asks for context.
3. Test impossible/unsafe task returns `no` with safety risk.
4. Implement minimal deterministic classifier.

**Redundancy Check:** This complements `predict_skill_outcome`: feasibility evaluates a user task before a skill is chosen; prediction evaluates a specific planned skill call.

---

### Task 4: Context Outcome Feedback

**Purpose:** Connect selected context to actual task outcomes so later scoring is evidence-based.

**Files:**
- Create: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_feedback.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
- Modify: `dimos/agents/mcp/mcp_client.py` only if automatic recording is needed after manual V1 proves useful.
- Test: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_feedback.py`

**Interface:**

```python
@skill
def record_context_feedback(
    self,
    task: str,
    context_evidence_json: str,
    selected_skill: str = "",
    outcome_json: str = "{}",
    helpful_sources_json: str = "[]",
    ignored_risks_json: str = "[]",
) -> SkillResult:
    """Record whether selected context evidence helped or hurt the task outcome."""
```

**Implementation Notes:**
- Validate `context_evidence_json` schema version.
- Store a bounded in-memory summary for immediate `get_context` use.
- Write the full feedback event to the Git ledger when available.
- Do not auto-record every tool call in V1; manual/LLM-triggered feedback is enough to validate shape.

**TDD Steps:**
1. Test valid feedback records helpful and harmful source counts.
2. Test invalid JSON fails.
3. Test summary is bounded and newest-first.
4. Test ledger integration with a stub writer.

**Redundancy Check:** This is not another SkillOutcomeStore. SkillOutcomeStore records tool success/failure; ContextFeedback records whether selected evidence influenced the decision.

---

### Task 5: Context Evidence Policy Extraction

**Purpose:** Move evidence scoring/selection out of `ContextProvider` and make it tunable and testable.

**Files:**
- Create: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_evidence.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
- Test: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_evidence.py`
- Update: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`

**Interface:**

```python
def build_context_evidence(metadata: dict[str, Any], policy: ContextEvidencePolicy) -> dict[str, Any]:
    ...
```

**Policy Fields:**

```python
@dataclass(frozen=True)
class ContextEvidencePolicy:
    min_relevance_score: float = 0.0
    max_entries: int = 12
    include_low_confidence: bool = True
    require_robot_state_for_motion: bool = True
```

**Implementation Notes:**
- Keep the current `deterministic_source_coverage_v1` as the default policy.
- Add thresholds only after Task 4 feedback exists.
- Make the policy pure Python with no module or RPC dependencies.
- `ContextProvider` should call this helper and remain a coordinator.

**TDD Steps:**
1. Test default policy preserves current `context_evidence.v1` shape.
2. Test low-relevance spatial evidence can be filtered.
3. Test `max_entries` is enforced deterministically.
4. Test motion tasks retain robot/safety evidence.

**Redundancy Check:** This is a refactor plus policy, not a new memory router. It owns selection rules only.

---

### Task 6: Skill Evolution Proposal

**Purpose:** Let the agent propose skill interface improvements without modifying executable skill code.

**Files:**
- Create: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_proposal.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
- Modify: `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_registry.py` only if read APIs need small additions.
- Test: `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_skill_proposal.py`

**Interface:**

```python
@skill
def propose_skill_interface(
    self,
    task: str,
    failure_context_json: str = "{}",
) -> SkillResult:
    """Propose a new or revised skill interface without changing executable code."""
```

**Proposal Shape:**

```json
{
  "schema": "dimos.skill_proposal.v1",
  "proposal_id": "...",
  "task": "...",
  "problem": "...",
  "existing_skill_gaps": [],
  "proposed_interface": {
    "skill_name": "...",
    "domain": "...",
    "required_args": [],
    "optional_args": [],
    "risk_class": "low|medium|high",
    "recommended_preflight": []
  },
  "validation_plan": [],
  "requires_human_review": true
}
```

**Implementation Notes:**
- Read Layer 5 skill contracts and recent failures.
- Generate proposals deterministically from repeated known gaps in V1.
- Write proposals under `.dimos/evolution/proposals/` via the ledger.
- Never add `@skill` methods or modify system prompts automatically in V1.

**TDD Steps:**
1. Test repeated missing-argument/context failures produce no new skill proposal; they should recommend better tool use.
2. Test repeated "skill missing capability" feedback produces a proposal file.
3. Test proposal always sets `requires_human_review=True`.
4. Test proposal can be compared against current contracts to avoid duplicates.

**Redundancy Check:** This is a proposal generator, not a dynamic skill runtime. Existing skill code remains the only executable path.

---

## Completion Gates

Each task is complete only when:

- Focused pytest passes.
- Ruff check and format check pass for touched Python files.
- Relevant Layer 3/4/5 design docs are updated.
- `dimos/robot/unitree/go2/PROJECT_CHANGELOG.md` has a scoped entry.
- MCP-exposed tools have docstrings, primitive parameter types, and `SkillResult` returns.
- No new module imports heavy perception/model dependencies at package import time unless that module explicitly owns them.

## Defer Until Later

- Automatic code modification of skills.
- Automatic Git merge or push.
- Training/fine-tuning models.
- Replacing ChromaDB, SQLite, or existing memory modules.
- A robot-agnostic abstraction before Go2 contracts prove stable.
