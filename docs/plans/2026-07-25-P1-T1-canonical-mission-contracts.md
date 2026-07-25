# P1-T1 Canonical Mission Contracts Implementation Plan

> **For Codex:** REQUIRED SKILL: use `executing-plans` and complete only master task `P1-T1`.

**Goal:** Add one strict, provider-neutral contract for Go2 mission intent,
lifecycle, terminal result, evidence linkage, and valid state transitions
before any robot I/O.

**Architecture:** The domain contract lives in the user-editable Go2 Studio
extension because the future deterministic executor will also live there.
DimOS Studio keeps small HTTP ingress DTOs; it does not import the optional
extension package or duplicate the domain contract. The executor in P1-T2 will
translate the ingress payload or Agent output into `TaskSpec`.

**Tech Stack:** Python 3.12, Pydantic v2, `StrEnum`, Pytest, Ruff.

---

## Parent and scope

- Parent:
  `docs/plans/2026-07-25-go2-semantic-autonomy-master-plan.md`
- Task: `P1-T1 — Canonical mission contracts`
- In scope:
  - strict `TaskSpec`;
  - mission/state enums;
  - task lifecycle transition validation;
  - terminal result and evidence requirements;
  - strict Studio mission request DTOs.
- Out of scope:
  - language-to-task compilation;
  - mission execution;
  - robot I/O;
  - navigation;
  - MCP tools;
  - visual inference.

## Design decisions

1. Add `PAUSED` to the master lifecycle because P1-T2 explicitly requires
   pause/resume. A paused snapshot carries `resume_state`; this avoids resuming
   into an unrelated phase.
2. Every contract model uses `extra="forbid"` and immutable instances.
3. `TaskSpec` normalizes strings and UTC timestamps, while rejecting naive
   datetimes and invalid kind/field combinations.
4. `urgent` is initially accepted only for `come_to_user`.
5. A completed task requires a non-empty result and at least one evidence ID.
   For navigation-only tasks, the future executor can use an arrival/pose
   evidence record.
6. Failed and cancelled tasks require a terminal reason. Non-terminal states
   cannot contain terminal results or reasons.
7. Studio HTTP models remain transport DTOs. They become strict but do not
   depend on `dimos_go2_studio`, keeping the core DimOS package importable
   without the external extension.

## Task 1 — Write failing domain contract tests

**Files:**

- Create:
  `extensions/go2-studio-agent/tests/test_mission_contracts.py`

**Tests:**

- all five mission kinds and lifecycle states are stable string enums;
- IDs and whitespace are normalized and unknown fields are forbidden;
- naive timestamps are rejected and aware timestamps normalize to UTC;
- each mission kind enforces its required and forbidden fields;
- `come_to_user` normalizes its destination to `用户身边`;
- only `come_to_user` accepts urgent priority;
- valid lifecycle transitions pass and invalid/terminal transitions fail;
- paused state requires a resumable previous state;
- completed state requires a result and evidence;
- failed/cancelled states require a reason;
- non-terminal states reject terminal payloads.

**Run:**

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_mission_contracts.py -v
```

**Expected before implementation:** collection fails because
`dimos_go2_studio.mission_contracts` does not exist.

## Task 2 — Write failing Studio DTO tests

**Files:**

- Modify: `dimos/web/studio/test_mission.py`

**Tests:**

- mission objectives are trimmed;
- blank objectives are rejected;
- unknown mission request fields are rejected;
- action reasons are trimmed.

**Run:**

```bash
cd /Users/johnsonmac/ai_completion/dimos
.venv/bin/python -m pytest dimos/web/studio/test_mission.py -v
```

**Expected before implementation:** whitespace/extra-field assertions fail.

## Task 3 — Implement the minimal domain contract

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_contracts.py`

**Implementation:**

- `MissionKind`
- `TaskPriority`
- `TaskState`
- `TaskSpec`
- `TaskResult`
- `TaskSnapshot`
- `ALLOWED_TASK_TRANSITIONS`
- `validate_task_transition`

Keep this file pure: no DimOS modules, model imports, network calls, storage, or
robot I/O.

## Task 4 — Make Studio mission DTOs strict

**Files:**

- Modify: `dimos/web/studio/models.py`

Add a private mission request base with:

```python
ConfigDict(extra="forbid", str_strip_whitespace=True)
```

Use it for `MissionCreateRequest`, `MissionStartRequest`, and
`MissionActionRequest`. Do not import the extension package.

## Task 5 — Verify and document

**Commands:**

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_mission_contracts.py \
  dimos/web/studio/test_mission.py \
  dimos/web/studio/test_studio.py -v

.venv/bin/ruff check \
  extensions/go2-studio-agent/src/dimos_go2_studio/mission_contracts.py \
  extensions/go2-studio-agent/tests/test_mission_contracts.py \
  dimos/web/studio/models.py \
  dimos/web/studio/test_mission.py

git diff --check
```

**Acceptance boundary:**

- Invalid task combinations fail before robot I/O.
- Every task has a stable ID and validated lifecycle state.
- Completion cannot be represented without result/evidence.
- Cancellation/failure cannot be represented without a reason.
- No runtime, Agent, MCP, navigation, or hardware behavior changes.

## Rollback boundary

The task is isolated to the two new contract/plan files and the strict Studio
mission DTO/test changes. Reverting these files does not affect the running Go2
Blueprint or robot control.

## Execution result

**Status:** `DONE` on 2026-07-25.

**Implemented:**

- strict `MissionKind`, `TaskPriority`, and `TaskState` enums;
- `TaskSpec` validation for five supported mission kinds;
- UTC-aware timestamps, stable task IDs, immutable/extra-forbidden models;
- bounded lifecycle transitions including pause/resume;
- evidence-backed `TaskResult`;
- internally consistent `TaskSnapshot`;
- strict, whitespace-normalizing Studio mission ingress DTOs.

**Verification:**

```text
43 passed in 0.28s
Ruff: All checks passed
git diff --check: passed
```

**Boundary:** no Agent, MCP, runtime, navigation, vision, or hardware behavior
was added or changed. The next permitted master task is `P1-T2`.
