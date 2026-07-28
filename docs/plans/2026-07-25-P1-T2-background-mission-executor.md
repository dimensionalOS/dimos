# P1-T2 Background MissionExecutor Implementation Plan

> **For Codex:** REQUIRED SKILL: use `executing-plans` and complete only master task `P1-T2`.

**Goal:** Add one background, cancellable mission executor that owns a task from
validated `TaskSpec` through resolution, navigation, pause/resume, timeout, and
an evidence-backed terminal state.

**Architecture:** `MissionExecutor` is a DimOS `Module` and the only owner of
high-level mission lifecycle. `start_task` is a background skill holding
`CAP_MOVEMENT` through a `ToolStream` until the worker reaches a terminal
state. Navigation and destination resolution are dependency-injected so tests
are deterministic. Until P1-T4 supplies the semantic resolver, the production
Blueprint uses an explicit unavailable resolver and fails before `set_goal`
rather than inventing a pose.

**Tech Stack:** Python 3.12, DimOS Module/Skill/MCP capability lifecycle,
Pydantic mission contracts, `NavigationInterfaceSpec`, threads/events, Pytest,
Ruff.

---

## Parent and scope

- Parent:
  `docs/plans/2026-07-25-go2-semantic-autonomy-master-plan.md`
- Depends on: completed `P1-T1`
- Task: `P1-T2 — Background MissionExecutor lifecycle`
- In scope:
  - single-task mutual exclusion;
  - queued/resolving/navigating/recovering/completed/failed/cancelled/paused
    lifecycle;
  - injected destination resolver, navigator, clock, wait function, and event
    sink;
  - mission timeout;
  - idempotent cancellation that waits for navigation idle;
  - pause/resume with goal reissue;
  - background capability ownership;
  - high-level task status skills;
  - one executor in the existing Blueprint.
- Out of scope:
  - semantic place storage/resolution implementation (`P1-T4`);
  - navigation recovery actions (`P1-T3`);
  - visual evidence (`Phase 2`);
  - persistence across process restarts (`P3-T5`);
  - changing current navigation/controller weights;
  - real hardware movement.

## Design decisions

1. `start_task(task_json: str)` is
   `@skill(uses=[CAP_MOVEMENT], lifecycle="background")`.
2. It calls `start_tool("start_task")` before every early return. Invalid input
   closes the stream; same-tool reinvocation keeps the live stream so the
   capability token is not leaked.
3. The worker is a single daemon thread protected by a re-entrant lock and
   cancellation/wake events. A second active task is rejected with the active
   task ID.
4. `set_goal()` acknowledgment advances only to `NAVIGATING`. Completion
   requires both `is_goal_reached()` and `NavigationState.IDLE`.
5. `pause_task` changes the snapshot first, cancels the current goal, and waits
   for idle. `resume_task` restores the recorded state and lets the worker
   reissue the retained pose.
6. Time spent paused does not consume the mission timeout.
7. Every state change publishes an immutable `TaskSnapshot` to an injected
   event sink and emits a tool progress message when available.
8. Completion uses an arrival evidence ID produced only after goal-reached plus
   idle. It is contract evidence, not yet a persisted camera artifact.
9. A fresh executor starts with no active task. P3-T5 will later reconcile
   persisted interrupted tasks.
10. No embedded `McpClient` or conversational Agent is added.

## Task 1 — Write failing executor tests

**Files:**

- Create:
  `extensions/go2-studio-agent/tests/test_mission_executor.py`
- Modify:
  `extensions/go2-studio-agent/tests/test_blueprint.py`

**Test fixtures:**

- fake navigation with explicit accepted/reached/state controls;
- fake resolver returning a known `PoseStamped`, `None`, or an exception;
- manual clock and advancing waiter for deterministic timeout;
- test executor overriding tool-stream methods to avoid transport side effects;
- polling helper with a short real test deadline.

**Required tests:**

- `start_task` metadata is background and holds `CAP_MOVEMENT`;
- fresh status reports no active task;
- valid task transitions through resolving and navigating;
- `set_goal=True` alone does not complete;
- reached plus idle completes with arrival evidence;
- a second concurrent task is rejected;
- resolver failure and goal rejection become typed failures;
- timeout cancels navigation and becomes failed;
- pause cancels and reaches idle, resume reissues the retained goal;
- cancellation is idempotent and finishes idle;
- a fresh executor after simulated restart is not running;
- Blueprint includes exactly one `MissionExecutor`;
- Blueprint still excludes `McpClient`, `SpatialMemory`, and continuous VLM.

**Expected before implementation:**

```text
ModuleNotFoundError: dimos_go2_studio.mission_executor
```

## Task 2 — Implement minimal executor

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`

**Components:**

- `DestinationResolver` protocol;
- `UnavailableDestinationResolver`;
- `MissionExecutor`;
- pure status JSON serialization helpers;
- worker lifecycle and state transition helpers.

**Constraints:**

- do not publish `Twist`;
- do not accept raw coordinates through the Agent-facing skill;
- do not call a vision provider;
- do not treat resolver or `set_goal` acknowledgment as completion;
- always close the background tool stream on terminal state and module stop.

## Task 3 — Add executor to the product Blueprint

**Files:**

- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/skills.py`

Add exactly one `MissionExecutor.blueprint()`. Document in `skills.py` that
mission lifecycle methods belong to the executor so custom policy skills do not
become a second movement owner.

## Task 4 — Verify

**Commands:**

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_mission_contracts.py \
  extensions/go2-studio-agent/tests/test_mission_executor.py \
  extensions/go2-studio-agent/tests/test_blueprint.py \
  dimos/web/studio/test_mission.py \
  dimos/web/studio/test_studio.py -v

.venv/bin/ruff check \
  extensions/go2-studio-agent/src/dimos_go2_studio/mission_contracts.py \
  extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py \
  extensions/go2-studio-agent/src/dimos_go2_studio/skills.py \
  extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py \
  extensions/go2-studio-agent/tests/test_mission_contracts.py \
  extensions/go2-studio-agent/tests/test_mission_executor.py \
  extensions/go2-studio-agent/tests/test_blueprint.py

git diff --check
```

## Acceptance boundary

- Two movement missions cannot run concurrently.
- `set_goal()` acknowledgment is visibly different from arrival.
- Cancellation is idempotent and navigation is idle before it returns success.
- Timeout cancels navigation and produces a reasoned terminal failure.
- Pause/resume preserves the previous active phase and reissues the retained
  goal.
- Background `CAP_MOVEMENT` is released only when the task stream closes.
- Fresh process state is inactive rather than falsely running.
- The production resolver intentionally fails closed until `P1-T4`.
- No robot movement or live runtime test is claimed.

## Rollback boundary

The executor is isolated to the new module/tests and one Blueprint atom.
Removing that atom and the new files restores the P1-T1 runtime without changing
the existing Go2 planner, controller, MCP server, Studio, or native app.

## Execution result

**Status:** `DONE` on 2026-07-25.

**Implemented:**

- one background `MissionExecutor` connected to `NavigationInterfaceSpec`;
- `start_task`, `pause_task`, `resume_task`, `cancel_task`, and
  `get_task_status` skills;
- full-task `CAP_MOVEMENT` ownership through the DimOS ToolStream lifecycle;
- injected resolver, navigator, clock, waiter, and immutable event sink;
- single-task mutual exclusion;
- goal-acknowledged versus physically-arrived distinction;
- reached-plus-idle completion with arrival evidence;
- fail-closed resolver and goal rejection;
- bounded timeout and navigation cancellation;
- pause-to-idle and retained-goal resume;
- idempotent cancellation and inactive fresh-process state;
- exactly one executor in the lightweight product Blueprint.

**Verification:**

```text
57 passed in 3.93s
Ruff: All checks passed
git diff --check: passed
```

**Boundary:** the production semantic resolver remains deliberately unavailable
until `P1-T4`, so the new skill fails before publishing a goal. No replay,
simulation, live runtime, MCP network call, or real-Go2 movement was performed.
The next permitted master task is `P1-T3`.
