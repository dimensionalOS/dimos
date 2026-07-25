# P1-T3 Navigation Recovery Supervisor Implementation Plan

> **For Codex:** REQUIRED SKILL: use `executing-plans` and complete only master task `P1-T3`.

**Goal:** Replace repeated blind replanning with a bounded, observable recovery
sequence that can replan, rotate in place to refresh perception, skip unsafe
actions whose prerequisites are absent, and fail with a typed reason.

**Architecture:** Add a side-effect-free `RecoverySupervisor` that chooses one
action from the current attempt, failure cause, and explicit capability flags.
`GlobalPlanner` remains the only global navigation owner and `LocalPlanner`
remains the only `Twist` publisher. A rotate/rescan action is implemented by
overriding only the first orientation of a freshly planned path, causing the
existing local planner to rotate before translating. Recovery events are
published as structured JSON through the navigation module.

**Tech Stack:** Python 3.12, dataclasses, `StrEnum`, DimOS reactive streams,
`ReplanningAStarPlanner`, Pytest, Ruff.

---

## Parent and scope

- Parent:
  `docs/plans/2026-07-25-go2-semantic-autonomy-master-plan.md`
- Depends on: completed `P1-T1`, completed `P1-T2`
- Task: `P1-T3 — Navigation recovery supervisor`
- In scope:
  - preserve the existing 70-degree rotate-before-drive threshold;
  - preserve the six-second progress timeout;
  - preserve the eight-attempt same-area bound;
  - classify recovery causes;
  - bounded replan and alternating rotate/rescan decisions;
  - explicit capability gates for stale-local clearing, reverse motion, and
    alternate approach poses;
  - typed recovery events and failure reasons;
  - `NavigationState.RECOVERY` while a dispatched recovery is active;
  - immediate cancellation through the existing local-planner stop path.
- Out of scope:
  - deleting or clearing persistent map data;
  - blind reverse motion;
  - pretending the current occupancy grid has obstacle age/provenance;
  - semantic destination resolution (`P1-T4`);
  - vision or VLM calls;
  - real robot movement.

## Current behavior and constraints

1. A stall, path deviation, local obstacle, or local-planner error all call the
   same `_replan_path()` loop.
2. `ReplanLimiter` already bounds same-area retries to eight and resets after
   two metres.
3. `LocalPlanner` already rotates in place when the first path orientation is
   outside tolerance and remains the sole velocity-command owner.
4. `NavigationMap` stores only the latest occupancy grid. It has no separately
   addressable stale local layer and no observation timestamp/provenance.
5. There is no rear-clearance contract that can authorize reverse motion.
6. The planner has no alternate-approach-pose provider.

The last three facts require fail-closed capability gates. This task must not
weaken obstacle handling to claim a broader recovery feature.

## Task 1 — Write failing recovery tests

**Files:**

- Create:
  `dimos/navigation/replanning_a_star/test_recovery_supervisor.py`
- Preserve:
  `dimos/navigation/replanning_a_star/test_recovery_tuning.py`

**Required tests:**

- first failure requests a normal replan;
- repeated failure requests alternating rotate/rescan actions;
- rotate/rescan never requests forward or reverse motion directly;
- stale-local clearing is selected only when that exact capability exists;
- reverse is selected only when rear clearance is explicitly verified;
- alternate approach is selected only when a provider is available;
- missing capabilities are recorded as skipped, not silently attempted;
- attempt eight returns typed `attempts_exhausted` failure;
- recovery events serialize cause, action, attempt, outcome, and reason;
- cancel clears recovery state and commands the existing local planner to stop;
- a rotate/rescan path changes only the first heading and retains the final
  goal pose.

**Expected before implementation:**

```text
ModuleNotFoundError:
  dimos.navigation.replanning_a_star.recovery_supervisor
```

## Task 2 — Implement the pure recovery decision layer

**Files:**

- Create:
  `dimos/navigation/replanning_a_star/recovery_supervisor.py`

**Components:**

- `RecoveryCause`;
- `RecoveryAction`;
- `RecoveryOutcome`;
- `RecoveryCapabilities`;
- `RecoveryDecision`;
- `RecoveryEvent`;
- stateless `RecoverySupervisor.decide(...)`.

**Sequence:**

```text
attempt 0: replan
attempt 1: rotate/rescan left
attempt 2: replan
attempt 3: rotate/rescan right
attempt 4: clear stale local observations if supported, otherwise replan
attempt 5: bounded back-up only if rear clearance is verified,
           otherwise rotate/rescan
attempt 6: alternate approach if supported, otherwise replan
attempt 7: final rotate/rescan
attempt 8: typed failure
```

No action in this layer publishes velocity or mutates a map.

## Task 3 — Integrate only supported production actions

**Files:**

- Modify:
  `dimos/navigation/replanning_a_star/global_planner.py`
- Modify:
  `dimos/navigation/replanning_a_star/module.py`
- Modify only if a regression test requires clarification:
  `dimos/navigation/replanning_a_star/controllers.py`

**Implementation:**

1. Route stall, obstacle, deviation, and planner-error callbacks through the
   supervisor with distinct causes.
2. Execute `REPLAN` through the existing planner.
3. Execute `ROTATE_RESCAN` by changing the first pose heading of the new path;
   do not publish `Twist` from `GlobalPlanner`.
4. Keep unsupported capability flags false in production.
5. Publish structured recovery events as JSON.
6. Report `NavigationState.RECOVERY` from dispatch until the local planner
   resumes a navigation state or the goal is cancelled.
7. On exhaustion, emit a typed failure before cancelling.

## Task 4 — Verify

**Commands:**

```bash
cd /Users/johnsonmac/ai_completion/dimos

.venv/bin/python -m pytest \
  dimos/navigation/replanning_a_star/test_recovery_supervisor.py \
  dimos/navigation/replanning_a_star/test_recovery_tuning.py \
  dimos/navigation/replanning_a_star/test_global_planner.py -v

PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_mission_executor.py \
  extensions/go2-studio-agent/tests/test_blueprint.py \
  dimos/navigation/replanning_a_star/test_recovery_supervisor.py \
  dimos/navigation/replanning_a_star/test_recovery_tuning.py \
  dimos/navigation/replanning_a_star/test_global_planner.py -v

.venv/bin/ruff check \
  dimos/navigation/replanning_a_star/recovery_supervisor.py \
  dimos/navigation/replanning_a_star/global_planner.py \
  dimos/navigation/replanning_a_star/module.py \
  dimos/navigation/replanning_a_star/controllers.py \
  dimos/navigation/replanning_a_star/test_recovery_supervisor.py \
  dimos/navigation/replanning_a_star/test_recovery_tuning.py

git diff --check
```

## Acceptance boundary

- A temporary obstruction can progress from replan to rotate/rescan and another
  replan without immediately failing the mission.
- A 75-degree heading error still produces zero linear velocity.
- Same-area recovery terminates after eight retries with a typed reason.
- Cancellation preempts recovery through the existing zero-velocity stop path.
- No persistent map is cleared.
- No reverse action is executed without an explicit rear-clearance contract.
- Tests prove decisions and planner dispatch only; they do not prove live Go2
  obstacle recovery.

## Rollback boundary

Removing the new supervisor/test files and reverting the small
`GlobalPlanner`/module integration restores the current bounded-replan behavior.
The existing controller thresholds, stall timeout, limiter, Studio, Agent, MCP,
and hardware drivers remain independent.

## Execution result

**Status:** `DONE` on 2026-07-25.

**Implemented:**

- a stateless, bounded recovery policy with typed causes, actions, outcomes,
  decisions, capability gates, and JSON events;
- normal replan attempts alternating with left/right rotate-rescan headings;
- production execution of only `REPLAN` and `ROTATE_RESCAN`;
- fail-closed skips for stale-local clearing, reverse, and alternate approach
  when their evidence/interfaces are absent;
- `NavigationState.RECOVERY` while an action is active;
- event forwarding through `ReplanningAStarPlanner.recovery_event`;
- distinct causes for progress timeout, obstacle, path deviation, and planner
  error;
- typed `attempts_exhausted` failure after eight same-area retries;
- goal revisions that discard an A* result computed across cancellation or a
  newer goal;
- cancellation through the existing local-planner zero-velocity stop path.

**Verification:**

```text
Initial red test:
  ModuleNotFoundError: recovery_supervisor

Focused navigation:
  21 passed

Affected regression:
  86 passed, 4 existing archive-extraction deprecation warnings

Ruff:
  All checks passed

git diff --check:
  passed
```

**Boundary:** tests prove policy decisions, path-heading dispatch, typed
events, cancellation preemption, and stale-path rejection. They do not prove
that a live Go2 can escape a physical obstruction. No DimOS runtime, MCP
network call, simulation, replay, or robot movement was run. The current
occupancy grid still lacks stale-observation provenance, rear-clearance
verification, and an alternate-approach provider, so those actions remain
disabled instead of being guessed.

**Next permitted task:** `P1-T4 — Persistent semantic place store`.
