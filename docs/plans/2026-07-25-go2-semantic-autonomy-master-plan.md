# Go2 Semantic Autonomy Master Implementation Plan

> **Superseded on 2026-07-25:** 后续产品任务请以
> `docs/plans/2026-07-25-go2-agent-fusion-master-plan.md` 为唯一权威总计划。
> 本文件仅保留为融合前的技术历史，不再更新任务状态。

> **For Codex:** REQUIRED SKILL: use `executing-plans` for implementation, and execute one numbered child task at a time.

**Goal:** Turn the current DimOS-based Go2 stack into a product system that can understand a household request, navigate to a semantic destination, inspect and report evidence, follow a person without bypassing obstacle-aware navigation, and later accept commands from a smart ring.

**Architecture:** Keep DimOS as the only robot runtime. Local Python modules own sensor synchronization, mapping, metric localization, navigation, obstacle handling, motion cancellation, and deterministic task execution. A replaceable visual-language provider is called only for task interpretation or ambiguous semantic evidence; it never produces velocity commands or declares geometric arrival. MCP is the high-level task and observability interface, while DimOS streams remain the sensor and control data plane.

**Tech Stack:** Python 3.12, DimOS `0.0.14b1`, Unitree Go2 WebRTC/DDS, voxel and cost maps, `ReplanningAStarPlanner`, `SpatialMemory`, Pydantic contracts, MCP, FastAPI Studio, native Rerun viewer, optional OpenAI-compatible vision provider, Pytest, Ruff.

---

## 0. Authority, status, and working rules

This is the canonical product and implementation roadmap for Go2 semantic
autonomy. Existing point plans remain useful technical references, but if they
conflict with this document, this document wins:

- `docs/plans/2026-07-24-go2-local-mission-control-design.md`
- `docs/plans/2026-07-24-go2-mission-control-implementation.md`
- `docs/plans/2026-07-25-go2-candidate-vision-verification-design.md`
- `docs/plans/2026-07-25-go2-visual-semantic-mission-mvp.md`
- `docs/plans/2026-07-25-go2-gpt-vision-lidar-closed-loop.md`

The roadmap has three product phases:

| Phase | Product outcome | Status |
|---|---|---|
| Phase 1 | Reliable semantic-place navigation Demo | `IN_PROGRESS` |
| Phase 2 | Event-driven visual inspection and open-world target search | `NOT_STARTED` |
| Phase 3 | Obstacle-aware person following and smart-ring integration | `NOT_STARTED` |

Status values are `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `DONE`. A phase may
be marked `DONE` only when its phase gate passes, not when its code merely runs.

### Mandatory workflow for every future Codex task

1. Select exactly one task ID from this master plan.
2. Read this file, `docs/PROJECT_CONTEXT.md`, the listed source files, and the
   task's dependencies.
3. Create a child plan at
   `docs/plans/YYYY-MM-DD-P<phase>-T<task>-<slug>.md` before editing source.
4. In the child plan, record current behavior, exact files, failing tests,
   minimal implementation, verification commands, rollback boundary, and the
   claimed acceptance boundary.
5. Implement with tests first. Do not combine unrelated task IDs.
6. Run focused tests, relevant regression tests, Ruff, and
   `git diff --check`.
7. Update this roadmap's status table and `docs/PROJECT_CONTEXT.md`.
8. Record what was proved. A tool call being accepted is not proof that a
   physical task completed.
9. Commit only when the user has explicitly authorized Git commits.

### Definition of Done for every child task

- The child plan exists and names the parent task ID.
- New behavior has a failing test before its implementation.
- Focused tests and affected regression tests pass.
- Errors, cancellation, timeouts, and stale inputs are tested.
- Runtime observability is sufficient to diagnose failure without guessing.
- No second robot runtime, second movement owner, or hidden background Agent is
  introduced.
- Documentation describes the actual validated boundary.
- Hardware claims include captured pose/state/evidence, not only logs or UI.

## 1. Product boundary and non-negotiable decisions

### In scope

- Natural-language commands such as:
  - `去门口`
  - `去厨房看看有没有水瓶`
  - `来到我身边`
  - `跟着我`
- Autonomous mapping and persistent semantic place names.
- Metric navigation, exploration, obstacle handling, recovery, cancellation,
  and arrival confirmation.
- Event-driven visual target verification and evidence-backed reporting.
- Map, pose, path, target, task state, and evidence visualization.
- Later smart-ring command ingress through the computer.

### Out of scope for these three phases

- Picking up food, opening doors, carrying objects, or other manipulation.
- End-to-end learned locomotion policies.
- A visual model directly emitting `Twist`, motor, or sport commands.
- Frame-by-frame cloud vision.
- Replacing the entire DimOS runtime with ROS 2, Isaac, or a new Agent runtime.
- Treating a CLIP camera viewpoint as a verified object pose.

### Architecture decisions

#### ADR-001: DimOS-first hybrid architecture

Use the current DimOS Go2 runtime and external Studio Blueprint. Add missing
task execution, recovery, semantic world-model, and evidence layers instead of
rewriting the robot stack.

#### ADR-002: Deterministic executor owns physical task progress

An LLM may compile language into a typed `TaskSpec`, but a Python state machine
or behavior tree owns execution, retries, cancellation, and completion.

#### ADR-003: Remain on 2D/2.5D navigation for the first product

The current target environment is flat indoor space without stairs. Do not
switch to the experimental MLS/Isaac 3D stack unless measured navigation
failures show that the present geometry is insufficient.

#### ADR-004: Visual-language models are event-driven semantic providers

Use local geometric perception continuously. Invoke a VLM only for a new
candidate, an ambiguous target, final semantic confirmation, or explicit user
inspection.

#### ADR-005: MCP is the task/control plane, not the real-time data plane

Expose high-level tools and inspectable resources over MCP. Keep images, point
clouds, odometry, cost maps, and velocity control on DimOS transports.

#### ADR-006: Person following must route through navigation

The current official `PersonFollowSkillContainer` directly performs visual
servoing and documents that it does not avoid obstacles. It must not be enabled
for the product. A tracker supplies a dynamic person pose; the planner supplies
collision-aware motion.

#### ADR-007: No custom model training before evidence proves it is needed

Use pretrained models and collect a replay/evidence corpus. Fine-tune only after
repeated failure clusters have names, counts, and reproducible examples.

## 2. Target runtime and data contracts

```text
Ring / Studio / external MCP client
                 |
                 v
        Natural-language compiler
                 |
                 v
          typed TaskSpec
                 |
                 v
     Deterministic MissionExecutor
       |          |            |
       v          v            v
 SemanticMap  RecoverySupervisor  EvidenceService
       |          |            |
       +----------+------------+
                  |
                  v
       DimOS NavigationInterface
                  |
                  v
               Go2

Camera + LiDAR + odometry + TF
                 |
                 v
  synchronized geometry / dynamic entities / semantic observations
                 |
        +--------+--------+
        v                 v
  Metric map        Semantic world model
```

### Required stable contracts

Create provider-neutral contracts rather than passing free-form dictionaries:

```python
class MissionKind(StrEnum):
    GO_TO_PLACE = "go_to_place"
    INSPECT_PLACE = "inspect_place"
    FIND_TARGET = "find_target"
    COME_TO_USER = "come_to_user"
    FOLLOW_PERSON = "follow_person"


class TaskSpec(BaseModel):
    task_id: str
    kind: MissionKind
    destination: str | None
    target_description: str | None
    question: str | None
    priority: Literal["normal", "urgent"]
    created_at: datetime


class TaskState(StrEnum):
    QUEUED = "queued"
    RESOLVING = "resolving"
    EXPLORING = "exploring"
    NAVIGATING = "navigating"
    RECOVERING = "recovering"
    VERIFYING = "verifying"
    FOLLOWING = "following"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class SemanticEntity(BaseModel):
    entity_id: str
    label: str
    map_pose: PoseEstimate | None
    confidence: float
    evidence_ids: list[str]
    last_seen_at: datetime
    persistence: Literal["static", "dynamic", "unknown"]


class ObservationBundle(BaseModel):
    frame_id: str
    captured_at: datetime
    camera_info_id: str
    robot_map_pose: PoseEstimate
    image_ref: str
    point_cloud_ref: str | None
    source: Literal["live", "replay", "simulation"]
```

The concrete implementation may refine field types, but it may not remove:
identity, timestamps, source, map pose, evidence linkage, task lifecycle, or
failure reason.

## 3. Master dependency graph

```text
P1-T1 contracts
  ├─> P1-T2 executor lifecycle
  ├─> P1-T3 recovery supervisor
  └─> P1-T4 semantic place store
          └─> P1-T5 map onboarding

P1-T2 + P1-T3 + P1-T4
  └─> P1-T6 MCP and UI integration
          └─> P1-T7 Phase 1 acceptance

P1 gate
  └─> P2-T1 synchronized observation bundle
        ├─> P2-T2 vision provider
        └─> P2-T3 2D-to-3D grounding
              └─> P2-T4 evidence fusion
                    ├─> P2-T5 inspection mission
                    └─> P2-T6 unknown-target search
                          └─> P2-T7 Phase 2 acceptance

P2 gate
  └─> P3-T1 dynamic person target
        └─> P3-T2 obstacle-aware follow coordinator
              └─> P3-T3 follow recovery
                    └─> P3-T4 ring command adapter
                          └─> P3-T5 product persistence/operations
                                └─> P3-T6 Phase 3 acceptance
```

# Phase 1 — Reliable semantic-place navigation Demo

## Phase 1 product outcome

The operator can run one DimOS instance, build or reload a map, confirm semantic
place names once, issue `去门口`, see the current task and path, have the robot
recover from a bounded obstruction, and receive a completion only after the
navigator is idle at the resolved destination.

## P1-T1 — Canonical mission contracts

**Status:** `DONE`

**Completed:** 2026-07-25. The strict contract is implemented in
`dimos_go2_studio.mission_contracts`; 43 focused/Studio tests, Ruff, and
`git diff --check` pass. No runtime or hardware behavior changed.

**Depends on:** none.

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_contracts.py`
- Create:
  `extensions/go2-studio-agent/tests/test_mission_contracts.py`
- Modify: `dimos/web/studio/models.py`

**Implementation plan:**

1. Write tests for mission kinds, state transitions, strict validation, task
   identity, cancellation, failure reasons, and evidence requirements.
2. Run the focused test and confirm it fails because the contracts do not exist.
3. Implement the minimal Pydantic models and enums.
4. Adapt Studio request/response models without duplicating the contracts.
5. Run focused and Studio model tests.

**Commands:**

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_mission_contracts.py \
  dimos/web/studio/test_mission.py -v
```

**Acceptance:**

- Invalid task combinations are rejected before robot I/O.
- Every task has a stable ID and terminal state.
- `COMPLETED` cannot be constructed without the required result/evidence.

## P1-T2 — Background MissionExecutor lifecycle

**Status:** `DONE`

**Completed:** 2026-07-25. One background `MissionExecutor` now exposes typed
start/pause/resume/cancel/status skills, holds `CAP_MOVEMENT` until terminal
state, and fails closed before `set_goal` while P1-T4's semantic resolver is
absent. A 57-test regression set, Ruff, and `git diff --check` pass. No live
runtime or hardware movement was executed.

**Depends on:** P1-T1.

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`
- Create:
  `extensions/go2-studio-agent/tests/test_mission_executor.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/skills.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`

**Implementation plan:**

1. Test queued, resolving, navigating, completed, failed, cancelled, and timeout
   transitions with fake navigation and clock dependencies.
2. Confirm a `set_goal()` acknowledgment does not mark the mission completed.
3. Implement one background executor with dependency-injected navigation,
   semantic resolver, clock, and event sink.
4. Hold `CAP_MOVEMENT` for the full physical task lifecycle.
5. Make cancellation idempotent and wait for navigation to become idle.
6. Expose `start_task`, `pause_task`, `resume_task`, `cancel_task`, and
   `get_task_status` as high-level skills.

**Acceptance:**

- Two movement tasks cannot run concurrently.
- Cancellation reaches an idle navigation state and is observable.
- A process restart leaves no task falsely marked `RUNNING`.
- No embedded conversational Agent is added to the Blueprint.

## P1-T3 — Navigation recovery supervisor

**Status:** `DONE`

**Completed:** 2026-07-25. A pure bounded `RecoverySupervisor` now classifies
stall, obstacle, path-deviation, and planner-error causes; alternates replanning
with rotate/rescan headings; exposes typed JSON recovery events; reports
`NavigationState.RECOVERY`; and discards stale A* results after cancellation or
goal replacement. Unsupported stale-layer clearing, reverse, and alternate
approach actions remain fail-closed behind explicit capability flags. An
86-test regression set, Ruff, and `git diff --check` pass. No runtime or
hardware movement was executed.

**Depends on:** P1-T1, P1-T2.

**Files:**

- Create:
  `dimos/navigation/replanning_a_star/recovery_supervisor.py`
- Create:
  `dimos/navigation/replanning_a_star/test_recovery_supervisor.py`
- Modify:
  `dimos/navigation/replanning_a_star/module.py`
- Modify:
  `dimos/navigation/replanning_a_star/global_planner.py`
- Modify:
  `dimos/navigation/replanning_a_star/controllers.py`

**Recovery sequence:**

```text
progress timeout
  -> request replan
  -> rotate and rescan
  -> clear only stale local obstacle observations
  -> bounded back-up when free space is confirmed
  -> generate an alternative approach pose
  -> fail with typed reason after bounded attempts
```

**Implementation plan:**

1. Freeze the existing 70-degree rotation, six-second stall, and eight-replan
   constants with regression tests.
2. Test each recovery transition with fake odometry, cost map, and time.
3. Implement a side-effect-free recovery decision state machine.
4. Connect one action at a time to the planner; do not combine clearing,
   backing, and rotating in one untestable callback.
5. Emit structured recovery events with attempt, cause, action, and outcome.
6. Preserve immediate stop/cancel preemption during every recovery state.

**Acceptance:**

- A 75-degree heading error rotates without forward motion.
- A temporary obstacle can trigger rescan/replan without mission failure.
- A bounded dead end ends in typed failure rather than infinite pushing.
- The recovery supervisor never clears the persistent global semantic map.

## P1-T4 — Persistent semantic place store

**Status:** `NOT_STARTED`

**Depends on:** P1-T1.

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py`
- Create:
  `extensions/go2-studio-agent/tests/test_semantic_world.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`
- Modify: `dimos/perception/spatial_memory_spec.py`

**Implementation plan:**

1. Test stable entity IDs, aliases, map poses, confidence, evidence links,
   timestamps, persistence class, and serialization.
2. Add `SpatialMemory` in persistent mode (`new_memory=False`) without adding a
   continuous VLM loop.
3. Separate exact confirmed places from CLIP observation candidates.
4. Implement resolver order:
   confirmed place -> alias -> semantic candidate -> unresolved.
5. Return a typed resolution result; never silently convert a low-confidence
   image match into a navigation target.

**Acceptance:**

- Restarting the runtime does not delete confirmed place names.
- `门口` and configured aliases resolve to the same stable entity.
- A CLIP result is labelled `candidate`, not `confirmed`.
- Every confirmed semantic place points to a map-frame pose and evidence.

## P1-T5 — One-time map onboarding

**Status:** `NOT_STARTED`

**Depends on:** P1-T4.

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/map_onboarding.py`
- Create:
  `extensions/go2-studio-agent/tests/test_map_onboarding.py`
- Modify: `dimos/web/studio/app.py`
- Modify: `dimos/web/studio/static/app.js`
- Modify: `dimos/web/studio/static/index.html`

**Implementation plan:**

1. Add onboarding states: mapping, proposing, awaiting confirmation, saved.
2. Use exploration and existing observation candidates to propose place names.
3. Let the operator confirm, rename, merge, or reject proposed places once.
4. Save confirmed entities with map ID/version.
5. Detect a mismatched or missing map instead of navigating with stale poses.

**Acceptance:**

- The operator is not required to manually click a coordinate for every task.
- An unconfirmed proposal cannot be used as a known destination.
- Reloading the same map restores its confirmed place names.
- Loading a different map invalidates incompatible poses visibly.

## P1-T6 — High-level MCP and unified task UI

**Status:** `NOT_STARTED`

**Depends on:** P1-T2, P1-T3, P1-T4, P1-T5.

**Files:**

- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/skills.py`
- Modify:
  `extensions/go2-studio-agent/tests/test_blueprint.py`
- Modify: `dimos/web/studio/app.py`
- Modify: `dimos/web/studio/service.py`
- Modify: `dimos/web/studio/static/app.js`
- Modify: `dimos/web/studio/static/index.html`
- Modify: `dimos/web/studio/test_studio.py`

**Implementation plan:**

1. Test the exact MCP surface; reject raw velocity and unsafe sport actions from
   the simplified product wrapper.
2. Return structured status plus text fallback from every high-level tool.
3. Add task progress events for resolve, navigate, recover, arrive, and fail.
4. Show map, robot pose, route, destination, mission state, recovery state, and
   stop/cancel in one view.
5. Do not open additional browser tabs or start a second Viewer/runtime.

**Acceptance:**

- The visible UI and MCP status show the same task ID and state.
- `cancel_task` works even if the visual model or UI is unavailable.
- Raw camera frames are not streamed through high-frequency MCP calls.
- Only one runtime owns Go2 movement.

## P1-T7 — Phase 1 replay, simulation, and hardware acceptance

**Status:** `NOT_STARTED`

**Depends on:** P1-T1 through P1-T6.

**Files:**

- Create: `tests/acceptance/go2_semantic_navigation_matrix.md`
- Create: `scripts/run_go2_phase1_acceptance.sh`
- Create: `artifacts/go2-phase1/.gitkeep` only if artifact retention is approved.

**Validation order:**

1. Unit tests.
2. Recorded replay.
3. Simulation.
4. Hardware read-only sensor validation.
5. One short supervised hardware route.
6. Ten known-place routes and ten bounded-obstruction trials.

**Phase 1 gate:**

- Known semantic-place arrival: at least 9/10.
- Bounded blocked-path recovery: at least 8/10 without manual driving.
- Final horizontal error: at most 0.6 m unless the destination defines a larger
  stand-off.
- Local cancel-to-zero-command p95: below 200 ms at the application boundary.
- Map and semantic names survive a restart and relocalize successfully.
- No duplicate runtime, indefinite recovery loop, or false completion.
- Map, task state, destination, path, and final pose are visible and retained.

Phase 2 may not start until this gate is recorded as passed or the user accepts
a written exception with the failed metric.

# Phase 2 — Event-driven vision and open-world inspection

## Phase 2 product outcome

The operator can say `去厨房看看有没有水瓶` or ask the robot to find an
unregistered target. The robot uses the semantic map and exploration to reach
useful viewpoints, invokes a visual provider only at meaningful events,
geometrically grounds a candidate with LiDAR/pose, confirms it from multiple
views, and returns evidence-backed found/not-found/uncertain results.

## P2-T1 — Synchronized ObservationBundle

**Status:** `NOT_STARTED`

**Depends on:** Phase 1 gate.

**Files:**

- Create: `dimos/perception/observation_bundle.py`
- Create: `dimos/perception/test_observation_bundle.py`
- Modify: `dimos/robot/unitree/go2/connection.py`

**Implementation plan:**

1. Test freshness, monotonic frame IDs, camera info identity, robot map pose,
   source, and optional point-cloud reference.
2. Reject stale images, missing pose, and excessive camera/LiDAR timestamp skew.
3. Provide a snapshot API; do not change continuous stream ownership.
4. Persist only selected evidence frames, not the full video by default.

**Acceptance:**

- Every image submitted for semantic inference has a timestamp and map pose.
- A stale or unsynchronized bundle produces `uncertain`, never a movement goal.
- Replay and live bundles share the same contract.

## P2-T2 — Provider-neutral vision gateway

**Status:** `NOT_STARTED`

**Depends on:** P2-T1.

**Files:**

- Modify: `dimos/perception/target_verification.py`
- Modify: `dimos/perception/test_target_verification.py`
- Create: `dimos/perception/vision_gateway.py`
- Create: `dimos/perception/test_vision_gateway.py`

**Implementation plan:**

1. Define structured outputs: labels, region/point, confidence, verdict, short
   evidence, model metadata, latency, and failure class.
2. Preserve a local/no-upload provider and an OpenAI-compatible provider.
3. Add timeout, cancellation, size limit, redaction hook, and no automatic
   retry for expired observations.
4. Trigger calls only for new candidates, final verification, or explicit
   inspection.
5. Benchmark providers on retained replay frames before selecting defaults.

**Acceptance:**

- Navigation, stopping, and obstacle handling work when the provider is offline.
- The gateway cannot output a velocity or direct navigation command.
- Model/provider selection is configuration, not mission code.
- Every response records the source frame IDs and latency.

## P2-T3 — Camera/LiDAR/pose target grounding

**Status:** `NOT_STARTED`

**Depends on:** P2-T1, P2-T2.

**Files:**

- Create: `dimos/perception/semantic_grounding.py`
- Create: `dimos/perception/test_semantic_grounding.py`
- Reuse without direct visual servo:
  `dimos/navigation/visual_servoing/detection_navigation.py`
- Reuse: `dimos/perception/detection/module3D.py`

**Implementation plan:**

1. Test camera projection and map-frame transformation with synthetic points.
2. Reuse the existing 2D-to-3D projection path, not its direct `Twist` control.
3. Estimate target pose and uncertainty from valid depth/LiDAR points.
4. Generate a collision-checked stand-off pose, not the object's centroid.
5. Reject targets with insufficient depth support or invalid TF.

**Acceptance:**

- A vision label alone cannot become a map goal.
- Synthetic projection tests meet numerical tolerances.
- On a measured replay set, median grounded position error is at most 0.5 m.
- Every approach goal records its target pose, stand-off, and uncertainty.

## P2-T4 — Multi-view semantic evidence fusion

**Status:** `NOT_STARTED`

**Depends on:** P2-T2, P2-T3.

**Files:**

- Create: `dimos/perception/semantic_evidence.py`
- Create: `dimos/perception/test_semantic_evidence.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py`

**Implementation plan:**

1. Test stable candidate IDs, duplicate-frame rejection, viewpoint diversity,
   contradictory evidence, confidence decay, and expiry.
2. Require independent observations before promoting ambiguous objects.
3. Fuse evidence into a semantic entity while retaining every source bundle.
4. Add dynamic/static classification and time-to-live behavior.

**Acceptance:**

- Repeating one frame cannot increase confidence.
- A candidate requires at least two fresh, sufficiently distinct viewpoints
  unless the task explicitly permits single-view inspection.
- Contradictory evidence yields `uncertain`, not forced success.

## P2-T5 — Inspection mission and evidence report

**Status:** `NOT_STARTED`

**Depends on:** P2-T1 through P2-T4.

**Files:**

- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`
- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/evidence_service.py`
- Create:
  `extensions/go2-studio-agent/tests/test_inspection_mission.py`
- Modify: `dimos/web/studio/static/app.js`

**Implementation plan:**

1. Test resolve -> navigate -> confirm idle -> settle -> inspect -> report.
2. Add found, not_found, uncertain, cancelled, and provider_unavailable results.
3. Require post-arrival frames for a destination inspection.
4. Show thumbnails, timestamps, poses, provider result, and final verdict.

**Acceptance:**

- Pre-arrival images cannot satisfy a post-arrival inspection.
- A report includes destination, arrival result, question, verdict, evidence
  IDs, and failure reason.
- The task does not claim object manipulation.

## P2-T6 — Unknown-target active search

**Status:** `NOT_STARTED`

**Depends on:** P2-T4, P2-T5.

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_search.py`
- Create:
  `extensions/go2-studio-agent/tests/test_semantic_search.py`
- Reuse:
  `dimos/navigation/frontier_exploration/wavefront_frontier_goal_selector.py`

**Implementation plan:**

1. Rank known semantic candidates before starting exploration.
2. Select informative frontier/viewpoint goals instead of walking randomly.
3. Stop movement before final candidate verification.
4. Continue with a new viewpoint after rejection.
5. Bound search by time, distance, explored coverage, and candidate count.

**Acceptance:**

- Search choices record why each viewpoint was selected.
- A rejected candidate is not immediately revisited without new evidence.
- Timeout produces `not_found` or `uncertain`, not infinite exploration.
- The robot stops before an ambiguous final semantic decision.

## P2-T7 — Phase 2 dataset and acceptance

**Status:** `NOT_STARTED`

**Depends on:** P2-T1 through P2-T6.

**Files:**

- Create: `tests/acceptance/go2_semantic_vision_matrix.md`
- Create: `scripts/run_go2_phase2_acceptance.sh`
- Create: `docs/evaluations/go2-vision-provider-benchmark.md`

**Phase 2 gate:**

- Evaluation set contains at least 50 retained target/non-target situations
  across multiple viewpoints and lighting conditions.
- Candidate recall is at least 90% on the declared test set.
- False verified-target rate is below 5%.
- At least 95% of evidence records have fresh image, pose, source, and provider
  metadata.
- Grounded target median position error is at most 0.5 m.
- Event-driven semantic verification p95 is below five seconds on the chosen
  provider, or the UI visibly reports a slower non-real-time mode.
- Provider loss never disables local stop, obstacle handling, or navigation.
- Ten end-to-end inspection missions produce correct arrival and evidence
  lifecycle in at least 9/10 trials.

Phase 3 may not start until this gate is recorded as passed or the user accepts
a written exception.

# Phase 3 — Obstacle-aware following and smart-ring commands

## Phase 3 product outcome

The robot can follow a selected person through a mapped indoor environment at a
configured stand-off while the planner avoids obstacles, recover from temporary
target loss, and accept deduplicated high-level commands from a smart ring via
the computer. The map and task UI show both robot and target state.

## P3-T1 — Dynamic person target stream

**Status:** `NOT_STARTED`

**Depends on:** Phase 2 gate.

**Files:**

- Create: `dimos/perception/dynamic_target.py`
- Create: `dimos/perception/test_dynamic_target.py`
- Modify: `dimos/agents/skills/person_follow.py` only to expose reusable
  detection/tracking pieces; do not enable direct servoing.

**Implementation plan:**

1. Test identity, map pose, velocity estimate, timestamp, confidence, and lost
   state.
2. Use the detector/tracker to create a map-frame person pose.
3. Reject stale or geometrically unsupported person observations.
4. Publish target updates separately from robot velocity commands.

**Acceptance:**

- The person tracker emits no `Twist`.
- Target identity does not switch silently between nearby people.
- Stale target data causes hold/search behavior, not blind pursuit.

## P3-T2 — Obstacle-aware follow coordinator

**Status:** `NOT_STARTED`

**Depends on:** P3-T1.

**Files:**

- Create: `dimos/navigation/dynamic_follow.py`
- Create: `dimos/navigation/test_dynamic_follow.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`

**Implementation plan:**

1. Test stand-off target generation, update rate limiting, dead band,
   obstacle-induced detours, cancellation, and target loss.
2. Convert the dynamic person pose to a navigation goal behind the desired
   stand-off.
3. Route all motion through `NavigationInterfaceSpec`.
4. Avoid constant full replans for target jitter.
5. Expose follow state and target distance to MCP/UI.

**Acceptance:**

- Follow motion is generated by obstacle-aware navigation, not visual servoing.
- Desired stand-off defaults to 1.0–1.5 m and is configurable.
- Small target jitter does not cause oscillation.
- Cancel immediately preempts follow and leaves navigation idle.

## P3-T3 — Lost-target and blocked-follow recovery

**Status:** `NOT_STARTED`

**Depends on:** P3-T2.

**Files:**

- Modify: `dimos/navigation/dynamic_follow.py`
- Modify: `dimos/navigation/test_dynamic_follow.py`
- Modify:
  `dimos/navigation/replanning_a_star/recovery_supervisor.py`

**Implementation plan:**

1. Test short occlusion, long loss, blocked corridor, person stopped, and
   person reappearing with the same/different identity.
2. Hold briefly for short loss.
3. Rotate/search from the last valid pose for bounded time.
4. Fail or request operator selection after identity ambiguity.
5. Never back up or turn blindly without local obstacle clearance.

**Acceptance:**

- A brief occlusion does not terminate follow.
- Long loss produces a bounded search and visible terminal state.
- The robot does not select an arbitrary new person.

## P3-T4 — Smart-ring command adapter

**Status:** `NOT_STARTED`

**Depends on:** P1-T1, P3-T2.

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/ring_adapter.py`
- Create:
  `extensions/go2-studio-agent/tests/test_ring_adapter.py`
- Modify: `dimos/web/studio/app.py`

**Implementation plan:**

1. Define a transport-neutral `RingCommandEnvelope` with command ID, timestamp,
   source device, payload, and optional user location.
2. Test authentication boundary, deduplication, ordering, retry, expiry, and
   explicit cancel.
3. Translate only into the same `TaskSpec` used by Studio/MCP.
4. Keep BLE/device-specific code outside the mission executor.
5. Return acknowledgment and task ID to the ring bridge.

**Acceptance:**

- Retried ring messages create one task, not duplicate tasks.
- Expired or out-of-order commands do not move the robot.
- Studio, MCP, and ring use the same task contract.
- Ring disconnection does not disable local task cancellation.

## P3-T5 — Persistence, relocalization, telemetry, and operations

**Status:** `NOT_STARTED`

**Depends on:** P3-T2, P3-T4.

**Files:**

- Create:
  `extensions/go2-studio-agent/src/dimos_go2_studio/task_store.py`
- Create:
  `extensions/go2-studio-agent/tests/test_task_store.py`
- Modify:
  `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py`
- Modify: `dimos/web/studio/service.py`
- Modify: `dimos/web/studio/static/app.js`

**Implementation plan:**

1. Persist task events, map version, semantic entities, evidence references, and
   final outcomes.
2. Test crash/restart reconciliation and map mismatch.
3. Add health metrics: sensor age, target age, nav state, recovery count,
   provider latency, command latency, and runtime owner.
4. Add exportable mission replay metadata without storing secrets.

**Acceptance:**

- Restart reconciles an interrupted task as interrupted/failed, never still
  running.
- Relocalization is verified before using old semantic poses.
- One exported task record can explain what the robot saw, decided, attempted,
  and why it ended.

## P3-T6 — Phase 3 end-to-end product acceptance

**Status:** `NOT_STARTED`

**Depends on:** P3-T1 through P3-T5.

**Files:**

- Create: `tests/acceptance/go2_follow_ring_matrix.md`
- Create: `scripts/run_go2_phase3_acceptance.sh`
- Create: `docs/evaluations/go2-product-readiness.md`

**Phase 3 gate:**

- Follow distance remains within 1.0–1.5 m for at least 90% of clear-path
  samples.
- Twenty controlled follow trials complete without direct visual-servo
  collision behavior.
- Temporary person occlusion recovers in at least 8/10 trials.
- Ambiguous identity produces hold/operator-selection rather than target switch.
- Ring command ingress-to-task-ack p95 is below one second on the local
  computer path.
- Duplicate ring commands produce exactly one task.
- Stop/cancel remains local and independent of ring/cloud availability.
- Restart, map reload, and relocalization pass three consecutive trials.
- One unified UI shows robot trajectory, target/person position, semantic
  places, task status, recovery, and evidence.

# 4. Cross-phase failure matrix

| Failure | Required response | Forbidden response |
|---|---|---|
| Duplicate DimOS runtime | Reject second owner and report process IDs | Allow both to publish movement |
| Stale camera frame | Mark observation uncertain | Send stale frame as current evidence |
| Missing odometry/TF | Refuse geometric target | Guess distance from RGB |
| Cloud/VLM timeout | Continue local nav/stop; report provider unavailable | Freeze cancellation or obstacle handling |
| Low-confidence semantic match | Gather another viewpoint | Navigate as if confirmed |
| Planner makes no progress | Run bounded recovery sequence | Push forever or only increase retry count |
| Map version mismatch | Relocalize or require onboarding | Use old semantic coordinates silently |
| Person identity ambiguity | Hold and request selection | Follow the nearest arbitrary person |
| Ring duplicate/retry | Deduplicate by command ID | Start multiple physical tasks |
| Process restart mid-task | Reconcile as interrupted | Display stale `RUNNING` state |

# 5. Upgrade decision gates

Do not add ROS 2/Nav2, Jetson, Isaac ROS, or model training because they sound
more advanced. Add them only against measured failures.

## ROS 2/Nav2 backend gate

Evaluate a navigation backend adapter only if, after P1-T3:

- known-place success remains below 9/10;
- recovery remains below 8/10;
- dynamic obstacle behavior cannot be corrected without invasive planner
  changes; or
- current planner lacks required progress/collision interfaces.

The adapter must preserve `NavigationInterfaceSpec` so MissionExecutor and MCP
do not change.

## Jetson/Isaac ROS gate

Evaluate Linux/Jetson edge hardware only if:

- event-driven vision cannot meet the Phase 2 latency/accuracy gate on the
  current Mac/provider mix;
- continuous local open-vocabulary perception becomes a real requirement; or
- depth/3D reconstruction throughput is the measured bottleneck.

## Model training gate

Consider fine-tuning only when the retained evaluation corpus contains at least
100 labelled failures in a repeated category and a pretrained/provider change
cannot reach the acceptance target.

# 6. Codex child-plan prompt

Use this prompt to start every implementation task:

```text
请执行总计划中的 [TASK_ID]。

先阅读：
1. docs/plans/2026-07-25-go2-semantic-autonomy-master-plan.md
2. docs/PROJECT_CONTEXT.md
3. [TASK_ID] 列出的源码和测试

然后：
- 创建 docs/plans/YYYY-MM-DD-[TASK_ID]-<slug>.md 小计划；
- 只实现这一个任务，不顺手实现后续阶段；
- 先写失败测试，再做最小实现；
- 保持一个 DimOS 运行时和一个运动控制 owner；
- 运行小计划列出的 focused tests、相关回归、Ruff、git diff --check；
- 不把 MCP 调用成功当成真实任务完成；
- 更新总计划状态和 docs/PROJECT_CONTEXT.md；
- 汇报实际验证边界、未解决问题和下一个允许执行的 TASK_ID。

若涉及真实机器运动，先说明本次要验证的精确动作、范围、停止方式和
需要记录的证据；没有明确授权时只做 replay/simulation/read-only 验证。
不要自行提交 Git，除非我明确要求。
```

# 7. Current next action

The next allowed implementation task is **P1-T4 — Persistent semantic place
store**. P1-T5 and later tasks must not start until P1-T4's child plan, tests,
implementation, and documentation are complete.
