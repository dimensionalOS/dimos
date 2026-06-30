# Draft Plan: Agentic Skill Benchmark Harness

Status: exploratory draft, not an active OpenSpec change
Date: 2026-06-24

Related concrete change:

- `openspec/changes/framework-robosuite-integration/` defines the first implementation slice for the benchmark runtime framework plus Robosuite integration. That change intentionally focuses on protocol, prelaunch orchestration, sidecar isolation, local motor bridging, and two plumbing demos; it does not attempt the full agentic benchmark roadmap described in this draft.

## Goal

Build a DimOS-native benchmark harness for evaluating agents that solve robotics tasks by calling existing skills.

This is intentionally **not** a code-as-policy benchmark. The unit under test is:

```text
LLM agent + system prompt + available MCP skills + DimOS blueprint + task scenario
```

The first milestone should prove that DimOS can run reproducible skill-calling episodes, score the final world state externally, and archive enough artifacts to debug failures.

## Non-Goals

- Do not execute model-generated Python code.
- Do not clone Cap-X's `ApiBase.functions()` / code-action interface yet.
- Do not introduce a large benchmark taxonomy in the first milestone.
- Do not require Ray, sandboxing, or parallel rollout infrastructure initially.
- Do not expose raw geometry helper APIs unless a pilot task needs them.

## Core Episode Loop

```text
┌──────────────────────────────┐
│ Agentic benchmark task spec   │
│ prompt / blueprint / skills   │
│ reset / timeout / scorer      │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Reset scenario                │
│ sim, replay, or real setup    │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Start DimOS blueprint         │
│ modules + MCP server/client   │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Send task prompt to agent     │
│ agent calls allowed skills    │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Observe outcome externally    │
│ streams / RPC / sim state     │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Score + archive artifacts     │
│ score.json / tool calls / logs│
└──────────────────────────────┘
```

## Task Spec Shape

The task spec should be declarative enough to run the same task across agents and models.

Conceptual YAML:

```yaml
name: pick-cup-place-on-marker
suite: tabletop-manipulation
blueprint: xarm-perception-sim-agent

prompt: |
  Find the cup and place it on the red marker.

skill_profile: manipulation_basic
allowed_skills:
  - get_scene_info
  - look
  - scan_objects
  - pick
  - place
  - place_back
  - drop_on
  - open_gripper
  - close_gripper

limits:
  timeout_s: 180
  max_tool_calls: 20

reset:
  type: simulation_scene
  scene: tabletop_basic
  seed: 1

success:
  type: object_near_target
  object: cup
  target: red_marker
  radius_m: 0.08

artifacts:
  record_agent_messages: true
  record_tool_calls: true
  record_module_logs: true
  record_final_observation: true
  record_rerun: true
```

## Skill Profiles

Skill profiles are benchmark contracts over existing DimOS MCP tools. They make evaluation fair by controlling what the agent can call.

### `manipulation_basic`

Initial profile for tabletop manipulation tasks:

- `get_scene_info(robot_name=None)`
- `look(robot_name=None)`
- `scan_objects(min_duration=0.0, robot_name=None)`
- `pick(object_name, object_id=None, robot_name=None)`
- `place(x, y, z, robot_name=None)`
- `place_back(robot_name=None)`
- `drop_on(target_object_name, z_offset=0.1, robot_name=None)`
- `open_gripper(robot_name=None)`
- `close_gripper(robot_name=None)`
- `move_to_pose(x, y, z, roll=None, pitch=None, yaw=None, robot_name=None)`
- `reset()`

Candidate source modules:

- `dimos/manipulation/manipulation_module.py`
- `dimos/manipulation/pick_and_place_module.py`

### `navigation_basic`

Future profile for mobile robot tasks:

- `navigate_with_text(query)`
- `tag_location(location_name)`
- `stop_navigation()`
- `relative_move(forward=0.0, left=0.0, degrees=0.0)`
- `wait(seconds)`

Candidate source modules:

- `dimos/agents/skills/navigation.py`
- `dimos/robot/unitree/unitree_skill_container.py`

### `drone_basic`

Future profile for aerial tasks:

- `takeoff(altitude)`
- `land()`
- `move(x, y, z, duration)`
- `fly_to(lat, lon, alt)`
- `follow_object(object_description, duration)`
- `observe()`

Candidate source module:

- `dimos/robot/drone/connection_module.py`

## Observation Model

For the first milestone, observation should be **agent-facing**, not tensor-first.

The agent should observe through existing skills such as:

- `get_scene_info()`
- `look()`
- `scan_objects()`
- `get_robot_state()`
- `observe()` for drone tasks

The harness may also collect a separate evaluator-facing snapshot for scoring:

```text
EvaluatorObservation
  - robot pose / joint state
  - end-effector pose
  - gripper state
  - detected objects
  - simulator ground truth, when available
  - fault / task state
```

The evaluator-facing snapshot should not automatically be exposed to the agent.

## Scoring Principles

The agent must not self-report success. Success is computed by the harness from world state, module state, simulator state, or trusted evaluator RPCs.

Example scorer types:

### Manipulation

- object detected
- object lifted above height threshold
- object within radius of target pose
- object near or on another object
- gripper empty/full after task
- robot/module not in fault state

### Navigation

- robot pose within radius of target
- visited required waypoint sequence
- target person/object followed for minimum duration
- no timeout or navigation failure

### Drone

- reached GPS/relative pose target
- maintained object in frame for minimum duration
- landed safely
- no disarm/failsafe violation

## Trial Artifacts

Each trial should emit a self-contained artifact directory.

```text
trial_<id>/
  task.yaml
  resolved_config.yaml
  prompt.txt
  available_skills.json
  agent_messages.jsonl
  tool_calls.jsonl
  skill_results.jsonl
  module_logs.jsonl
  final_observation.json
  score.json
  rerun.rrd or video.mp4
```

`score.json` should include at least:

```json
{
  "success": true,
  "score": 1.0,
  "reason": "cup within 0.08m of red_marker",
  "timeout": false,
  "tool_calls": 7,
  "duration_s": 93.4
}
```

## MVP Suite: Tabletop Manipulation

Start with a small suite that exercises actual agent skill choice, retry behavior, and interpretation of tool results.

### Task 1: `scan-visible-object`

Prompt:

```text
Find the cup in the scene.
```

Success:

- target object appears in detection snapshot or evaluator-visible object list.

Purpose:

- validates observation skills and basic agent-tool loop.

### Task 2: `pick-object`

Prompt:

```text
Pick up the cup.
```

Success:

- cup is lifted above a threshold or held by gripper according to evaluator state.

Purpose:

- validates `scan_objects`, `pick`, and recovery from missing object IDs.

### Task 3: `place-at-coordinate`

Prompt:

```text
Place the cup at the marked target position.
```

Success:

- cup is within radius of the target pose.

Purpose:

- validates pick/place sequencing and spatial instruction following.

### Task 4: `drop-on-object`

Prompt:

```text
Put the block on the tray.
```

Success:

- block is near or above tray region.

Purpose:

- validates object-to-object spatial reasoning through existing high-level skills.

### Task 5: `recover-after-failed-pick`

Prompt:

```text
Pick up the cup. If the first attempt fails, inspect the scene and try again.
```

Success:

- object eventually held or lifted within tool/time budget.

Purpose:

- validates agent recovery behavior and skill-result interpretation.

## Implementation Phases

### Concrete Slice: `framework-robosuite-integration`

Purpose: establish the simulator runtime substrate needed before agentic skill benchmarks can be reliable.

Scope:

- lightweight shared runtime protocol package outside the main `dimos` package
- network boundary between DimOS and simulator sidecars
- local SHM-only bridge between the DimOS runtime client and ControlCoordinator-facing WholeBodyAdapter
- prelaunch orchestration that starts the sidecar first, derives a resolved runtime plan, then launches a DimOS blueprint
- Robosuite Panda Lift as the first real backend plumbing demo
- fake sidecar smoke demo for dependency-light validation

Explicitly out of scope for this slice:

- new `dimos benchmark` CLI command
- LLM/agent task-success demo
- broad benchmark taxonomy
- code-as-policy execution

This slice should be completed before the roadmap moves into repeatable agent skill profiles and task scoring.

### Phase 0: Harness Design Spike

Purpose: finalize the declarative task schema and identify current repo seams.

Tasks:

- Confirm how to launch/stop target blueprints in-process or through `dimos run`.
- Confirm how to send prompts to an existing agent, likely reusing `dimos/agents/agent_test_runner.py` patterns.
- Confirm where MCP tool calls and agent messages can be captured.
- Define a minimal scorer interface.
- Define artifact directory layout.

Output:

- schema draft for `AgenticBenchmarkTask`
- first task YAML examples
- scorer interface sketch

### Phase 1: Black-Box Agent Runner

Purpose: run one prompt against one blueprint and collect artifacts.

Responsibilities:

- start or connect to a blueprint
- wait for MCP/agent readiness
- apply skill profile filtering, if supported
- send task prompt
- wait for completion, timeout, or max tool calls
- collect messages, tool calls, skill results, logs

Success criterion:

- one manually configured task produces a complete artifact directory and pass/fail score.

### Phase 2: Manipulation Scorers

Purpose: score the MVP tabletop suite externally.

Initial scorer interface:

```text
score(task, final_observation, artifact_dir) -> ScoreResult
```

Candidate scorer implementations:

- `object_detected`
- `object_lifted`
- `object_near_pose`
- `object_near_object`
- `module_not_faulted`

Success criterion:

- the five MVP manipulation tasks can be scored without asking the agent whether it succeeded.

### Phase 3: Batch Runs and Summaries

Purpose: run repeatable evaluations over tasks, seeds, and model configs.

Responsibilities:

- run multiple tasks sequentially
- aggregate results into `summary.json` and Markdown report
- preserve per-trial artifacts
- support reruns with the same seed/config

Success criterion:

- a suite run emits pass rate, average duration, average tool calls, failure reasons, and links to artifacts.

### Phase 4: Broaden Profiles

Purpose: reuse the harness for non-manipulation embodied tasks.

Candidate suites:

- Go2 navigation and person-following tasks
- drone observe/follow/fly-to tasks
- mixed perception/navigation tasks

## Key Design Questions

1. Should skill filtering happen by MCP server configuration, MCP client prompt/tool list filtering, or harness-side validation of tool calls?
2. What is the minimum reliable signal for “agent done” across current DimOS agents?
3. Should reset be required for all tasks, or can early tasks support `setup`/`teardown` only?
4. For simulated manipulation, where should ground-truth object poses come from?
5. Should scorer code call DimOS RPCs directly, read streams, or parse module outputs?
6. How strict should max-tool-call limits be for recovery-oriented tasks?

## Risks

- Existing skills return human-readable strings; scorers need structured state from modules or simulator, not just skill output text.
- Skill availability is currently blueprint-derived; task-scoped skill profiles may need a filtering layer.
- Reset semantics may be inconsistent across replay, simulation, and real hardware.
- Manipulation success may require simulator ground truth or reliable perception snapshots.
- Agent completion detection may need tighter integration than simple timeout-based waiting.

## Recommended First Milestone

Implement the smallest useful loop:

```text
one manipulation sim blueprint
one task YAML
one prompt
one skill profile
one external scorer
one artifact directory
```

Suggested first task:

```text
scan-visible-object
```

Reason: it validates benchmark plumbing before adding manipulation execution complexity.

After that passes, move to `pick-object`, then `place-at-coordinate`.
