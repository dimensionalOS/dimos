# GOAL_1: Go2 Architecture Layers

This file records the first-stage architecture goal for the Go2 blueprint
layers. The stage-1 goal is to name and organize the Layer 3 through Layer 6
composition boundaries without moving underlying implementation modules or
breaking existing public blueprint names.

## Layer Mapping

- Layer 3, Agent Brain: ExpertRouter, ContextProvider, SkillOutcomeStore,
  SkillOutcomePredictor, CausalWorldModel, MCP server/client, and the LLM
  agent loop.
- Layer 4, World State: spatial memory today, with a lazy temporal-memory
  factory for the existing temporal-memory blueprint.
- Layer 5, Skill Interface: MCP-callable skills and skill containers that
  bridge the agent to navigation, perception, speech, and Unitree commands.
- Layer 6, Robot Body / Local Policy: the existing Go2 robot stack, including
  perception, mapping, navigation, movement management, and hardware connection.

All blueprint variables in this directory are intentionally prefixed with `_`.
The registry generator skips those names, so these internal layer nodes do not
become new CLI blueprints.

## Directory Layout

Each architecture layer has its own directory:

```text
layers/
  layer_3_agent_brain/
    __init__.py                  # internal Layer 3 blueprint composition
    context_provider.py          # context aggregation for the LLM agent
    expert_router.py             # deterministic expert-domain routing
    prompt_policy.py             # Layer 3 tool-use policy for McpClient
    skill_outcome_store.py       # recent skill outcome memory
    skill_outcome_predictor.py   # rule-based preflight risk checks
    causal_world_model.py        # before/action/result/after causal memory
    test_*.py                    # focused Layer 3 tests
  layer_4_world_state/
    __init__.py                  # spatial/temporal memory blueprint pieces
    structured_world_state.py    # normalized Layer 4 world snapshot
    semantic_temporal_map.py     # spatial/temporal memory evidence view
    world_state_spec.py          # Layer 4 RPC specs
    DESIGN.md                    # Layer 4 implementation logic
    test_*.py                    # focused Layer 4 tests
  layer_5_skill_interface/
    __init__.py                  # Go2 skill-interface blueprint pieces
    skill_interface_registry.py  # static Go2 skill contract registry
    skill_interface_spec.py      # Layer 5 RPC spec
    DESIGN.md                    # Layer 5 implementation logic
    test_*.py                    # focused Layer 5 tests
  layer_6_robot_body/
    __init__.py                  # Go2 robot-body/local-policy blueprint piece
    robot_body_state.py          # Layer 6 body/local-policy state facade
    robot_body_spec.py           # Layer 6 RPC spec
    DESIGN.md                    # Layer 6 implementation logic
    test_*.py                    # focused Layer 6 tests
```

The public internal import paths intentionally stay stable at the layer level,
for example `dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain`.

## ContextProvider Direction

`ContextProvider` belongs in Layer 3 because it prepares context for the LLM
agent. It should not own the underlying robot or world state. Instead, it should
aggregate and compress context from these sources:

- LLM/task context: user goal, decomposed subtasks, dialogue summary, and current
  tool-call intent.
- Layer 4 world state: `SpatialMemory`, `TemporalMemory`, and later structured
  world-state or semantic-temporal map modules.
- Layer 5 skill state: recent skill outcomes, failure reasons, risks, and
  recovery suggestions.
- Layer 6 robot state: odometry, navigation state, connection state, safety
  state, and perception summaries.
- Runtime context: replay/simulation/hardware mode, blueprint/config values, and
  available modules.
- External context: web input, human input, external LLM/VLM results, and future
  connected services.

The current implementation exposes a compact MCP skill,
`get_context(task: str, focus: str = "", spatial_limit: int = 3)`, returning
the minimum context needed for the agent's next decision. It now prefers the
Layer 4 `WorldStateSpec.get_world_snapshot(...)` RPC when available, and keeps
direct spatial/temporal/odom reads as fallback behavior. Planning and tool
execution remain with the agent and Layer 5 skills.

## Layer 4 Direction

Layer 4 owns the structured robot/world-state view. It should normalize state
from memory, odom, navigation, runtime config, and later safety/connection
sources, then expose RPC methods that upper layers can read.

Current Layer 4 modules:

- `_Go2SemanticTemporalMap`: combines spatial-memory matches and temporal-memory
  summaries/answers for one query. It does not create a new database.
- `_Go2StructuredWorldState`: returns one `get_world_snapshot(...)` payload with
  `runtime`, `robot_state`, `memory_state`, and `semantic_temporal_map` fields.

Layer 4 V1 is intentionally a facade over existing modules. The existing
`SpatialMemory` and lazy `TemporalMemory` implementations stay in their current
packages.

## Layer 5 Direction

Layer 5 owns the skill-interface boundary. It should tell upper layers which
Go2 skills exist, what arguments each skill expects, which skills are
motion-sensitive, and what preflight checks should run before execution.

Current Layer 5 V1 keeps existing skill containers unchanged and adds
`_Go2SkillInterfaceRegistry` as an RPC-only contract registry. The registry
does not execute skills. It returns static contracts for current Go2 skills,
including navigation, person following, Unitree motion/utility commands,
perception lookout, security patrol, and speech. `ContextProvider` reads this
registry through `SkillInterfaceSpec` and includes a compact skill-interface
summary in `get_context(...)`.

Layer 5 V2 should compare the static contracts with the MCP server tool list so
renamed or missing skills are detected automatically.

## Layer 6 Direction

Layer 6 owns the robot body and local-policy boundary. It includes the existing
Go2 connection, sensor streams, mapping, navigation, patrolling, movement
manager, and local safety behavior. Current Layer 6 V1 keeps those
implementations in place and adds `_Go2RobotBodyState` as an RPC-only
observational facade.

`_Go2RobotBodyState` tracks odom, color-image, and lidar stream liveness,
reports runtime connection mode/configuration, and exposes conservative
local-policy/safety context. It does not move the robot or replace physical
safety checks. Layer 4 reads it through `RobotBodyStateSpec` and includes the
body/local-policy sections in `robot_state`.

## ExpertRouter Direction

`ExpertRouter` belongs in Layer 3 because it routes an LLM task toward the
right expert domain before tool selection. It does not execute robot actions or
own world state. The current implementation exposes
`route_task(task: str, context: str = "")` as an MCP skill. It uses deterministic
rules to return a domain, confidence, matched keywords, recommended tools, and a
flag indicating whether the agent should call `get_context` first.

## Layer 3 Version Roadmap

- V1 implemented: Keep Layer 3 as explicit, testable MCP tools.
  `ContextProvider` aggregates task, world, robot, runtime, skill-outcome, and
  external-context placeholders.
  `ExpertRouter` performs deterministic domain routing for Go2 skills. The
  existing `McpClient` remains the LLM/VLM agent loop.
- V2 implemented: Add a shared skill-outcome store and
  `SkillOutcomePredictor` so Layer 3 can warn about likely failures before tool
  execution. The Go2 Layer 3 prompt now asks the agent to use
  `route_task -> get_context -> predict_skill_outcome` for non-trivial tasks,
  and `McpClient` automatically records non-internal MCP tool outcomes when
  `record_skill_outcome` is available.
- V3 implemented: Add `CausalWorldModel` as an event/transition recorder that links
  before-context, chosen tool, tool result, after-context, inferred cause, and
  recovery suggestion. Use it to summarize repeated failure patterns.
- Out of scope for this stage: promoting Layer 3 pieces into robot-agnostic
  modules under `dimos.agents`. Do this only after Layers 4, 5, and 6 have
  clearer Go2 boundaries and the V3 causal loop has been validated.

## Skill Outcome Direction

`SkillOutcomeStore` is the Layer 3 memory for recent skill calls. The current
implementation exposes `record_skill_outcome(...)` and
`summarize_skill_outcomes(...)` as MCP skills, and exposes an internal
`get_recent_outcomes(...)` RPC for other Layer 3 modules. Agent-driven tool
calls are recorded automatically by `McpClient` when the outcome store is
present; manual recording is still available for external events or outcomes
outside the agent's MCP tool path.

`SkillOutcomePredictor` performs conservative preflight checks with
`predict_skill_outcome(skill_name: str, args_json: str = "", context: str = "")`.
It uses deterministic rules plus recent outcomes from `SkillOutcomeStore`.
This is not a trained model yet; it is a risk gate that helps the agent avoid
obvious retries, missing arguments, or movement-sensitive calls without context.

## Causal World Model Direction

`CausalWorldModel` records compact Layer 3 transitions:

```text
task + before_context + chosen skill + args + prediction + outcome
  -> after_context + inferred_cause + recovery suggestion
```

The current implementation exposes `record_causal_transition(...)` and
`summarize_causal_patterns(...)` as MCP skills, and exposes
`get_recent_transitions(...)` as an internal RPC for other Layer 3 modules.
It stores recent transitions in memory only. `ContextProvider` includes recent
causal transitions in its context payload, and `SkillOutcomePredictor` raises
risk when the same skill repeatedly fails for the same inferred cause.
