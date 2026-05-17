# Go2 Architecture Layers

This directory names the Go2 blueprint composition boundaries that map to
layers 3 through 6 of the project architecture diagram. These files do not move
implementation modules; they only group existing blueprints so the runtime
structure can evolve without breaking current import paths.

## Layer Mapping

- Layer 3, Agent Brain: ExpertRouter, ContextProvider, MCP server/client, and
  the LLM agent loop.
- Layer 4, World State: spatial memory today, with a lazy temporal-memory
  factory for the existing temporal-memory blueprint.
- Layer 5, Skill Interface: MCP-callable skills and skill containers that
  bridge the agent to navigation, perception, speech, and Unitree commands.
- Layer 6, Robot Body / Local Policy: the existing Go2 robot stack, including
  perception, mapping, navigation, movement management, and hardware connection.

All blueprint variables in this directory are intentionally prefixed with `_`.
The registry generator skips those names, so these internal layer nodes do not
become new CLI blueprints.

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
the minimum context needed for the agent's next decision. Planning and tool
execution remain with the agent and Layer 5 skills.

## ExpertRouter Direction

`ExpertRouter` belongs in Layer 3 because it routes an LLM task toward the
right expert domain before tool selection. It does not execute robot actions or
own world state. The current implementation exposes
`route_task(task: str, context: str = "")` as an MCP skill. It uses deterministic
rules to return a domain, confidence, matched keywords, recommended tools, and a
flag indicating whether the agent should call `get_context` first.

## Layer 3 Version Roadmap

- V1: Keep Layer 3 as explicit, testable MCP tools. `ContextProvider` aggregates
  task, world, robot, runtime, and external-context placeholders.
  `ExpertRouter` performs deterministic domain routing for Go2 skills. The
  existing `McpClient` remains the LLM/VLM agent loop.
- V2: Add a shared skill-outcome store and `SkillOutcomePredictor` so Layer 3
  can warn about likely failures before tool execution. Upgrade routing to use
  recent skill outcomes, navigation state, and memory availability.
- V3: Add `CausalWorldModel` as an event/transition recorder that links
  before-context, chosen tool, tool result, after-context, inferred cause, and
  recovery suggestion. Use it to summarize repeated failure patterns.
- V4: Promote stable Go2-specific Layer 3 pieces into robot-agnostic modules
  under `dimos.agents`, while keeping robot-specific routing rules in the
  robot blueprint layer.
