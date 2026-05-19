# Layer 3 Agent Brain Design

This document explains how the Go2 Layer 3 implementation works and why it is
shaped this way. It is the companion to `GOAL_1.md`: `GOAL_1.md` records the
stage goal, while this file explains the implementation logic.

## Current Scope

Layer 3 is the decision layer above world state and skill execution. It does
not move the robot directly. It prepares context, routes tasks, estimates risk,
and exposes tools through MCP so the LLM agent can choose safer next actions.

Current Layer 3 modules:

- `McpClient`: existing LLM agent loop.
- `McpServer`: exposes `@skill` methods as MCP tools.
- `ContextProvider`: gathers compact task/world/robot/runtime/skill context.
- `ExpertRouter`: maps a task to an expert domain and recommended tools.
- `PromptPolicy`: appends Layer 3 tool-use rules to the Go2 MCP client prompt.
- `SkillOutcomeStore`: records recent skill outcomes in memory.
- `SkillOutcomePredictor`: checks planned tool calls for obvious risk.
- `CausalWorldModel`: records before/action/result/after transitions and
  summarizes repeated failure causes.

## Runtime Flow

The intended task flow is:

```text
user task
  -> McpClient / LLM
  -> route_task(task)
  -> get_context(task, focus)
  -> predict_skill_outcome(skill_name, args_json, context)
  -> call a Layer 5 skill
  -> McpClient auto-records the outcome through record_skill_outcome(...)
  -> get_context(task, focus) for after-context
  -> record_causal_transition(...)
  -> summarize_causal_patterns(...) before risky retries
```

The final `record_skill_outcome(...)` call is automatic for agent-driven,
non-internal MCP tool calls when the outcome store is present. Manual recording
is still useful for external events or important outcomes that did not flow
through `McpClient`.

## Module Responsibilities

`ContextProvider`

- Reads task text passed by the agent.
- Optionally reads `SpatialMemory`, `TemporalMemory`, navigation state, odom,
  runtime config, and recent skill outcomes.
- Returns a compact `SkillResult` with text plus structured metadata.
- Does not plan, execute skills, or own persistent state.

`ExpertRouter`

- Uses deterministic keyword rules.
- Returns a domain such as `navigation`, `perception`, `safety`, `memory`, or
  `person_follow`.
- Recommends existing MCP tools.
- Does not call those tools.

`SkillOutcomeStore`

- Stores recent outcomes in a bounded in-memory `deque`.
- Keeps at most 100 records.
- Clears when the blueprint/process restarts.
- Is intentionally not backed by a database in this stage.
- Receives automatic records from `McpClient` for non-internal MCP tool calls.

`SkillOutcomePredictor`

- Is not a trained model.
- Uses rule checks over `skill_name`, `args_json`, context text, and recent
  same-skill outcomes.
- Returns `low`, `medium`, or `high` risk, with reasons and recovery
  suggestions.

`CausalWorldModel`

- Stores recent causal transitions in a bounded in-memory `deque`.
- Links task, before-context, chosen skill, args, prediction, outcome,
  after-context, inferred cause, and recovery suggestion.
- Uses deterministic cause rules, not a learned causal model.
- Does not execute skills or own world state.

## Implementation Documentation Standard

Every new Layer 3 requirement that changes behavior should add or update a
function-level implementation note in this document. The goal is that a reader
can understand what changed without reverse-engineering every branch from the
code.

Each implementation note should cover:

- Entry point: class, function, file, and whether it is an MCP skill, RPC-only
  helper, stream callback, or private helper.
- Purpose: what problem it solves and what it deliberately does not do.
- Inputs: user arguments, injected Specs, streams, global config, and helper
  state.
- Storage: whether it writes memory, a file, a database, or no persistent state.
- Data shape: the exact fields written or returned.
- Algorithm: the branch/order of operations used to produce the result.
- Error handling: validation failures, caught dependency errors, and fallback
  behavior.
- Return shape: success/failure `SkillResult`, RPC return, metadata keys, and
  important message text.
- Tests: focused tests or compile checks that protect the behavior.
- Limits and next version: known shortcuts and what should change in V2/V3.

## Function Implementation Notes

### `_Go2ContextProvider.get_context(...)`

- File: `layer_3_agent_brain/context_provider.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: aggregate compact context for the LLM before it selects a tool.
  It does not plan, execute robot actions, or own world-state storage.
- Inputs:
  - Direct arguments: `task`, `focus`, `spatial_limit`.
  - Stream state: latest `odom` message cached by `_on_odom`.
  - Optional injected Specs: `SpatialMemorySpec`, `TemporalMemorySpec`,
    `NavigationInterfaceSpec`, `SkillOutcomeStoreSpec`.
  - Runtime config: `global_config.simulation`, `replay`, `robot_ip`, `viewer`,
    `mcp_port`, and `n_workers`.
- Storage:
  - No database.
  - No persistent file writes.
  - Only keeps the latest odom message in `_latest_odom` while the process is
    running.
- Data read:
  - Spatial memory: `query_by_text(task, limit=spatial_limit)`.
  - Temporal memory: `get_rolling_summary()` and `get_state()`.
  - Navigation: `get_state()` and `is_goal_reached()`.
  - Skill history: `get_recent_outcomes(limit=5)`.
- Algorithm:
  - Trim `task` and `focus`.
  - Reject empty `task` with `SkillResult.fail("INVALID_INPUT", ...)`.
  - Clamp `spatial_limit` to `0..10`.
  - Build a metadata dictionary with `sources`, `runtime`, `robot_state`,
    `world_state`, `skill_state`, and `external_context`.
  - Catch dependency failures per source and append readable warnings to
    `errors` instead of failing the whole context request.
  - Convert metadata through `_to_jsonable(...)` so MCP responses stay compact
    and serializable.
  - Format a short text summary for the LLM.
- Return shape:
  - `SkillResult(success=True, message=<summary>, metadata=<jsonable dict>)`.
  - Metadata keys: `task`, `focus`, `sources`, `runtime`, `robot_state`,
    `world_state`, `skill_state`, `external_context`, and optional `errors`.
- Current limits:
  - External context is represented as unavailable.
  - Context compression is deterministic formatting, not learned retrieval.
  - It reads existing stores but does not decide whether a skill should run.

### `_Go2ExpertRouter.route_task(...)`

- File: `layer_3_agent_brain/expert_router.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: classify a task into a Go2 expert domain and recommend candidate MCP
  tools. It does not call those tools.
- Inputs:
  - Direct arguments: `task`, optional `context`.
  - Static rule table: `_ROUTE_RULES`.
- Storage:
  - No database.
  - No persistent state.
  - Rules are Python constants in the module.
- Data shape:
  - Each `_RouteRule` contains `domain`, `keywords`, `tools`, `reason`, and
    `needs_context`.
  - Domains currently include `safety`, `speech`, `memory`, `person_follow`,
    `perception`, `navigation`, `security`, `robot_motion`, `utility`,
    `human_help`, and fallback `general`.
- Algorithm:
  - Trim inputs and reject empty `task`.
  - Combine `task` and `context`, then `casefold()` for matching.
  - For each rule, collect keywords contained in the combined text.
  - Select the rule with the highest keyword-match count.
  - Use the fallback `general` rule when no domain-specific keyword matches.
  - If the selected rule needs context and no context was supplied, mark
    `needs_context=True` and ensure `get_context` is recommended.
  - Map keyword count to confidence: `0=low`, `1=medium`, `2+=high`.
- Return shape:
  - `SkillResult(success=True, message=<route summary>, metadata=<dict>)`.
  - Metadata keys: `task`, `context_used`, `domain`, `confidence`,
    `matched_keywords`, `recommended_tools`, `needs_context`, and `reason`.
- Current limits:
  - Routing is deterministic keyword matching, not semantic classification.
  - Rule order only matters when match counts tie; the current implementation
    keeps the earlier best route.

### `_Go2SkillOutcomeStore.record_skill_outcome(...)`

- File: `layer_3_agent_brain/skill_outcome_store.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: record recent Layer 5 tool results so Layer 3 can include history in
  context and prediction. It does not execute, retry, or repair the skill.
- Inputs:
  - `skill_name`, `success`, `domain`, `error_code`, `message`, `risk`, and
    `recovery`.
- Writers:
  - `McpClient._mcp_tool_call(...)` writes automatically after non-internal MCP
    tool calls when `record_skill_outcome` is available.
  - LLM or human operators may still call this skill manually for external
    events or non-MCP outcomes.
- Storage:
  - Storage backend is an in-memory `collections.deque`.
  - The deque lives at `self._outcomes` inside `_Go2SkillOutcomeStore`.
  - Maximum retained records: `_max_outcomes = 100`.
  - No SQL database, vector database, file, Redis, or external service is used.
  - Restarting the blueprint or worker process clears the history.
- Data written:
  - Each record is a `_SkillOutcome` dataclass with `timestamp`, `skill_name`,
    `success`, `domain`, `error_code`, `message`, `risk`, and `recovery`.
  - `timestamp` is recorded with `time.time()` when the outcome is written.
  - `to_dict()` rounds timestamp to three decimals for JSON-shaped responses.
- Algorithm:
  - Strip text inputs.
  - Reject empty `skill_name`.
  - Validate `risk` against `low`, `medium`, `high`, and `unknown`.
  - Create `_SkillOutcome`.
  - Append to `self._outcomes`; the oldest item is dropped automatically after
    the 100-record cap.
  - Return the written outcome and current count.
- Related read APIs:
  - `summarize_skill_outcomes(...)` is an MCP skill for human/LLM inspection.
  - `get_recent_outcomes(...)` is RPC-only and returns newest records first.
  - Filtering uses exact `skill_name` and `domain` matches.
- Return shape:
  - Success: `SkillResult.ok("Recorded outcome for ...", outcome=<dict>,
    total_outcomes=<int>)`.
  - Failure: `SkillResult.fail("INVALID_INPUT", ...)`.
- Current limits:
  - Automatic recording only covers the agent's `McpClient` tool path.
    Direct external MCP clients can still call `record_skill_outcome(...)`
    manually if they need history.
  - There is no durable audit log yet.

### `_Go2SkillOutcomePredictor.predict_skill_outcome(...)`

- File: `layer_3_agent_brain/skill_outcome_predictor.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: perform a conservative preflight risk check before the agent calls a
  physical or recovery-sensitive skill. It does not execute the skill and it is
  not a trained ML model.
- Inputs:
  - Direct arguments: `skill_name`, `args_json`, `context`.
  - Optional injected Spec: `SkillOutcomeStoreSpec` for recent same-skill
    outcomes.
- Storage:
  - No database.
  - No persistent state.
  - Reads recent outcomes if the store is wired, but does not write them.
  - Reads recent causal transitions if `CausalWorldModel` is wired.
- Data read:
  - `args_json` is parsed as a JSON object string because MCP tool schemas are
    simpler with primitive string arguments.
  - Recent history is read with `get_recent_outcomes(limit=5,
    skill_name=skill_name)`.
- Algorithm:
  - Trim `skill_name` and `context`.
  - Reject empty `skill_name`.
  - Parse `args_json`; reject invalid JSON and non-object JSON.
  - Build risk reasons from five signals: skill name, parsed args, context
    text, recent same-skill outcomes, and recent same-skill causal transitions.
  - Add history reasons for one recent failure or repeated recent failures.
  - Add movement-context reasons for `navigate_with_text`, `relative_move`, and
    `follow_person` when context or odom appears unavailable.
  - Add argument-specific blockers:
    - `navigate_with_text` requires a non-empty `query`.
    - `follow_person` requires a non-empty `query`.
    - `look_out_for` requires `description_of_things`.
  - Downgrade stop, utility, and speech tools so missing robot context does not
    block urgent or simple actions.
  - Map reasons to risk: hard blockers or repeated failures become `high`, any
    other reason becomes `medium`, and no reasons becomes `low`.
  - Produce recovery suggestions based on skill family.
- Return shape:
  - `SkillResult(success=True, message=<risk summary>, metadata=<dict>)`.
  - Metadata keys: `skill_name`, `args`, `risk`, `predicted_success`,
    `failure_reasons`, `recovery_suggestions`, `recent_outcomes`,
    `recent_causal_transitions`, `outcome_store_available`, and
    `causal_world_model_available`.
- Current limits:
  - `predicted_success` is `risk != "high"`, not a probability.
  - Risk rules are handcrafted and should be revised after real outcome data is
    available.

### `_go2_layer_3_system_prompt(...)`

- File: `layer_3_agent_brain/prompt_policy.py`
- Entry point: private helper used by `_go2_agent_brain_with_client(...)`.
- Purpose: append the Go2 Layer 3 decision policy to the MCP client's system
  prompt without changing the public `McpClient` API.
- Inputs:
  - Optional `base_prompt`; defaults to `dimos.agents.system_prompt.SYSTEM_PROMPT`.
- Storage:
  - No database.
  - No runtime state.
  - Returns a composed prompt string.
- Algorithm:
  - Trim the base prompt.
  - If the Layer 3 policy header is already present, return the prompt
    unchanged.
  - Otherwise append a policy block telling the agent to use
    `route_task`, `get_context`, and `predict_skill_outcome` before risky
    physical tools.
  - Tell the agent that normal MCP tool outcomes are recorded automatically and
    manual `record_skill_outcome(...)` is only for external/non-MCP events.
- Return shape:
  - `str`.
- Current limits:
  - This is prompt guidance, not a hard planner. It improves consistency but
    cannot prove the LLM will always follow the intended flow.

### `McpClient._mcp_tool_call(...)` Outcome Recording

- File: `dimos/agents/mcp/mcp_client.py`
- Entry point: internal MCP client helper used by LangChain tools and
  continuation execution.
- Purpose: automatically write recent outcomes into `SkillOutcomeStore` after
  agent-driven MCP tool calls.
- Inputs:
  - Tool name and arguments.
  - MCP tool response content.
  - `_tool_registry`, used to detect whether `record_skill_outcome` is
    available.
- Storage:
  - Does not store data locally.
  - Calls the MCP `record_skill_outcome` tool with `record_outcome=False` to
    avoid recursive self-recording.
- Data shape written:
  - `skill_name`, `success`, `domain`, `error_code`, `message`, `risk`, and
    `recovery`.
- Algorithm:
  - Send the original `tools/call` request with a progress token.
  - If the call raises, record a failed outcome with `risk="high"` when the
    store is available, then re-raise.
  - If the call succeeds, skip Layer 3/internal tools such as `get_context`,
    `route_task`, `predict_skill_outcome`, `record_skill_outcome`, server
    status, and utility agent-send tools.
  - For `SkillResult` JSON responses, preserve `success`, `message`, and
    `error_code`.
  - For plain string responses, treat normal MCP responses as successful and
    obvious server error text as failed.
  - Infer a coarse domain from known Go2 tool names when possible.
- Return shape:
  - Returns the original MCP response unchanged.
- Current limits:
  - Plain string tools cannot provide precise success/failure semantics unless
    they migrate to `SkillResult`.
  - Direct external MCP calls that bypass `McpClient` are not auto-recorded.

### `_Go2CausalWorldModel.record_causal_transition(...)`

- File: `layer_3_agent_brain/causal_world_model.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: record one compact causal transition after an important skill call.
  It does not execute, retry, or repair the skill.
- Inputs:
  - Direct arguments: `task`, `skill_name`, `args_json`, `before_context`,
    `after_context`, `prediction_json`, `outcome_json`, and `domain`.
  - Optional injected Spec: `SkillOutcomeStoreSpec` for latest same-skill
    outcome fallback when `outcome_json` is omitted.
- Storage:
  - Storage backend is an in-memory `collections.deque`.
  - The deque lives at `self._transitions` inside `_Go2CausalWorldModel`.
  - Maximum retained records: `_max_transitions = 200`.
  - No SQL database, vector database, file, Redis, or external service is used.
  - Restarting the blueprint or worker process clears the history.
- Data written:
  - Each record is a `_CausalTransition` dataclass with `timestamp`, `task`,
    `domain`, `skill_name`, `args`, `before_context`, `prediction_risk`,
    `prediction_reasons`, `outcome_success`, `outcome_error_code`,
    `outcome_message`, `after_context`, `inferred_cause`, `recovery`, and
    `confidence`.
- Algorithm:
  - Trim `task`, `skill_name`, context strings, and domain.
  - Reject empty `task` or `skill_name`.
  - Parse `args_json`, `prediction_json`, and `outcome_json` as JSON objects.
  - If `outcome_json` is omitted, read the latest same-skill outcome from
    `SkillOutcomeStore` when available.
  - Flatten `metadata` from SkillResult-shaped JSON so prediction/outcome fields
    are easy to read.
  - Infer cause with deterministic rules:
    missing args, repeated failures, unavailable odom, missing map target,
    unavailable world state, unavailable visual target, explicit error code,
    unknown failure, or success.
  - Append the transition to the bounded deque.
- Return shape:
  - Success: `SkillResult.ok("Recorded causal transition ...",
    transition=<dict>, total_transitions=<int>)`.
  - Failure: `SkillResult.fail("INVALID_INPUT", ...)`.
- Current limits:
  - Cause inference is rule-based and coarse.
  - The agent must explicitly call this after important physical/recovery tool
    calls; `McpClient` only auto-records simple skill outcomes, not full causal
    transitions.
  - Context snapshots are compact strings, not full world-state copies.

### `_Go2CausalWorldModel.summarize_causal_patterns(...)`

- File: `layer_3_agent_brain/causal_world_model.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: summarize repeated recent failure causes before the agent retries a
  skill.
- Inputs:
  - Optional filters: `skill_name`, `domain`, and `limit`.
- Storage:
  - Reads the in-memory causal transition deque.
  - Does not write new data.
- Algorithm:
  - Read newest transitions first through `get_recent_transitions(...)`.
  - Keep failure transitions where `outcome_success is False`.
  - Count transitions by `inferred_cause`.
  - Return ordered patterns with cause, count, affected skill names, latest
    message, and recovery suggestion.
- Return shape:
  - `SkillResult.ok(<summary>, patterns=<list>, transitions=<list>)`.
- Current limits:
  - Pattern summaries only cover recent in-memory transitions.
  - It does not yet aggregate across process restarts.

## Version Boundaries

V1 implemented:

- Layer 3 is explicit MCP tools rather than hidden agent internals.
- `ContextProvider` gives the agent a compact context view.
- `ExpertRouter` gives deterministic task-domain routing.
- Existing `McpClient` remains the LLM/VLM agent loop.

V2 implemented:

- `SkillOutcomeStore` records recent skill results.
- `SkillOutcomePredictor` performs preflight risk checks.
- `ContextProvider` includes recent skill outcomes when available.
- Go2 Layer 3 prompt policy asks the LLM to route, gather context, and predict
  risk before non-trivial or movement-sensitive tools.
- `McpClient` automatically records non-internal MCP tool outcomes when
  `record_skill_outcome` is available.

V3 implemented:

- Added `CausalWorldModel` as an event transition recorder:
  before-context, action, result, after-context, inferred cause, recovery.
- `ContextProvider` includes recent causal transitions when available.
- `SkillOutcomePredictor` raises risk when same-skill causal failures repeat.
- `PromptPolicy` asks the agent to record causal transitions after important
  physical or recovery-sensitive tool calls.

Out of scope for this stage:

- Do not promote Layer 3 pieces into robot-agnostic `dimos.agents` modules yet.
  The current architecture work should stop at V3 until Layers 4, 5, and 6 have
  clearer Go2 boundaries and the causal loop has been validated.

## Design Rules

- Keep layer files internal by using `_`-prefixed blueprint variables/classes.
- Preserve existing public blueprint names and imports at the layer level.
- Do not make Layer 3 execute physical actions directly.
- Prefer deterministic rules first; introduce models only after data and tests
  are stable.
- Update this file when Layer 3 behavior changes in a way that affects task
  flow, stored data, prediction logic, or version boundaries.
- Update the implementation notes above whenever a new requirement adds a
  function, changes storage, changes returned metadata, or changes decision
  logic.
