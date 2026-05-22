# Layer 5 Skill Interface Design

This document explains the Go2 Layer 5 implementation logic. Layer 5 is the
boundary between the decision layer and executable robot skills. It describes
what tools exist, how they should be called, and what preflight checks upper
layers should run before execution.

## Current Scope

Layer 5 V1 keeps all existing skill implementations in place. It does not move,
wrap, or rewrite `NavigationSkillContainer`, `PersonFollowSkillContainer`,
`UnitreeSkillContainer`, `PerceiveLoopSkill`, `SecurityModule`, `WebInput`, or
`SpeakSkill`.

The new Layer 5 module is `_Go2SkillInterfaceRegistry`. It is an RPC-only
contract registry for the current Go2 MCP-callable skills.

The package entrypoint uses lazy blueprint construction. Importing
`skill_interface_spec.py` or `skill_interface_registry.py` should stay light and
must not import the full skill stack. The heavy skill containers are imported
only when a blueprint asks for `_go2_spatial_skill_interface`,
`_go2_agentic_skill_interface`, or `_go2_skill_interface`.

## Runtime Flow

The intended task flow is:

```text
Layer 3 ContextProvider
  -> SkillInterfaceSpec.get_skill_interface_snapshot()
  -> compact skill list in get_context(...)
  -> LLM chooses a Layer 5 skill
  -> Layer 3 predictor validates likely risk
  -> existing Layer 5 skill container executes the skill
```

The registry is read-only. It does not execute robot actions and does not
replace the existing MCP skill containers.

Layer 5 V2 compares the static contract registry against an MCP `tools/list`
payload so renamed, missing, or unregistered Go2 skills are detected in focused
tests before the agent relies on stale prompt context.

## Module Responsibilities

`_Go2SkillInterfaceRegistry`

- Stores static skill contracts for the Go2 agentic blueprint.
- Returns all contracts or contracts filtered by domain.
- Returns one contract by `skill_name`.
- Validates planned arguments against required fields and simple JSON types.
- Compares known contracts against MCP tool names while ignoring Layer 3 and
  MCP server internal tools.
- Does not call, retry, interrupt, or monitor the skill itself.

Existing skill containers

- Continue to expose the actual `@skill` methods.
- Continue to own their injected specs, streams, background loops, and robot
  side effects.
- Keep their current import paths and public skill names.

## Function Implementation Notes

### `_Go2SkillInterfaceRegistry.get_skill_interface_snapshot(...)`

- File: `layer_5_skill_interface/skill_interface_registry.py`
- Entry point: RPC-only helper.
- Purpose: expose a structured list of Go2 skill contracts to upper layers.
- Inputs:
  - Direct argument: optional `domain` filter.
  - Static data: `_SKILL_CONTRACTS`.
- Storage:
  - No database.
  - No persistent file writes.
  - Contracts are Python constants in the module.
- Data returned:
  - `available`, `version`, `source`, `domain_filter`, `domains`,
    `skill_count`, and `skills`.
  - Each skill includes domain, module, summary, required/optional arguments,
    argument types, context requirements, risk class, and recommended preflight
    tools.
- Algorithm:
  - Strip the domain filter.
  - Select all contracts if no domain is given, otherwise exact-match the
    domain.
  - Convert contracts to JSON-shaped dictionaries.
  - Derive sorted domains from the selected contracts.
- Current limits:
  - Contracts are manually maintained and should be updated when Go2 skills are
    added, removed, or renamed.

### `_Go2SkillInterfaceRegistry.get_skill_contract(...)`

- File: `layer_5_skill_interface/skill_interface_registry.py`
- Entry point: RPC-only helper.
- Purpose: return one skill contract for a planned MCP tool name.
- Inputs:
  - Direct argument: `skill_name`.
- Storage:
  - No database or persistent state.
- Algorithm:
  - Strip `skill_name`.
  - Find the exact contract in `_SKILL_CONTRACTS`.
  - Return `None` when no contract exists.
- Return shape:
  - Contract dictionary or `None`.

### `_Go2SkillInterfaceRegistry.validate_skill_request(...)`

- File: `layer_5_skill_interface/skill_interface_registry.py`
- Entry point: RPC-only helper.
- Purpose: catch obvious skill-call mistakes before execution.
- Inputs:
  - `skill_name`: the MCP tool name.
  - `args_json`: JSON object string containing planned tool arguments.
- Storage:
  - No database or persistent state.
- Algorithm:
  - Reject unknown skills.
  - Parse `args_json`; reject invalid JSON and non-object JSON.
  - Check all required arguments for missing, empty-string, or empty-list
    values.
  - Check simple argument types: `str`, `bool`, `float`, `list[float]`,
    `list[str]`, and `dict`.
  - Add warnings when the skill is context-dependent or motion-sensitive.
- Return shape:
  - `valid`, `skill_name`, `errors`, `warnings`, and `contract`.
- Current limits:
  - This is schema/risk metadata validation, not semantic validation. For
    example, it can confirm that `query` exists, but not that a target location
    is actually reachable.

### `_Go2SkillInterfaceRegistry.compare_mcp_tools(...)`

- File: `layer_5_skill_interface/skill_interface_registry.py`
- Entry point: RPC-only helper.
- Purpose: detect drift between the static Layer 5 contracts and the MCP tool
  names actually exposed by the full Go2 agentic blueprint.
- Inputs:
  - `tools_json`: JSON from MCP `tools/list`, a JSON-RPC result wrapper, a list
    of tool-name strings, or a list of tool objects with `name` fields.
- Storage:
  - No database or persistent state.
- Algorithm:
  - Parse tool names from the payload.
  - Compare them with `_SKILL_CONTRACTS`.
  - Ignore known non-Layer-5 MCP tools from Layer 3 and `McpServer`, such as
    `get_context`, `route_task`, `record_skill_outcome`, and `server_status`.
  - Report missing contracts and unexpected MCP tools separately.
- Return shape:
  - `valid`, `errors`, `contract_skill_count`, `mcp_tool_count`,
    `contract_skill_names`, `mcp_tool_names`, `known_non_layer5_mcp_tools`,
    `missing_contracts`, and `unregistered_mcp_tools`.
  - `valid` is true only when parsing succeeds and both drift lists are empty.
- Current limits:
  - The focused full-blueprint test uses static `@skill` inspection rather than
    starting an MCP server process. That catches renamed or newly exposed skill
    methods without needing robot hardware.

## Version Boundaries

V1 implemented:

- A static Go2 skill contract registry.
- RPC methods for skill snapshot, single-skill lookup, and planned-call
  validation.
- ContextProvider integration so Layer 3 can see the Layer 5 skill interface.
- Lazy Layer 5 package imports so Layer 3 specs/tests do not pull in the full
  skill dependency stack.

V2 implemented:

- Compare the static contract registry against the MCP server tool list during
  tests or startup, so missing/renamed skills are detected automatically.
- Added contracts for MCP-exposed Go2 skills that previously bypassed the
  registry: `observe`, `begin_exploration`, `end_exploration`, `start_patrol`,
  and `stop_patrol`.

Remaining V2 work:

- Add more precise argument constraints for Unitree sport commands and
  perception callbacks.

V3 planned:

- Feed validated skill contracts into a structured planner/policy so Layer 3
  can choose tools from contracts instead of relying only on prompt text.

## Design Rules

- Keep Layer 5 contracts close to Go2 while the skill set is still evolving.
- Do not move or wrap existing skill implementations in this stage.
- Use exact public skill names as contract keys.
- Treat physical-motion skills as context-dependent and preflight-sensitive.
- Update this document and `GOAL_1.md` whenever Layer 5 contract fields,
  validation logic, or included skills change.
