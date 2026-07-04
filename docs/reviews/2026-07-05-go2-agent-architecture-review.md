# Go2 Agent Architecture Review Notes

Date: 2026-07-05
Branch: `refactor/go2-architecture-layers`
Local HEAD before upstream merge: `646e7ec5`
Upstream merged target: `upstream/main` at `6e813a72`
Merge base: `1f544d05`

This document is the entry point for a full review of the current fork
changes. It separates upstream changes from local feature work, then describes
each local feature in enough detail to review implementation logic rather than
only file names.

## Current Git State

The branch had 16 local commits not in upstream, and upstream had 160 commits
not in the branch. The accurate conflict surface from the merge base was 15
overlapping files; the actual merge conflicts were 7 files:

- `dimos/agents/mcp/mcp_server.py`
- `dimos/agents/mcp/test_mcp_server.py`
- `dimos/perception/spatial_perception.py`
- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py`
- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_temporal_memory.py`
- `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_spatial.py`
- `dimos/robot/unitree/go2/test_connection.py`

Resolution strategy:

- Keep upstream infrastructure changes, especially Zenoh transport, capability
  gating, AES key forwarding, docs and packaging updates.
- Keep the local Go2 layer architecture as the authoritative Go2 agentic wiring.
- Use upstream's new perception module paths, while preserving the local
  state-dir based spatial-memory default.
- Combine non-overlapping tests rather than dropping either side.

Important environment note:

- Upstream now defaults macOS transport to `zenoh`. That requires
  `eclipse-zenoh`; I installed `eclipse-zenoh==1.9.0` in the local `.venv`.
- The machine already has a live `dimos` process listening on MCP port `9990`.
  MCP tests must use another port, for example `MCP_PORT=9991`, or they will
  talk to the live server instead of the test server.
- `git diff --cached --check` passes when excluding upstream `.patch` files.
  Full `diff --check` reports trailing whitespace inside upstream patch files;
  those are patch context lines and should be handled separately if we want a
  repository-wide whitespace cleanup.

## Review Granularity

Use this structure when reviewing each local feature. The goal is to avoid
approving a feature just because the file names sound plausible.

- Entry point: Which blueprint, module, MCP skill, RPC method, or helper is the
  first public surface?
- Inputs: Which JSON strings, spec providers, memory providers, or runtime
  states feed it?
- Outputs: Which `SkillResult`, dict schema, Spec return value, file artifact,
  or dashboard payload does the interface return?
- Decision owner: Is the decision made by deterministic code, the LLM reading
  context, a small online statistical model, or a human reviewer?
- State: Is the feature stateless, in-memory only, persisted to JSON, persisted
  to Git, or stored in an existing RAG/vector database?
- Write boundary: Which exact method writes, and which methods are read-only?
- Failure mode: Does missing data produce `unavailable`, `uncertain`, a
  clarifying question, a failed `SkillResult`, or a proposal artifact?
- Safety boundary: Can it execute robot motion, or only advise another layer?
- Review artifact: Which schema should be stable enough for future tooling?
- Tests: Which test proves the data path, and which test proves the failure path?

The most important distinction in this branch is between three very different
things:

- Context selection: deterministic filtering and summarization before the LLM
  sees context.
- Agent judgment: the LLM deciding what to do with selected context and tools.
- Self-evolution artifact: reviewable JSON events/proposals, not automatic code
  mutation.

## Architecture Diagram

![DimOS Go2 agent self-evolution architecture](2026-07-05-go2-agent-architecture.png)

The purple right-side column is optional observability/storage. The current
`unitree_go2_agentic` blueprint does not wire `WebsocketVisModule`; the Rerun /
WebsocketVis panel only receives world-model state if a separate visualization
blueprint wires `WebsocketVisSpec`.

## Interface Input/Output Quick Reference

Use this section when reviewing API boundaries. It lists the stable public
surfaces added or changed by the local Go2 architecture work; helper functions
that are not consumed across module boundaries are intentionally omitted.

| Surface | Inputs | Outputs |
| --- | --- | --- |
| Go2 blueprint factories (`unitree_go2_agentic`, `unitree_go2_spatial`, `unitree_go2_temporal_memory`) | CLI blueprint name plus `GlobalConfig` values already resolved by DimOS. Lazy layer imports have no runtime arguments. | DimOS `Blueprint` objects composed with `autoconnect()`. They output module wiring, streams, and Spec injection paths, not user-facing JSON. |
| `RobotBodyStateSpec.get_robot_body_snapshot()` | No arguments; reads recent odom/image/lidar observations and connection/local-policy summaries from Layer 6. | `dict` with connection state, sensor state, local policy state, safety state, and freshness/availability markers. |
| `RobotBodyStateSpec.get_connection_state()` | No arguments. | `dict` describing connection availability/config-derived mode. |
| `RobotBodyStateSpec.get_sensor_state()` | No arguments. | `dict` with observed sensor counters/freshness such as image, lidar, and odom availability. |
| `RobotBodyStateSpec.get_local_policy_state()` | No arguments. | `dict` describing local policy/readiness state used as evidence by upper layers. |
| `WorldStateSpec.get_world_snapshot(task, spatial_limit)` | `task: str`, `spatial_limit: int`; reads Layer 6 body state plus spatial/temporal memory providers when wired. | `dict` world snapshot with `robot_state`, `runtime`, `memory_state`, `semantic_temporal_map`, `sources`, and snapshot-storage metadata. |
| `WorldStateSpec.get_robot_state()` | No arguments. | `dict` robot/body state view for upper-layer preflight. |
| `WorldStateSpec.get_runtime_state()` | No arguments. | `dict` runtime mode/config summary such as replay/simulation/hardware and MCP/viewer settings. |
| `WorldStateSpec.get_memory_state(task, spatial_limit)` | `task: str`, `spatial_limit: int`. | `dict` memory section with spatial and temporal availability, matches/summaries, and errors when a provider probe fails. |
| `WorldStateSpec.get_snapshot_storage_policy()` | No arguments. | `dict` explaining whether snapshots are transient, written to memory, or persisted elsewhere. |
| `SemanticTemporalMapSpec.query_semantic_temporal_map(query, spatial_limit)` | `query: str`, `spatial_limit: int`; reads spatial and temporal memory providers. | `dict` with spatial section, temporal section, fused evidence entries, confidence/location/time summaries, and source errors. |
| `SkillInterfaceSpec.get_skill_interface_snapshot(domain)` | Optional `domain: str` filter. | `dict` with `available`, `version`, `source`, `domain_filter`, `domains`, `skill_count`, and a `skills` list of static contracts. |
| `SkillInterfaceSpec.get_skill_contract(skill_name)` | `skill_name: str`. | Matching skill-contract `dict`, or `None` when no contract exists. |
| `SkillInterfaceSpec.validate_skill_request(skill_name, args_json)` | `skill_name: str`, `args_json: str` JSON object. | `dict` with `valid`, `errors`, `warnings`, and the matched `contract`; unknown skills return `valid=False`. |
| `SkillInterfaceSpec.compare_mcp_tools(tools_json)` | `tools_json: str` containing an MCP `tools/list` style payload or list. | `dict` with `valid`, parser errors, contract/MCP counts, missing contracts, unregistered MCP tools, and known internal tools. |
| `route_task(task, context)` MCP skill | `task: str`, optional `context: str`. | `SkillResult` metadata: `domain`, `confidence`, `matched_keywords`, `recommended_tools`, `needs_context`, `reason`, and `context_used`. |
| `get_context(task, focus, spatial_limit)` MCP skill | `task: str`, optional `focus: str`, `spatial_limit: int`; reads Layer 4, memory, skill, feedback, outcome, and world-model providers when wired. | `SkillResult` message plus metadata containing `sources`, `runtime`, `robot_state`, `world_state`, `skill_state`, `context_feedback`, `causal_state`, `world_model_state`, `context_evidence`, and provider `errors`. |
| `build_context_evidence(metadata, policy)` helper | Metadata dict from ContextProvider and `ContextEvidencePolicy` thresholds. | `context_evidence.v1` dict describing selected evidence, dropped/low-confidence evidence, selected sources, and policy effects. |
| `memory_backend_status()` MCP skill | No arguments; read probes optional providers. | `SkillResult` metadata with schema `go2_memory_backend_status.v1`, per-provider wired/available/probe fields, and warnings. |
| `record_evolution_event(event_type, task, payload_json, commit)` MCP skill | `event_type: str`, optional `task: str`, `payload_json: str` JSON object, `commit: bool`. | `SkillResult` metadata with `schema`, `event_type`, `task`, `ledger_dir`, `event_path`, `commit_requested`, optional `commit_sha`, `warnings`, and full `event`. |
| `record_skill_proposal(proposal_json, commit)` MCP skill / ledger RPC | `proposal_json: str` using `dimos.skill_proposal.v1`, `commit: bool`. | `SkillResult` metadata with `schema`, `proposal_id`, `ledger_dir`, `proposal_path`, `commit_requested`, optional `commit_sha`, `warnings`, and validated `proposal`. |
| `EvolutionLedgerSpec.write_evolution_event(event_type, task, payload, commit)` | Structured payload `dict` from another Layer 3 module. | Same ledger event record dict as `record_evolution_event`, without MCP JSON parsing. |
| `EvolutionLedgerSpec.write_skill_proposal(proposal, commit)` | Validated proposal `dict`. | Same proposal record dict as `record_skill_proposal`, without MCP JSON parsing. |
| `evaluate_task_feasibility(task, context_json)` MCP skill | `task: str`, `context_json: str` JSON object, usually from `get_context` metadata. Reads Layer 5 contracts when wired. | `SkillResult` metadata: `feasible` (`yes`, `no`, `uncertain`), `missing_context`, `required_skills`, `available_skills`, `missing_skills`, `safety_risks`, `recommended_next_action`, `clarifying_question`, `evidence_sources`, and warnings. |
| `record_context_feedback(task, context_evidence_json, selected_skill, outcome_json, helpful_sources_json, ignored_risks_json)` MCP skill | Task text, `context_evidence.v1` JSON, optional selected skill, outcome JSON object, helpful-source JSON list, ignored-risk JSON list. | `SkillResult` metadata with one `go2_context_feedback.v1` feedback record, `total_feedback`, and optional ledger warnings. |
| `ContextFeedbackSpec.get_recent_context_feedback(limit, source)` | `limit: int`, optional source filter. | Newest-first list of `go2_context_feedback.v1` feedback dicts. |
| `ContextFeedbackSpec.get_context_feedback_summary(limit)` | `limit: int`. | Aggregate `dict` with counts for success/failure/unknown outcomes plus helpful/harmful source counters. |
| `record_skill_outcome(skill_name, success, domain, error_code, message, risk, recovery)` MCP skill | Skill name, success boolean, optional domain/error/message/risk/recovery strings. | `SkillResult` metadata with recorded outcome dict and `total_outcomes`. |
| `summarize_skill_outcomes(limit, skill_name, domain)` MCP skill | Limit plus optional skill/domain filters. | `SkillResult` metadata with filtered newest-first `outcomes` list. |
| `SkillOutcomeStoreSpec.get_recent_outcomes(limit, skill_name, domain)` | Limit plus optional exact skill/domain filters. | Newest-first list of outcome dicts: timestamp, skill name, success, domain, error code, message, risk, recovery. |
| `predict_skill_outcome(skill_name, args_json, context)` MCP skill | Skill name, planned args JSON object, optional context text. Reads SkillOutcomeStore and CausalWorldModel when wired. | `SkillResult` metadata with `risk`, `predicted_success`, `failure_reasons`, `recovery_suggestions`, recent outcomes/transitions, optional world-model prediction, and provider availability flags. |
| `record_causal_transition(...)` MCP skill | Task, skill name, args JSON, before/after context text, prediction JSON, outcome JSON, domain, before/after state JSON. | `SkillResult` metadata with transition dict, `total_transitions`, `autosave_error`, and `dashboard_error`. |
| `predict_world_transition(snapshot_json, action_json, goal, horizon)` MCP skill / `predict_next_state(...)` RPC | Layer 4 snapshot JSON, action JSON with `skill_name` and `args`, optional goal, bounded horizon. | `dimos.world_model_prediction.v1` dict with action, snapshot summary, risk, predicted success, score, confidence, predicted symbolic delta, failure modes, reasons, model output, causal attribution, SCM explanation, and intervention evidence. |
| `score_action(snapshot_json, action_json, goal)` RPC | Same snapshot/action/goal inputs as prediction. | Compact dict with score, risk, confidence, predicted success, failure modes, reasons, model/causal/SCM/intervention evidence. |
| `summarize_causal_patterns(skill_name, domain, limit)` MCP skill | Optional skill/domain filters and limit. | `SkillResult` metadata with repeated causal `patterns` and filtered `transitions`. |
| `record_intervention(...)` MCP skill | Task, intervention name, target variable, before/after value JSON, action JSON, before/after snapshot JSON, outcome JSON, causal hypothesis. | `SkillResult` metadata with intervention record, `total_interventions`, `autosave_error`, and `dashboard_error`. |
| `save_world_model_state(path)` / `load_world_model_state(path)` MCP skills | Optional path; required unless `DIMOS_GO2_WORLD_MODEL_STATE` is set. | `SkillResult` summary with saved/loaded model-state counts plus optional dashboard error. |
| `CausalWorldModelSpec.get_recent_transitions(limit, skill_name, domain, cause)` | Limit plus optional exact filters. | Newest-first list of transition dicts. |
| `CausalWorldModelSpec.get_intervention_log(limit, target_variable, intervention_name)` | Limit plus optional exact filters. | Newest-first list of intervention dicts. |
| `CausalWorldModelSpec.get_model_state()` | No arguments. | Dict with online model sample/weights summary, provider contract, causal estimator snapshot, intervention log snapshot, SCM snapshot, and persistence config. |
| `CausalWorldModelSpec.get_provider_contract()` | No arguments. | `dimos.world_model_provider.v1` dict naming provider, model type, capabilities, and output schemas. |
| `propose_skill_interface(task, failure_context_json)` MCP skill | Task text and failure-context JSON object with missing-capability evidence. Reads Layer 5 contracts/outcomes when wired. | `SkillResult` metadata: `proposal_created`, `proposal` using `dimos.skill_proposal.v1` when created, `existing_skill_matches`, `recommended_next_action`, optional `proposal_path` or warnings. |
| MCP `tools/list` / `tools/call` runtime surface | JSON-RPC requests; `tools/call` also carries MCP arguments and optional `_mcp_context` progress/capability token. | Tool schemas with capability metadata; tool-call text/content results; SSE progress frames for background tools; capability release on instant return or stopped frame. |
| Optional `WebsocketVisSpec.set_world_model_state(state)` | `state: dict` expected to use `dimos.world_model_dashboard_state.v1`. This is only active when a blueprint wires `WebsocketVisModule`; current `unitree_go2_agentic` does not. | `dict` acknowledgement/current display state for optional Rerun/WebsocketVis clients. |

## Upstream Changes To Review

These are not local self-evolution features, but they can affect this branch.

1. Zenoh transport integration

   Files:

   - `dimos/core/transport_factory.py`
   - `dimos/core/transport.py`
   - `dimos/protocol/pubsub/impl/zenohpubsub.py`
   - `dimos/protocol/service/zenohservice.py`
   - `dimos/protocol/rpc/pubsubrpc.py`
   - `dimos/protocol/tf/tf.py`
   - `dimos/core/global_config.py`

   Implementation logic:

   - `GlobalConfig.transport` can be `lcm` or `zenoh`.
   - On Darwin, upstream now defaults to `zenoh`; elsewhere it defaults to
     `lcm`.
   - `make_transport()` maps logical names to either LCM topics or Zenoh key
     expressions. Zenoh topics are namespaced under `dimos/`.
   - RPC and TF backends are selected through `rpc_backend()` and `tf_backend()`.
   - High-rate sensor channels get best-effort/latest-wins QoS; human/agent
     channels get reliable/blocking QoS.

   Review risk:

   - macOS default behavior changed. If existing local runs assume LCM, set
     `DIMOS_TRANSPORT=lcm`.
   - Zenoh creates native pyo3 callback threads. Existing pytest thread-leak
     checks can flag these if cleanup is delayed.
   - Tests involving long-lived services must avoid collisions with an already
     running MCP server.

2. Capability gating for MCP tools

   Files:

   - `dimos/agents/capabilities.py`
   - `dimos/agents/mcp/mcp_server.py`
   - `dimos/agents/mcp/tool_stream.py`
   - `dimos/agents/annotation.py`
   - `dimos/agents/test_capabilities.py`

   Implementation logic:

   - `SkillInfo` now carries capability metadata such as `uses` and
     `lifecycle`.
   - `tools/list` includes `_meta.dimos/uses` and `_meta.dimos/lifecycle` when
     relevant.
   - `tools/call` acquires capability locks before invoking a skill.
   - Instant holders may be waited on; background holders require a stop tool.
   - Background tool capability release is tied to a tool-stream stopped frame.

   Review risk:

   - Skill annotations must accurately declare long-running motion/control
     capabilities, otherwise the server cannot serialize conflicting behaviors.
   - Tool-stream stop frames must always be emitted for background tools that
     hold capabilities.

3. Coordinator RPC and dynamic blueprint loading

   Files:

   - `dimos/core/coordination/coordinator_rpc.py`
   - `dimos/core/coordination/module_coordinator.py`
   - `dimos/core/coordination/worker_manager_python.py`

   Implementation logic:

   - `ModuleCoordinator` can expose a singleton coordinator RPC service.
   - Clients can list modules and load blueprints through that coordinator RPC.
   - Existing deployed module proxies are protected by a lock during dynamic
     loading and restart operations.

   Local interaction:

   - Our local runtime plumbing preserved non-blocking system-module
     notification for `McpClient`, so agent startup is not blocked by direct
     tool registration.
   - Validation passed under `DIMOS_TRANSPORT=lcm`.

4. Memory and perception migration

   Files:

   - `dimos/perception/image_embedding.py`
   - `dimos/perception/spatial_vector_db.py`
   - `dimos/perception/visual_memory.py`
   - `dimos/perception/spatial_perception.py`
   - `dimos/memory2/*`

   Implementation logic:

   - Upstream moved visual/spatial memory helpers out of
     `dimos.agents_deprecated.memory` into `dimos.perception`.
   - Old deprecated agent memory modules were removed.
   - A new memory2 TF service/replay/tooling area was added upstream.

   Local interaction:

   - We use upstream's new module paths in `spatial_perception.py`.
   - We keep local default persistence under `STATE_DIR / "spatial_memory"`,
     with `DIMOS_SPATIAL_MEMORY_DIR` override, instead of upstream's project
     assets output path.

5. Control, manipulation, learning, docs, and LFS updates

   Summary:

   - Control tasks were split into package directories with registries.
   - Roboplan/learning/data-prep modules were added.
   - Navigation docs were reorganized for Mintlify.
   - New LFS data packages and native build scripts were added.

   Review risk:

   - These are broad upstream changes and not directly part of the Go2
     self-evolution logic. They should be reviewed mainly for dependency,
     packaging, and blueprint registry side effects.

## Local Feature Inventory

### 1. Go2 Layered Blueprint Architecture

Files:

- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py`
- `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_spatial.py`
- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_temporal_memory.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/__init__.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/__init__.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/__init__.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/__init__.py`

Implementation logic:

- `unitree_go2_agentic` is now composed from four explicit layers:
  `_go2_robot_body`, `_go2_spatial_world_state`, `_go2_skill_interface`, and
  `_go2_agent_brain`.
- Layer packages use lazy `__getattr__` construction so importing one layer does
  not eagerly import heavy perception, VLM, or robot stacks.
- `unitree_go2_spatial` is the non-agentic spatial stack built from Layer 6,
  Layer 4 spatial world state, and Layer 5 spatial skills.
- `unitree_go2_temporal_memory` adds temporal memory through Layer 4's lazy
  `_go2_temporal_memory_world_state()` helper after CLI config has been applied.

Why this matters:

- Reviewers can inspect robot body, world state, skill interface, and agent
  reasoning independently.
- It prevents the agent brain from becoming an unstructured import hub.
- It preserves DimOS blueprint semantics: modules are still composed with
  `autoconnect()` and RPC refs are injected by specs.

Primary risk:

- The layering must not hide missing blueprint modules. The static registry test
  and Layer 4 wiring test are the main protections.

Components:

- `unitree_go2_agentic`: the full agentic Go2 stack. It composes the physical
  body, structured world state, skill interface registry/containers, and Layer
  3 MCP/LLM brain.
- `unitree_go2_spatial`: the perception/world-state stack without the LLM brain.
  It is useful for validating Layer 4 and Layer 5 without agent behavior.
- `unitree_go2_temporal_memory`: an additive blueprint that wraps the agentic
  stack and adds temporal memory through the Layer 4 factory.
- Layer package `__init__.py` files: lazy blueprint factories. They are part of
  the architecture, not just import convenience, because they prevent import
  side effects from pulling in heavy perception/model dependencies too early.

Construction flow:

1. CLI imports a named blueprint lazily through the registry.
2. The blueprint imports only the layer handles it needs.
3. Each layer handle builds an `autoconnect()` blueprint for its modules.
4. The final blueprint is still ordinary DimOS composition, so stream wiring and
   Spec-based RPC injection happen in `ModuleCoordinator`.
5. Tests validate that Layer 3 can reach Layer 4/5/6 through the intended
   contracts and that the generated blueprint registry remains current.

State and persistence:

- The blueprint layer itself stores no robot data.
- State is owned by modules inside the layers: body state in Layer 6, memory
  and world snapshots in Layer 4, skill contracts in Layer 5, context/outcome
  and causal state in Layer 3.
- Any persistence belongs to those modules, not the blueprint file.

Safety boundary:

- The blueprint decides what modules are deployed together.
- It does not decide which skill to call, and it does not execute motion.
- Reviewers should treat wiring mistakes as safety risks because the LLM may
  see missing or wrong tools/context.

### 2. Layer 6 Robot Body State

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/robot_body_state.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/robot_body_spec.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_6_robot_body/test_robot_body_state.py`

Implementation logic:

- `_Go2RobotBodyState` is a small module that exposes robot-body status through
  a spec-facing API.
- It sits next to the real Go2 connection stack rather than replacing it.
- Higher layers consume robot state through a typed spec rather than directly
  reaching into the hardware connection module.

Review focus:

- Make sure it reports enough state for safety-sensitive agent preflight.
- Make sure it does not duplicate live control authority.

Components:

- `_Go2RobotBodyState`: the module that adapts low-level robot status into a
  compact Layer 6 provider.
- `robot_body_spec.py`: the typed RPC contract consumed by upper layers.
- Layer 6 package factory: wires the underlying Go2 robot blueprint with the
  body-state module.

Data flow:

1. The underlying Go2 connection and robot modules own live hardware/simulation
   communication.
2. `_Go2RobotBodyState` exposes read-oriented body-state summaries through its
   spec.
3. Layer 4/Layer 3 use the spec for safety preflight and context evidence.
4. The agent brain sees summarized state, not direct hardware handles.

Decision owner:

- The body-state module does not make task decisions.
- It is an evidence provider. Layer 3 deterministic preflight and the LLM use
  its output to decide whether to wait, clarify, or call a skill.

State and write boundary:

- It should be treated as a current-state adapter.
- It should not persist memory and should not issue control commands.
- Any future body-state caching should be explicitly bounded and timestamped.

Failure mode:

- If the underlying robot state is missing, the correct behavior is explicit
  unavailable/unknown state, not fabricated readiness.
- Motion-sensitive preflight should treat missing robot body state as risk.

### 3. Layer 4 Structured World State

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/structured_world_state.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/semantic_temporal_map.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/world_state_spec.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py`

Implementation logic:

- `_Go2StructuredWorldState` exposes a compact robot/world snapshot through a
  typed spec.
- `_Go2SemanticTemporalMap` combines spatial and temporal memory summaries into
  a more agent-friendly world-state surface.
- Layer 3 consumes this via RPC spec injection instead of importing memory
  implementations directly.

What "effective context" means here:

- This layer does not ask the LLM to judge memory relevance directly.
- It normalizes available world and memory sources into a structured provider
  surface; Layer 3 then scores, filters, and packages evidence for the LLM.

Review focus:

- Check that missing memory backends degrade to explicit unavailable statuses.
- Check that snapshot schemas are stable enough for prompt and dashboard
  consumers.

Components:

- `_Go2StructuredWorldState`: normalizes robot/world information into a compact
  snapshot provider.
- `_Go2SemanticTemporalMap`: fuses spatial memory and temporal memory summaries
  into semantic evidence for agent context.
- `world_state_spec.py`: the RPC contract Layer 3 relies on.
- `_go2_temporal_memory_world_state()`: lazy factory that delays TemporalMemory
  construction until CLI flags such as `--new-memory` are applied.

Data flow:

1. SpatialMemory and TemporalMemory remain the storage/search systems.
2. Layer 4 queries or receives summaries from those systems.
3. Layer 4 shapes them into a stable world-state object with explicit source
   availability.
4. Layer 3 asks for world state through the spec and then selects evidence for
   the current task.

State and persistence:

- Layer 4 may read from persistent memory backends, but its own structured
  snapshot is a derived view.
- Temporal memory persistence is controlled by the TemporalMemory module and CLI
  config.
- Spatial memory persistence is controlled by `SpatialConfig` and the state-dir
  path logic in `spatial_perception.py`.

Decision owner:

- Layer 4 decides how to normalize provider output.
- Layer 4 does not decide whether the context is sufficient for action. That
  decision belongs to Layer 3 preflight policy and the LLM.

Failure mode:

- Provider unavailable means the snapshot should include unavailable metadata.
- Empty search results are not the same as backend failure; reviewers should
  check that these are represented differently.

### 4. Layer 5 Skill Interface Registry

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_registry.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/skill_interface_spec.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_5_skill_interface/test_skill_interface_registry.py`

Implementation logic:

- Skill contracts are explicit static records: skill name, domain, module,
  required args, optional args, motion sensitivity, context requirements,
  preflight recommendations, risk class, and outcome shape.
- The registry exposes contracts to Layer 3 as data.
- Known non-Layer-5 MCP tools are excluded from contract mismatch checks so
  internal status/preflight tools do not pollute skill coverage.

Self-evolution relationship:

- The robot does not mutate skills automatically.
- Missing capability evidence can create a reviewable proposed skill interface.
- Existing contracts are the ground truth for duplicate suppression.

Review focus:

- Verify every exposed action skill has an accurate contract.
- Treat risk class and required context as safety-critical metadata.

Components:

- Static `_SkillContract` records: the human-authored interface description for
  each expected action skill.
- `_Go2SkillInterfaceRegistry`: exposes contracts through RPC/MCP-facing
  methods for Layer 3.
- `skill_interface_spec.py`: typed contract for modules that consume skill
  metadata.
- Layer 5 blueprint factory: wires action skill containers, perception/security
  skills, and the registry.

Data flow:

1. Action modules expose `@skill` methods through MCP.
2. The registry provides a parallel contract view with richer safety/context
   metadata than the MCP JSON schema alone.
3. ContextProvider and TaskFeasibility read contracts to decide what context,
   arguments, and preflight checks are needed.
4. SkillProposal reads the same contracts to avoid proposing duplicate skills.

State and write boundary:

- Contracts are code/static data, not learned state.
- The registry is read-only at runtime.
- Self-evolution proposals never mutate this file directly. A developer edits
  contracts after review.

Decision owner:

- Humans own the source-of-truth contract text and risk metadata.
- Deterministic Layer 3 code consumes it.
- The LLM may use contract information, but should not invent hidden skill
  capabilities outside the registry/MCP list.

Failure mode:

- Missing contract for an exposed skill is a review failure because preflight
  cannot know risk/context needs.
- Contract exists but MCP tool is absent is also a review failure unless it is
  intentionally disabled for a blueprint.

### 5. Layer 3 Expert Router

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/expert_router.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/prompt_policy.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_expert_router.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_prompt_policy.py`

Implementation logic:

- The router classifies task domains and recommends which expert/tool family
  should be used.
- Prompt policy injects Layer 3 self-evolution guidance into the MCP client
  system prompt without replacing robot-specific prompts entirely.
- The output is deterministic routing metadata, not an LLM call.

Judgment subject:

- Deterministic code performs the first-pass routing and evidence packaging.
- The LLM remains the actor that reads the final context and decides how to
  execute or ask clarification, unless a deterministic MCP skill is explicitly
  called.

Review focus:

- Verify routing labels line up with actual skill contracts.
- Watch for prompt text that could overclaim autonomy or code mutation.

Components:

- `_Go2ExpertRouter`: deterministic task/domain router.
- Prompt policy helpers: merge layer-specific instructions into the agent system
  prompt.
- Router tests: pin expected labels and guard against accidental broadening.

Data flow:

1. A user task or subtask is passed to the router.
2. The router classifies the task into a small set of domains such as
   navigation, perception, person follow, manipulation-like unsupported work,
   or general conversation.
3. It returns routing metadata and recommended next tools/preflight.
4. ContextProvider can include routing output in context.
5. The LLM uses routing as a hint, not as an irreversible planner.

Decision owner:

- Domain classification is deterministic and inspectable.
- Tool choice remains an LLM/tool-calling decision unless a deterministic
  preflight skill refuses or asks for clarification.

State and persistence:

- The router is stateless.
- It does not write ledger events or feedback by itself.

Failure mode:

- Unknown tasks should route to uncertainty/general handling, not fabricate a
  capability.
- Safety-sensitive domains should bias toward preflight and clarification.

### 6. Context Provider And Evidence Selection

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_evidence.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_provider.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_evidence.py`

Implementation logic:

- `get_context()` gathers available data from task text, runtime metadata,
  robot state, RAG/spatial memory, temporal memory, skill contracts, skill
  outcomes, causal world model, and context feedback.
- `context_evidence.py` contains the pure policy helper
  `build_context_evidence(metadata, ContextEvidencePolicy)`.
- The policy supports thresholds such as `min_relevance_score`, `max_entries`,
  `include_low_confidence`, and `require_robot_state_for_motion`.
- The returned context contains a compact answer plus `context_evidence.v1`,
  which explains which evidence was selected and why.

What "effective" means today:

- Effective means selected by deterministic policy from available providers:
  relevance score, confidence, source availability, task type, and safety needs.
- It is not yet learned from replay metrics.
- The LLM consumes the selected context and may still judge it insufficient.

Components:

- `_Go2ContextProvider`: MCP-facing context bundle provider. It is the
  aggregator that reads optional Layer 4/5/3 providers and formats the compact
  context returned to the agent.
- `context_evidence.py`: pure evidence selector. It owns the deterministic
  ranking/filtering policy and can be unit-tested without a running robot,
  MCP server, memory backend, or LLM.
- Optional provider refs: world state, spatial/RAG memory, temporal memory,
  skill interface registry, skill outcome store, causal world model, and
  context feedback store.
- `context_evidence.v1`: review artifact embedded in the response. It records
  which evidence entries were selected, dropped, or marked low-confidence.

Inputs:

- Task text and runtime metadata supplied by the caller.
- Provider snapshots/summaries from wired DimOS modules.
- Policy values such as relevance threshold, entry cap, low-confidence
  handling, and whether motion tasks require robot state.
- Prior feedback/outcome/world-model summaries when those providers are wired.

Data flow:

1. The caller invokes `get_context()` with task/runtime metadata.
2. ContextProvider asks wired providers for world state, memory summaries,
   skill contracts, outcome summaries, causal-world-model state, and context
   feedback. If a provider is not wired, the provider contributes an explicit
   unavailable status rather than throwing away the source silently.
3. Metadata is passed to `build_context_evidence()`. That helper is pure:
   given metadata and a `ContextEvidencePolicy`, it returns selected evidence
   without calling RPC, writing memory, or consulting an LLM.
4. ContextProvider formats the compact context and attaches
   `context_evidence.v1` so reviewers can see why the context contained those
   entries.
5. The LLM is still the consumer and final user-facing reasoning actor. The
   provider does not decide to execute a skill.

State and write boundary:

- ContextProvider itself is read-mostly. It does not write RAG/vector memory,
  temporal memory, skill contracts, or robot state.
- The only durable side channel is indirect: it may include summaries from
  feedback/outcome/ledger-aware modules, but those modules own their writes.
- The selected context is a transient prompt artifact unless another caller
  explicitly records feedback or an evolution event.

Decision owner:

- Deterministic code decides what evidence enters the context bundle.
- The LLM decides how to reason over that bundle, whether to ask a clarifying
  question, and whether to call an exposed tool.
- Human reviewers decide whether the deterministic policy is acceptable for a
  given robot safety posture.

Failure mode:

- Missing providers should appear as explicit unavailable metadata.
- Malformed provider payloads should degrade that source, not erase the whole
  context bundle.
- Low relevance or low confidence evidence should be dropped or marked
  according to policy, never silently upgraded to trusted evidence.

What is not implemented:

- No learned ranker is trained from replay logs yet.
- No RAG/vector database entries are written by ContextProvider.
- Context feedback is summarized as a signal, but it does not override provider
  data by itself.

Review focus:

- Check whether relevance/confidence defaults are too permissive.
- Check whether motion tasks always require robot state.
- Check whether low-confidence evidence should be visible but marked, or hidden.

### 7. Memory Backend Status

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/memory_backend_status.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_memory_backend_status.py`

Implementation logic:

- Adds MCP skill `memory_backend_status()`.
- Reports whether providers are wired for world state, spatial memory, temporal
  memory, skill outcomes, causal world model, and skill interface.
- Performs small read probes where possible.
- Returns status metadata only; it does not create another memory database.

Review focus:

- This is diagnostic, not a memory scheduler.
- It should never write to RAG, spatial memory, temporal memory, or Git.

Components:

- `_Go2MemoryBackendStatus`: diagnostic module exposed as an MCP skill.
- Optional provider specs: world state, spatial memory, temporal memory,
  outcome store, causal world model, and skill interface registry.
- Probe helpers: small read attempts that report availability and errors.

Data flow:

1. The skill checks which optional provider references are injected.
2. For each provider, it records `wired`, `available`, and any probe result.
3. Probe failures are captured as status metadata rather than raised to the
   agent as hard crashes.
4. The result is returned as JSON/text for the LLM or human operator.

State and write boundary:

- It is read-only.
- It does not persist a status snapshot.
- It does not initialize missing databases. If a database is not running or not
  wired, the status should say so.

Decision owner:

- The skill does not decide which memory to use.
- It answers the operational question "what is connected and readable right
  now?" ContextProvider and the LLM decide how to respond to that.

Failure mode:

- A probe failure should identify the provider and error string.
- A missing provider should be explicit enough to distinguish blueprint wiring
  problems from empty memory results.

### 8. Git-Backed Evolution Ledger

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_event.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_ledger.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/evolution_proposal.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_evolution_ledger.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_evolution_proposal.py`

Implementation logic:

- Events use schema `dimos.evolution_event.v1`.
- Proposals use schema records validated by `evolution_proposal.py`.
- Default storage is local repository path:
  `.dimos/evolution/events/YYYY/MM/DD/*.json` and
  `.dimos/evolution/proposals/*.json`.
- `DIMOS_EVOLUTION_LEDGER_DIR` can override that location.
- Writes are JSON files with deterministic schema and reviewable payloads.
- `commit=False` by default.
- If `commit=True`, the module creates a local Git commit containing only the
  event/proposal file.

Components:

- `evolution_event.py`: schema and validation for low-level self-evolution
  observations such as feasibility assessments, context feedback, and runtime
  review notes.
- `evolution_proposal.py`: schema and validation for higher-level proposed
  changes that need human review before becoming code or configuration.
- `_EvolutionLedger`: filesystem and optional Git writer. It owns path
  selection, JSON serialization, and commit-mode staging/commit behavior.
- Tests: verify schema validation, path shape, commit isolation, and proposal
  persistence.

Inputs:

- Event/proposal payload supplied by a Layer 3 MCP tool.
- Optional caller-controlled commit flag.
- Optional `DIMOS_EVOLUTION_LEDGER_DIR` override.
- Current Git repository context when commit mode is enabled.

Where does Git data go:

- It goes into this local repository's working tree and local `.git` history.
- It does not push to GitHub.
- GitHub only receives it if a human later runs `git push` or opens a PR.

Write data flow:

1. A caller invokes a ledger-facing MCP skill such as
   `record_evolution_event()` or `record_skill_proposal()`.
2. The module validates the schema and normalizes the payload.
3. The module chooses the storage root. The default is the repo-local
   `.dimos/evolution` tree; `DIMOS_EVOLUTION_LEDGER_DIR` overrides it.
4. The JSON file is written as the review artifact.
5. If `commit=False`, Git is not touched beyond the working tree file.
6. If `commit=True`, only that new event/proposal file is staged and committed
   locally. There is no push.

State and write boundary:

- The ledger is durable filesystem state, not an in-memory learning model.
- It writes only JSON artifacts under the configured ledger root.
- It does not mutate skill code, prompts, contracts, memory databases, or model
  weights.
- Commit mode is intentionally local-only and narrow: stage the artifact, make
  a local commit, stop.

Decision owner:

- Caller modules decide when an event or proposal is worth recording.
- The ledger decides only whether the artifact is schema-valid and where it is
  stored.
- Human reviewers decide whether ledger artifacts should become code changes,
  PRs, ignored local traces, or training/evaluation data later.

Failure mode:

- Invalid schema should fail before writing.
- Unsafe paths or path traversal should be rejected before filesystem access.
- Git commit failure should leave the JSON artifact visible in the worktree
  rather than pretending the record was committed.

Privacy and review boundary:

- These files can contain task text, outcome messages, context summaries, and
  proposal rationale. Decide whether `.dimos/evolution` belongs in normal Git
  history before enabling commit mode on real robot runs.
- The ledger is a control-plane audit trail. It is not a replacement for
  SpatialMemory, TemporalMemory, SkillOutcomeStore, or the causal model state.

Review focus:

- Confirm no path traversal is possible through event/proposal fields.
- Confirm commit mode stages only the new ledger file.
- Decide whether `.dimos/evolution` should be ignored, committed, or exported
  elsewhere depending on privacy policy.

### 9. Task Feasibility Preflight

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/task_feasibility.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_task_feasibility.py`

Implementation logic:

- Adds MCP skill `evaluate_task_feasibility(task, context_json)`.
- Reads skill contracts, world/robot context, and optional JSON context.
- Returns a deterministic result with feasibility status, missing context,
  required skills, available skills, safety risks, recommended next action,
  optional clarifying question, and evidence sources.
- May write a ledger event when an evolution ledger is wired.

Self-evolution meaning:

- This is not model self-training.
- It is agent self-assessment: before acting, the agent can ask a deterministic
  evaluator whether the task has enough context and capability support.

Components:

- `_Go2TaskFeasibility`: MCP skill surface for deterministic preflight.
- Skill contract reader: consumes Layer 5 skill metadata such as required
  arguments, context requirements, risk class, and motion sensitivity.
- Context parser: merges explicit `context_json` with wired world/robot
  provider summaries.
- Ledger integration: optional event writer for review traces.

Inputs:

- `task`: natural-language user request or subtask.
- `context_json`: optional structured context supplied by the caller.
- Layer 5 skill contracts and availability metadata.
- Layer 4 robot/world state when wired.
- Optional outcome/context feedback summaries when available through the
  context bundle.

Decision/data flow:

1. Parse the task and optional `context_json`.
2. Match the task against known skill contracts and expert domains.
3. Check required arguments and context requirements from Layer 5 contracts.
4. Check robot/world-state availability for motion-sensitive or risky skills.
5. Produce one of three broad outcomes:
   `feasible`, `uncertain`, or not feasible.
6. If data is missing, recommend a clarifying question or context-gathering
   action.
7. If capability is missing, report it as missing capability evidence. That is
   the input that can later justify a skill-interface proposal.

State and write boundary:

- It is read-only with respect to robot state, memory, and skill contracts.
- It does not call the target skill.
- It does not create a new skill.
- If the ledger is wired, it may write an audit event describing the preflight
  result, but not a code change.

Decision owner:

- Feasibility classification is deterministic code.
- The LLM may call this tool and use its result, but the LLM is not the
  classifier inside the tool.
- Human review decides whether the deterministic rules are conservative enough
  for real hardware.

Failure mode:

- Missing required arguments should produce clarification, not a new skill
  proposal.
- Missing context should produce context-gathering or uncertainty, not a false
  feasible result.
- Missing capability evidence should be specific enough for SkillProposal to
  distinguish it from transient runtime failure.

Review focus:

- Check false positives for motion-sensitive tasks.
- Check that missing context leads to clarification instead of skill proposal.

### 10. Context Feedback Store

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_feedback.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_context_feedback.py`

Implementation logic:

- Adds MCP skill `record_context_feedback(...)`.
- Keeps a bounded in-memory deque of the most recent 100 feedback entries.
- Uses schema `go2_context_feedback.v1`.
- Optionally writes feedback events through the evolution ledger.
- ContextProvider includes a compact feedback summary in future context.

Review focus:

- Feedback is session memory unless ledger is wired.
- It should not become unbounded or silently override RAG/temporal memory.

Components:

- `_Go2ContextFeedbackStore`: in-memory bounded store and MCP skill surface.
- Feedback entry schema: task/context identifiers, rating or usefulness signal,
  source labels, optional explanation, and timestamp.
- Optional evolution ledger connection: durable audit trail when wired.
- ContextProvider integration: compact aggregate feedback summary.

Data flow:

1. The agent or a human-facing process calls `record_context_feedback(...)`
   after a context bundle is helpful, incomplete, stale, or misleading.
2. The store validates and normalizes feedback fields.
3. The entry is appended to a deque capped at 100 records.
4. If the ledger is wired, the feedback is also written as an evolution event.
5. ContextProvider reads a summary, not the full raw feedback list, to avoid
   bloating prompts.

State and persistence:

- Without the ledger, feedback is process-local and lost on restart.
- With the ledger, feedback becomes reviewable JSON but still does not train a
  ranker automatically.

Decision owner:

- Feedback recording is an input signal.
- The evidence policy and LLM decide how to treat the signal. There is no
  automatic prompt rewrite or memory deletion.

Failure mode:

- Malformed feedback should fail validation.
- Excess feedback should evict oldest entries, not grow unbounded.

### 11. Skill Outcome Store And Predictor

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_outcome_store.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_outcome_predictor.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_skill_outcomes.py`

Implementation logic:

- Records recent skill outcomes with bounded storage.
- Summarizes success/failure patterns by skill.
- Predictor estimates risk or likely outcome based on recent history and skill
  contract metadata.
- ContextProvider can use these summaries as task evidence.

Review focus:

- The predictor is heuristic, not a learned model.
- Make sure stale failures do not permanently block useful skills.

Components:

- `_Go2SkillOutcomeStore`: bounded recent outcome history.
- `_Go2SkillOutcomePredictor`: heuristic scorer over recent history and skill
  contract metadata.
- Outcome summary methods: aggregate successes, failures, repeated errors, and
  recent messages by skill.

Data flow:

1. After a skill call, an outcome can be recorded with skill name, success,
   error code, message, and metadata.
2. The outcome store appends it to bounded memory.
3. Summaries group recent records by skill and error patterns.
4. The predictor combines recent outcomes with skill contract risk/context
   metadata to return risk and rationale.
5. ContextProvider can include the summary as evidence for future calls.

State and persistence:

- Current store is bounded runtime memory unless a future integration writes it
  elsewhere.
- It is separate from the Git ledger. The ledger records review/audit events;
  the outcome store supports immediate runtime adaptation.

Decision owner:

- The predictor gives a heuristic risk hint.
- It should not block a skill by itself; TaskFeasibility or the LLM should use
  it as one evidence source.

Failure mode:

- Unknown skill should return low-confidence or no-history output.
- Repeated failure should increase caution but still allow recovery/stop skills.

### 12. Causal World Model And Dashboard Contract

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/causal_world_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/causal_effect_estimator.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/online_transition_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/structural_causal_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/intervention_log.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/world_model_contract.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_causal_world_model.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_world_model_contract.py`

Implementation logic:

- Records causal transitions and interventions.
- Maintains an online transition model for recent world-state changes.
- Estimates causal effects from recorded events.
- Produces versioned contract outputs:
  `dimos.world_model_prediction.v1`,
  `dimos.world_model_provider.v1`, and
  `dimos.world_model_dashboard.v1`.
- Supports save/load of model state.

Is it a trained model?

- Strict answer: there is a very small online model, but there is no offline
  training pipeline and no neural world model.
- `OnlineTransitionOutcomeModel` is a lightweight logistic-style linear model.
  It stores feature weights in a dictionary and updates them incrementally when
  `record_causal_transition()` receives an observed success/failure outcome.
- If no transitions have been recorded, its `sample_count` is zero and its
  prediction should be treated as low-confidence prior behavior, not learned
  robot knowledge.
- The symbolic SCM is hand-authored. It is not learned from data.
- The causal effect estimator is observational statistics. It compares smoothed
  success rates for active features and does not control hidden confounders.

Components:

- `OnlineTransitionOutcomeModel`: online success-probability scorer. Features
  include bias, skill name, domain, odometry presence, navigation state, spatial
  memory availability, spatial match count, temporal availability, and selected
  action argument presence/value. The update is one-step gradient adjustment
  with L2 shrinkage.
- `CausalEffectEstimator`: bounded observation store with treated/control
  success-rate differences for symbolic features. It can produce risk factors,
  supporting factors, and intervention suggestions, but its own output declares
  the assumption that unobserved confounders are not controlled.
- `StructuralCausalModel`: hand-authored causal graph for variables such as
  `spatial_has_matches`, `target_resolvability`, `odom_ready`,
  `motion_safety`, and `navigation_success`. It also emits counterfactual
  suggestions such as "tag or perceive target before semantic navigation".
- `InterventionLog`: records explicit interventions and lets prediction surface
  prior intervention evidence.
- `_WorldTransition`: the compact event record tying task, skill, before/after
  state, predicted risk, outcome, inferred cause, and recovery suggestion
  together.

Data flow for recording:

1. `record_causal_transition()` receives task, skill name, optional args,
   before/after Layer 4 snapshots, prediction metadata, and outcome metadata.
2. Inputs are parsed as JSON objects and validated. Missing outcome can fall
   back to the latest same-skill outcome if SkillOutcomeStore is wired.
3. The module infers a coarse cause and recovery suggestion from context,
   prediction reasons, outcome success, error code, and message.
4. It computes a symbolic state delta from before/after snapshots.
5. It calls `_outcome_model.update(...)` and `_causal_estimator.update(...)`.
6. It appends the transition to an in-memory deque capped at 200 transitions.
7. It autosaves to `DIMOS_GO2_WORLD_MODEL_STATE` only if that env path is set.
8. It publishes dashboard state if WebsocketVis is wired.

Data flow for prediction:

1. `predict_next_state()` parses a snapshot and candidate action.
2. It gets recent same-skill transitions and derives repeated failure modes.
3. It computes rule-based risk reasons from the current snapshot and action.
4. It asks the online model for success probability, score, risk, confidence,
   and top weighted feature contributions.
5. It asks the causal estimator for observational risk/support factors.
6. It asks the hand-authored SCM for variables, edges, and counterfactuals.
7. It asks the intervention log for matching intervention evidence.
8. It combines rule risk and model risk into a final risk and score.
9. It returns `dimos.world_model_prediction.v1` and optionally publishes the
   dashboard contract.

Trust boundary:

- The output is advisory. It should help the LLM choose preflight, perception,
  clarification, or a safer skill path.
- It should not directly command robot motion.
- High confidence currently requires enough recent samples for the same skill;
  otherwise the model should be treated as a low-confidence heuristic.

Review focus:

- Treat current causal estimates as advisory evidence.
- Validate that prediction schemas remain stable for dashboard and LLM prompt
  consumers.
- Check that persistence does not mix data across runs unexpectedly.
- Check that the UI/prompt labels do not imply a fully trained physical world
  model.
- Check that replay or simulation evaluation is added before relying on this
  for autonomous action selection.

### 13. Skill Interface Proposal Generator

Files:

- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/skill_proposal.py`
- `dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/test_skill_proposal.py`

Implementation logic:

- Adds MCP skill `propose_skill_interface(task, failure_context_json)`.
- Generates `dimos.skill_proposal.v1` review artifacts only when there is
  explicit missing capability evidence.
- Suppresses proposals for missing arguments or missing context.
- Suppresses duplicate proposals when an existing contract already covers the
  capability.
- Writes proposal files through the evolution ledger when wired.
- Does not modify Python code, skill decorators, or blueprint wiring.

Decision tree:

1. Parse `failure_context_json`.
2. Look for explicit missing capability evidence, not just failed execution.
3. Check existing Layer 5 contracts to avoid proposing a duplicate interface.
4. Reject cases that are only missing arguments, missing context, or temporary
   provider unavailability.
5. Build a proposal artifact with task, evidence, suggested interface shape,
   expected inputs, expected output, and review rationale.
6. Write it through the evolution ledger if a ledger is wired.

Human boundary:

- This tool is intentionally proposal-only. A developer still writes code,
  chooses the container, adds `@skill`, updates contracts/prompts, and tests the
  behavior.

Review focus:

- This is the safest self-evolution boundary: proposal-only, human-reviewed.
- Check evidence requirements carefully so ordinary user ambiguity does not
  generate noisy skill proposals.

Components:

- `_Go2SkillProposalGenerator`: MCP skill surface for proposal generation.
- Existing Layer 5 contracts: duplicate-suppression source of truth.
- Evolution ledger: optional proposal writer.
- Proposal schema: review artifact describing the missing capability and
  proposed interface, not executable code.

Inputs:

- `task`: the user/subtask text that failed.
- `failure_context_json`: structured evidence about why existing skills failed
  or were insufficient.
- Existing contracts and known MCP tool names.

Accepted evidence:

- Explicit missing capability.
- Repeated failure that indicates no available skill can perform the required
  operation.
- A domain/action gap not covered by existing contracts.

Rejected evidence:

- Missing required arguments for an existing skill.
- Missing context or unavailable memory/provider.
- Temporary runtime failure.
- Capability already covered by an existing contract.

Output boundary:

- The output is a JSON proposal and optional ledger file.
- It does not import, write, or generate Python code.
- It does not update the prompt or registry automatically.

### 14. MCP Runtime Plumbing

Files:

- `dimos/agents/mcp/mcp_client.py`
- `dimos/agents/mcp/mcp_server.py`
- `dimos/agents/mcp/tool_stream.py`
- `dimos/agents/mcp/test_mcp_client_unit.py`
- `dimos/agents/mcp/test_mcp_server.py`
- `dimos/agents/mcp/test_tool_stream.py`

Implementation logic:

- `McpServer.on_system_modules()` avoids querying its own proxy over RPC. For
  its own skills and actor-class backed proxies, it collects skill schemas
  locally.
- `McpClient` defers direct tool registration outside the RPC path so startup
  does not block on the server being fully initialized.
- Tool-stream progress frames can be delivered over persistent SSE and include
  progress-token metadata.
- Capability release for background tools is tied to stopped frames.
- `agent_send()` now uses `make_transport()` so it respects the active backend.

Review focus:

- Validate no startup path waits for a tool that depends on itself.
- Keep MCP test ports isolated from live robot sessions.

Components:

- `McpServer`: exposes MCP JSON-RPC endpoints, tool list/call handling, SSE
  progress stream, server status tools, capability gating, and agent-send.
- `McpClient`: LLM agent module that discovers tools and exposes them to the
  agent runtime.
- `tool_stream.py`: progress/log notification path for long-running skills.
- Capability registry: shared server-side lock manager for mutually exclusive
  tool capabilities.

Startup data flow:

1. ModuleCoordinator deploys workers and starts modules.
2. `McpServer.start()` starts HTTP/SSE serving and subscribes to tool-stream
   frames.
3. `on_system_modules()` registers tool schemas. For the server itself and
   actor-class backed modules, schemas are collected locally to avoid RPC
   self-deadlock.
4. `McpClient` initializes the agent without blocking startup on direct tool
   registration outside the RPC path.

Tool call flow:

1. JSON-RPC `tools/call` arrives.
2. Server resolves `SkillInfo` and RPC call target.
3. Capability locks are acquired if the skill declares `uses`.
4. Progress token and capability token are injected through reserved
   `_mcp_context`.
5. RPC call runs in an executor.
6. Instant skills release capability after return; background skills hand
   release to tool-stream stopped frames.

State and failure boundary:

- Server state is process-local: skills, rpc calls, SSE queues, capability
  registry.
- Port conflicts are external environment failures. Tests should use isolated
  MCP ports when a live server is running.
- Tool not found should return a structured MCP text result, not crash the
  server.

### 15. Runtime Hardening

Files:

- `dimos/core/coordination/module_coordinator.py`
- `dimos/core/coordination/worker_manager_python.py`
- `dimos/simulation/mujoco/mujoco_process.py`
- `dimos/robot/unitree/mujoco_connection.py`
- `dimos/robot/unitree/go2/connection.py`
- `dimos/robot/unitree/go2/test_connection.py`

Implementation logic:

- Coordinator system-module notifications avoid waiting on known agent clients.
- Worker manager/coordinator changes preserve module restart, reload, and stream
  rewiring behavior.
- MuJoCo helper code can read available stderr without blocking.
- macOS headless MuJoCo uses the current Python executable where needed.
- Go2 WebRTC connection forwards AES key configuration from `GlobalConfig`.

Review focus:

- These are operational fixes. Review them for process lifecycle correctness,
  not agent reasoning behavior.

Components:

- ModuleCoordinator notification path: controls build/start/system-module
  notification order.
- WorkerManagerPython: deploys modules into workers and handles worker pool
  lifecycle.
- MuJoCo process helpers: subprocess executable selection and stderr draining.
- Go2 connection config: replay stop behavior and WebRTC AES key forwarding.

Data flow:

1. Build/deploy happens through ModuleCoordinator and worker managers.
2. Stream transports and RPC references are wired before modules receive
   system-module notifications.
3. Some notifications use nowait behavior to avoid waiting on an agent client
   that is itself initializing from MCP.
4. Restart/reload paths must preserve transports and rewired module refs.
5. Simulator helper code isolates platform-specific process quirks from the
   rest of the robot stack.

State and write boundary:

- Coordinator owns deployed module proxy maps, transport registry, module
  transports, and reload aliases.
- These changes do not alter skill semantics.
- They do affect whether modules start, stop, reload, and reconnect cleanly.

Failure mode:

- A stuck build/start should stop managers and surface the original exception.
- Restart should not orphan old transports or leave consumers attached to dead
  proxies.
- ReplayConnection stop should be a no-op, not an exception.

### 16. Optional Rerun/WebsocketVis World-Model Panel

Files:

- `dimos/web/websocket_vis/websocket_vis_module.py`
- `dimos/web/websocket_vis_spec.py`
- `dimos/web/templates/rerun_dashboard.html`
- `dimos/web/websocket_vis/test_websocket_vis_module.py`

Implementation logic:

- Websocket visualization can carry world-model/dashboard state in addition to
  existing visualization payloads.
- Dashboard payload shape is aligned with `world_model_contract.py`.
- This is optional observability. The current `unitree_go2_agentic` blueprint
  does not include `WebsocketVisModule`, so this path is inactive unless a
  visualization blueprint wires `WebsocketVisSpec`.

Review focus:

- Dashboard state should remain display-only.
- Avoid making dashboard consumers depend on unversioned internal Python
  objects.
- Do not treat this as a required dependency of the Go2 self-evolution runtime.

Components:

- `websocket_vis_spec.py`: typed surface for visualization updates.
- `websocket_vis_module.py`: runtime module that stores/publishes dashboard
  state.
- `rerun_dashboard.html`: browser-facing display surface.
- `world_model_contract.py`: schema source for world-model dashboard payloads.

Data flow:

1. CausalWorldModel produces a dashboard payload using
   `dimos.world_model_dashboard.v1`.
2. If WebsocketVis is wired, CausalWorldModel calls `set_world_model_state(...)`.
3. WebsocketVis stores and serves the latest state to dashboard clients.
4. The dashboard renders the state as inspection data for humans.

State and persistence:

- Dashboard state is the latest display snapshot, not the source of truth.
- Durable state, if configured, belongs to CausalWorldModel save/load JSON.
- Browser/dashboard consumers should rely on versioned payload fields only.

Safety boundary:

- Dashboard updates must not trigger robot actions.
- UI should not present advisory predictions as guaranteed outcomes.
- If `WebsocketVisModule` is not wired, CausalWorldModel still works; the
  dashboard publication path is skipped.

## Review Order I Recommend

1. Start with Go2 blueprint wiring:
   `unitree_go2_agentic.py`, Layer 3/4/5/6 `__init__.py`, and
   `test_world_state.py`.
2. Review MCP runtime plumbing:
   `mcp_server.py`, `mcp_client.py`, `tool_stream.py`, and their tests.
3. Review self-evolution write boundaries:
   `evolution_ledger.py`, `evolution_proposal.py`, `skill_proposal.py`.
4. Review context quality:
   `context_provider.py`, `context_evidence.py`, `context_feedback.py`.
5. Review action safety:
   `skill_interface_registry.py`, `task_feasibility.py`,
   `skill_outcome_predictor.py`.
6. Review advisory world-model logic:
   `causal_world_model.py` and `world_model_contract.py`.
7. Review upstream transport changes separately, especially Zenoh defaults on
   macOS.

## Validation Run During Upstream Sync

Passed:

- `.venv/bin/python -m pytest dimos/agents/mcp/test_mcp_server.py -q`
  - `10 passed, 1 warning`
- `.venv/bin/python -m pytest dimos/robot/unitree/go2/test_connection.py -q`
  - `5 passed`
- `.venv/bin/python -m pytest dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain -q`
  - `62 passed`
- `.venv/bin/python -m pytest dimos/robot/unitree/go2/blueprints/layers/layer_4_world_state/test_world_state.py -q`
  - `3 passed, 1 warning`
- `.venv/bin/python -m pytest dimos/perception/test_spatial_perception_config.py -q`
  - `2 passed`
- `DIMOS_TRANSPORT=lcm .venv/bin/python -m pytest dimos/core/coordination/test_module_coordinator.py -q`
  - `35 passed`
- `DIMOS_TRANSPORT=lcm MCP_PORT=9991 .venv/bin/python -m pytest dimos/agents/mcp/test_mcp_client_unit.py dimos/agents/mcp/test_tool_stream.py -q`
  - `44 passed, 1 warning`
- `.venv/bin/python -m pytest dimos/robot/test_all_blueprints_generation.py -q`
  - `1 passed`
- `git diff --cached --check -- ':(exclude)*.patch'`
  - passed

Known caveats:

- Running MCP tests on default port `9990` fails on this machine because a live
  `dimos-live` process already owns that port.
- Running some tests with upstream's new macOS default `zenoh` backend exposed
  native pyo3 thread lifetime issues in pytest. The same coordinator and MCP
  suites pass under `DIMOS_TRANSPORT=lcm`. Reviewers should decide whether to
  treat Zenoh-on-macOS test behavior as an upstream follow-up or to pin local
  tests to LCM until Zenoh cleanup semantics are stable.
