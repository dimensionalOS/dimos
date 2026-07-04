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

Call path and ownership:

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

Where does Git data go:

- It goes into this local repository's working tree and local `.git` history.
- It does not push to GitHub.
- GitHub only receives it if a human later runs `git push` or opens a PR.

Write path:

1. A caller invokes a ledger-facing MCP skill such as
   `record_evolution_event()` or `record_skill_proposal()`.
2. The module validates the schema and normalizes the payload.
3. The module chooses the storage root. The default is the repo-local
   `.dimos/evolution` tree; `DIMOS_EVOLUTION_LEDGER_DIR` overrides it.
4. The JSON file is written as the review artifact.
5. If `commit=False`, Git is not touched beyond the working tree file.
6. If `commit=True`, only that new event/proposal file is staged and committed
   locally. There is no push.

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

Decision tree:

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

Boundary:

- It does not call the target skill.
- It does not create a new skill.
- It does not ask the LLM to classify feasibility.

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

### 16. Rerun/Websocket Dashboard World-Model State

Files:

- `dimos/web/websocket_vis/websocket_vis_module.py`
- `dimos/web/websocket_vis_spec.py`
- `dimos/web/templates/rerun_dashboard.html`
- `dimos/web/websocket_vis/test_websocket_vis_module.py`

Implementation logic:

- Websocket visualization can carry world-model/dashboard state in addition to
  existing visualization payloads.
- Dashboard payload shape is aligned with `world_model_contract.py`.

Review focus:

- Dashboard state should remain display-only.
- Avoid making dashboard consumers depend on unversioned internal Python
  objects.

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
