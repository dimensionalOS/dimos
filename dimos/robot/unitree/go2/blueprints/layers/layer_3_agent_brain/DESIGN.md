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
  It also emits `context_evidence.v1`, a deterministic evidence ledger that
  records which sources were selected, why they were selected, and their
  relevance/confidence/risk/cost labels.
- `ContextEvidencePolicy`: pure helper policy for selecting and thresholding
  `context_evidence.v1` entries.
- `MemoryBackendStatus`: reports which context, memory, outcome, causal, and
  skill-interface backends are wired and whether basic read probes succeed.
- `EvolutionLedger`: writes low-volume self-evolution decisions, feedback, and
  human-reviewable proposals to `.dimos/evolution` JSON files, with optional
  local Git commits.
- `TaskFeasibilityEvaluator`: evaluates whether a user task is possible and
  safe before a physical skill is selected.
- `ContextFeedbackStore`: records whether selected `context_evidence.v1`
  sources helped or hurt observed task outcomes.
- `SkillProposalGenerator`: creates human-reviewable skill-interface proposals
  when repeated evidence shows a missing capability.
- `ExpertRouter`: maps a task to an expert domain and recommended tools.
- `PromptPolicy`: appends Layer 3 tool-use rules to the Go2 MCP client prompt.
- `SkillOutcomeStore`: records recent skill outcomes in memory.
- `SkillOutcomePredictor`: checks planned tool calls for obvious risk.
- `CausalWorldModel`: records before/action/result/after transitions and
  summarizes repeated failure causes. Its predictions and dashboard state use
  explicit DimOS world-model schemas.

## Runtime Flow

The intended task flow is:

```text
user task
  -> McpClient / LLM
  -> evaluate_task_feasibility(task)
  -> route_task(task)
  -> get_context(task, focus)
  -> evaluate_task_feasibility(task, context_json)
  -> predict_skill_outcome(skill_name, args_json, context)
  -> call a Layer 5 skill
  -> McpClient auto-records the outcome through record_skill_outcome(...)
  -> get_context(task, focus) for after-context
  -> record_causal_transition(...)
  -> record_context_feedback(...) when the context influenced the decision
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
  runtime config, recent skill outcomes, and recent context feedback.
- Returns a compact `SkillResult` with text plus structured metadata.
- Does not plan, execute skills, or own persistent state.

`ContextEvidencePolicy`

- Builds `context_evidence.v1` from already gathered ContextProvider metadata.
- Applies relevance, confidence, max-entry, and motion robot-state retention
  policy.
- Has no Module, MCP, RPC, database, or robot dependencies.
- Does not query RAG or decide which memory backend should be used.

`MemoryBackendStatus`

- Reads optional backend Specs already wired into the Layer 3 blueprint.
- Reports availability and small health/count signals for world state, spatial
  memory, temporal memory, skill outcomes, causal transitions, and Layer 5
  skill contracts.
- Catches backend probe failures and returns warnings instead of failing the
  whole report.
- Does not write memory, mutate stores, or decide which context is effective.

`EvolutionLedger`

- Writes low-volume JSON events under `.dimos/evolution/events/YYYY/MM/DD`.
- Creates `.dimos/evolution/proposals` for later proposal files.
- Optionally commits only the just-written event file with Git.
- Validates proposal target files before writing proposal files.
- Does not store raw images, embeddings, Chroma/SQLite data, or large logs.
- Does not merge, push, or modify executable robot skill code.

`TaskFeasibilityEvaluator`

- Reads the user task, optional `get_context` metadata, Layer 5 skill
  contracts, and the evolution ledger if wired.
- Returns `yes`, `no`, or `uncertain` plus missing context, required skills,
  safety risks, and the next recommended action.
- Uses deterministic rules in V1; it does not call an LLM.
- Does not execute skills or replace `SkillOutcomePredictor`.

`ContextFeedbackStore`

- Stores recent context-feedback records in a bounded in-memory `deque`.
- Writes full feedback events to `EvolutionLedger` when available.
- Aggregates helpful and harmful evidence-source counts for immediate
  `ContextProvider` use.
- Does not record skill success/failure generally; that remains
  `SkillOutcomeStore`.

`SkillProposalGenerator`

- Reads Layer 5 skill contracts, recent outcome misuse signals, and optional
  failure context.
- Avoids proposals for missing arguments or missing context; those are better
  handled by correct existing skill use.
- Writes review-only proposals through `EvolutionLedger` when available.
- Does not add `@skill` methods, patch executable code, or update prompts.

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
    `NavigationInterfaceSpec`, `SkillOutcomeStoreSpec`,
    `SkillInterfaceSpec`, and `ContextFeedbackSpec`.
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
  - Skill interface: `get_skill_interface_snapshot()`.
  - Context feedback: `get_recent_context_feedback(limit=5)` and
    `get_context_feedback_summary(limit=20)`.
  - Causal/world model: recent transitions, model state, and recent
    interventions when wired.
- Algorithm:
  - Trim `task` and `focus`.
  - Reject empty `task` with `SkillResult.fail("INVALID_INPUT", ...)`.
  - Clamp `spatial_limit` to `0..10`.
  - Build a metadata dictionary with `sources`, `runtime`, `robot_state`,
    `world_state`, `skill_state`, `context_feedback`, `causal_state`,
    `world_model_state`, and `external_context`.
  - Catch dependency failures per source and append readable warnings to
    `errors` instead of failing the whole context request.
  - Include Layer 5 skill-interface contracts under
    `skill_state.interface` when the registry is wired.
  - Build `context_evidence` from the selected metadata sections. V1 uses a
    deterministic source-coverage policy: always include task/runtime, then add
    robot state, spatial RAG matches, temporal summaries, fused Layer 4 memory,
    skill outcomes, skill contracts, causal transitions, and predictive
    world-model state only when those sections are available and informative.
  - Each evidence entry contains `source`, `query`, `relevance_score`,
    `recency`, `confidence`, `risk_impact`, `cost`, `selected_reason`, and a
    short `summary`.
  - Convert metadata through `_to_jsonable(...)` so MCP responses stay compact
    and serializable.
  - Format a short text summary for the LLM.
- Return shape:
  - `SkillResult(success=True, message=<summary>, metadata=<jsonable dict>)`.
  - Metadata keys: `task`, `focus`, `sources`, `runtime`, `robot_state`,
    `world_state`, `skill_state`, `context_feedback`, `causal_state`,
    `world_model_state`, `context_evidence`, `external_context`, and optional
    `errors`.
  - `context_evidence` keys: `version`, `selection_policy`, `query`, `focus`,
    `entry_count`, `selected_sources`, and `entries`.
- Current limits:
  - External context is represented as unavailable.
  - Context compression and evidence selection are deterministic formatting,
    not learned retrieval or reranking.
  - Relevance/confidence/risk/cost labels are coarse heuristics over already
    retrieved data. They explain why the context was admitted; they are not a
    statistical success estimate.
  - It reads existing stores but does not decide whether a skill should run.
  - Layer 5 contract details are summarized; full contract validation stays in
    `SkillInterfaceRegistry`.

### `build_context_evidence(...)`

- File: `layer_3_agent_brain/context_evidence.py`
- Entry point: pure helper function used by `ContextProvider`.
- Purpose: build and threshold `context_evidence.v1` from already gathered
  metadata. It does not fetch context, call memory backends, or use RPC.
- Inputs:
  - `metadata`: ContextProvider metadata dictionary.
  - `ContextEvidencePolicy`: `min_relevance_score`, `max_entries`,
    `include_low_confidence`, and `require_robot_state_for_motion`.
- Storage:
  - No database.
  - No file writes.
  - No module state.
- Algorithm:
  - Build the same deterministic source coverage entries used by
    ContextProvider V1: task, runtime, robot state, spatial/temporal memory,
    semantic-temporal map, skill outcomes, skill contracts, causal transitions,
    and predictive world-model state.
  - Apply relevance and low-confidence filtering while protecting task/runtime
    evidence.
  - For motion-like tasks, retain robot-state evidence when available.
  - Enforce `max_entries` deterministically by preserving source order.
- Return shape:
  - Dict with `version`, `selection_policy`, `query`, `focus`, `entry_count`,
    `selected_sources`, and `entries`.
- Current limits:
  - Scores remain coarse heuristics over already selected data.
  - Feedback-driven threshold tuning is now possible but not automatic.

### `_Go2MemoryBackendStatus.memory_backend_status(...)`

- File: `layer_3_agent_brain/memory_backend_status.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: give the agent and operator a compact, read-only status report for
  the memory/context stack before relying on RAG, temporal summaries, outcome
  history, causal transitions, or skill-interface contracts. It does not rank
  context, execute skills, or choose a storage backend.
- Inputs:
  - No direct user arguments.
  - Optional injected Specs: `WorldStateSpec`, `SpatialMemorySpec`,
    `TemporalMemorySpec`, `SkillOutcomeStoreSpec`, `CausalWorldModelSpec`, and
    `SkillInterfaceSpec`.
- Storage:
  - No database.
  - No persistent file writes.
  - No in-memory state beyond local variables while building the report.
- Data read:
  - World state: `get_world_snapshot(task="memory backend status",
    spatial_limit=0)`.
  - Spatial memory: `query_by_text("__memory_status_probe__", limit=1)`.
  - Temporal memory: `get_state()` and `get_graph_db_stats()`.
  - Skill history: `get_recent_outcomes(limit=5)`.
  - Causal/world model: `get_recent_transitions(limit=5)` and optional
    `get_model_state()`.
  - Skill interface: `get_skill_interface_snapshot()`.
- Algorithm:
  - Build one status section for each optional backend.
  - Mark a section `wired=False` when the Spec is not connected.
  - Run only small read probes against connected backends.
  - Catch each backend failure independently and append a readable warning such
    as `spatial_memory: ...`.
  - Count wired sections and return a short summary message.
- Return shape:
  - `SkillResult.ok("Memory backends: <n> wired, <m> warning(s)", ...)`.
  - Metadata keys: `schema`, `world_state`, `spatial_memory`,
    `temporal_memory`, `skill_outcomes`, `causal_world_model`,
    `skill_interface`, and `warnings`.
  - Schema version: `go2_memory_backend_status.v1`.
- Current limits:
  - Spatial status uses a harmless probe query, so `match_count_probe` is only a
    liveness hint, not a corpus-size estimate.
  - A wired backend with zero records is reported as available but empty.
  - The skill reports whether backends can be read; it deliberately does not
    judge whether a returned context is useful for a particular task.

### `_Go2EvolutionLedger.record_evolution_event(...)`

- File: `layer_3_agent_brain/evolution_ledger.py` and
  `layer_3_agent_brain/evolution_event.py`; proposal validation lives in
  `layer_3_agent_brain/evolution_proposal.py`
- Entry point: MCP skill exposed by `@skill`; `write_evolution_event(...)` is
  the RPC surface used by other Layer 3 modules. `record_skill_proposal(...)`
  is a proposal-only MCP skill.
- Purpose: record low-volume, reviewable self-evolution decisions and feedback
  in a local ledger. It does not store raw observations or replace RAG memory.
- Inputs:
  - Direct skill arguments: `event_type`, `task`, `payload_json`, and `commit`.
  - Environment: optional `DIMOS_EVOLUTION_LEDGER_DIR` override and
    `DIMOS_RUN_ID` inherited from `dimos run`.
- Storage:
  - Default path: repo-root `.dimos/evolution`.
  - Event path: `events/YYYY/MM/DD/<timestamp>-<event_type>.json`.
  - Proposal directory: `proposals/`.
  - Writes are atomic via a same-directory temp file and rename.
- Data shape:
  - Event schema: `dimos.evolution_event.v1`.
  - Fields: `timestamp`, `event_type`, `task`, `run_id`, `payload`, and
    `git.commit_requested` / `git.commit_sha`.
  - Proposal schema: `dimos.skill_proposal.v1`.
- Algorithm:
  - Validate `event_type` and parse `payload_json` as a JSON object.
  - Build the event with current UTC timestamp.
  - Create the ledger directory, README, event day directory, and proposals
    directory if needed.
  - Write the event JSON file.
  - If `commit=True`, detect the Git worktree and commit only the event file.
  - If no Git worktree is present, keep the file and return a warning.
  - For proposals, require human review, relative target files, no `..` path
    traversal, no evolution event-log targets, and no large binary artifact
    targets.
- Return shape:
  - Success: `SkillResult.ok("Recorded evolution event ...", ...)`.
  - Metadata keys: `schema`, `event_type`, `task`, `ledger_dir`,
    `event_path`, `commit_requested`, `commit_sha`, `warnings`, and `event`.
  - Proposal success returns `proposal_path`, `ledger_dir`, `commit_requested`,
    `commit_sha`, `warnings`, and the validated `proposal`.
- Current limits:
  - Commit SHA is returned in MCP metadata after the commit. The event file
    itself is written before the commit and keeps `git.commit_sha=""`.
  - There is no push, merge, branch creation, or automatic review workflow.

### `_Go2TaskFeasibilityEvaluator.evaluate_task_feasibility(...)`

- File: `layer_3_agent_brain/task_feasibility.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: decide whether the current user task is feasible here and now before
  a physical skill is selected. It complements `predict_skill_outcome(...)`,
  which evaluates a concrete planned skill call.
- Inputs:
  - Direct arguments: `task` and optional `context_json`.
  - Optional injected Specs: `SkillInterfaceSpec` and `EvolutionLedgerSpec`.
- Storage:
  - No local persistent state.
  - Writes a `task_feasibility` event to `EvolutionLedger` when wired.
- Data read:
  - Layer 5 contracts through `get_skill_interface_snapshot()`.
  - Optional `context_json` from `get_context` metadata, including robot state,
    world state, and `context_evidence`.
- Algorithm:
  - Reject empty task and invalid/non-object context JSON.
  - Map task keywords to likely required skills.
  - Compare required skills against Layer 5 contracts.
  - Check required context from contracts: robot state, world state, and
    context evidence.
  - Refuse clearly unsafe tasks such as disabling safety or running into a
    person.
  - Return `yes`, `no`, or `uncertain`, with a next action of `proceed`,
    `get_context`, `ask_clarifying_question`, or `refuse`.
- Return shape:
  - Metadata keys: `feasible`, `missing_context`, `required_skills`,
    `available_skills`, `safety_risks`, `recommended_next_action`,
    `clarifying_question`, `evidence_sources`, `missing_skills`, and
    `skill_interface_available`.
- Current limits:
  - V1 is deterministic keyword/rule matching, not semantic planning.
  - It does not infer detailed tool arguments; Layer 5 validation still checks
    concrete skill calls.

### `_Go2ContextFeedbackStore.record_context_feedback(...)`

- File: `layer_3_agent_brain/context_feedback.py`
- Entry point: MCP skill exposed by `@skill`; read APIs are RPC-only.
- Purpose: connect selected `context_evidence.v1` to observed task outcomes so
  later evidence policy changes can be based on feedback. It does not replace
  `SkillOutcomeStore`.
- Inputs:
  - Direct arguments: `task`, `context_evidence_json`, `selected_skill`,
    `outcome_json`, `helpful_sources_json`, and `ignored_risks_json`.
  - Optional injected Spec: `EvolutionLedgerSpec`.
- Storage:
  - Bounded in-memory `deque`, max 100 records.
  - Full feedback event written to `EvolutionLedger` when available.
- Data shape:
  - Feedback schema: `go2_context_feedback.v1`.
  - Each record includes task, selected skill, outcome success/error, evidence
    sources, helpful sources, ignored risks, helpful source counts, and harmful
    source counts.
- Algorithm:
  - Validate `context_evidence_json.version == "context_evidence.v1"`.
  - Parse outcome, helpful sources, and ignored risks.
  - Mark helpful sources from the explicit helpful list.
  - Mark harmful sources from ignored risk/source labels and high-risk evidence
    entries when the observed outcome failed.
  - Append newest feedback to the bounded deque.
  - Write a `context_feedback` ledger event when wired.
- Related read APIs:
  - `get_recent_context_feedback(limit, source)` returns newest records first.
  - `get_context_feedback_summary(limit)` aggregates helpful/harmful source
    counts and success/failure totals.
- Current limits:
  - Feedback is manual/LLM-triggered in V1. It is not automatically recorded
    after every tool call.
  - Harmful source counts are attribution hints, not causal proof.

### `_Go2SkillProposalGenerator.propose_skill_interface(...)`

- File: `layer_3_agent_brain/skill_proposal.py`
- Entry point: MCP skill exposed by `@skill`.
- Purpose: propose a new or revised skill interface when repeated evidence
  shows the current Go2 skill contracts lack a capability. It never modifies
  executable skill code.
- Inputs:
  - Direct arguments: `task` and `failure_context_json`.
  - Optional injected Specs: `SkillInterfaceSpec`, `SkillOutcomeStoreSpec`,
    and `EvolutionLedgerSpec`.
- Storage:
  - No local persistent state.
  - Writes a proposal to `EvolutionLedger.record_skill_proposal(...)` when the
    ledger is wired.
- Data read:
  - Layer 5 contracts through `get_skill_interface_snapshot()`.
  - Recent outcomes only to detect repeated existing-skill misuse.
- Algorithm:
  - Reject empty task and invalid/non-object failure context JSON.
  - Compare the requested capability with current skill contract names,
    summaries, and domains to avoid duplicates.
  - If repeated failures are missing arguments, invalid arguments, or missing
    context, return no proposal and recommend correct existing skill use.
  - Only create a proposal for explicit missing capability evidence.
  - Generate a `dimos.skill_proposal.v1` payload with target files, proposed
    interface, validation plan, and `requires_human_review=True`.
  - Validate the proposal before writing it through the ledger.
- Return shape:
  - No proposal: metadata includes `proposal_created=False`,
    `recommended_next_action`, and optional `existing_skill_matches`.
  - Proposal: metadata includes `proposal_created=True`, `proposal`,
    `recommended_next_action="review_proposal"`, and optional `proposal_path`.
- Current limits:
  - V1 generation is deterministic and conservative.
  - It proposes interface contracts, not executable implementations.

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
    `evaluate_task_feasibility`, `route_task`, `get_context`, and
    `predict_skill_outcome` before risky physical tools.
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
  - `predict_next_state(...)` returns schema `dimos.world_model_prediction.v1`.
  - Dashboard publications use schema `dimos.world_model_dashboard.v1`.
  - `get_provider_contract()` returns schema `dimos.world_model_provider.v1`.
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

Self-evolution plan Task 1 implemented:

- `MemoryBackendStatus` exposes a read-only MCP skill for checking whether
  memory/context backends are wired and whether small read probes work.
- The status report is observability only. It does not create a new memory
  store, replace RAG retrieval, or decide context effectiveness.

Self-evolution plan Tasks 2-4 implemented:

- `EvolutionLedger` writes low-volume decision and feedback events to local
  JSON files under `.dimos/evolution`, with optional event-only Git commits.
- `TaskFeasibilityEvaluator` adds a deterministic task-level preflight before
  physical skills are selected.
- `ContextFeedbackStore` records whether selected context evidence helped or
  hurt outcomes, then exposes recent summaries back to `ContextProvider`.
- `EvolutionLedger` also validates and records human-reviewable proposal files
  without applying patches or mutating skill code.
- `CausalWorldModel` predictions, provider contracts, and dashboard snapshots
  now carry explicit DimOS schemas.

Self-evolution plan Tasks 5-6 implemented:

- `ContextEvidencePolicy` moves evidence scoring and selection out of
  `ContextProvider` into a pure helper with threshold and max-entry policy.
- `SkillProposalGenerator` creates validated, human-reviewable skill-interface
  proposals only when repeated evidence points to a missing capability.
- Missing arguments, missing context, and existing skill contracts suppress new
  skill proposals.

Out of scope for this stage:

- Do not promote these self-evolution pieces into robot-agnostic
  `dimos.agents` modules yet. Keep validating them in the Go2 layered stack
  until the contracts and feedback data prove stable.

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
