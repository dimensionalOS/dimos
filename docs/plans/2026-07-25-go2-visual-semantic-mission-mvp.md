# Go2 Visual Semantic Mission MVP Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use `executing-plans` to implement this plan task-by-task.

**Goal:** Build a Demo-grade Go2 mission loop that understands a natural-language household request, navigates to a known or semantically resolved place, performs an on-demand visual inspection after confirmed arrival, and reports an evidence-backed result without attempting manipulation.

**Architecture:** Keep one Go2/DimOS runtime. The external conversation Agent converts user language into a typed `MissionSpec`; a deterministic Python mission executor resolves the destination, waits for the existing A* navigator to reach it, then runs an on-demand vision provider on fresh camera frames. Mapping, obstacle avoidance, navigation state, cancellation, and motion remain local; the visual model returns evidence but never sends motion commands.

**Tech Stack:** Python 3.12, DimOS `0.0.14b1` source checkout, Go2 camera/LiDAR/odometry, `SpatialMemory` + CLIP, `ReplanningAStarPlanner`, local Moondream with a provider-neutral adapter, MCP, TypeScript Pi Agent, Pytest/Vitest.

---

## 1. Product answer and Demo boundary

### What can be demonstrated with the current product stack

| Request | Demo definition | Feasible |
|---|---|---|
| “我有紧急情况，来到我身边” | Pre-register an exact location named `用户身边`; preempt the current mission and navigate there | Yes |
| “去门口帮我拿外卖” | Navigate to `门口`, inspect for a parcel/takeout bag, report found/not found; do not pick it up | Yes |
| “去看厨房有没有水瓶” | Navigate to `厨房`, inspect fresh camera views for the requested object, report evidence | Yes |
| Find an unregistered room in a previously explored map | Use CLIP spatial memory to propose a camera-viewpoint candidate, then verify after arrival | Demo-grade only |
| Find the user anywhere in the house | Requires a live user position from ring/phone/UWB or a separately validated person-search/follow flow | Not in this MVP |
| Pick up, carry, open doors, press switches | Requires manipulation hardware and a separate manipulation stack | Not supported |

No custom model training is required for this MVP. The first release uses existing models and modules, while keeping model providers replaceable.

### Success means

A mission is complete only when:

1. the Agent produced a valid typed intent;
2. the destination resolved to a map pose;
3. the navigator reported goal reached;
4. a fresh post-arrival image was inspected;
5. the result includes the destination, arrival state, visual verdict, frame IDs, and failure reason if any.

An MCP call being accepted is not completion. A visual model answer without confirmed arrival is not completion.

## 2. Current source findings

### Existing useful capabilities

- `GO2Connection.observe()` returns the latest camera frame.
- `unitree_go2` already contains camera, LiDAR, voxel mapping, cost mapping, A* replanning, exploration, patrol, and `MovementManager`.
- `SpatialMemory` stores camera views with robot pose and supports CLIP text-to-image search.
- `NavigationSkillContainer` supports tagged locations and semantic-map navigation.
- `PerceiveLoopSkill` supports local on-demand target detection and continuations.
- Local `MoondreamVlModel` supports free-form visual questions, object boxes, and target points.
- `target_verification.py` already provides a provider-neutral, structured candidate-verification boundary and an optional OpenAI implementation.
- `ReturnToUserAndGreetSkill` already demonstrates the correct pattern of setting a goal, polling navigation state, and acting only after confirmed arrival.

### Current gaps that must not be hidden

1. No Go2, MCP wrapper, or upper Agent process is currently running.
2. The active lightweight `dimos-go2-studio.go2` Blueprint deliberately excludes `SpatialMemory`, `PerceiveLoopSkill`, `NavigationSkillContainer`, and embedded `McpClient`.
3. Mission Control currently sends text through `agent_send`, but the lightweight Blueprint has no embedded Agent to consume it.
4. The current TypeScript MCP client and Python wrapper preserve text only; image content returned by `observe` is discarded before the upper Pi Agent sees it.
5. The current mission controller stores state and forwards a prompt, but there is no live deterministic executor that waits for arrival and then inspects.
6. The current mission prompt refers to semantic memory even though the active lightweight Blueprint does not include it.
7. `SpatialMemory` defaults `new_memory=True`; using it unchanged can clear the persistent semantic map on startup.
8. CLIP results are camera-viewpoint candidates, not object bounding boxes or guaranteed metric object positions.
9. Local Moondream defaults to CPU on this Mac because `LocalModelConfig` only auto-selects CUDA or CPU; MPS availability is not currently selected.

## 3. Chosen approach

### Option A: Run the full official `unitree-go2-agentic` Blueprint

- Fastest path to official Agent, semantic memory, navigation, and visual skills.
- Also brings embedded model dependencies, speech, official person following, broad tool exposure, CUDA-only EdgeTAM, and extra runtime complexity.
- Reintroduces a second conversation owner and conflicts with the current lightweight external-Agent direction.

**Decision:** Do not use as the primary Demo architecture.

### Option B: One lightweight Go2 runtime plus deterministic semantic mission skills

- Add persistent `SpatialMemory`, exact location tags, on-demand visual inspection, and a background mission executor to the current external Blueprint.
- Keep the upper Pi Agent as the only natural-language conversation owner.
- Run vision inside the Python robot runtime and return structured JSON text, avoiding the current image-stripping MCP wrapper.
- Keep navigation and obstacle avoidance in existing DimOS modules.

**Decision:** Recommended.

### Option C: Build a new end-to-end Agent/runtime outside DimOS

- Maximum flexibility.
- Duplicates mapping, navigation, lifecycle, MCP, visualization, and cancellation already present in DimOS.

**Decision:** Reject for the Demo.

## 4. Runtime data flow

```text
User / ring / Studio
        |
        v
Upper Pi Agent: natural language -> MissionSpec
        |
        v
MCP run_demo_mission(MissionSpec)
        |
        v
Deterministic MissionExecutor
  RESOLVE_LOCATION
      exact tag -> semantic candidate -> fail
  NAVIGATE
      set_goal -> poll state -> reached/retry/fail
  SETTLE
      cancel residual motion -> wait 1 s
  INSPECT
      fresh camera frames -> local VisionProvider
  REPORT
      typed evidence -> upper Agent -> user
```

The visual model does not choose a velocity, publish `cmd_vel`, change a cost map, or declare physical arrival.

## 5. Mission contract

Use a small typed vocabulary for the first Demo:

```python
class MissionKind(StrEnum):
    COME_TO_USER = "come_to_user"
    GO_TO_LOCATION = "go_to_location"
    INSPECT_LOCATION = "inspect_location"


class MissionSpec(BaseModel):
    kind: MissionKind
    location_name: str
    question: str | None = None
    target_description: str | None = None
    priority: Literal["normal", "emergency"] = "normal"
```

Examples:

```json
{
  "kind": "come_to_user",
  "location_name": "用户身边",
  "priority": "emergency"
}
```

```json
{
  "kind": "inspect_location",
  "location_name": "门口",
  "question": "门口有没有外卖袋或包裹？",
  "target_description": "takeout bag or parcel"
}
```

```json
{
  "kind": "inspect_location",
  "location_name": "厨房",
  "question": "厨房里有没有水瓶？",
  "target_description": "water bottle"
}
```

## 6. Implementation tasks

### Task 1: Add typed mission and result contracts

**Files:**

- Create: `extensions/go2-studio-agent/src/dimos_go2_studio/mission_contracts.py`
- Create: `extensions/go2-studio-agent/tests/test_mission_contracts.py`

**Step 1: Write failing tests**

Test:

- only the three MVP mission kinds are accepted;
- `come_to_user` always normalizes the location to exact `用户身边`;
- inspection requires a non-empty question;
- emergency priority is valid only for `come_to_user`;
- unknown fields are rejected;
- result states are `queued/resolving/navigating/arrived/inspecting/completed/failed/cancelled`;
- completed inspection requires at least one evidence frame ID;
- failed missions require a non-empty failure reason.

**Step 2: Run the focused test**

```bash
cd /Users/johnsonmac/ai_completion/dimos
.venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_mission_contracts.py -v
```

Expected: fail because `mission_contracts.py` does not exist.

**Step 3: Implement minimal frozen Pydantic contracts**

Do not put robot I/O, model calls, or Agent prompting in this file.

**Step 4: Re-run the test**

Expected: pass.

### Task 2: Restore persistent semantic memory without restoring continuous VLM

**Files:**

- Modify: `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`
- Modify: `extensions/go2-studio-agent/tests/test_blueprint.py`

**Step 1: Change the Blueprint test first**

Assert:

- exactly one `GO2Connection`;
- exactly one `MovementManager`;
- exactly one `SpatialMemory`;
- `SpatialMemory.kwargs["new_memory"] is False`;
- `SpatialMemory.kwargs["min_distance_threshold"] >= 0.20`;
- `SpatialMemory.kwargs["min_time_threshold"] >= 2.0`;
- `NavigationSkillContainer` is present;
- `PerceiveLoopSkill`, embedded `McpClient`, and `SpeakSkill` remain absent.

**Step 2: Run the test and confirm failure**

```bash
cd /Users/johnsonmac/ai_completion/dimos
PYTHONPATH=extensions/go2-studio-agent/src \
  .venv/bin/python -m pytest \
  extensions/go2-studio-agent/tests/test_blueprint.py -v
```

**Step 3: Add the modules**

Compose:

```python
SpatialMemory.blueprint(
    new_memory=False,
    min_distance_threshold=0.25,
    min_time_threshold=2.0,
)
NavigationSkillContainer.blueprint()
```

Do not add `PerceiveLoopSkill`; this MVP performs explicit, post-arrival inspection instead of continuously running a VLM.

**Step 4: Verify startup configuration**

Run the Blueprint test and a replay startup smoke test. Confirm an existing semantic DB is not deleted on restart.

### Task 3: Add a provider-neutral on-demand visual inspection module

**Files:**

- Create: `extensions/go2-studio-agent/src/dimos_go2_studio/visual_inspection.py`
- Create: `extensions/go2-studio-agent/tests/test_visual_inspection.py`
- Reuse: `dimos/perception/target_verification.py`
- Reference: `dimos/models/vl/moondream.py`

**Step 1: Write failing unit tests with a fake provider**

Test:

- no frame returns `status=no_frame`;
- a frame older than 1 second returns `status=stale_frame`;
- the module samples at most three fresh frames;
- the provider receives only those frames and the requested target/question;
- `yes/no/uncertain` are the only accepted verdicts;
- provider timeout or malformed output becomes `uncertain`;
- the JSON response contains `frame_ids`, timestamps, verdict, answer, and model identifier;
- the module never invokes navigation or movement.

**Step 2: Define the generic interfaces**

```python
class InspectionRequest(BaseModel):
    question: str
    target_description: str | None = None
    frame_ids: list[str]


class InspectionResult(BaseModel):
    verdict: Literal["yes", "no", "uncertain"]
    answer: str
    target_description: str | None
    frame_ids: list[str]
    model: str
```

```python
class SceneVisionProvider(Protocol):
    def inspect(
        self,
        request: InspectionRequest,
        frames: list[Image],
    ) -> InspectionResult: ...
```

**Step 3: Add `MoondreamSceneVisionProvider`**

- Load lazily on the first inspection.
- Use `query_detections` for target-presence questions when `target_description` exists.
- Use `query` for the concise scene answer.
- Do not use Moondream's hard-coded `confidence=1.0` as a calibrated confidence score.
- Return `uncertain` when detection and textual answer disagree.

**Step 4: Add an optional adapter to the existing external verifier**

This is not required to pass the local MVP. It allows later candidate-only cloud verification without changing `MissionExecutor`.

**Step 5: Benchmark the current Mac**

Run 20 replay frames on CPU. Separately test an explicit `device="mps"` configuration; retain MPS only if model load and all 20 queries are correct and stable.

Record:

- model load time;
- median and p95 inspection latency;
- peak resident memory;
- any MPS dtype/operator failure.

Do not claim MPS support merely because `torch.backends.mps.is_available()` is true.

### Task 4: Add a destination resolver

**Files:**

- Create: `extensions/go2-studio-agent/src/dimos_go2_studio/location_resolver.py`
- Create: `extensions/go2-studio-agent/tests/test_location_resolver.py`

**Step 1: Write failing tests**

Resolution order:

1. exact tagged location;
2. semantic tag match for non-emergency tasks;
3. CLIP spatial-memory camera-viewpoint candidate;
4. structured failure.

Test:

- emergency `come_to_user` accepts only the exact `用户身边` tag;
- it never substitutes a semantically similar location;
- semantic candidates below the existing similarity threshold are rejected;
- CLIP results are marked `candidate_only=True`;
- missing pose metadata is rejected;
- a candidate returns the stored camera pose, not a fabricated object pose.

**Step 2: Implement `LocationResolver` over `SpatialMemorySpec`**

The resolver returns:

```python
class ResolvedDestination(BaseModel):
    name: str
    pose: PoseStamped
    source: Literal["exact_tag", "semantic_tag", "clip_viewpoint"]
    candidate_only: bool
```

**Step 3: Add setup guidance**

Before the first Demo, manually place the dog at and tag:

- `用户身边`
- `门口`
- `厨房`
- one additional room used in the scripted Demo

Exact tags are the primary reliable path. Semantic location discovery is a fallback, not the first acceptance gate.

### Task 5: Implement the deterministic background mission executor

**Files:**

- Create: `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`
- Create: `extensions/go2-studio-agent/tests/test_mission_executor.py`
- Reference: `components/dimos-mcp/src/dimos_dog_mcp/return_to_user.py`
- Reference: `dimos/navigation/navigation_spec.py`

**Step 1: Write failing tests using fake navigation, resolver, and vision**

Test the exact sequence:

```text
queued
-> resolving
-> navigating
-> arrived
-> inspecting
-> completed
```

Also test:

- inspection is never called before `is_goal_reached() is True`;
- navigation rejection fails the mission;
- navigator returning to `IDLE` without goal reached is a failure;
- 120-second timeout cancels the goal;
- cancellation stops the tracker/inspection and cancels navigation;
- emergency mission preempts a normal mission before setting the new goal;
- `come_to_user` fails if the exact tag is absent;
- `GO_TO_LOCATION` completes at arrival without requiring a visual verdict;
- `INSPECT_LOCATION` settles for one second before using post-arrival frames;
- a CLIP candidate requires final visual inspection before completion;
- all errors produce a typed terminal result.

**Step 2: Implement `SemanticMissionExecutor`**

Dependencies:

- `NavigationInterfaceSpec`
- `SpatialMemorySpec` through `LocationResolver`
- `VisualInspectionSpec`

The executor owns the `CAP_MOVEMENT` background lifecycle. It must not call `navigate_with_text`, because the current official method returns immediately and releases its movement capability before physical navigation ends.

**Step 3: Keep retries bounded**

- Let the existing A* planner perform its internal replans.
- At mission level, allow at most one destination re-resolution after a failed semantic candidate.
- Exact-tag navigation failure ends the mission and reports the planner state.

**Step 4: Add thread and shutdown tests**

Every terminal path must:

- stop the background thread;
- cancel an active goal when appropriate;
- release `CAP_MOVEMENT`;
- preserve the final result for `semantic_mission_status`.

### Task 6: Expose a minimal MCP skill surface

**Files:**

- Create: `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_mission_skills.py`
- Create: `extensions/go2-studio-agent/tests/test_semantic_mission_skills.py`
- Modify: `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`
- Modify: `extensions/go2-studio-agent/tests/test_blueprint.py`

**Step 1: Add four skills**

```text
run_demo_mission
semantic_mission_status
cancel_semantic_mission
inspect_current_view
```

`run_demo_mission` accepts the typed mission fields, not arbitrary executable plans.

**Step 2: Keep stopping deterministic**

`cancel_semantic_mission` calls the executor's cancellation RPC. The Studio global red stop must call it alongside existing exploration/navigation cancellation and runtime shutdown.

**Step 3: Return JSON text**

Vision inference stays inside Python and returns structured text. Do not send raw camera images through the current text-only wrapper in the MVP.

### Task 7: Connect the upper Pi Agent without starting a second dog runtime

**Files in `/Users/johnsonmac/ai_completion/agent`:**

- Modify: `components/agent-framework/dimos-mcp-wrapper/src/dimos_mcp_wrapper/dog_tools.py`
- Modify: `components/agent-framework/dimos-mcp-wrapper/src/dimos_mcp_wrapper/server.py`
- Modify: `components/agent-framework/dimos-mcp-wrapper/tests/test_dog_tools.py`
- Modify: `components/agent-framework/dimos-mcp-wrapper/tests/test_dimos_integration.py`
- Modify: `components/agent-framework/agent-webhook-gateway/src/agent-runtime.ts`
- Modify: `components/agent-framework/agent-webhook-gateway/test/agent-runtime.test.ts`

**Step 1: Add the four same-name forwarding tools**

They must forward once, preserve arguments, and never retry a movement request automatically.

**Step 2: Update the system prompt**

Map:

- “紧急情况，来到我身边” -> `come_to_user`, exact `用户身边`, emergency priority.
- “去门口拿外卖” -> explain no manipulation, then `inspect_location` at `门口` for a parcel.
- “去厨房看有没有水瓶” -> `inspect_location` at `厨房`.
- “停” remains the existing fast stop path; other cancellation language calls `cancel_semantic_mission`.

The Agent must not replace a missing location with timed straight-line motion.

**Step 3: Preserve one runtime**

In this operating mode:

- run `dimos-go2-studio.go2` as the only robot runtime;
- point the wrapper upstream at that runtime's MCP endpoint;
- do not also start `dimos_dog_mcp.blueprint`.

Add a startup check that rejects a second process owning the Go2/DimOS runtime ports.

**Step 4: Run tests**

```bash
cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/dimos-mcp-wrapper
/Users/johnsonmac/ai_completion/dimos/.venv/bin/python \
  -m unittest discover -s tests -v
```

```bash
cd /Users/johnsonmac/ai_completion/agent/components/agent-framework/agent-webhook-gateway
npm test
npm run check
npm run build
```

### Task 8: Route Studio Mission Control to the real upper Agent

**Files:**

- Modify: `dimos/web/studio/models.py`
- Modify: `dimos/web/studio/service.py`
- Modify: `dimos/web/studio/test_studio.py`
- Modify: `dimos/web/studio/static/app.js`
- Modify: `dimos/web/studio/static/index.html`

**Step 1: Write the failing service test**

The current lightweight runtime has no embedded Agent, so `StudioService.start_mission()` must not use `agent_send`.

Test:

- Studio posts the objective to the configured local Agent gateway;
- returned `instruction_id` is stored in the mission record;
- gateway failure pauses the mission;
- no second robot runtime is started;
- mission status shows Agent acceptance separately from physical progress.

**Step 2: Replace `agent_send` with `AgentGatewayClient`**

Keep this client local and text-only. The semantic mission tool itself performs visual inspection inside the Python runtime.

**Step 3: Add visible mission evidence**

Show:

- parsed mission kind;
- resolved destination and source;
- navigation state;
- arrived/not arrived;
- visual verdict and answer;
- evidence frame IDs;
- failure/cancellation reason.

Do not show private chain-of-thought.

### Task 9: Documentation and complete validation

**Files:**

- Modify: `docs/PROJECT_CONTEXT.md`
- Modify: `extensions/go2-studio-agent/README.md` if created during implementation
- Modify: `/Users/johnsonmac/ai_completion/agent/USAGE.md`
- Modify: `/Users/johnsonmac/ai_completion/agent/CONTEXT.md`

Document:

- one-runtime topology;
- exact location setup procedure;
- local Moondream/CLIP roles;
- optional external provider boundary;
- text-only MCP wrapper limitation;
- no manipulation claim;
- live-user-location limitation;
- start, status, cancellation, and evidence interpretation.

## 7. Validation gates

### Gate 0: Static and unit tests

```bash
cd /Users/johnsonmac/ai_completion/dimos
.venv/bin/python -m pytest extensions/go2-studio-agent/tests -v
.venv/bin/ruff check extensions/go2-studio-agent dimos/web/studio
git diff --check
```

All existing navigation-recovery tests must remain green.

### Gate 1: Replay with no robot movement

Use recorded camera/pose/map data to verify:

- exact tag resolution;
- CLIP fallback candidate resolution;
- arrival-before-inspection ordering;
- fresh-frame rejection;
- local visual yes/no/uncertain results;
- cancellation and timeout.

### Gate 2: Live read-only vision

Connect one Go2 runtime with movement disabled:

- confirm camera, LiDAR, odometry, and map freshness;
- inspect current view for five scripted objects;
- measure model latency;
- confirm no motion tool or `cmd_vel` is produced.

### Gate 3: Known-location navigation only

Tag `用户身边`, `门口`, and `厨房`.

Run three navigation-only missions and confirm:

- correct resolved map pose;
- A* path visible;
- planner reports reached;
- final odometry is within the configured planner tolerance;
- cancel works during motion.

### Gate 4: Full scripted Demo

Run:

1. “紧急情况，来到我身边。”
2. “去门口帮我拿外卖。”
3. “去厨房看一下有没有水瓶。”

Acceptance:

- all three intents map to the expected `MissionSpec`;
- no timed straight-line fallback occurs;
- inspection begins only after arrival;
- every result includes evidence frame IDs;
- the takeout mission says only “arrived and found/not found,” never “picked up”;
- the emergency mission uses the exact saved user location;
- the operator can cancel any mission.

### Gate 5: Semantic fallback

Only after the exact-tag Demo passes:

- remove one non-emergency exact tag;
- resolve the place from CLIP spatial memory;
- navigate to the stored camera viewpoint;
- re-inspect and either complete or report mismatch;
- never treat CLIP similarity alone as success.

## 8. Model decision

Start with:

- **CLIP:** continuous low-cost semantic location memory.
- **Moondream:** explicit post-arrival object detection and short scene answers.
- **Upper Pi Agent model:** natural-language intent to `MissionSpec`.

Do not train a model for the first Demo.

Keep the existing `SceneVisionProvider` boundary so later tests can select:

- a stronger external vision API for selected frames;
- a remote NVIDIA vision service;
- a better local Mac model;
- a specialized object detector.

The provider choice must not change navigation, stop, or mission-state semantics.

## 9. Recommended implementation order

1. Typed mission contract.
2. Persistent exact location tags and semantic memory.
3. On-demand visual inspection in replay.
4. Deterministic navigation/arrival executor.
5. MCP tools and cancellation.
6. Upper Agent intent mapping.
7. Studio UI routing.
8. Read-only live validation.
9. Three scripted physical Demo missions.
10. Semantic fallback only after the known-location path is stable.

This order proves the simple, reliable Demo first and leaves open-world exploration, live user localization, and manipulation for later phases.
