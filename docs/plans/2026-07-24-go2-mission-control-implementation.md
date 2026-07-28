# Go2 Mission Control Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a beginner-usable local Mission Control page for a Unitree Go2 that combines DimOS camera/map visualization, natural-language mission control, explicit safety gating, and an observation-first autonomous door-search workflow.

**Architecture:** Reuse the existing DimOS `unitree_go2_agentic` blueprint, frontier explorer, navigation stack, spatial memory, Rerun viewer, and WebSocket command center. Add a pure-Python mission domain/state machine inside Studio, expose it through a small FastAPI facade, and render the existing visualization beside mission controls. The first release stays movement-locked by default and only permits autonomous motion after the existing hardware confirmation plus a mission-specific safety gate.

**Tech Stack:** Python 3.12, FastAPI, Pydantic, DimOS modules/blueprints/MCP, vanilla HTML/CSS/JavaScript, pytest, Ruff

---

### Task 1: Add the mission domain and safety gate

**Files:**
- Create: `dimos/web/studio/mission.py`
- Modify: `dimos/web/studio/models.py`
- Test: `dimos/web/studio/test_mission.py`

**Step 1: Write the failing tests**

Cover:

- a new mission starts in `draft`
- a movement-locked mission cannot start
- a mission requires a running DimOS runtime
- a mission pauses when a person is within 1.5 m
- a mission resumes only after the scene has been clear for 3 seconds
- emergency stop always transitions to `stopped`
- repeated navigation failures transition to `failed`

**Step 2: Run tests to verify they fail**

Run: `uv run pytest dimos/web/studio/test_mission.py -q`

Expected: FAIL because the mission domain does not exist.

**Step 3: Implement the minimum domain model**

Implement:

- `MissionPolicy` with conservative defaults
- `SafetySnapshot`
- `MissionRecord`
- `MissionController`
- deterministic transition methods for create, start, observe, pause, resume, complete, fail, and emergency stop

No robot I/O belongs in this file.

**Step 4: Run tests to verify they pass**

Run: `uv run pytest dimos/web/studio/test_mission.py -q`

Expected: PASS.

**Step 5: Commit**

```bash
git add dimos/web/studio/mission.py dimos/web/studio/models.py dimos/web/studio/test_mission.py
git commit -m "feat: add Go2 mission safety state machine"
```

### Task 2: Expose mission control through the Studio API

**Files:**
- Modify: `dimos/web/studio/app.py`
- Modify: `dimos/web/studio/service.py`
- Modify: `dimos/web/studio/models.py`
- Modify: `dimos/web/studio/test_studio.py`

**Step 1: Write the failing API tests**

Cover:

- `GET /api/mission/status`
- `POST /api/mission`
- `POST /api/mission/start`
- `POST /api/mission/pause`
- `POST /api/mission/resume`
- `POST /api/mission/stop`
- `POST /api/mission/estop`
- mission start forwards an observation-first task to MCP only after the gate passes
- emergency stop is locally recorded even when MCP is unavailable

**Step 2: Run tests to verify they fail**

Run: `uv run pytest dimos/web/studio/test_studio.py -q`

Expected: FAIL with missing endpoints.

**Step 3: Implement the API facade**

Add request models and service methods. Persist only mission metadata and event history; never persist image frames or secrets. Format the Agent prompt so that it:

- observes before moving
- uses frontier exploration
- seeks a door candidate
- verifies the candidate across multiple observations
- approaches no closer than 1 m
- stops for people and reports uncertainty
- cannot override the Studio movement lock

**Step 4: Run tests to verify they pass**

Run: `uv run pytest dimos/web/studio/test_studio.py -q`

Expected: PASS.

**Step 5: Commit**

```bash
git add dimos/web/studio/app.py dimos/web/studio/service.py dimos/web/studio/models.py dimos/web/studio/test_studio.py
git commit -m "feat: expose safe mission control API"
```

### Task 3: Build the unified Mission Control page

**Files:**
- Modify: `dimos/web/studio/static/index.html`
- Modify: `dimos/web/studio/static/app.js`
- Modify: `dimos/web/studio/static/styles.css`
- Test: `dimos/web/studio/test_studio.py`

**Step 1: Write the failing page contract test**

Assert that the Studio HTML includes:

- one Mission Control tab
- embedded DimOS visualizer URL
- task input and create/start/pause/resume/stop controls
- prominent E-STOP
- visible robot/runtime/mission/safety state
- crowd-yield and movement-lock explanations

**Step 2: Run the test to verify it fails**

Run: `uv run pytest dimos/web/studio/test_studio.py -q`

Expected: FAIL because the page is absent.

**Step 3: Implement the page**

Add a dense functional layout:

- left: embedded `http://127.0.0.1:7779/` DimOS dashboard containing camera/Rerun and the command center
- right: task editor, state, current phase, safety reason, policy summary, event log, and controls
- poll mission/runtime status every two seconds
- disable controls that are invalid for the current state
- never issue direct velocity commands from Studio

**Step 4: Verify**

Run:

- `uv run pytest dimos/web/studio/test_studio.py -q`
- `node --check dimos/web/studio/static/app.js`

Expected: PASS.

**Step 5: Commit**

```bash
git add dimos/web/studio/static/index.html dimos/web/studio/static/app.js dimos/web/studio/static/styles.css dimos/web/studio/test_studio.py
git commit -m "feat: add unified Go2 mission control page"
```

### Task 4: Add the observation-first door mission skills

**Files:**
- Modify: `extensions/go2-studio-agent/src/dimos_go2_studio/skills.py`
- Create: `extensions/go2-studio-agent/src/dimos_go2_studio/mission_policy.py`
- Create: `extensions/go2-studio-agent/tests/test_mission_policy.py`
- Modify: `extensions/go2-studio-agent/pyproject.toml`

**Step 1: Write the failing policy tests**

Cover:

- door candidates require multiple independent observations
- candidates with low confidence are rejected
- approach goal stops at the configured stand-off distance
- person presence yields `pause`
- stale perception yields `stop`
- Agent output is advisory and cannot directly unlock movement

**Step 2: Run tests to verify they fail**

Run: `uv run pytest extensions/go2-studio-agent/tests/test_mission_policy.py -q`

Expected: FAIL because the policy module does not exist.

**Step 3: Implement pure policy helpers and typed skills**

Expose typed `@skill` methods for:

- beginning an observation-first door-search plan
- recording a door observation
- deciding whether a candidate is verified
- reporting the next safe action

Keep navigation effects in existing DimOS skills; this module returns decisions and evidence only.

**Step 4: Run tests to verify they pass**

Run: `uv run pytest extensions/go2-studio-agent/tests/test_mission_policy.py -q`

Expected: PASS.

**Step 5: Commit**

```bash
git add extensions/go2-studio-agent
git commit -m "feat: add evidence-based door mission skills"
```

### Task 5: Wire a mission-ready blueprint without bypassing safety

**Files:**
- Modify: `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`
- Test: `extensions/go2-studio-agent/tests/test_blueprint.py`
- Modify: `docs/PROJECT_CONTEXT.md`

**Step 1: Write the failing blueprint smoke test**

Verify that the external blueprint loads and includes:

- existing agentic Go2 stack
- existing frontier explorer
- existing map/camera visualization
- custom mission skills

**Step 2: Run the smoke test to verify it fails**

Run: `uv run pytest extensions/go2-studio-agent/tests/test_blueprint.py -q`

Expected: FAIL until the mission integration is explicit.

**Step 3: Implement the minimum wiring**

Keep the existing agentic blueprint as the base. Add only explicit mission policy modules and config. Do not add a second movement publisher and do not connect any model output directly to `cmd_vel`.

**Step 4: Run full static and unit verification**

Run:

- `uv run pytest dimos/web/studio extensions/go2-studio-agent/tests -q`
- `uv run ruff check dimos/web/studio extensions/go2-studio-agent`
- `node --check dimos/web/studio/static/app.js`

Expected: PASS.

**Step 5: Update project context and commit**

Record the implemented state, commands, validation, unresolved real-hardware gates, and next staged checks in `docs/PROJECT_CONTEXT.md`.

```bash
git add extensions/go2-studio-agent docs/PROJECT_CONTEXT.md
git commit -m "docs: record Go2 mission control implementation"
```

### Task 6: Perform staged non-moving validation

**Files:**
- Modify: `docs/PROJECT_CONTEXT.md`

**Step 1: Validate the Studio API locally**

Start Studio on a non-conflicting test port and verify:

- health
- settings
- mission create
- movement-locked start rejection
- emergency stop

**Step 2: Validate the DimOS blueprint without real movement**

Run the blueprint in simulation or hardware-read-only mode. Confirm:

- runtime appears in Studio
- `7779` visualization loads
- camera/map panels connect when streams exist
- mission controls remain movement-locked

**Step 3: Record the exact boundary**

Do not claim real autonomous operation until all later gates pass:

1. simulation/replay
2. real hardware read-only
3. empty controlled area at 0.1 m/s
4. one controlled pedestrian
5. only then a cluttered venue

**Step 4: Commit**

```bash
git add docs/PROJECT_CONTEXT.md
git commit -m "test: record non-moving mission control validation"
```
