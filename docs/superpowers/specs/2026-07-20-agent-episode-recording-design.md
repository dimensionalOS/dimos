# Agent-Triggered Episode Recording — Design

**Date:** 2026-07-20
**Branch:** `feat/agent-episode-recording`
**Status:** approved design, pre-implementation

## Context

DimOS is meant to gather its own training data for VLA/robot-LLM models, but
today an operator can only start/stop a labeled recording episode with physical
Quest controller buttons. There is no way to do it from the agent at runtime.

The recording and export machinery already exists and is reused unchanged:
- `dimos/learning/collection/recorder.py` — `CollectionRecorder(Recorder)` writes
  declared streams to a SQLite session DB.
- `dimos/learning/collection/episode_monitor.py` — `EpisodeMonitorModule` runs a
  start/save/discard state machine and emits `EpisodeStatus` (with `task_label`).
- `dimos/learning/dataprep/` — `dimos dataprep build` segments episodes offline
  from the recorded `status` stream and exports LeRobot v3.0 / HDF5.

The only missing piece is a runtime trigger the agent can call. This feature adds
that trigger and wires a recorder into a Go2 agentic blueprint so the loop works
end to end.

## Goals / non-goals

**Goals**
- The agent can start, stop (save), and discard a labeled recording episode via
  natural language, over MCP.
- A new opt-in Go2 blueprint records camera + odom + action + episode-boundary
  streams to a session DB while the agent drives.
- The recorded DB is consumable by the existing `dimos dataprep build` exporter
  with no changes to dataprep.

**Non-goals (explicit)**
- No changes to the export pipeline (`dataprep`) — it already works.
- No new capture backend; we reuse `Recorder`.
- Real-robot motor-level (`lowcmd`) action capture — follow-up (see Caveats).
- Fixing the recorder's latest-coalescing frame drop — follow-up (see Caveats).

## Components

### 1. Runtime trigger: `set_episode` RPC on `EpisodeMonitorModule`
File: `dimos/learning/collection/episode_monitor.py`

Add a public RPC that drives the existing private `_transition(event, ts)` state
machine:

```python
@rpc
def set_episode(self, event: str, task_label: str | None = None) -> str:
    """Drive the episode state machine from an external caller (e.g. an agent).

    event: one of "start", "save", "discard".
    task_label: applied when event == "start".
    Returns a human-readable status string.
    """
```

Behavior:
- Validates `event ∈ {"start","save","discard"}`; unknown → returns an error
  string (no exception, so the calling skill can report it to the agent).
- On `"start"`: sets the active `task_label`, then `_transition("start", ts)`.
  If a take is already in progress, the existing state machine auto-commits it
  first (preserve current behavior; documented, not changed).
- On `"save"`/`"discard"`: `_transition(event, ts)`.
- Timestamp: uses the module's existing time source (same one `_on_buttons`/
  `_on_keyboard` use when they call `_transition`) so runtime and offline
  segmentation agree.
- Button/keyboard input paths are untouched; this is an additional entry point.
- Publishing of `EpisodeStatus` is unchanged (it already fires inside
  `_transition` via `_snapshot`).

### 2. Injection contract: `EpisodeControlSpec`
File: `dimos/learning/collection/episode_monitor_spec.py` (new)

```python
class EpisodeControlSpec(Spec, Protocol):
    def set_episode(self, event: str, task_label: str | None = None) -> str: ...
```
Follows the `dimos/navigation/navigation_spec.py` convention. `EpisodeMonitorModule`
satisfies it structurally via the `set_episode` `@rpc`.

### 3. Agent skill container: `EpisodeRecordingSkillContainer`
File: `dimos/agents/skills/episode_recording.py` (new)

`Module` subclass with the injected control spec and three `@skill`s
(`lifecycle="instant"`, no capability lock — recording does not block movement):

```python
class EpisodeRecordingSkillContainer(Module):
    _episode: EpisodeControlSpec

    @rpc
    def start(self) -> None: super().start()
    @rpc
    def stop(self) -> None: super().stop()

    @skill
    def start_recording(self, task_label: str) -> str:
        """Begin recording a training episode, labeled with the task.
        Example: start_recording("navigate to the kitchen")"""
        return self._episode.set_episode("start", task_label)

    @skill
    def stop_recording(self) -> str:
        """Stop and SAVE the current episode as a successful demonstration."""
        return self._episode.set_episode("save")

    @skill
    def discard_recording(self) -> str:
        """Stop and DISCARD the current episode (bad take, do not keep)."""
        return self._episode.set_episode("discard")
```

Docstrings are the LLM tool descriptions; they include example phrasings so the
model maps "record this as a kitchen run" → `start_recording("kitchen run")`.

### 4. Go2 recorder: `Go2CollectionRecorder`
File: `dimos/learning/collection/go2_recorder.py` (new) — or colocated with the
blueprint if that better matches repo layout.

`Recorder` subclass declaring the streams worth training on:
```python
class Go2CollectionRecorder(Recorder):
    color_image: In[Image]      # camera
    odom: In[PoseStamped]       # pose
    cmd_vel: In[Twist]          # action channel
    status: In[EpisodeStatus]   # episode boundaries (from the monitor)
```
DB path: `STATE_DIR/recordings/session_go2_<YYYYMMDD_HHMMSS>.db`, reusing the
`_session_db` helper pattern from `dimos/learning/collection/blueprint.py`.

### 5. Opt-in blueprint: `unitree-go2-agentic-record`
File: `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_record.py` (new)

```python
unitree_go2_agentic_record = autoconnect(
    unitree_go2_agentic,                       # existing agent + MCP + skills + Go2 stack
    Go2CollectionRecorder.blueprint(db_path=_session_db("go2")),
    EpisodeMonitorModule.blueprint(),
    EpisodeRecordingSkillContainer.blueprint(),
)
```
Presence in the graph auto-exposes the skills via MCP (`McpServer.on_system_modules`
collects every module's `@skill`s — no manual registration). `autoconnect` matches
the recorder's In ports to the existing `color_image`, `odom`, `cmd_vel`, and the
monitor's `status` output by `(name, type)`.

### 6. System prompt guidance
File: `dimos/agents/system_prompt.py`

Add a short `## Episode Recording` block under `# SKILL COORDINATION`: when the user
asks to record/capture a demo, call `start_recording` with a concise task label;
call `stop_recording` to keep a good run or `discard_recording` to drop a bad one.

### 7. Registry regen
Run `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate
`dimos/robot/all_blueprints.py` so `unitree-go2-agentic-record` is registered.
CI fails if this is stale.

## Data flow

```
agent-send "record this as a kitchen run"
  → LLM → start_recording("kitchen run")
  → EpisodeControlSpec.set_episode("start","kitchen run")  [RPC]
  → EpisodeMonitorModule emits EpisodeStatus(start, label)
  → Go2CollectionRecorder appends color_image/odom/cmd_vel/status → session.db
... agent drives (relative_move etc.) ...
agent-send "stop recording"
  → stop_recording() → set_episode("save") → EpisodeStatus(save) → boundary marked
Offline:
  dimos dataprep build --source <session>.db --config <cfg>.json
  → LeRobot v3.0 dataset (unchanged pipeline)
```

## Error handling
- `set_episode` with an invalid `event` → returns `"error: unknown event '<x>'..."`;
  the skill returns that string to the agent (no crash).
- `stop_recording`/`discard_recording` when idle → the state machine's existing
  no-op/guard behavior; `set_episode` returns an informative string.
- `start_recording` while already recording → existing auto-commit-then-start
  behavior; return string states that the previous take was saved.
- Missing recorder/monitor in the graph → `autoconnect` fails at build time (the
  intended, early failure), not at runtime. The record blueprint always includes
  both, so this only guards against misuse.

## Testing

### Unit tests
File: `dimos/learning/collection/test_episode_monitor.py` (extend or new)
- `set_episode("start", "task")` → state `recording`, `last_event="start"`,
  `task_label="task"`.
- `set_episode("save")` → state `idle`, `episodes_saved` incremented, success path.
- `set_episode("discard")` → state `idle`, `episodes_discarded` incremented.
- `set_episode("bogus")` → returns error string, no state change.
- start-while-recording auto-commits the prior take.

### Skill-container test
File: `dimos/agents/skills/test_episode_recording.py` (new), mirroring
`test_unitree_skill_container.py`:
- `StubEpisodeControl(Module)` with an `@rpc set_episode` recording calls.
- `MockedEpisodeRecordingSkillContainer` bound to the stub.
- `agent_setup`-driven test: `HumanMessage("record this as a test run")` →
  assert the returned string and that the stub received `set_episode("start","test run")`;
  then "stop recording" → `set_episode("save")`.

### Blueprint generation test
- `pytest dimos/robot/test_all_blueprints_generation.py` passes and includes
  `unitree-go2-agentic-record`.

### End-to-end verification (the acceptance test)
Runs against the MuJoCo sim with a funded OpenAI key in `.env`:
1. Launch: `dimos --simulation --viewer none run unitree-go2-agentic-record`.
   Wait for `Discovered tools from MCP server` and confirm `record` skills appear
   in `dimos mcp list-tools` (`start_recording`, `stop_recording`, `discard_recording`).
2. `dimos agent-send "start recording a short forward-walk demo called walk_test"`
   → confirm an `EpisodeStatus(start, task_label="walk_test")` on the `status`
   stream (via `dimos topic echo`) and a new `session_go2_*.db` file created.
3. `dimos agent-send "walk forward 1 meter"` → robot moves; `cmd_vel`/`odom`
   flowing (already verified working earlier).
4. `dimos agent-send "stop recording"` → confirm `EpisodeStatus(save)`.
5. Inspect the DB: it contains streams `color_image`, `odom`, `cmd_vel`, `status`
   with a start→save boundary (a small read-only check script, or
   `dimos dataprep inspect`).
6. `dimos dataprep build --source <session>.db --config <cfg>.json` → produces a
   LeRobot dataset directory with ≥1 episode. This closes the loop: agent →
   labeled episode → trainable dataset.

A scripted version of steps 1–5 (launch, drive via `agent-send`, assert on the DB)
is the automated e2e check; step 6 confirms export compatibility.

## Caveats (documented, out of scope, follow-ups)
- **Frame drop under load:** `Recorder` dispatches via `process_observable`, which
  coalesces to the latest message; high-rate camera/lidar frames can be dropped.
  Acceptable for a v1 demo; a queued-capture path is a follow-up.
- **Action fidelity:** `cmd_vel` is a reasonable action channel in sim; real-robot
  VLA training will eventually want motor-level `lowcmd`. Follow-up.
- **Single active episode:** the state machine supports one take at a time; nested
  or concurrent episodes are not supported (and not requested).

## Files touched (summary)
- edit: `dimos/learning/collection/episode_monitor.py` (add `set_episode` @rpc)
- edit: `dimos/agents/system_prompt.py` (recording guidance)
- new:  `dimos/learning/collection/episode_monitor_spec.py`
- new:  `dimos/learning/collection/go2_recorder.py`
- new:  `dimos/agents/skills/episode_recording.py`
- new:  `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_record.py`
- new:  `dimos/agents/skills/test_episode_recording.py`
- edit/new: `dimos/learning/collection/test_episode_monitor.py`
- regen: `dimos/robot/all_blueprints.py` (via test, do not hand-edit)
