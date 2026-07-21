# Agent-Triggered Episode Recording — Implementation Plan

> **For agentic workers:** implement with TDD (superpowers:test-driven-development). Steps use checkbox (`- [ ]`) syntax.

**Goal:** Let the DimOS agent start / save / discard a labeled training-data episode by voice, on an opt-in Go2 blueprint, feeding the existing dataprep→LeRobot exporter.

**Architecture:** Add a runtime `set_episode` RPC to the existing `EpisodeMonitorModule` (drives its start/save/discard state machine + a per-episode `task_label`); expose it to the agent through a new `EpisodeRecordingSkillContainer` injected via `EpisodeControlSpec`; record the Go2 streams with a new `Go2CollectionRecorder`; wire all three onto `unitree_go2_agentic` in a new `unitree-go2-agentic-record` blueprint. Reuse `dataprep` unchanged.

**Tech Stack:** Python 3.12, DimOS module/blueprint/skill system, memory2 Recorder (SQLite), pytest, mypy strict.

**Spec:** `docs/superpowers/specs/2026-07-20-agent-episode-recording-design.md`

---

## Task 1: Per-episode label + `set_episode` RPC on `EpisodeMonitorModule`

**Files:**
- Modify: `dimos/learning/collection/episode_monitor.py`
- Test: `dimos/learning/collection/test_episode_monitor.py` (extend; mirror `make_monitor` fixture)

- [ ] **Step 1 — failing tests** (append to `test_episode_monitor.py`):
```python
def test_set_episode_start_sets_label(make_monitor):
    m = make_monitor()
    msg = m.set_episode("start", "kitchen run")
    ev = _events(m)[-1]
    assert ev.last_event == "start" and ev.state == "recording"
    assert ev.task_label == "kitchen run"
    assert "kitchen run" in msg

def test_set_episode_save_then_reset_label(make_monitor):
    m = make_monitor()
    m.set_episode("start", "kitchen run")
    m.set_episode("save")
    ev = _events(m)[-1]
    assert ev.last_event == "save" and ev.state == "idle"
    assert ev.episodes_saved == 1
    assert ev.task_label == "kitchen run"       # save event still carries the take's label
    # next take with no label falls back to the config default (None here)
    m.set_episode("start")
    assert _events(m)[-1].task_label is None

def test_set_episode_discard(make_monitor):
    m = make_monitor()
    m.set_episode("start", "bad take")
    m.set_episode("discard")
    ev = _events(m)[-1]
    assert ev.last_event == "discard" and ev.episodes_discarded == 1

def test_set_episode_invalid_event(make_monitor):
    m = make_monitor()
    msg = m.set_episode("bogus")
    assert "error" in msg.lower()
    assert _events(m) == []   # no transition emitted
```

- [ ] **Step 2 — run, expect fail:** `uv run pytest dimos/learning/collection/test_episode_monitor.py -k set_episode -q` → FAIL (`set_episode` missing).

- [ ] **Step 3 — implement** in `episode_monitor.py`:
  - add `from typing import ..., cast`
  - in `__init__`: `self._task_label: str | None = self.config.default_task_label`
  - in `_snapshot`: `task_label=self._task_label` (was `self.config.default_task_label`)
  - in `_transition`, after `status = self._snapshot(event, ts)` (still under lock): reset on terminal events
    ```python
    if event in ("save", "discard"):
        self._task_label = self.config.default_task_label
    ```
  - add RPC:
    ```python
    @rpc
    def set_episode(self, event: str, task_label: str | None = None) -> str:
        """Drive the episode state machine from an external caller (e.g. the agent).

        event: "start", "save", or "discard". task_label applies on "start".
        Returns a human-readable status string.
        """
        if event not in ("start", "save", "discard"):
            return f"error: unknown episode event '{event}'; use start, save, or discard."
        if event == "start" and task_label is not None:
            with self._lock:
                self._task_label = task_label
        self._transition(cast("EpisodeCommand", event), time.time())
        with self._lock:
            saved, discarded, label = self._saved, self._discarded, self._task_label
        if event == "start":
            return f"Recording started (task: {task_label})." if task_label else "Recording started."
        if event == "save":
            return f"Episode saved. Total saved this session: {saved}."
        return f"Episode discarded. Total discarded this session: {discarded}."
    ```

- [ ] **Step 4 — run, expect pass:** `uv run pytest dimos/learning/collection/test_episode_monitor.py -q` (all, incl. existing button tests → label now `None` by default, unchanged).

- [ ] **Step 5 — commit:** `feat(collection): add set_episode RPC + per-episode task_label to EpisodeMonitor`

## Task 2: `EpisodeControlSpec`

**Files:**
- Create: `dimos/learning/collection/episode_monitor_spec.py`

- [ ] **Step 1 — implement** (mirror `navigation_spec.py`; no test needed — pure Protocol, exercised in Task 3/5):
```python
from typing import Protocol
from dimos.spec.utils import Spec

class EpisodeControlSpec(Spec, Protocol):
    def set_episode(self, event: str, task_label: str | None = None) -> str: ...
```
- [ ] **Step 2 — verify import:** `uv run python -c "from dimos.learning.collection.episode_monitor_spec import EpisodeControlSpec"`
- [ ] **Step 3 — commit:** `feat(collection): add EpisodeControlSpec for RPC injection`

## Task 3: `EpisodeRecordingSkillContainer`

**Files:**
- Create: `dimos/agents/skills/episode_recording.py`
- Test: `dimos/agents/skills/test_episode_recording.py` (mirror `test_unitree_skill_container.py` pure-unit style — a `StubEpisodeMonitor(Module)` with `@rpc set_episode` and a built blueprint via a lightweight coordinator, OR direct-instantiation unit test of the container's skill methods against a stub bound to `_episode`).

- [ ] **Step 1 — failing test:** build the container + a `StubEpisodeMonitor` via `agent_setup`-style coordinator is heavy (needs LLM fixture); instead unit-test the skill methods directly by constructing the container with `_episode` set to a stub that records calls, asserting return strings and forwarded args. (See `test_did_you_mean` for pure-unit precedent.)
```python
class _StubControl:
    def __init__(self): self.calls = []
    def set_episode(self, event, task_label=None):
        self.calls.append((event, task_label)); return "ok"

def test_start_recording_forwards_label():
    c = _build_container_with(_StubControl())  # helper patches _episode
    out = c.start_recording("kitchen run")
    assert c._episode.calls[-1] == ("start", "kitchen run")
```
  (Construction helper mirrors `make_monitor`'s boot-patching so `Module.__init__` doesn't start a real loop/RPC.)

- [ ] **Step 2 — run, expect fail.**
- [ ] **Step 3 — implement** `episode_recording.py`:
```python
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.learning.collection.episode_monitor_spec import EpisodeControlSpec

class EpisodeRecordingSkillContainer(Module):
    _episode: EpisodeControlSpec

    @rpc
    def start(self) -> None: super().start()
    @rpc
    def stop(self) -> None: super().stop()

    @skill
    def start_recording(self, task_label: str) -> str:
        """Begin recording a training-data episode for later model training.

        Call this when the user asks to record / capture / demonstrate a task.
        Args:
            task_label: short description of the task being demonstrated,
                e.g. "navigate to the kitchen".
        Example: start_recording("walk to the door")
        """
        return self._episode.set_episode("start", task_label)

    @skill
    def stop_recording(self) -> str:
        """Stop and SAVE the current episode as a successful demonstration."""
        return self._episode.set_episode("save")

    @skill
    def discard_recording(self) -> str:
        """Stop and DISCARD the current episode (a bad or unwanted take)."""
        return self._episode.set_episode("discard")
```
- [ ] **Step 4 — run, expect pass.**
- [ ] **Step 5 — commit:** `feat(skills): add EpisodeRecordingSkillContainer`

## Task 4: `Go2CollectionRecorder`

**Files:**
- Create: `dimos/learning/collection/go2_recorder.py`

- [ ] **Step 1 — implement** (mirror `Go2Memory`; odom pose_setter avoids tf warnings; no separate test — exercised by blueprint gen (Task 6) + e2e (Task 7)):
```python
from __future__ import annotations
from pathlib import Path
from dimos.core.stream import In
from dimos.learning.collection.episode_monitor import EpisodeStatus
from dimos.memory2.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image

class Go2CollectionRecorderConfig(RecorderConfig):
    db_path: str | Path = "recording_go2_collection.db"

class Go2CollectionRecorder(Recorder):
    """Records Go2 camera + pose + action + episode-boundary streams for VLA training."""
    config: Go2CollectionRecorderConfig
    color_image: In[Image]      # observation (camera)
    odom: In[PoseStamped]       # observation (pose)
    cmd_vel: In[Twist]          # action channel
    status: In[EpisodeStatus]   # episode start/save/discard segmentation
    _last_odom_pose: Pose | None = None

    @pose_setter_for("odom")
    async def _odom_pose(self, msg: PoseStamped) -> Pose | None:
        self._last_odom_pose = msg
        return self._last_odom_pose
```
- [ ] **Step 2 — verify import:** `uv run python -c "from dimos.learning.collection.go2_recorder import Go2CollectionRecorder"`
- [ ] **Step 3 — commit:** `feat(collection): add Go2CollectionRecorder`

## Task 5: `unitree-go2-agentic-record` blueprint

**Files:**
- Create: `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_record.py`

- [ ] **Step 1 — implement:**
```python
from dimos.agents.skills.episode_recording import EpisodeRecordingSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.learning.collection.blueprint import _session_db
from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
from dimos.learning.collection.go2_recorder import Go2CollectionRecorder
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import unitree_go2_agentic

unitree_go2_agentic_record = autoconnect(
    unitree_go2_agentic,
    Go2CollectionRecorder.blueprint(db_path=_session_db("go2")),
    EpisodeMonitorModule.blueprint(),
    EpisodeRecordingSkillContainer.blueprint(),
)
```
- [ ] **Step 2 — verify import:** `uv run python -c "from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic_record import unitree_go2_agentic_record"`
- [ ] **Step 3 — commit:** `feat(go2): add unitree-go2-agentic-record blueprint`

## Task 6: System prompt + registry regen

**Files:**
- Modify: `dimos/agents/system_prompt.py` (add `## Episode Recording` under `# SKILL COORDINATION`)
- Regen: `dimos/robot/all_blueprints.py` (via test)

- [ ] **Step 1 — prompt:** add a short block: when the user asks to record/capture/demonstrate a task, call `start_recording` with a concise label; `stop_recording` to keep a good run, `discard_recording` to drop a bad one.
- [ ] **Step 2 — regen registry:** `uv run pytest dimos/robot/test_all_blueprints_generation.py -q` (regenerates `all_blueprints.py`; expect it to include `unitree-go2-agentic-record`).
- [ ] **Step 3 — verify listing:** `uv run dimos list | grep unitree-go2-agentic-record`
- [ ] **Step 4 — commit:** `feat: register record blueprint + prompt guidance`

## Task 7: Full checks + end-to-end sim verification

- [ ] **Step 1 — full fast test suite:** `uv run pytest dimos/learning/collection dimos/agents/skills -q` → PASS.
- [ ] **Step 2 — mypy on touched files:** `uv run mypy dimos/learning/collection dimos/agents/skills/episode_recording.py dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_record.py` → clean.
- [ ] **Step 3 — launch e2e (sim, funded key in `.env`):**
  `dimos --simulation --viewer none run unitree-go2-agentic-record` (background). Wait for `Discovered tools from MCP server`.
- [ ] **Step 4 — confirm skills exposed:** `dimos mcp list-tools | grep -E "start_recording|stop_recording|discard_recording"` → all three present.
- [ ] **Step 5 — drive:** `dimos agent-send "start recording a demo called walk_test"`; confirm `EpisodeStatus(start, task_label="walk_test")` (via `dimos topic echo /status` or logs) and a new `session_go2_*.db` under `STATE_DIR/recordings/`.
- [ ] **Step 6 — move + stop:** `dimos agent-send "walk forward 1 meter"` then `dimos agent-send "stop recording"`; confirm `EpisodeStatus(save)`.
- [ ] **Step 7 — verify DB + export:** read-only check that the DB has streams `color_image`, `odom`, `cmd_vel`, `status` with a start→save boundary; then `dimos dataprep build --source <session>.db --config <cfg>.json` produces a LeRobot dataset with ≥1 episode. (If a config is required, use/author a minimal one mapping `color_image`→image, `odom`→state, `cmd_vel`→action.)
- [ ] **Step 8 — stop sim:** `pkill -f dimos`.
- [ ] **Step 9 — final commit** of any e2e helper/config: `test: e2e verification for agent episode recording`.

---

## Notes / caveats (from spec)
- Recorder uses latest-coalescing dispatch → high-rate frames can drop; acceptable v1.
- `cmd_vel` is the sim action channel; real-robot `lowcmd` is a follow-up.
- Export pipeline (`dataprep`) is unchanged and out of scope.
