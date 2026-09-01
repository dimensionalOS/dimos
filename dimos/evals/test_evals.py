# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline unit tests: scorers, cases, environments, agents, runner artifacts.

No network, no robot, no LLM — chat models are fakes, the MCP tool set is a
stub, and the environment in runner tests is a plain object satisfying the
Environment protocol structurally.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any

from langchain_core.callbacks import CallbackManagerForLLMRun
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, ToolMessage
from langchain_core.outputs import ChatGeneration, ChatResult
import numpy as np
import pytest

from dimos.evals.agents.blind import BLIND_BLOCK, Blind
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.mcp_client import McpClientAgent
from dimos.evals.agents.pi import Pi
from dimos.evals.agents.question_answer import QuestionAnswer
from dimos.evals.environments.dataset import Dataset
from dimos.evals.environments.image_file import ImageFile
from dimos.evals.environments.sim import Sim
from dimos.evals.runner import EvalRunner
from dimos.evals.scorers import (
    choice,
    exact,
    final,
    first_number,
    floor,
    mean,
    ramp,
    within,
    yes_no,
)
from dimos.evals.types import (
    EvalCase,
    Observation,
    ObservationResult,
    Outcome,
    RunningEnvironment,
    ToolCall,
    Trajectory,
    recording,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3


def _pose(x: float, y: float) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
    )


def _open_store(path: Path) -> Any:
    from dimos.memory.store.sqlite import SqliteStore

    try:
        return SqliteStore(path=str(path))
    except Exception as e:  # pragma: no cover — sqlite-vec unavailable platforms
        pytest.skip(f"SqliteStore unavailable: {e}")


@pytest.fixture
def dataset(tmp_path: Path) -> str:
    """A tiny on-disk memory dataset: 5 odom poses walking 4m in +x over 4s."""
    path = tmp_path / "tiny.db"
    store = _open_store(path)
    stream = store.stream("odom", PoseStamped)
    for i in range(5):
        stream.append(_pose(float(i), 0.0), ts=1000.0 + i)
    store.stop()
    return str(path)


class SpyChat(BaseChatModel):
    """Captures the exact messages an agent sends; replies with a constant."""

    reply: str = "42"
    seen: list[list[BaseMessage]] = []

    @property
    def _llm_type(self) -> str:
        return "spy"

    def _generate(
        self,
        messages: list[BaseMessage],
        stop: list[str] | None = None,
        run_manager: CallbackManagerForLLMRun | None = None,
        **kwargs: Any,
    ) -> ChatResult:
        self.seen.append(messages)
        return ChatResult(generations=[ChatGeneration(message=AIMessage(content=self.reply))])


GO2_STACK = "unitree-go2 mcp-server unitree-skill-container"


def _sim(**kwargs: Any) -> Sim:
    return Sim(blueprint=GO2_STACK, **kwargs)


def _trajectory(answer: str, raw: Path, timed_out: bool = False) -> Trajectory:
    trajectory = TrajectoryBuilder("?", name="fake", model="fake")
    trajectory.step(message=answer, request=raw / "r", response=raw / "s")
    return trajectory.build("timeout" if timed_out else "answer")


def _assert_atif(doc: dict[str, Any]) -> None:
    """What Harbor's validator checks: required fields, 1-based sequential
    step ids, observations answering calls the step made, no nulls."""
    assert doc["schema_version"] == "ATIF-v1.7"
    assert {"name", "version", "model_name"} <= doc["agent"].keys()
    assert [s["step_id"] for s in doc["steps"]] == list(range(1, len(doc["steps"]) + 1))
    for step in doc["steps"]:
        assert {"timestamp", "source", "message"} <= step.keys()
        called = {c["tool_call_id"] for c in step.get("tool_calls", [])}
        assert all(
            r["source_call_id"] in called for r in step.get("observation", {}).get("results", [])
        )
    assert "final_metrics" in doc and "null" not in json.dumps(doc)


class FakeEnvironment:
    """A frozen recording with one declared artifact; records lifecycle calls."""

    artifacts = ("recording",)
    has_robot = False

    def __init__(self, path: Path, calls: list[str]) -> None:
        self.path = path
        self.calls = calls
        self.settled_budget: float | None = None

    def preflight(self, agent: Any) -> None:
        self.calls.append("preflight")

    def start(self, modules: str) -> RunningEnvironment:
        self.calls.append("start")
        return RunningEnvironment(mcp_url="", streams=(), artifacts={"recording": self.path})

    def settle(self, budget_s: float) -> None:
        self.calls.append("settle")
        self.settled_budget = budget_s

    def stop(self) -> None:
        self.calls.append("stop")


class FakeAgent:
    """Replies with a canned answer after an optional delay; can fail."""

    modules = ""

    def __init__(self, answer: str = "", delay_s: float = 0.0, fail: bool = False) -> None:
        self.answer, self.delay_s, self.fail = answer, delay_s, fail

    def preflight(self, environment: Any) -> None:
        pass

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        return ("fake_tool",)

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        time.sleep(min(self.delay_s, timeout_s))
        if self.fail:
            raise RuntimeError("boom")
        return _trajectory(self.answer, run_dir / "raw", timed_out=self.delay_s > timeout_s)


def _text(messages: list[BaseMessage]) -> str:
    blocks = [b for m in messages for b in (m.content if isinstance(m.content, list) else [m])]
    return " ".join(
        b["text"]
        if isinstance(b, dict) and b.get("type") == "text"
        else str(getattr(b, "content", ""))
        for b in blocks
    )


# -- scorers ------------------------------------------------------------------------


def test_scorer_math() -> None:
    assert exact(6, 6) == 1.0
    assert exact("yes", "no") == 0.0
    assert within(2.0)(10.0, 10.0) == 1.0
    assert within(2.0)(10.0, 11.0) == 0.5
    assert within(2.0)(10.0, 13.0) == 0.0
    assert ramp(0.0, band=2.0) == 1.0
    assert ramp(1.0, band=2.0) == 0.5
    assert ramp(5.0, band=2.0) == 0.0
    assert final([0.1, 0.9]) == 0.9
    assert floor([0.4, 0.2, 0.8]) == 0.2
    assert mean([0.0, 1.0]) == 0.5


def test_parsers() -> None:
    assert first_number("about 12.5 meters") == 12.5
    assert first_number("-3") == -3.0
    with pytest.raises(ValueError):
        first_number("none")
    assert yes_no("Yes, there is.") == "yes"
    assert yes_no("no") == "no"
    with pytest.raises(ValueError):
        yes_no("maybe")
    compass = choice(["north", "northeast", "east"])
    assert compass(" Northeast. ") == "northeast"
    assert compass("it drifts north, then finally east") == "east"  # last named wins
    assert compass("north-east") == "northeast"  # not "east"
    with pytest.raises(ValueError):
        compass("no idea")


# -- environments -------------------------------------------------------------------


def test_dataset_start_hands_out_the_selection(dataset: str) -> None:
    env = Dataset(dataset, select=(lambda s: s.streams.odom.limit(2),))
    running = env.start("")
    try:
        (odom,) = running.streams
        assert odom.name == "odom"
        observations = list(odom)
        assert [o.ts for o in observations] == [1000.0, 1001.0]
        assert observations[0].data.position.x == 0.0
        assert running.mcp_url == "" and not env.has_robot
        assert running.artifacts["recording"] == Path(dataset)
    finally:
        env.stop()


def test_dataset_preflight_reports_missing_stream(dataset: str) -> None:
    env = Dataset(dataset, select=(lambda s: s.streams.lidar.limit(1),))
    with pytest.raises(AttributeError, match="No stream 'lidar'"):
        env.preflight(QuestionAnswer())


def test_dataset_preflight_checks_added_modules(dataset: str) -> None:
    """A tool-using agent's modules become the launched stack, so preflight
    validates the names; adding modules to an attached dimos is a conflict."""
    with pytest.raises(ValueError, match="Unknown blueprint or module: 'no-such-module'"):
        Dataset(dataset).preflight(McpClientAgent(modules="no-such-module"))
    match = "already attaches to http://x/mcp; McpClientAgent also adds modules"
    with pytest.raises(RuntimeError, match=match):
        Dataset(dataset, mcp_url="http://x/mcp").preflight(McpClientAgent(modules="unitree-go2"))


def test_dataset_launches_the_agents_modules(dataset: str, monkeypatch: pytest.MonkeyPatch) -> None:
    """A frozen recording has no dimos, so a non-empty ``modules`` becomes its
    own launched stack — ``dimos run <modules>``, no simulator — waited on at
    the MCP url and torn down with the case."""
    from dimos.evals.environments.lib.launch import default_mcp_url

    calls: list[str] = []

    class FakeProc:
        simulator: str | None = "mujoco"
        demo_args: list[str] | None = None

        def start(self) -> None:
            calls.append(f"start [{' '.join(self.demo_args or [])}] simulator={self.simulator}")

        def stop(self) -> None:
            calls.append("stop")

    class FakeAdapter:
        def __init__(self, url: str) -> None:
            calls.append(f"wait {url}")

        def wait_for_ready(self, timeout: float, interval: float = 1.0) -> bool:
            return True

    monkeypatch.setattr("dimos.e2e_tests.dimos_cli_call.DimosCliCall", FakeProc)
    monkeypatch.setattr("dimos.agents.mcp.mcp_adapter.McpAdapter", FakeAdapter)
    env = Dataset(dataset)
    running = env.start("mcp-server mcp-client")
    env.stop()
    assert running.mcp_url == default_mcp_url()
    assert calls == [
        "start [run mcp-server mcp-client] simulator=None",
        f"wait {running.mcp_url}",
        "stop",
    ]


def test_image_file_environment(tmp_path: Path) -> None:
    from dimos.msgs.sensor_msgs.Image import Image

    path = tmp_path / "frame.png"
    Image.from_numpy(np.full((8, 8, 3), 200, dtype=np.uint8)).save(path)
    env = ImageFile(path)
    env.preflight(QuestionAnswer())
    running = env.start("")
    try:
        (image_stream,) = running.streams
        (obs,) = list(image_stream)
        assert obs.data.agent_encode()[0]["type"] == "image_url"
        assert running.artifacts == {"image": path}
    finally:
        env.stop()
    with pytest.raises(FileNotFoundError):
        ImageFile(tmp_path / "missing.png").preflight(QuestionAnswer())


def test_sim_is_not_a_case_without_its_stack() -> None:
    with pytest.raises(TypeError, match="blueprint"):
        Sim()  # type: ignore[call-arg]


def _driving_sim(poses: int, **kwargs: Any) -> tuple[Sim, threading.Thread]:
    """A Sim over a live store whose robot drives for *poses* samples, then rests."""
    from dimos.memory.store.memory import MemoryStore

    store = MemoryStore()
    odom = store.stream("odom", PoseStamped)
    odom.append(_pose(0.0, 0.0), ts=time.time())

    def drive() -> None:
        for i in range(poses):
            odom.append(_pose(0.2 * (i + 1), 0.0), ts=time.time())
            time.sleep(0.02)

    env = _sim(**kwargs)
    env._recording = store
    return env, threading.Thread(target=drive)


def test_sim_settle_waits_until_the_robot_is_at_rest() -> None:
    env, drive = _driving_sim(poses=15, at_rest_s=0.1, settle_poll_s=0.02)  # ~0.3s of motion
    drive.start()
    time.sleep(0.05)  # motion underway before settle first samples
    t0 = time.monotonic()
    env.settle(10.0)
    elapsed = time.monotonic() - t0
    drive.join(timeout=2.0)
    assert 0.3 <= elapsed < 5.0, "returns once motion ends, not at the budget"


def test_sim_settle_gives_up_at_the_budget() -> None:
    env, drive = _driving_sim(poses=1, at_rest_s=5.0, settle_poll_s=0.02)  # never satisfiable
    drive.start()
    t0 = time.monotonic()
    env.settle(0.3)
    elapsed = time.monotonic() - t0
    drive.join(timeout=2.0)
    assert 0.3 <= elapsed < 2.0, "a world that never settles is bounded by the budget"


def test_sim_settle_without_motion_data_returns_immediately() -> None:
    from dimos.memory.store.memory import MemoryStore

    env = _sim()
    t0 = time.monotonic()
    env.settle(10.0)  # not started: no recording
    env._recording = MemoryStore()
    env.settle(10.0)  # recording without an odom stream
    assert time.monotonic() - t0 < 1.0


# -- agent / environment mismatches fail in preflight, naming both sides --------------


def test_agent_preflight_mismatches(dataset: str, monkeypatch: pytest.MonkeyPatch) -> None:
    frozen = Dataset(dataset)
    with pytest.raises(RuntimeError, match="McpClientAgent needs a running McpClient"):
        McpClientAgent().preflight(frozen)
    McpClientAgent(modules="mcp-server mcp-client").preflight(frozen)  # brings its own
    # attach mode (nothing added) needs a dimos to attach to, else the turn never ends
    monkeypatch.setattr("dimos.core.run_registry.list_runs", lambda alive_only=True: [])
    with pytest.raises(RuntimeError, match="no dimos is running to attach to"):
        McpClientAgent().preflight(_sim(attach=True))


def test_a_limit_an_agent_cannot_honor_is_not_a_parameter_it_has() -> None:
    with pytest.raises(TypeError, match="max_steps"):
        QuestionAnswer(max_steps=3)  # type: ignore[call-arg]
    with pytest.raises(TypeError, match="model"):
        McpClientAgent(model="gpt-4o")  # type: ignore[call-arg]


# -- agents -------------------------------------------------------------------------


def test_question_answer_encodes_the_recording_into_one_call(dataset: str, tmp_path: Path) -> None:
    spy = SpyChat(reply="4")
    spy.seen.clear()
    env = Dataset(dataset)
    running = env.start("")
    try:
        trajectory = QuestionAnswer(chat_model=spy).run(
            "how far?", running, tmp_path / "case", timeout_s=60.0
        )
    finally:
        env.stop()

    assert trajectory.final_answer == "4" and trajectory.extra.ended_by == "answer"
    assert trajectory.agent.model_name == "SpyChat", (
        "an injected model is recorded, not the default"
    )
    assert [s.source for s in trajectory.steps] == ["user", "agent"] and len(spy.seen) == 1
    assert trajectory.steps[0].message == "how far?"
    text = _text(spy.seen[0])
    assert "stream 'odom'" in text and "4.000" in text and "how far?" in text
    # every call is recorded whole; a fake model has no wire, so normalized
    extra = trajectory.steps[1].extra
    assert extra and extra.request.exists() and extra.response.exists()
    assert json.loads(extra.request.read_text())["normalized"] is True
    assert extra.request.parent == tmp_path / "case" / "raw"


def test_question_answer_refuses_an_empty_recording(tmp_path: Path) -> None:
    running = RunningEnvironment(mcp_url="", streams=(), artifacts={})
    with pytest.raises(RuntimeError, match="would be blind"):
        QuestionAnswer(chat_model=SpyChat()).run("?", running, tmp_path, timeout_s=60.0)


def test_blind_never_reads_the_recording(dataset: str, tmp_path: Path) -> None:
    spy = SpyChat(reply="7")
    spy.seen.clear()
    env = Dataset(dataset)
    running = env.start("")
    try:
        trajectory = Blind(chat_model=spy).run("how far?", running, tmp_path, timeout_s=60.0)
    finally:
        env.stop()
    text = _text(spy.seen[0])
    assert BLIND_BLOCK["text"] in text and "odom" not in text
    assert trajectory.final_answer == "7"


# -- runner -------------------------------------------------------------------------


def test_runner_uses_unique_directory_when_timestamps_match(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr("dimos.evals.runner.time.strftime", lambda _: "run-20260831-120000-")
    first = EvalRunner(out_dir=tmp_path)
    second = EvalRunner(out_dir=tmp_path)

    first.run([], FakeAgent())
    second.run([], FakeAgent())

    assert first.run_dir != second.run_dir
    assert first.run_dir.parent == second.run_dir.parent == tmp_path
    assert first.run_dir.name.startswith("run-20260831-120000-")
    assert second.run_dir.name.startswith("run-20260831-120000-")


def test_runner_end_to_end_offline(dataset: str, tmp_path: Path) -> None:
    from dimos.evals.runner import EvalRunner, summarize

    calls: list[str] = []
    env = FakeEnvironment(Path(dataset), calls)
    cases = [
        EvalCase(
            id="disp",
            inputs="straight-line distance in meters?",
            environment=env,
            grade=lambda o: within(1.0)(4.0, first_number(o.trajectory.final_answer)),
        ),
        EvalCase(  # grade failure -> error result, run survives
            id="unparseable",
            inputs="?",
            environment=env,
            grade=lambda o: exact(1.0, first_number("no numbers here")),
        ),
        EvalCase(  # preflight failure -> error result, run survives
            id="missing_stream",
            inputs="?",
            environment=Dataset(dataset, select=(lambda s: s.streams.lidar,)),
            grade=lambda o: 1.0,
        ),
    ]
    runner = EvalRunner(out_dir=tmp_path / "evals")
    results = runner.run(cases, FakeAgent(answer="4.0"))

    by_id = {r.case_id: r for r in results}
    assert by_id["disp"].passed and by_id["disp"].score == 1.0
    assert by_id["disp"].final_answer == "4.0" and by_id["disp"].steps == 2  # instruction + call
    assert by_id["disp"].ended_by == "answer"
    assert "ValueError" in by_id["unparseable"].error
    assert by_id["missing_stream"].error.startswith("preflight:")

    s = summarize(results)
    assert s.n == 3 and s.errors == 2

    run_dir = runner.run_dir
    lines = (run_dir / "results.jsonl").read_text().strip().splitlines()
    assert len(lines) == 3
    assert [json.loads(line)["case_id"] for line in lines] == [
        "disp",
        "unparseable",
        "missing_stream",
    ]
    summary = json.loads((run_dir / "summary.json").read_text())
    assert summary["manifest"] == "manifest.json"
    assert "agent" not in summary
    manifest = json.loads((run_dir / "manifest.json").read_text())
    assert manifest["selection"]["case_ids"] == ["disp", "unparseable", "missing_stream"]
    assert manifest["runner"] == {"threshold": 1.0, "strict": False}
    assert manifest["source"] == {"kind": "unavailable"}
    trajectory = json.loads(Path(by_id["disp"].trajectory).read_text())
    _assert_atif(trajectory)
    assert trajectory["agent"]["tool_definitions"] == [{"name": "fake_tool"}]
    assert (
        trajectory["steps"][1]["message"] == "4.0" and trajectory["extra"]["ended_by"] == "answer"
    )


def test_manifest_exists_when_strict_preflight_fails(dataset: str, tmp_path: Path) -> None:
    case = EvalCase(
        id="missing_stream",
        inputs="?",
        environment=Dataset(dataset, select=(lambda store: store.streams.lidar,)),
        grade=lambda outcome: 1.0,
    )
    runner = EvalRunner(out_dir=tmp_path, strict=True)

    with pytest.raises(AttributeError, match="lidar"):
        runner.run([case], FakeAgent())

    manifest = json.loads((runner.run_dir / "manifest.json").read_text())
    assert manifest["selection"]["case_ids"] == ["missing_stream"]
    assert not (runner.run_dir / "missing_stream").exists()


@pytest.mark.parametrize(
    "case_id",
    [
        "",
        "/tmp/escape",
        "../escape",
        "a/b",
        r"a\b",
        ".",
        "..",
        "manifest.json",
        "summary.json",
        "results.jsonl",
    ],
)
def test_runner_rejects_unsafe_and_artifact_case_ids(case_id: str, tmp_path: Path) -> None:
    case = EvalCase(
        id=case_id,
        inputs="?",
        environment=FakeEnvironment(tmp_path / "artifact", []),
        grade=lambda outcome: 1.0,
    )
    out_dir = tmp_path / "runs"

    with pytest.raises(ValueError, match="unsafe eval case ID"):
        EvalRunner(out_dir=out_dir).run([case], FakeAgent())

    assert not out_dir.exists()
    assert not (tmp_path / "escape").exists()


def test_runner_rejects_duplicate_case_ids_before_preflight(tmp_path: Path) -> None:
    calls: list[str] = []
    case = EvalCase(
        id="duplicate",
        inputs="?",
        environment=FakeEnvironment(tmp_path / "artifact", calls),
        grade=lambda outcome: 1.0,
    )

    with pytest.raises(ValueError, match="duplicate eval case IDs"):
        EvalRunner(out_dir=tmp_path / "runs").run([case, case], FakeAgent())

    assert calls == []


def test_programmatic_run_manifest_marks_provenance_unavailable(tmp_path: Path) -> None:
    runner = EvalRunner(out_dir=tmp_path)
    runner.run([], FakeAgent())

    manifest = json.loads((runner.run_dir / "manifest.json").read_text())
    assert manifest["source"] == {"kind": "unavailable"}
    assert manifest["agent"] is None


def test_runner_stops_the_environment_before_grading_and_on_failure(
    dataset: str, tmp_path: Path
) -> None:
    from dimos.evals.runner import EvalRunner

    calls: list[str] = []
    env = FakeEnvironment(Path(dataset), calls)

    def grade(o: Outcome) -> float:
        calls.append("grade")
        return 1.0

    case = EvalCase(id="c", inputs="x", environment=env, grade=grade)
    runner = EvalRunner(out_dir=tmp_path)
    runner.run([case], FakeAgent(answer="ok"))
    assert calls.index("stop") < calls.index("grade"), "the agent phase ends before grading"

    calls.clear()
    result = runner.run([case], FakeAgent(fail=True))[0]
    assert "boom" in result.error and result.score == 0.0
    assert calls == ["preflight", "start", "stop"]


def test_runner_gives_the_world_the_budget_the_agent_did_not_use(
    dataset: str, tmp_path: Path
) -> None:
    from dimos.evals.runner import EvalRunner

    calls: list[str] = []
    env = FakeEnvironment(Path(dataset), calls)
    case = EvalCase(id="c", inputs="x", environment=env, grade=lambda o: 1.0, timeout_s=30.0)
    EvalRunner(out_dir=tmp_path).run([case], FakeAgent(answer="ok"))

    assert calls.index("start") < calls.index("settle") < calls.index("stop")
    assert env.settled_budget == pytest.approx(30.0, abs=1.0)  # FakeAgent answers at once


def test_runner_timeout_marks_the_trajectory(dataset: str, tmp_path: Path) -> None:
    from dimos.evals.runner import EvalRunner

    env = FakeEnvironment(Path(dataset), [])
    case = EvalCase(id="slow", inputs="x", environment=env, grade=lambda o: 0.5, timeout_s=0.2)
    result = EvalRunner(out_dir=tmp_path).run([case], FakeAgent(delay_s=5.0))[0]
    assert result.ended_by == "timeout" and not result.error
    assert result.score == 0.5 and not result.passed  # the world is still graded


def test_runner_missing_artifact_is_an_error(tmp_path: Path) -> None:
    from dimos.evals.runner import EvalRunner

    graded: list[Outcome] = []
    env = FakeEnvironment(tmp_path / "never-written.db", [])
    case = EvalCase(id="c", inputs="x", environment=env, grade=lambda o: graded.append(o) or 1.0)
    result = EvalRunner(out_dir=tmp_path).run([case], FakeAgent(answer="ok"))[0]
    assert result.error == "missing artifacts: ['recording']" and not graded


def test_recording_helper_opens_the_artifact(dataset: str) -> None:
    store = recording(
        Outcome(trajectory=_trajectory("", Path()), artifacts={"recording": Path(dataset)})
    )
    try:
        assert store.streams.odom.last().data.position.x == 4.0
    finally:
        store.stop()


def test_count_rooms_grader_scores_reply_and_coverage(tmp_path: Path) -> None:
    """Half credit for the exact room count, half for the fraction of room
    points the recorded odometry approached; an unparseable reply loses the
    count half but the world still scores."""
    from dimos.evals.suites.dimsim_pointcloud_mapping import (
        N_ROOMS,
        ROOMS,
        VISIT_RADIUS_M,
        grade_rooms,
    )

    def written(db: Path, points: list[tuple[float, float]]) -> Path:
        store = _open_store(db)
        stream = store.stream("odom", PoseStamped)
        for i, (x, y) in enumerate(points):
            stream.append(_pose(x, y), ts=1000.0 + i)
        store.stop()
        return db

    def score(db: Path, answer: str) -> float:
        outcome = Outcome(trajectory=_trajectory(answer, tmp_path), artifacts={"recording": db})
        return grade_rooms(outcome)

    rooms = list(ROOMS.values())
    two = written(tmp_path / "two.db", [(x + VISIT_RADIUS_M / 2, y) for x, y in rooms[:2]])
    coverage = 0.5 * 2 / N_ROOMS
    assert score(two, str(N_ROOMS)) == pytest.approx(0.5 + coverage)
    assert score(two, f"{N_ROOMS} rooms, I think") == pytest.approx(0.5 + coverage)
    assert score(two, str(N_ROOMS + 1)) == pytest.approx(coverage), "wrong count"
    assert score(two, "no idea") == pytest.approx(coverage), "unparseable reply"

    every = written(tmp_path / "every.db", rooms)
    assert score(every, str(N_ROOMS)) == 1.0
    assert score(written(tmp_path / "still.db", [(50.0, 50.0)]), str(N_ROOMS)) == 0.5


def test_suites_and_agents_importable() -> None:
    """Modules construct without data or network (lambdas stay lazy)."""
    from dimos.evals.cli import load_agent
    from dimos.evals.module import list_agents
    from dimos.evals.suites import (
        dimsim_house,
        dimsim_pointcloud_mapping,
        examples,
        go2_smoke,
        go2_vqa,
    )

    for module in (examples, go2_smoke, go2_vqa, dimsim_house, dimsim_pointcloud_mapping):
        assert module.SUITE, module.__name__
    agents = list_agents()
    assert {m.rsplit(".", 1)[1] for m in agents} == {"question_answer", "blind", "mcp_client", "pi"}
    for module in agents:
        assert callable(load_agent(module).run), module


def test_load_agent_is_the_module_plus_set_overrides() -> None:
    """``--agent`` names a module with one agent class; ``--set`` values are
    JSON where they parse, else text; a field the agent lacks is a TypeError."""
    from dimos.evals.cli import load_agent

    agent = load_agent(
        "dimos.evals.agents.pi", ["max_steps=null", "modules=rangefinder-skill", "model=x"]
    )
    assert (type(agent).__name__, agent.max_steps, agent.modules, agent.model) == (
        "Pi", None, "rangefinder-skill", "x"
    )  # fmt: skip
    assert load_agent("dimos.evals.agents.pi", ["max_steps=3"]).max_steps == 3
    with pytest.raises(TypeError, match="max_steps"):
        load_agent("dimos.evals.agents.blind", ["max_steps=3"])
    with pytest.raises(TypeError, match="0 agents"):
        load_agent("dimos.evals.agents.lib.chat")


@pytest.mark.parametrize("goes_idle", [True, False])
def test_mcp_client_agent_drives_a_turn_over_real_transports(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, goes_idle: bool
) -> None:
    """The production agent points the McpClient's raw capture at run_dir/raw,
    publishes on /human_input, reads the turn back on /agent until
    /agent_idle or its budget runs out, and links every model call to the
    McpClient's trace files (the message must arrive with no flush sleep: LCM
    publish is a synchronous send)."""
    from dimos.core.transport_factory import make_transport

    trace_dir = tmp_path / "case" / "raw"
    trace_dir.mkdir(parents=True)

    repointed: list[str] = []
    app = SimpleNamespace(
        McpClient=SimpleNamespace(set_trace_dir=repointed.append), stop=lambda: None
    )
    monkeypatch.setattr("dimos.porcelain.dimos.Dimos.connect", lambda: app)

    human, agent_t, idle = (
        make_transport("/human_input"),
        make_transport("/agent"),
        make_transport("/agent_idle"),
    )
    for t in (human, agent_t, idle):
        t.start()

    def fake_mcp_client(text: str) -> None:
        idle.publish(False)
        agent_t.publish(HumanMessage(content=text))
        for i in range(2):
            (trace_dir / f"{i:03d}-request.json").write_text("{}")
            (trace_dir / f"{i:03d}-response.json").write_text("{}")
        agent_t.publish(
            AIMessage(
                content="",
                tool_calls=[{"name": "move_to", "args": {"x": 1.0}, "id": "c1"}],
                usage_metadata={"input_tokens": 10, "output_tokens": 2, "total_tokens": 12},
            )
        )
        agent_t.publish(ToolMessage(content="arrived", tool_call_id="c1"))
        agent_t.publish(AIMessage(content="I am at the bed"))
        if goes_idle:
            idle.publish(True)

    unsubscribe = human.subscribe(
        lambda msg: threading.Thread(target=fake_mcp_client, args=(msg,)).start()
    )
    try:
        env = RunningEnvironment(mcp_url="http://localhost:1/mcp", streams=(), artifacts={})
        agent = McpClientAgent()
        trajectory = agent.run(
            "go to the bed", env, tmp_path / "case", timeout_s=10.0 if goes_idle else 0.5
        )
    finally:
        unsubscribe()
        for t in (human, agent_t, idle):
            t.stop()

    assert repointed == [str(trace_dir)]
    assert trajectory.extra.ended_by == ("answer" if goes_idle else "timeout")
    assert trajectory.final_answer == "I am at the bed"
    assert len(trajectory.steps) == 3 and trajectory.final_metrics.total_prompt_tokens == 10
    user, first, last = trajectory.steps
    assert user.source == "user" and user.message == "go to the bed"
    assert first.tool_calls == (
        ToolCall(tool_call_id="c1", function_name="move_to", arguments={"x": 1.0}),
    )
    assert first.observation == Observation(
        results=(ObservationResult(source_call_id="c1", content="arrived"),)
    )
    assert last.extra and last.extra.request == tmp_path / "case" / "raw" / "001-request.json"
    assert last.extra.request.exists()


# -- Pi -------------------------------------------------------------------------------

FAKE_PI = """\
import json, os, sys, time, urllib.request
from pathlib import Path

args = sys.argv[1:]
agent_dir = Path(os.environ["PI_CODING_AGENT_DIR"])
base = json.loads((agent_dir / "models.json").read_text())["providers"]["dimos"]["baseUrl"]
model = args[args.index("--model") + 1].split("/", 1)[1]  # pi strips the provider prefix
tool_steps = int(Path(__file__).with_name("tool_steps").read_text())
step_delay = Path(__file__).with_name("step_delay")


def call(n):
    req = urllib.request.Request(
        base + "/responses",
        data=json.dumps({"model": model, "n": n}).encode(),
        headers={"Content-Type": "application/json", "Authorization": "Bearer " + os.environ["OPENAI_API_KEY"]},
    )
    urllib.request.urlopen(req).read()


def emit(event):
    print(json.dumps(event), flush=True)


def assistant(content, stop):
    return {"role": "assistant", "model": "gpt-fake", "stopReason": stop, "content": content,
            "usage": {"input": 10, "output": 2, "cacheRead": 5, "cacheWrite": 1, "reasoning": 1,
                      "cost": {"total": 0.005}}}


emit({"type": "session", "version": 3})
for i in range(tool_steps):
    if step_delay.exists():
        time.sleep(float(step_delay.read_text()))
    call(i)
    emit({"type": "message_end", "message": assistant([
        {"type": "thinking", "thinking": "measuring"},
        {"type": "text", "text": "looking"},
        {"type": "toolCall", "id": f"c{i}", "name": "bash", "arguments": {"command": "python probe.py"}},
    ], "toolUse")})
    emit({"type": "tool_execution_start", "toolCallId": f"c{i}", "toolName": "bash", "args": {}})
    emit({"type": "tool_execution_end", "toolCallId": f"c{i}", "toolName": "bash", "isError": False,
          "result": {"content": [{"type": "text", "text": "x=4.0"}]}})
call(tool_steps)
emit({"type": "message_end", "message": assistant([{"type": "text", "text": "4.0"}], "stop")})
emit({"type": "agent_end"})
"""


@pytest.fixture
def fake_pi(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Path]:
    """A ``pi`` that calls the provider it was configured with and emits Pi's
    JSON events, plus an upstream that echoes each request. ``tool_steps``
    next to the script is how many tool rounds it plays before answering;
    ``step_delay`` (optional) is how long it sleeps before each of them."""
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Upstream(BaseHTTPRequestHandler):
        def do_POST(self) -> None:
            body = self.rfile.read(int(self.headers["Content-Length"]))
            reply = json.dumps({"echo": json.loads(body)}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)

        def log_message(self, format: str, *args: Any) -> None:
            return None

    server = ThreadingHTTPServer(("127.0.0.1", 0), Upstream)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    monkeypatch.setenv("OPENAI_BASE_URL", f"http://127.0.0.1:{server.server_port}/v1")
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test")
    script = tmp_path / "bin" / "pi"
    script.parent.mkdir()
    script.write_text(f"#!{sys.executable}\n{FAKE_PI}")
    script.chmod(0o755)
    script.with_name("tool_steps").write_text("1")
    yield script
    server.shutdown()


def test_pi_runs_headless_over_the_recording_and_records_every_call(
    dataset: str, fake_pi: Path, tmp_path: Path
) -> None:
    env = Dataset(dataset, select=(lambda s: s.streams.odom.limit(2),))
    running = env.start("")
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    try:
        agent = Pi(cli=str(fake_pi), model="gpt-fake")
        agent.preflight(env)
        trajectory = agent.run("how far?", running, run_dir, timeout_s=60.0)
    finally:
        env.stop()
    assert (trajectory.final_answer, trajectory.extra.ended_by, trajectory.agent.model_name) == (
        "4.0",
        "answer",
        "gpt-fake",
    )
    user, first, last = trajectory.steps
    assert user.source == "user" and user.message == "how far?"
    assert first.tool_calls == (
        ToolCall(tool_call_id="c0", function_name="bash", arguments={"command": "python probe.py"}),
    )
    assert first.observation == Observation(
        results=(ObservationResult(source_call_id="c0", content="x=4.0"),)
    )
    assert first.reasoning_content == "measuring" and not last.tool_calls
    totals = trajectory.final_metrics
    assert (totals.total_prompt_tokens, totals.total_completion_tokens) == (
        32,  # (input 10 + cacheWrite 1 + cacheRead 5) per step
        4,
    )
    assert totals.total_cached_tokens == 10 and totals.total_cost_usd == pytest.approx(0.01)
    assert sum(s.extra.reasoning_tokens for s in trajectory.steps if s.extra) == 2
    for i, step in enumerate((first, last)):  # every call captured whole, auth dropped
        assert step.extra is not None
        request = json.loads(step.extra.request.read_text())
        assert request["body"] == {"model": "gpt-fake", "n": i}
        assert "authorization" not in {k.lower() for k in request["headers"]}
        assert json.loads(step.extra.response.read_text())["body"] == {"echo": request["body"]}
    prompt = (run_dir / "system-prompt.txt").read_text()
    assert f"- recording: {run_dir / 'recording.db'}" in prompt and "dimos mcp call" not in prompt
    copy = _open_store(run_dir / "recording.db")  # the selection, not the source dataset
    try:
        assert [o.ts for o in copy.streams.odom] == [1000.0, 1001.0]
    finally:
        copy.stop()


def test_pi_is_told_the_robots_tools_and_stopped_at_max_steps(
    dataset: str, fake_pi: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import dimos.evals.agents.pi as pi_mod

    monkeypatch.setattr(pi_mod, "tool_listing", lambda mcp_url: "- move_to(x, y): Go there.")
    fake_pi.with_name("tool_steps").write_text("5")
    env = Dataset(dataset, mcp_url="http://localhost:1/mcp")
    running = env.start("")
    try:
        trajectory = Pi(cli=str(fake_pi), max_steps=2).run("go", running, tmp_path, timeout_s=60.0)
    finally:
        env.stop()
    assert (trajectory.extra.ended_by, len(trajectory.steps), trajectory.final_answer) == (
        "max_steps",
        3,  # the instruction, then the two calls allowed
        "",
    )
    prompt = (tmp_path / "system-prompt.txt").read_text()
    assert "dimos mcp call <tool> --json-args" in prompt and "- move_to(x, y): Go there." in prompt


def test_pi_is_killed_at_the_time_budget_and_keeps_its_steps(
    dataset: str, fake_pi: Path, tmp_path: Path
) -> None:
    fake_pi.with_name("tool_steps").write_text("50")
    fake_pi.with_name("step_delay").write_text("0.2")
    env = Dataset(dataset)
    running = env.start("")
    try:
        started = time.monotonic()
        trajectory = Pi(cli=str(fake_pi)).run("go", running, tmp_path, timeout_s=0.5)
        took = time.monotonic() - started
    finally:
        env.stop()
    assert trajectory.extra.ended_by == "timeout" and took < 5.0
    assert 2 <= len(trajectory.steps) <= 4  # the instruction plus the calls it got to
    assert all(s.tool_calls for s in trajectory.steps[1:]) and trajectory.final_answer == ""


def test_pi_preflight_and_tool_rendering(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.evals.agents.pi import render_tools

    monkeypatch.setenv("OPENAI_API_KEY", "sk-test")
    with pytest.raises(RuntimeError, match="not on PATH"):
        Pi(cli=str(tmp_path / "missing")).preflight(ImageFile(tmp_path / "x.png"))
    tools = [
        {
            "name": "move_to",
            "description": "Go to a pose.\n\nArgs: x, y",
            "inputSchema": {"properties": {"x": {}, "y": {}}},
        }
    ]
    assert render_tools(tools) == "- move_to(x, y): Go to a pose."


def test_pi_skills_become_native_flags_and_preflight_checks_paths(
    dataset: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Explicit skill paths become repeated ``--skill`` flags, absolute (Pi's
    cwd is the run dir) with ambient discovery still off; a missing path or a
    bare string fails preflight before a simulator or model starts."""
    skill = tmp_path / "spatial" / "SKILL.md"
    skill.parent.mkdir()
    skill.write_text("# spatial skill")
    monkeypatch.chdir(tmp_path)

    env = Dataset(dataset)
    running = env.start("")
    for d in ("with-skills", "bare"):
        (tmp_path / d).mkdir()
    try:
        agent = Pi(skills=(str(skill), "spatial/SKILL.md"))  # absolute and relative
        command = agent._command("go", running, tmp_path / "with-skills")
        bare = Pi()._command("go", running, tmp_path / "bare")
    finally:
        env.stop()
    assert [command[i + 1] for i, a in enumerate(command) if a == "--skill"] == [
        str(skill),
        str(skill),
    ]
    assert "--no-skills" in command, "ambient discovery stays off alongside explicit skills"
    assert "--skill" not in bare

    with pytest.raises(RuntimeError, match="do not exist"):
        Pi(skills=(str(tmp_path / "missing.md"),)).preflight(env)
    with pytest.raises(RuntimeError, match="list of paths"):
        Pi(skills="spatial/SKILL.md").preflight(env)


def test_agents_report_every_available_tool() -> None:
    environment_tools = ("move_to", "speak")

    assert QuestionAnswer().available_tools(environment_tools) == ()
    assert Blind().available_tools(environment_tools) == ()
    assert McpClientAgent().available_tools(environment_tools) == environment_tools
    assert Pi().available_tools(environment_tools) == (
        "read",
        "bash",
        "edit",
        "write",
        *environment_tools,
    )
