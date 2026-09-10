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
stub, and runner environments implement the same lifecycle as real environments.
"""

from __future__ import annotations

from collections.abc import Sequence
from datetime import datetime
import json
from pathlib import Path
import threading
from types import SimpleNamespace
from typing import Any

from langchain_core.language_models.fake_chat_models import FakeListChatModel
from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
import numpy as np
from pydantic import ValidationError
import pytest
from pytest_mock import MockerFixture

from dimos.core.transport_factory import make_transport
from dimos.evals.agents.base import Agent
from dimos.evals.agents.blind import BLIND_BLOCK, Blind
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.mcp_client_adapter import McpClientAdapter
from dimos.evals.agents.question_answer import QuestionAnswer
from dimos.evals.cli import load_agent
from dimos.evals.environments.base import Environment
from dimos.evals.environments.dataset import Dataset
from dimos.evals.environments.image_file import ImageFile
from dimos.evals.environments.lib.launch import default_mcp_url
from dimos.evals.environments.sim import Sim
from dimos.evals.module import list_agents
from dimos.evals.runner import EvalRunner, summarize
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
from dimos.evals.suites import dimsim_house, dimsim_pointcloud_mapping, examples, go2_smoke, go2_vqa
from dimos.evals.suites.dimsim_pointcloud_mapping import N_ROOMS, ROOMS, grade_rooms
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
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.msgs.sensor_msgs.Image import Image


def _pose(x: float, y: float) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
    )


def _sim(**kwargs: Any) -> Sim:
    return Sim(blueprint=["unitree-go2", "mcp-server", "unitree-skill-container"], **kwargs)


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


class FakeEnvironment(Environment):
    """A frozen recording that records lifecycle calls."""

    def __init__(self, path: Path, calls: list[str]) -> None:
        super().__init__()
        self.path = path
        self.calls = calls
        self.settled_budget: float | None = None

    def preflight(self, agent: Any) -> None:
        self.calls.append("preflight")

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        self.calls.append("start")
        return RunningEnvironment(mcp_url="", streams=(), artifacts={"recording": self.path})

    def settle(self, budget_s: float) -> None:
        self.calls.append("settle")
        self.settled_budget = budget_s

    def stop(self) -> None:
        self.calls.append("stop")
        super().stop()


class FakeAgent(Agent):
    """Replies with a canned answer or timeout."""

    def __init__(self, answer: str = "", timed_out: bool = False) -> None:
        super().__init__()
        self.answer, self.timed_out = answer, timed_out

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        return ("fake_tool",)

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        return _trajectory(self.answer, run_dir / "raw", timed_out=self.timed_out)


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


def test_dataset_start_hands_out_the_selection(dataset: str) -> None:
    env = Dataset(dataset, select=(lambda s: s.streams.odom.limit(2),))
    running = env.start(())
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


def test_dataset_preflight_checks_added_modules(dataset: str) -> None:
    """A tool-using agent's modules become the launched stack, so preflight
    validates the names; adding modules to an attached dimos is a conflict."""
    with pytest.raises(ValueError, match="Unknown blueprint or module: 'no-such-module'"):
        Dataset(dataset).preflight(McpClientAdapter(modules=("no-such-module",)))
    match = "already attaches to http://x/mcp; McpClientAdapter also adds modules"
    with pytest.raises(RuntimeError, match=match):
        Dataset(dataset, mcp_url="http://x/mcp").preflight(
            McpClientAdapter(modules=("unitree-go2",))
        )


def test_dataset_launches_and_cleans_up_the_agents_modules(
    dataset: str, mocker: MockerFixture
) -> None:
    proc = mocker.patch("dimos.evals.environments.dataset.DimosCliCall").return_value
    adapter = mocker.patch("dimos.evals.environments.dataset.McpAdapter")
    adapter.return_value.wait_for_ready.return_value = True
    env = Dataset(dataset)

    try:
        running = env.start(("mcp-server", "mcp-client"))
        assert running.mcp_url == default_mcp_url()
    finally:
        env.stop()

    assert proc.demo_args == ["run", "mcp-server", "mcp-client"]
    assert proc.simulator is None
    proc.start.assert_called_once_with()
    proc.stop.assert_called_once_with()
    adapter.assert_called_once_with(default_mcp_url())


def test_dataset_cleans_up_when_mcp_is_not_ready(dataset: str, mocker: MockerFixture) -> None:
    proc = mocker.patch("dimos.evals.environments.dataset.DimosCliCall").return_value
    adapter = mocker.patch("dimos.evals.environments.dataset.McpAdapter")
    adapter.return_value.wait_for_ready.return_value = False
    env = Dataset(dataset)
    try:
        with pytest.raises(RuntimeError, match="not ready"):
            env.start(("mcp-server",))
    finally:
        env.stop()
    proc.stop.assert_called_once_with()


def test_dataset_stops_the_process_when_closing_its_store_fails(
    dataset: str, mocker: MockerFixture
) -> None:
    proc = mocker.patch("dimos.evals.environments.dataset.DimosCliCall").return_value
    mocker.patch("dimos.evals.environments.dataset.McpAdapter")
    store = mocker.patch("dimos.evals.environments.dataset.open_dataset").return_value
    store.stop.side_effect = RuntimeError("cleanup failed")
    env = Dataset(dataset)

    env.start(("mcp-server",))
    with pytest.raises(RuntimeError, match="cleanup failed"):
        env.stop()
    env.stop()

    proc.stop.assert_called_once_with()


def test_sim_attach_rejects_added_modules() -> None:
    with pytest.raises(RuntimeError, match="attaches.*also adds modules"):
        _sim(attach=True).preflight(McpClientAdapter(modules=("mcp-client",)))


def test_sim_launches_base_blueprints_and_agent_modules_in_order(
    dataset: str, mocker: MockerFixture
) -> None:
    proc = mocker.patch("dimos.evals.environments.sim.DimosCliCall").return_value
    adapter = mocker.patch("dimos.evals.environments.sim.McpAdapter")
    adapter.return_value.wait_for_ready.return_value = True
    sim_client = mocker.patch("dimos.evals.environments.sim.DimSimClient")
    setup = mocker.Mock()
    env = _sim(
        scene="empty",
        launch_timeout_s=4.0,
        setup=setup,
        disable=("wavefront-frontier-explorer", "patrolling-module"),
    )
    mocker.patch.object(env, "_wait_recording", return_value=Path(dataset))

    try:
        env.start(("mcp-client", "speak-skill"))
        assert proc.demo_args == [
            "run",
            "unitree-go2",
            "mcp-server",
            "unitree-skill-container",
            "mcp-client",
            "speak-skill",
            "--disable",
            "wavefront-frontier-explorer",
            "--disable",
            "patrolling-module",
        ]
        assert proc.global_args == ["--dimsim-scene", "empty", "--record"]
        adapter.return_value.wait_for_ready.assert_called_once_with(timeout=4.0, interval=2.0)
        setup.assert_called_once_with(sim_client.return_value)
        proc.start.assert_called_once_with()
    finally:
        env.stop()


def test_image_file_environment(tmp_path: Path) -> None:
    path = tmp_path / "frame.png"
    Image.from_numpy(np.full((8, 8, 3), 200, dtype=np.uint8)).save(path)
    env = ImageFile(path)
    env.preflight(QuestionAnswer())
    running = env.start(())
    try:
        (image_stream,) = running.streams
        (obs,) = list(image_stream)
        assert obs.data.agent_encode()[0]["type"] == "image_url"
        assert running.artifacts == {"image": path}
    finally:
        env.stop()
    with pytest.raises(FileNotFoundError):
        ImageFile(tmp_path / "missing.png").preflight(QuestionAnswer())


@pytest.mark.parametrize(
    ("moving_until", "budget_s", "expected_s"), [(0.3, 10.0, 0.4), (10.0, 0.3, 0.3)]
)
def test_sim_settle_stops_at_rest_or_at_the_budget(
    monkeypatch: pytest.MonkeyPatch, moving_until: float, budget_s: float, expected_s: float
) -> None:
    elapsed = 0.0
    with MemoryStore() as store:
        odom = store.stream("odom", PoseStamped)
        odom.append(_pose(0.0, 0.0), ts=0.0)

        def advance(seconds: float) -> None:
            nonlocal elapsed
            elapsed += seconds
            if elapsed <= moving_until:
                odom.append(_pose(elapsed, 0.0), ts=elapsed)

        monkeypatch.setattr(
            "dimos.evals.environments.sim.time",
            SimpleNamespace(monotonic=lambda: elapsed, sleep=advance),
        )
        env = _sim(at_rest_m=0.01, at_rest_s=0.1, settle_poll_s=0.02)
        env._recording = store
        env.settle(budget_s)

    assert elapsed == pytest.approx(expected_s, abs=env.config.settle_poll_s)


def test_sim_settle_without_motion_data_returns_immediately(mocker: MockerFixture) -> None:
    sleep = mocker.patch("dimos.evals.environments.sim.time.sleep")
    env = _sim()
    env.settle(10.0)
    with MemoryStore() as store:
        env._recording = store
        env.settle(10.0)

    sleep.assert_not_called()


def test_agent_preflight_mismatches(dataset: str) -> None:
    frozen = Dataset(dataset)
    with pytest.raises(RuntimeError, match="McpClientAdapter needs a running McpClient"):
        McpClientAdapter().preflight(frozen)
    McpClientAdapter(modules=("mcp-server", "mcp-client")).preflight(frozen)  # brings its own
    McpClientAdapter().preflight(_sim())  # the environment will launch the stack


def test_question_answer_encodes_the_recording_into_one_call(
    dataset: str, tmp_path: Path, mocker: MockerFixture
) -> None:
    clock = mocker.patch("dimos.evals.agents.lib.single_call.time")
    clock.time.return_value = 1_700_000_000.0
    clock.monotonic.side_effect = [10.0, 10.25]
    chat = FakeListChatModel(responses=["4"])
    generate = mocker.spy(FakeListChatModel, "generate")
    env = Dataset(dataset)
    running = env.start(())
    try:
        trajectory = QuestionAnswer(chat_model=chat).run(
            "how far?", running, tmp_path / "case", timeout_s=60.0
        )
    finally:
        env.stop()

    assert trajectory.final_answer == "4" and trajectory.extra.ended_by == "answer"
    assert trajectory.agent.model_name == "FakeListChatModel", (
        "an injected model is recorded, not the default"
    )
    assert [s.source for s in trajectory.steps] == ["user", "agent"] and generate.call_count == 1
    assert trajectory.steps[0].message == "how far?"
    _, (messages,) = generate.call_args.args
    text = str(messages[-1].content)
    assert (
        "stream 'odom'" in text and '"position_m": [4.0, 0.0, 0.0]' in text and "how far?" in text
    )
    # every call is recorded whole; a fake model has no wire, so normalized
    extra = trajectory.steps[1].extra
    assert extra and extra.request.exists() and extra.response.exists()
    assert json.loads(extra.request.read_text())["normalized"] is True
    assert extra.request.parent == tmp_path / "case" / "raw"
    response = json.loads(extra.response.read_text())
    assert response["normalized"] is True
    assert response["result"]["generations"][0][0]["message"]["content"] == "4"
    assert extra.latency_s == 0.25
    assert datetime.fromisoformat(trajectory.steps[1].timestamp).timestamp() == 1_700_000_000.0


def test_question_answer_refuses_an_empty_recording(tmp_path: Path) -> None:
    running = RunningEnvironment(mcp_url="", streams=(), artifacts={})
    with pytest.raises(ValueError, match="would be blind"):
        QuestionAnswer(chat_model=FakeListChatModel(responses=["42"])).run(
            "?", running, tmp_path, timeout_s=60.0
        )


def test_blind_never_reads_the_recording(
    dataset: str, tmp_path: Path, mocker: MockerFixture
) -> None:
    chat = FakeListChatModel(responses=["7"])
    generate = mocker.spy(FakeListChatModel, "generate")
    env = Dataset(dataset)
    running = env.start(())
    try:
        trajectory = Blind(chat_model=chat).run("how far?", running, tmp_path, timeout_s=60.0)
    finally:
        env.stop()
    _, (messages,) = generate.call_args.args
    text = str(messages[-1].content)
    assert BLIND_BLOCK["text"] in text and "odom" not in text
    assert trajectory.final_answer == "7"


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
    assert manifest["agent"] is None
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


@pytest.mark.parametrize("method", ["start", "settle"])
def test_runner_stops_after_an_environment_failure(
    tmp_path: Path, mocker: MockerFixture, method: str
) -> None:
    calls: list[str] = []
    env = FakeEnvironment(tmp_path, calls)
    mocker.patch.object(env, method, side_effect=RuntimeError("environment failed"))
    grade = mocker.Mock()
    case = EvalCase(id="c", inputs="x", environment=env, grade=grade)

    result = EvalRunner(out_dir=tmp_path).run([case], FakeAgent())[0]

    assert result.error == "RuntimeError('environment failed')"
    assert calls.count("stop") == 1
    grade.assert_not_called()


def test_runner_stops_after_an_agent_failure(tmp_path: Path, mocker: MockerFixture) -> None:
    calls: list[str] = []
    env = FakeEnvironment(tmp_path, calls)
    agent = FakeAgent()
    mocker.patch.object(agent, "run", side_effect=RuntimeError("agent failed"))
    grade = mocker.Mock()
    case = EvalCase(id="c", inputs="x", environment=env, grade=grade)

    result = EvalRunner(out_dir=tmp_path).run([case], agent)[0]

    assert result.error == "RuntimeError('agent failed')"
    assert calls == ["preflight", "start", "stop"]
    grade.assert_not_called()


@pytest.mark.parametrize(("elapsed_s", "remaining_s"), [(7.0, 23.0), (35.0, 0.0)])
def test_runner_gives_the_world_the_budget_the_agent_did_not_use(
    tmp_path: Path, mocker: MockerFixture, elapsed_s: float, remaining_s: float
) -> None:
    clock = mocker.patch("dimos.evals.runner.time.monotonic", return_value=100.0)
    env = FakeEnvironment(tmp_path, [])
    agent = FakeAgent()

    def run(*args: Any, **kwargs: Any) -> Trajectory:
        clock.return_value += elapsed_s
        return _trajectory("ok", tmp_path / "raw")

    mocker.patch.object(agent, "run", side_effect=run)
    case = EvalCase(id="c", inputs="x", environment=env, grade=lambda o: 1.0, timeout_s=30.0)
    EvalRunner(out_dir=tmp_path).run([case], agent)

    assert env.settled_budget == remaining_s


def test_runner_stops_before_grading_a_timeout(tmp_path: Path) -> None:
    calls: list[str] = []
    env = FakeEnvironment(tmp_path, calls)

    def grade(o: Outcome) -> float:
        calls.append("grade")
        return 0.5

    case = EvalCase(id="slow", inputs="x", environment=env, grade=grade, timeout_s=0.2)
    result = EvalRunner(out_dir=tmp_path).run([case], FakeAgent(timed_out=True))[0]
    assert result.ended_by == "timeout" and not result.error
    assert result.score == 0.5 and not result.passed  # the world is still graded
    assert calls == ["preflight", "start", "settle", "stop", "grade"]


def test_runner_missing_artifact_is_an_error(tmp_path: Path) -> None:
    graded: list[Outcome] = []
    env = FakeEnvironment(tmp_path / "never-written.db", [])
    case = EvalCase(id="c", inputs="x", environment=env, grade=lambda o: graded.append(o) or 1.0)
    result = EvalRunner(out_dir=tmp_path).run([case], FakeAgent(answer="ok"))[0]
    assert result.error == "missing artifacts: ['recording']" and not graded


def test_recording_helper_opens_the_artifact(dataset: str) -> None:
    with recording(
        Outcome(trajectory=_trajectory("", Path()), artifacts={"recording": Path(dataset)})
    ) as store:
        assert store.streams.odom.last().data.position.x == 4.0


def test_count_rooms_grader_scores_reply_and_coverage(tmp_path: Path) -> None:
    """Half credit for the exact room count, half for the fraction of room
    points the recorded odometry approached; an unparseable reply loses the
    count half but the world still scores."""
    radius = 1.5
    grade = grade_rooms(radius)

    def written(db_path: Path, points: list[tuple[float, float]]) -> Path:
        with SqliteStore(path=str(db_path)) as store:
            stream = store.stream("odom", PoseStamped)
            for i, (x, y) in enumerate(points):
                stream.append(_pose(x, y), ts=1000.0 + i)
        return db_path

    def score(db: Path, answer: str) -> float:
        outcome = Outcome(trajectory=_trajectory(answer, tmp_path), artifacts={"recording": db})
        return grade(outcome)

    rooms = list(ROOMS.values())
    two = written(tmp_path / "two.db", [(x + radius / 2, y) for x, y in rooms[:2]])
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

    for module in (examples, go2_smoke, go2_vqa, dimsim_house, dimsim_pointcloud_mapping):
        assert module.SUITE, module.__name__
    agents = list_agents()
    assert {m.rsplit(".", 1)[1] for m in agents} == {
        "question_answer",
        "blind",
        "mcp_client_adapter",
        "pi",
    }
    for module in agents:
        assert callable(load_agent(module).run), module


def test_load_agent_is_the_module_plus_set_overrides() -> None:
    """``--agent`` names a module with one agent class; ``--set`` values are
    JSON where they parse, else text; a field the agent lacks is a ValidationError."""

    agent = load_agent(
        "dimos.evals.agents.question_answer",
        ["chat_model=null", 'modules=["rangefinder-skill"]', "model=x"],
    )
    assert (type(agent).__name__, agent.config.chat_model, agent.config.modules, agent.config.model) == (
        "QuestionAnswer", None, ("rangefinder-skill",), "x"
    )  # fmt: skip
    loaded = load_agent("dimos.evals.agents.question_answer", ["frames_per_stream=3"])
    assert loaded.config.frames_per_stream == 3
    with pytest.raises(ValidationError, match="frames_per_stream"):
        load_agent("dimos.evals.agents.blind", ["frames_per_stream=3"])
    with pytest.raises(ValidationError, match="model"):
        load_agent("dimos.evals.agents.mcp_client_adapter", ["model=gpt-4o"])
    with pytest.raises(TypeError, match="0 agents"):
        load_agent("dimos.evals.agents.lib.single_call")
    with pytest.raises(ValidationError, match="modules"):
        load_agent("dimos.evals.agents.mcp_client_adapter", ["modules=mcp-server mcp-client"])
    with pytest.raises(ValidationError, match="frames_per_stream"):
        load_agent("dimos.evals.agents.question_answer", ["frames_per_stream=0"])


@pytest.mark.parametrize("goes_idle", [True, False])
def test_mcp_client_adapter_drives_a_turn_over_real_transports(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, goes_idle: bool
) -> None:
    """The production agent points the McpClient's raw capture at run_dir/raw,
    publishes on /human_input, reads the turn back on /agent until
    /agent_idle or its budget runs out, and links every model call to the
    McpClient's trace files (the message must arrive with no flush sleep: LCM
    publish is a synchronous send)."""

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
            (trace_dir / f"{i:03d}-request.json").write_text(
                json.dumps({"started_at": 1_700_000_000.0 + i})
            )
            (trace_dir / f"{i:03d}-response.json").write_text(json.dumps({"latency_s": 0.25 + i}))
        agent_t.publish(
            AIMessage(
                content=[{"type": "reasoning", "summary": [{"text": "Navigate to the bed."}]}],
                response_metadata={"model_provider": "openai"},
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
        agent = McpClientAdapter()
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
    assert first.reasoning_content == "Navigate to the bed."
    assert user.source == "user" and user.message == "go to the bed"
    assert first.tool_calls == (
        ToolCall(tool_call_id="c1", function_name="move_to", arguments={"x": 1.0}),
    )
    assert first.observation == Observation(
        results=(ObservationResult(source_call_id="c1", content="arrived"),)
    )
    assert last.extra and last.extra.request == tmp_path / "case" / "raw" / "001-request.json"
    assert last.extra.request.exists()
    assert first.extra and first.extra.latency_s == 0.25
    assert last.extra.latency_s == 1.25
    assert datetime.fromisoformat(first.timestamp).timestamp() == 1_700_000_000.0
    assert datetime.fromisoformat(last.timestamp).timestamp() == 1_700_000_001.0


def test_agents_report_every_available_tool() -> None:
    environment_tools = ("move_to", "speak")

    assert QuestionAnswer().available_tools(environment_tools) == ()
    assert Blind().available_tools(environment_tools) == ()
    assert McpClientAdapter().available_tools(environment_tools) == environment_tools
