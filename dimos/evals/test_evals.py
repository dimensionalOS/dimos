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

"""Offline unit tests: scorers, case dispatch, preflight, runner artifacts.

No network, no robot, no LLM — the chat model is a fake and the rig in case
tests is a plain object satisfying the EvalRig protocol structurally.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import json
from pathlib import Path
from typing import Any

from pydantic import BaseModel
import pytest

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
from dimos.evals.types import EvalCase, InteractiveEval, PassiveEval
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3


def _pose(x: float, y: float) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
    )


@pytest.fixture
def dataset(tmp_path: Path) -> str:
    """A tiny on-disk mem2 dataset: 5 odom poses walking 4m in +x over 4s."""
    from dimos.memory.store.sqlite import SqliteStore

    path = tmp_path / "tiny.db"
    try:
        store = SqliteStore(path=str(path))
    except Exception as e:  # pragma: no cover — sqlite-vec unavailable platforms
        pytest.skip(f"SqliteStore unavailable: {e}")
    stream = store.stream("odom", PoseStamped)
    for i in range(5):
        stream.append(_pose(float(i), 0.0), ts=1000.0 + i)
    store.stop()
    return str(path)


class FakeRig:
    """Structural EvalRig for case-level tests."""

    blind = False
    mcp_url = "http://localhost:9990/mcp"

    def __init__(self, answer: str = "", series: list[tuple[float, float]] | None = None):
        self.answer = answer
        self.series = series or []
        self.calls: list[str] = []

    def open_dataset(self, name: str) -> Any:
        from dimos.memory.cli.dataset import open_dataset

        return open_dataset(name)

    def live_store(self) -> Any:
        raise NotImplementedError

    def encode(self, stream: Any) -> list[dict[str, Any]]:
        return [{"type": "text", "text": f"{len(list(stream))} observations"}]

    def ask(self, context: Sequence[dict[str, Any]], question: str) -> str:
        self.calls.append("ask")
        return self.answer

    def ask_structured(
        self, context: Sequence[dict[str, Any]], question: str, schema: type[Any]
    ) -> Any:
        raise NotImplementedError

    def call_skill(self, name: str, args: Mapping[str, object]) -> str:
        self.calls.append(f"skill:{name}")
        return self.answer

    def agent_loop(self, case: EvalCase) -> str:
        self.calls.append("agent_loop")
        return self.answer

    def mcp_ready(self) -> bool:
        return False

    def setup_env(self, case: InteractiveEval) -> None:
        self.calls.append("setup_env")

    def check_env(self, case: InteractiveEval) -> None:
        pass

    def instruct(self, text: str) -> None:
        self.calls.append(f"instruct:{text}")

    def sample(
        self, score: Callable[[Any], float], interval_s: float, timeout_s: float
    ) -> list[tuple[float, float]]:
        return self.series


class _AnswerResponse(BaseModel):
    answer: str


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
    assert choice(" Chairs. ") == "chairs"


def test_runner_uses_model_native_structured_output(tmp_path: Path) -> None:
    from dimos.evals.runner import EvalRunner

    seen: list[tuple[type[BaseModel], object]] = []

    class StructuredModel:
        def with_structured_output(self, schema: type[BaseModel]) -> StructuredModel:
            seen.append((schema, None))
            return self

        def invoke(self, messages: object) -> BaseModel:
            seen[-1] = (seen[-1][0], messages)
            return _AnswerResponse(answer="yes")

    runner = EvalRunner(chat_model=StructuredModel(), out_dir=tmp_path)

    response = runner.ask_structured([], "Is there a chair?", _AnswerResponse)

    assert response == _AnswerResponse(answer="yes")
    assert seen[0][0] is _AnswerResponse
    assert seen[0][1] is not None


def test_runner_rejects_wrong_structured_response_type(tmp_path: Path) -> None:
    from dimos.evals.runner import EvalRunner

    class WrongTypeModel:
        def with_structured_output(self, schema: type[BaseModel]) -> WrongTypeModel:
            return self

        def invoke(self, messages: object) -> dict[str, str]:
            return {"answer": "yes"}

    runner = EvalRunner(chat_model=WrongTypeModel(), out_dir=tmp_path)

    with pytest.raises(TypeError, match="expected _AnswerResponse"):
        runner.ask_structured([], "Is there a chair?", _AnswerResponse)


# -- case dispatch ---------------------------------------------------------------------


def test_passive_model_path(dataset: str) -> None:
    case = PassiveEval(
        id="disp",
        inputs="how far?",
        expected=4.0,
        parse=first_number,
        score=within(1.0),
        context=(lambda s: s.streams.odom,),
        dataset=dataset,
    )
    rig = FakeRig(answer="about 4 meters")
    result = case.evaluate(rig)
    assert result.score == 1.0
    assert rig.calls == ["ask"]


def test_passive_skill_path(dataset: str) -> None:
    case = PassiveEval(
        id="sk",
        inputs="",
        skill="detect",
        skill_args={"query": "person"},
        expected="yes",
        parse=yes_no,
        dataset=dataset,
    )
    rig = FakeRig(answer="yes")
    assert case.evaluate(rig).score == 1.0
    assert rig.calls == ["skill:detect"]


def test_interactive_dispatch() -> None:
    case = InteractiveEval(
        id="nav",
        inputs="go to the bed",
        score=lambda store: 1.0,
        aggregate=floor,
        simulator="",
    )
    rig = FakeRig(series=[(0.0, 0.2), (1.0, 0.6), (2.0, 0.9)])
    result = case.evaluate(rig)
    assert result.score == 0.2  # floor aggregate
    assert result.series == ((0.0, 0.2), (1.0, 0.6), (2.0, 0.9))
    assert rig.calls == ["setup_env", "instruct:go to the bed"]


def test_interactive_no_samples_is_error() -> None:
    case = InteractiveEval(id="n", inputs="x", score=lambda s: 1.0, simulator="")
    assert "no samples" in case.evaluate(FakeRig()).error


# -- preflight ----------------------------------------------------------------------


def test_preflight_missing_stream(dataset: str) -> None:
    case = PassiveEval(
        id="bad",
        inputs="?",
        expected=1.0,
        parse=first_number,
        context=(lambda s: s.streams.lidar.limit(1),),
        dataset=dataset,
    )
    with pytest.raises(AttributeError, match="No stream 'lidar'"):
        case.preflight(FakeRig())


def test_preflight_needs_mcp(dataset: str) -> None:
    case = PassiveEval(
        id="needs_mcp",
        inputs="?",
        expected="yes",
        parse=yes_no,
        tools=True,
        dataset=dataset,
    )
    with pytest.raises(RuntimeError, match="needs MCP"):
        case.preflight(FakeRig())


# -- runner ------------------------------------------------------------------------


def test_runner_end_to_end_offline(dataset: str, tmp_path: Path) -> None:
    from langchain_core.language_models.fake_chat_models import FakeListChatModel

    from dimos.evals.runner import EvalRunner, summarize

    cases = [
        PassiveEval(
            id="disp",
            inputs="straight-line distance in meters?",
            expected=4.0,
            parse=first_number,
            score=within(1.0),
            context=(lambda s: s.streams.odom,),
            dataset=dataset,
        ),
        PassiveEval(  # parse failure -> error result, run survives
            id="unparseable",
            inputs="?",
            expected=1.0,
            parse=first_number,
            context=(lambda s: s.streams.odom.limit(1),),
            dataset=dataset,
        ),
        PassiveEval(  # preflight failure -> error result, run survives
            id="missing_stream",
            inputs="?",
            expected=1.0,
            parse=first_number,
            context=(lambda s: s.streams.lidar,),
            dataset=dataset,
        ),
    ]
    runner = EvalRunner(
        chat_model=FakeListChatModel(responses=["4.0", "no numbers here"]),
        out_dir=tmp_path / "evals",
    )
    results = runner.run(cases)

    by_id = {r.case_id: r for r in results}
    assert by_id["disp"].passed and by_id["disp"].score == 1.0
    assert "ValueError" in by_id["unparseable"].error
    assert by_id["missing_stream"].error.startswith("preflight:")

    s = summarize(results)
    assert s.n == 3 and s.errors == 2

    run_dir = runner.run_dir
    lines = (run_dir / "results.jsonl").read_text().strip().splitlines()
    assert len(lines) == 3
    summary = json.loads((run_dir / "summary.json").read_text())
    assert summary["n"] == 3 and "model" in summary


def test_runner_encode_budget(dataset: str, tmp_path: Path) -> None:
    from dimos.evals.runner import EvalRunner

    runner = EvalRunner(context_budget=3, out_dir=tmp_path)
    store = runner.open_dataset(dataset)
    try:
        blocks = runner.encode(store.streams.odom)
    finally:
        store.stop()
    # 1 header + 3 sampled observations, all text (PoseStamped str fallback)
    assert len(blocks) == 4
    assert all(b["type"] == "text" for b in blocks)
    assert "pos=" in blocks[1]["text"]


def test_suites_importable() -> None:
    """Suite modules construct without data or network (lambdas stay lazy)."""
    from dimos.evals.suites import dimsim_house, examples, go2_smoke, go2_vqa

    for module in (examples, go2_smoke, go2_vqa, dimsim_house):
        assert module.SUITE, module.__name__


def test_instruct_delivers_over_real_transport() -> None:
    """EvalRunner.instruct publishes on /human_input and tears down immediately;
    the message must still reach an already-subscribed listener (PR #3411 review:
    no flush sleep needed — LCM publish is a synchronous send)."""
    import threading

    from dimos.core.transport_factory import make_transport
    from dimos.evals.runner import EvalRunner

    got = threading.Event()
    received: list[str] = []
    listener = make_transport("/human_input")
    unsubscribe = listener.subscribe(lambda msg: (received.append(msg), got.set()))
    try:
        EvalRunner().instruct("go to the bed")
        assert got.wait(timeout=5.0), "instruct() message never arrived"
        assert received == ["go to the bed"]
    finally:
        unsubscribe()
        listener.stop()


def test_interactive_env_torn_down_per_case(monkeypatch: pytest.MonkeyPatch) -> None:
    """Each interactive case gets its own sim process, stopped even on failure —
    the leak Paul flagged on PR #3411 (only the last _proc was ever stopped)."""
    from dataclasses import dataclass

    import dimos.e2e_tests.dimos_cli_call as cli_call_mod
    from dimos.evals.runner import EvalRunner
    from dimos.evals.types import EvalRig

    launched: list[Any] = []

    class FakeProc:
        def __init__(self) -> None:
            self.simulator = ""
            self.global_args: list[str] = []
            self.demo_args: list[str] = []
            self.stopped = False
            launched.append(self)

        def start(self) -> None:
            pass

        def stop(self) -> None:
            self.stopped = True

    monkeypatch.setattr(cli_call_mod, "DimosCliCall", FakeProc)
    monkeypatch.setattr(EvalRunner, "_wait_mcp", lambda self, timeout: True)

    @dataclass(frozen=True, kw_only=True)
    class BoomCase(InteractiveEval):
        def evaluate(self, rig: EvalRig) -> Any:
            rig.setup_env(self)
            raise RuntimeError("boom")

    runner = EvalRunner()
    case = BoomCase(id="boom", inputs="x", score=lambda s: 1.0)
    for _ in range(2):
        result = runner._guarded(case)
        assert "boom" in (result.error or "")

    assert len(launched) == 2, "each case must launch its own env"
    assert all(p.stopped for p in launched), "every launched env must be stopped"
    assert runner._proc is None
