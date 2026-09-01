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

"""Integration tests for the memory <-> eval connection.

Frozen: a case's ``Dataset.select`` pulls real Streams from a recording, the
``QuestionAnswer`` agent encodes them, and the *actual observation data*
(image blocks, pose text) reaches the model prompt.

Live: the environment's recording is written while the agent acts (the
Recorder's role); the grader reads the whole history afterwards.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any

from langchain_core.callbacks import CallbackManagerForLLMRun
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import AIMessage, BaseMessage
from langchain_core.outputs import ChatGeneration, ChatResult
import numpy as np
import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.question_answer import QuestionAnswer
from dimos.evals.environments.dataset import Dataset
from dimos.evals.runner import EvalRunner
from dimos.evals.scorers import first_number, ramp, within
from dimos.evals.types import (
    EvalCase,
    RunningEnvironment,
    Trajectory,
    recording,
)
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


def _open_store(path: Path) -> Any:
    from dimos.memory.store.sqlite import SqliteStore

    try:
        return SqliteStore(path=str(path))
    except Exception as e:  # pragma: no cover — sqlite-vec unavailable platforms
        pytest.skip(f"SqliteStore unavailable: {e}")


class SpyChat(BaseChatModel):
    """Captures the exact messages the agent sends; replies with a constant."""

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


def _qa(spy: SpyChat, **kwargs: Any) -> QuestionAnswer:
    return QuestionAnswer(chat_model=spy, model="spy", system_prompt="sys", **kwargs)


# -- frozen: recording -> select -> encode -> prompt ---------------------------------


def test_selected_streams_reach_the_prompt(tmp_path: Path) -> None:
    store = _open_store(tmp_path / "rec.db")
    odom = store.stream("odom", PoseStamped)
    for i in range(20):
        odom.append(_pose(float(i), 2.5), ts=1000.0 + i)
    frame = np.full((16, 16, 3), 200, dtype=np.uint8)
    images = store.stream("color_image", Image)
    for i in range(3):
        images.append(Image.from_numpy(frame, frame_id="cam", ts=1000.0 + i), ts=1000.0 + i)
    store.stop()

    case = EvalCase(
        id="wiring",
        inputs="how far along x did you travel?",
        environment=Dataset(
            str(tmp_path / "rec.db"),
            select=(lambda s: s.streams.odom, lambda s: s.streams.color_image.limit(2)),
        ),
        grade=lambda o: within(1.0)(19.0, first_number(o.trajectory.final_answer)),
    )

    spy = SpyChat(reply="19")
    spy.seen.clear()
    results = EvalRunner(out_dir=tmp_path / "evals").run([case], _qa(spy))

    assert results[0].passed, results[0]
    blocks = [b for m in spy.seen[0] for b in (m.content if isinstance(m.content, list) else [])]
    image_blocks = [b for b in blocks if b.get("type") == "image_url"]
    text = " ".join(b["text"] for b in blocks if b.get("type") == "text")
    # the actual observation data crossed from memory into the prompt:
    assert len(image_blocks) == 2, "both selected image observations should be encoded"
    assert image_blocks[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "19.000" in text or "19.0" in text, "last odom pose must reach the prompt"
    assert case.inputs in text


def test_frames_per_stream_downsamples_not_truncates(tmp_path: Path) -> None:
    store = _open_store(tmp_path / "rec.db")
    odom = store.stream("odom", PoseStamped)
    for i in range(100):
        odom.append(_pose(float(i), 0.0), ts=1000.0 + i)
    store.stop()

    spy = SpyChat(reply="99")
    spy.seen.clear()
    env = Dataset(str(tmp_path / "rec.db"))
    running = env.start("")
    try:
        _qa(spy, frames_per_stream=5).run("?", running, tmp_path)
    finally:
        env.stop()
    texts = [b["text"] for b in spy.seen[0][-1].content if b.get("type") == "text"]  # type: ignore[union-attr]
    stamped = [t for t in texts if t.startswith("[t=")]
    assert len(stamped) == 5
    assert "0.000" in stamped[0] and "99.000" in stamped[-1], "spread must cover the whole window"


# -- live: the recording is written while the agent acts; the grader reads it after ----


def test_grader_reads_the_history_the_environment_recorded(tmp_path: Path) -> None:
    """A writer thread plays the Recorder: the robot approaches the goal while
    the agent 'acts'; the grader then reads the final pose and the path from
    the recording artifact."""
    db = tmp_path / "live.db"
    store = _open_store(db)
    odom = store.stream("odom", PoseStamped)
    odom.append(_pose(5.0, 0.0), ts=time.time())  # robot starts 5m from goal
    stop = threading.Event()

    def writer() -> None:
        for i in range(1, 26):
            if stop.is_set():
                return
            odom.append(_pose(max(0.0, 5.0 - i * 0.2), 0.0), ts=time.time())
            time.sleep(0.02)

    thread = threading.Thread(target=writer)

    class LiveEnvironment:
        artifacts = ("recording",)
        has_robot = False

        def preflight(self, agent: Any) -> None:
            pass

        def start(self, modules: str) -> RunningEnvironment:
            from dimos.memory.store.sqlite import SqliteStore

            thread.start()
            return RunningEnvironment(
                mcp_url="",
                recording=SqliteStore(path=str(db), must_exist=True),
                artifacts={"recording": db},
            )

        def settle(self, budget_s: float) -> None:
            return None

        def stop(self) -> None:
            stop.set()
            thread.join(timeout=5.0)

    class WaitingAgent:
        modules = ""

        def preflight(self, environment: Any) -> None:
            pass

        def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
            return ()

        def run(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> Trajectory:
            time.sleep(1.0)
            return TrajectoryBuilder(inputs, name="none").build("answer")

    def grade(o: Any) -> float:
        rec = recording(o)
        try:
            poses = [obs.data.position.x for obs in rec.streams.odom]
        finally:
            rec.stop()
        assert len(poses) >= 3, "the whole history is in the recording"
        assert poses == sorted(poses, reverse=True), "monotonic approach must be visible"
        return ramp(abs(poses[-1]), band=2.0)

    case = EvalCase(id="live", inputs="go to the goal", environment=LiveEnvironment(), grade=grade)
    try:
        result = EvalRunner(out_dir=tmp_path / "evals").run([case], WaitingAgent())[0]
    finally:
        stop.set()
        store.stop()

    assert not result.error, result.error
    assert result.score >= 0.99  # the last recorded pose is at the goal
