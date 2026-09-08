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

"""Integration tests for the memory <-> EvalCase connection.

Passive: a case's context Selects pull real Streams from a recording, the
runner encodes them, and the *actual observation data* (image blocks, pose
text) reaches the model prompt.

Interactive: a case's score callable reads the *live* store while a writer is
appending — the mem2 analogue of a robot's Recorder running mid-task.
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

from dimos.evals.runner import EvalRunner
from dimos.evals.scorers import final, first_number, ramp, within
from dimos.evals.types import InteractiveEval, PassiveEval
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
    """Captures the exact messages the runner sends; replies with a constant."""

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


# -- passive: recording -> Select -> encode -> prompt --------------------------------


def test_passive_streams_reach_the_prompt(tmp_path: Path) -> None:
    store = _open_store(tmp_path / "rec.db")
    odom = store.stream("odom", PoseStamped)
    for i in range(20):
        odom.append(_pose(float(i), 2.5), ts=1000.0 + i)
    frame = np.full((16, 16, 3), 200, dtype=np.uint8)
    images = store.stream("color_image", Image)
    for i in range(3):
        images.append(Image.from_numpy(frame, frame_id="cam", ts=1000.0 + i), ts=1000.0 + i)
    store.stop()

    case = PassiveEval(
        id="wiring",
        inputs="how far along x did you travel?",
        expected=19.0,
        parse=first_number,
        score=within(1.0),
        context=(
            lambda s: s.streams.odom.range_time(0, 100),
            lambda s: s.streams.color_image.limit(2),
        ),
        dataset=str(tmp_path / "rec.db"),
    )

    spy = SpyChat(reply="19")
    spy.seen.clear()
    runner = EvalRunner(chat_model=spy, out_dir=tmp_path / "evals")
    results = runner.run([case])

    assert results[0].passed, results[0]
    blocks = [b for m in spy.seen[0] for b in (m.content if isinstance(m.content, list) else [])]
    image_blocks = [b for b in blocks if b.get("type") == "image_url"]
    text = " ".join(b["text"] for b in blocks if b.get("type") == "text")
    # the actual observation data crossed from mem2 into the prompt:
    assert len(image_blocks) == 2, "both selected image observations should be encoded"
    assert image_blocks[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "pos=[0.000, 2.500" in text.replace("  ", " ") or "0.000" in text
    assert "19.000" in text or "19.0" in text, "last odom pose must reach the prompt"
    assert case.inputs in text


def test_passive_context_budget_downsamples_not_truncates(tmp_path: Path) -> None:
    store = _open_store(tmp_path / "rec.db")
    odom = store.stream("odom", PoseStamped)
    for i in range(100):
        odom.append(_pose(float(i), 0.0), ts=1000.0 + i)
    store.stop()

    runner = EvalRunner(context_budget=5, out_dir=tmp_path / "evals")
    reopened = runner.open_dataset(str(tmp_path / "rec.db"))
    try:
        blocks = runner.encode(reopened.streams.odom)
    finally:
        reopened.stop()
    texts = [b["text"] for b in blocks[1:]]  # skip header
    assert len(texts) == 5
    assert "0.000" in texts[0] and "99.000" in texts[-1], "spread must cover the whole window"


# -- interactive: live store -> score sampling ----------------------------------------


def test_interactive_scores_live_store_while_writing(tmp_path: Path) -> None:
    """A writer thread plays the Recorder role: the case's score callable must
    see fresh observations appear in the live store as they are appended."""
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
            time.sleep(0.05)

    thread = threading.Thread(target=writer)

    class NoEnvRunner(EvalRunner):
        """Rig with the sim/MCP environment stubbed out — mem2 path stays real."""

        def check_env(self, case: InteractiveEval) -> None:
            pass

        def setup_env(self, case: InteractiveEval) -> None:
            thread.start()

        def instruct(self, text: str) -> None:
            pass

    case = InteractiveEval(
        id="live_wiring",
        inputs="go to the goal",
        score=lambda s: ramp(abs(s.streams.odom.last().data.position.x), band=2.0),
        aggregate=final,
        interval_s=0.1,
        timeout_s=10.0,
        simulator="",
    )

    runner = NoEnvRunner(live_db=str(db), out_dir=tmp_path / "evals")
    try:
        results = runner.run([case])
    finally:
        stop.set()
        if thread.ident is not None:
            thread.join(timeout=5.0)
        store.stop()

    r = results[0]
    assert not r.error, r.error
    assert len(r.series) >= 3, "sampler must observe multiple live states"
    scores = [s for _, s in r.series]
    assert scores[0] < 0.9, "first sample sees the robot far from the goal"
    assert scores[-1] >= 0.99, "last sample sees the robot arrive (live data flowed)"
    assert r.score >= 0.99  # aggregate=final
    assert scores == sorted(scores), "monotonic approach must be visible in the series"
