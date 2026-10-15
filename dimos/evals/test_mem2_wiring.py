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

Frozen: a case's ``Dataset.config.select`` pulls real Streams from a recording, the
``QuestionAnswer`` agent encodes them, and the *actual observation data*
(image blocks, pose text) reaches the model prompt.

Live: the environment's recording is written while the agent acts (the
Recorder's role); the grader reads the whole history afterwards.
"""

from __future__ import annotations

from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from langchain_core.language_models.fake_chat_models import FakeListChatModel
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.evals.agents.base import Agent
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.question_answer import QuestionAnswer
from dimos.evals.environments.dataset import Dataset
from dimos.evals.runner import EvalRunner
from dimos.evals.scorers import first_number, ramp, within
from dimos.evals.types import (
    EvalCase,
    Outcome,
    RunningEnvironment,
    Trajectory,
    recording,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.image import image_from_array
from dimos.msgs.time import time_from_seconds


def _pose(x: float, y: float) -> PoseStamped:
    return PoseStamped(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
        pose=Pose(
            position=Point(x=x, y=y, z=0.0), orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        ),
    )


@pytest.fixture
def store(tmp_path: Path):
    with SqliteStore(path=str(tmp_path / "rec.db")) as store:
        yield store


def test_selected_streams_reach_the_prompt(
    tmp_path: Path, store: SqliteStore, mocker: MockerFixture
) -> None:
    odom = store.stream("odom", PoseStamped)
    for i in range(20):
        odom.append(_pose(float(i), 2.5), ts=1000.0 + i)
    frame = np.full((16, 16, 3), 200, dtype=np.uint8)
    images = store.stream("color_image", Image)
    for i in range(3):
        images.append(
            image_from_array(
                frame,
                encoding="rgb8",
                header=Header(frame_id="cam", stamp=time_from_seconds(1000.0 + i)),
            ),
            ts=1000.0 + i,
        )

    case = EvalCase(
        id="wiring",
        inputs="how far along x did you travel?",
        environment=Dataset(
            str(tmp_path / "rec.db"),
            select=(lambda s: s.streams.odom, lambda s: s.streams.color_image.limit(2)),
        ),
        grade=lambda o: within(1.0)(19.0, first_number(o.trajectory.final_answer)),
    )

    chat = FakeListChatModel(responses=["19"])
    generate = mocker.spy(FakeListChatModel, "generate")
    results = EvalRunner(out_dir=tmp_path / "evals").run([case], QuestionAnswer(chat_model=chat))

    assert results[0].passed, results[0]
    _, (messages,) = generate.call_args.args
    blocks = messages[-1].content
    assert isinstance(blocks, list)
    image_blocks = [b for b in blocks if b.get("type") == "image_url"]
    text = " ".join(b["text"] for b in blocks if b.get("type") == "text")
    # the actual observation data crossed from memory into the prompt:
    assert len(image_blocks) == 2, "both selected image observations should be encoded"
    assert image_blocks[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "position=(19.000, 2.500, 0.000)" in text, "last odom pose must reach the prompt"
    assert "frame='world'" in text and "quaternion=(0.000, 0.000, 0.000, 1.000)" in text
    assert case.inputs in text


@pytest.mark.parametrize(
    ("frames_per_stream", "positions"), [(1, [0]), (3, [0, 2, 4]), (8, [0, 1, 2, 3, 4])]
)
def test_frames_per_stream_downsamples_not_truncates(
    tmp_path: Path,
    store: SqliteStore,
    frames_per_stream: int,
    positions: list[int],
    mocker: MockerFixture,
) -> None:
    odom = store.stream("odom", PoseStamped)
    for i in range(5):
        odom.append(_pose(float(i), 0.0), ts=1000.0 + i)

    chat = FakeListChatModel(responses=["42"])
    generate = mocker.spy(FakeListChatModel, "generate")
    env = RunningEnvironment(mcp_url="", streams=(odom,), artifacts={})
    QuestionAnswer(chat_model=chat, frames_per_stream=frames_per_stream).run(
        "?", env, tmp_path, timeout_s=60.0
    )

    _, (messages,) = generate.call_args.args
    content = messages[-1].content
    assert isinstance(content, list)
    stamped = [
        block["text"]
        for block in content
        if block["type"] == "text" and block["text"].startswith("[t=")
    ]
    assert stamped == [
        f"[t={float(i):.1f}s] frame='world' position=({float(i):.3f}, 0.000, 0.000) "
        "quaternion=(0.000, 0.000, 0.000, 1.000)"
        for i in positions
    ]


def test_grader_reads_the_history_the_environment_recorded(
    tmp_path: Path, store: SqliteStore
) -> None:
    """Grading sees the initial pose and every pose recorded during agent execution."""
    odom = store.stream("odom", PoseStamped)
    odom.append(_pose(5.0, 0.0), ts=1000.0)

    class RecordingAgent(Agent):
        def run(
            self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
        ) -> Trajectory:
            for i, x in enumerate((2.0, 1.0, 0.0), start=1):
                odom.append(_pose(x, 0.0), ts=1000.0 + i)
            return TrajectoryBuilder(inputs, name="none").build("answer")

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as rec:
            poses = [obs.data.pose.position.x for obs in rec.streams.odom]
        assert poses == [5.0, 2.0, 1.0, 0.0]
        return ramp(abs(poses[-1]), band=2.0)

    case = EvalCase(
        id="live", inputs="go to the goal", environment=Dataset(store.config.path), grade=grade
    )
    result = EvalRunner(out_dir=tmp_path / "evals").run([case], RecordingAgent())[0]

    assert not result.error, result.error
    assert result.score == 1.0
