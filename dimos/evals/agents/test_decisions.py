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

from collections import deque
import json
import threading
import time

import httpx
import pytest

from dimos.evals.agents import decisions
from dimos.evals.agents.decisions import (
    DecisionsPolicy,
    axis_views,
    decision_request,
    past_actions,
    twist_from,
)
from dimos.evals.types import RunningEnvironment
from dimos.robot.raw_robot_bridge import RawTopics


def test_each_axis_takes_its_most_likely_level() -> None:
    def levels(*p: float) -> dict:  # type: ignore[type-arg]
        return {"probabilities": [{"value": i, "probability": v} for i, v in enumerate(p)]}

    answers = {"x": levels(0.2, 0.3, 0.5), "y": levels(0.45, 0.1, 0.45), "z": levels(0.7, 0.2, 0.1)}
    assert twist_from(answers, 0.05) == {"vx": 0.05, "vy": -0.05, "vz": -0.05}


def test_a_bare_score_rounds_to_its_level() -> None:
    answers = {"x": {"score": 2.0}, "y": {"score": 0.3}, "z": {"score": 1.0}}
    # Without probabilities, the score rounds to its nearest level.
    assert twist_from(answers, 0.05) == {"vx": 0.05, "vy": -0.05, "vz": 0.0}
    assert twist_from({}, 0.05) == {"vx": 0.0, "vy": 0.0, "vz": 0.0}  # missing: stay


# Optical frames as quaternions (x, y, z, w): the wrist looks straight down with image up
# along base +x; the overview looks along base -x from in front of the robot.
WRIST_DOWN = [0.7071068, -0.7071068, 0.0, 0.0]
OVERVIEW_FACING_ROBOT = [0.5, 0.5, -0.5, -0.5]
POSES = {"wrist": WRIST_DOWN, "overview": OVERVIEW_FACING_ROBOT}


def test_base_axes_are_described_as_they_look_in_each_image() -> None:
    views = axis_views(POSES)
    assert views["x"][1] == "wrist image: toward the top; workspace image: toward the camera"
    assert views["y"][1] == "wrist image: toward the left; workspace image: toward the right"
    assert views["z"][0] == "wrist image: away from the camera; workspace image: toward the bottom"


def test_request_has_both_images_and_every_question() -> None:
    request = decision_request(
        "gpt-6-luna", "Stack the cubes.", b"wrist", b"overview", axis_views(POSES), "Guide."
    )
    content = request["input"][0]["content"]
    assert [c["type"] for c in content] == ["input_text", "input_image", "input_image"]
    assert "Stack the cubes." in content[0]["text"]
    names = [(q["name"], q["type"]) for q in request["questions"]]
    assert names == [
        ("x", "score"),
        ("y", "score"),
        ("z", "score"),
        ("gripper", "choice"),
        ("done", "predicate"),
    ]
    assert all(len(q["levels"]) == 3 for q in request["questions"][:3])


def test_policy_drives_the_bridge_until_done(tmp_path, monkeypatch) -> None:  # type: ignore[no-untyped-def]
    endpoint = "tcp/127.0.0.1:17461"
    bridge = RawTopics(endpoint, listen=True)
    commands: list[tuple[str, dict[str, float]]] = []
    bridge.subscribe("arm/twist/json", lambda data, _: commands.append(("twist", json.loads(data))))
    bridge.subscribe(
        "arm/gripper/json", lambda data, _: commands.append(("gripper", json.loads(data)))
    )
    stop = threading.Event()

    def stream_frames() -> None:
        while not stop.wait(0.05):
            bridge.put("camera/jpeg", b"wrist")
            bridge.put("overview/jpeg", b"overview")
            for key, quaternion in (("camera_pose/json", WRIST_DOWN),
                                    ("overview/camera_pose/json", OVERVIEW_FACING_ROBOT)):  # fmt: skip
                bridge.put(key, json.dumps({"quaternion_xyzw": quaternion}))

    replies = iter(
        [
            {"x": 2.0, "gripper": "open", "done": 0.1},
            {"x": 1.0, "gripper": "close", "done": 0.2},
            {"x": 1.0, "gripper": "hold", "done": 0.9},
        ]
    )

    def answer(request: httpx.Request) -> httpx.Response:
        reply = next(replies)
        return httpx.Response(
            200,
            json={
                "answers": [
                    {"type": "score", "name": "x", "score": reply["x"]},
                    {"type": "score", "name": "y", "score": 1.0},
                    {"type": "score", "name": "z", "score": 1.0},
                    {"type": "choice", "name": "gripper", "choice": reply["gripper"]},
                    {"type": "predicate", "name": "done", "probability": reply["done"]},
                ],
                "usage": {"input_tokens": 1000},
            },
        )

    real_client = httpx.Client
    monkeypatch.setattr(
        httpx,
        "Client",
        lambda **kw: real_client(transport=httpx.MockTransport(answer), **kw),
    )
    monkeypatch.setenv("OPENAI_API_KEY", "test")
    thread = threading.Thread(target=stream_frames, daemon=True)
    thread.start()
    try:
        env = RunningEnvironment(
            mcp_url="unused", streams=(), artifacts={}, raw_endpoint=endpoint, raw_guide=None
        )
        trajectory = DecisionsPolicy(hold_s=0.5).run("Stack.", env, tmp_path, timeout_s=20.0)
        time.sleep(0.3)
    finally:
        stop.set()
        bridge.close()

    assert trajectory.extra.ended_by == "answer"
    assert len(trajectory.steps) == 4  # the task, then three decisions
    assert trajectory.final_metrics.total_cost_usd == pytest.approx(3 * 1000 * 0.10 / 1e6)
    twists = [c for kind, c in commands if kind == "twist"]
    grips = [c["opening"] for kind, c in commands if kind == "gripper"]
    assert grips == [1.0, 0.0]
    assert twists[0]["vx"] == 0.0  # the first grip change stands still
    assert twists[-1] == {"t": 0.5}  # stopped on the way out
    assert (tmp_path / "raw" / "001-wrist.jpg").read_bytes() == b"wrist"


def test_transient_server_errors_are_retried(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    monkeypatch.setattr("dimos.evals.agents.decisions.time.sleep", lambda _: None)
    statuses = iter([504, 429, 200])

    def answer(request: httpx.Request) -> httpx.Response:
        return httpx.Response(next(statuses), json={"answers": []})

    with httpx.Client(transport=httpx.MockTransport(answer), base_url="http://test") as http:
        assert decisions.post_decision(http, {}, retries=3) == {"answers": []}

    def always_down(request: httpx.Request) -> httpx.Response:
        return httpx.Response(504)

    with httpx.Client(transport=httpx.MockTransport(always_down), base_url="http://test") as http:
        with pytest.raises(httpx.HTTPStatusError):
            decisions.post_decision(http, {}, retries=1)


def test_past_actions_are_listed_oldest_first() -> None:
    assert past_actions(deque()) == "Your recent actions: none yet."
    history = deque(["x+0.5 y+0.0 z-1.0 open", "x+0.0 y+0.0 z+0.0 close"], maxlen=50)
    assert past_actions(history) == (
        "Your last 2 actions, oldest first, on the -1..+1 scale of the options: "
        "x+0.5 y+0.0 z-1.0 open; x+0.0 y+0.0 z+0.0 close."
    )
