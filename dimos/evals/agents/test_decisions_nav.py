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

import base64
import io
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

import httpx
import numpy as np
from PIL import Image
import pytest

from dimos.evals.agents.decisions_nav import (
    DecisionsNavPolicy,
    NavMap,
    base_twist,
    goal_words,
    mark_goal,
)
from dimos.evals.types import RunningEnvironment
from dimos.robot.raw_robot_bridge import RawTopics

K = [100.0, 0.0, 64.0, 0.0, 100.0, 48.0, 0.0, 0.0, 1.0]  # 128 x 96 camera


def _spec(goal: tuple[float, float] = (2.0, 0.0)) -> dict:  # type: ignore[type-arg]
    png = io.BytesIO()
    Image.fromarray(np.full((40, 60), 255, dtype=np.uint8)).save(png, format="PNG")
    return {
        "meters_per_pixel": 0.1,
        "origin_ros_xy": [2.0, 3.0],  # pixel (0, 0) is world x=2, y=3
        "goal_ros": [goal[0], goal[1], 0.0],
        "navmesh_png_base64": base64.b64encode(png.getvalue()).decode(),
    }


def _jpeg() -> bytes:
    buffer = io.BytesIO()
    Image.new("RGB", (128, 96), "white").save(buffer, format="JPEG")
    return buffer.getvalue()


def test_map_pixels_put_ros_x_up_and_y_left() -> None:
    nav = NavMap(_spec())
    assert nav.pixel(2.0, 3.0, 1) == (0.0, 0.0)
    assert nav.pixel(1.0, 3.0, 1) == (0.0, 10.0)  # -x is down
    assert nav.pixel(2.0, 2.0, 2) == (20.0, 0.0)  # -y is right, scaled


def test_goal_distance_and_bearing_are_robot_relative() -> None:
    nav = NavMap(_spec(goal=(0.0, 2.0)))
    distance, bearing = nav.goal_relative(0.0, 0.0, 0.0)
    assert distance == pytest.approx(2.0) and math.degrees(bearing) == pytest.approx(90.0)
    _, facing = nav.goal_relative(0.0, 0.0, math.pi / 2)
    assert facing == pytest.approx(0.0)
    assert goal_words(2.2) == "the goal is about 2.0 m away in a straight line."
    assert goal_words(7.38) == "the goal is about 7.5 m away in a straight line."


def test_goal_ahead_is_a_pole_and_behind_is_an_edge_arrow() -> None:
    ahead = np.asarray(
        Image.open(io.BytesIO(mark_goal(_jpeg(), K, (0, 0, 0, 0), (3.0, 0.0), 0.45)))
    )
    red = (ahead[:, :, 0] > 200) & (ahead[:, :, 1] < 80)
    assert red[:, 60:68].any() and not red[:, :20].any()  # on the image centre column
    behind = np.asarray(
        Image.open(io.BytesIO(mark_goal(_jpeg(), K, (0, 0, 0, 0), (-3.0, 1.0), 0.45)))
    )
    red = (behind[:, :, 0] > 200) & (behind[:, :, 1] < 80)
    assert red[:, :20].any() and not red[:, 100:].any()  # left edge: the goal is to the left


def test_scores_become_vx_and_wz() -> None:
    levels = [{"value": i, "probability": p} for i, p in enumerate((0.6, 0.3, 0.1))]
    answers: dict[str, dict[str, Any]] = {
        "forward": {"score": 2.0},
        "turn": {"probabilities": levels},
    }
    assert base_twist(answers, 0.4, 0.8) == {"vx": 0.4, "wz": -0.8}


def test_policy_drives_until_it_arrives(tmp_path: Path, monkeypatch) -> None:  # type: ignore[no-untyped-def]
    nav_map = tmp_path / "map.json"
    nav_map.write_text(json.dumps(_spec(goal=(1.0, 0.0))))
    endpoint = "tcp/127.0.0.1:17467"
    bridge = RawTopics(endpoint, listen=True)
    commands: list[dict[str, float]] = []
    x = {"value": 0.0}
    bridge.subscribe("cmd_vel/json", lambda data, _: commands.append(json.loads(data)))
    stop = threading.Event()

    def stream() -> None:
        while not stop.wait(0.05):
            bridge.put("camera/jpeg", _jpeg())
            bridge.put("camera_info/json", json.dumps({"width": 128, "height": 96, "K": K}))
            odom = {"t": 0, "x": x["value"], "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}
            bridge.put("odom/json", json.dumps(odom))

    texts: list[str] = []

    def answer(request: httpx.Request) -> httpx.Response:
        texts.append(json.loads(request.content)["input"][0]["content"][0]["text"])
        x["value"] += 0.3  # each decision drives the robot 0.3 m toward the goal
        return httpx.Response(
            200,
            json={
                "answers": [
                    {"type": "score", "name": "forward", "score": 2.0},
                    {"type": "score", "name": "turn", "score": 1.0},
                    {"type": "predicate", "name": "done", "probability": 0.0},
                ],
                "usage": {"input_tokens": 2000},
            },
        )

    real_client = httpx.Client
    monkeypatch.setattr(
        httpx, "Client", lambda **kw: real_client(transport=httpx.MockTransport(answer), **kw)
    )
    monkeypatch.setenv("OPENAI_API_KEY", "test")
    threading.Thread(target=stream, daemon=True).start()
    try:
        env = RunningEnvironment(
            mcp_url="unused",
            streams=(),
            artifacts={"nav_map": nav_map},
            raw_endpoint=endpoint,
            raw_guide=None,
        )
        policy = DecisionsNavPolicy(min_period_s=0.2, hold_s=0.5)
        trajectory = policy.run("Go.", env, tmp_path, timeout_s=20.0)
        time.sleep(0.3)
    finally:
        stop.set()
        bridge.close()

    assert trajectory.extra.ended_by == "answer"  # arrived within arrive_m
    assert "Your recent actions: none yet." in texts[0]
    assert "Your last 1 actions, oldest first" in texts[1] and "forward+1.0 turn+0.0" in texts[1]
    assert commands[0] == {"vx": 0.4, "wz": 0.0, "t": 0.5}
    assert commands[-1] == {"t": 0.5}  # stopped on the way out
    assert (tmp_path / "raw" / "0001-map.png").exists()
    assert (tmp_path / "raw" / "0001-camera.jpg").exists()


def test_map_turns_with_the_robot_so_ahead_is_up() -> None:
    nav = NavMap(_spec(goal=(0.0, 1.0)))  # the goal is 1 m along world +y
    facing_y = np.asarray(Image.open(io.BytesIO(nav.render([(0.0, 0.0)], math.pi / 2, 1, 2.0))))
    red = np.argwhere((facing_y[:, :, 0] > 200) & (facing_y[:, :, 1] < 60))
    centre = facing_y.shape[0] / 2
    assert red[:, 0].mean() < centre - 5  # ahead of the robot: above the centre
    assert abs(red[:, 1].mean() - centre) < 3  # straight ahead: on the centre column
    facing_x = np.asarray(Image.open(io.BytesIO(nav.render([(0.0, 0.0)], 0.0, 1, 2.0))))
    red = np.argwhere((facing_x[:, :, 0] > 200) & (facing_x[:, :, 1] < 60))
    assert red[:, 1].mean() < centre - 5  # +y is the robot's left: left of the centre
