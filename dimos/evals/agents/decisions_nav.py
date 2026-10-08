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

"""A System-1 navigation policy: the Decisions API drives a Go2 to a goal from two images.

Every loop sends the suite's ground-truth map (with the robot, its trail and the
goal drawn on it), the front camera frame (with the goal marked when in view) and
the goal's distance and bearing, and asks for a forward and a turn score. The
most likely level of each score becomes a base twist on the raw robot bridge.

The environment hands over the map as its ``nav_map`` artifact, a JSON file made by
``misc/habitat/navmesh_map.py``.

dimos evals run dimos.evals.suites.habitat_go2_goto --agent dimos.evals.agents.decisions_nav
"""

from __future__ import annotations

import base64
from collections import deque
import io
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any

import httpx
import numpy as np
from PIL import Image, ImageDraw
from pydantic import Field

from dimos.evals.agents.base import Agent, ModelAgentConfig
from dimos.evals.agents.decisions import (
    API_LATENCY_S,
    decision_metrics,
    jpeg_data_url,
    past_actions,
    post_decision,
    save_json,
    top_level,
)
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment
from dimos.evals.types import EndedBy, RunningEnvironment, ToolCall, Trajectory
from dimos.robot.raw_robot_bridge import RawTopics

GUIDANCE = (
    "You drive {robot} through a house to a goal, one short step at a time. Each decision "
    "drives at most about {step_m:.1f} m or turns at most about {step_deg:.0f} degrees.\n"
    "Image 1, map: the floor around the robot seen from above, {map_m:.0f} m across, turned "
    "so the robot always faces up. The green arrow in the centre is the robot: up on the map "
    "is straight ahead, left on the map is the robot's left, right is its right. White is "
    "floor the robot can drive on; dark grey is walls and furniture. The blue line is where "
    "it has driven. The red dot is the goal; when the goal is off this map, a red arrow on "
    "the edge points toward it. The robot fits through doorways that show as white gaps.\n"
    "Image 2, front camera: what the robot sees ahead. A red pole marks the goal when it is "
    "in view; a red arrow at the edge shows which side the goal is on when it is not.\n"
    "Plan a route through the white floor on the map, around walls, toward the red dot. "
    "The red pole can show through gaps the robot cannot drive through, so follow the white "
    "floor on the map rather than heading straight at the pole. "
    "Turn to face the next part of the route before driving; drive forward only when the "
    "way ahead is clear."
)


class DecisionsNavConfig(ModelAgentConfig):
    model: str = "gpt-6-luna"
    robot: str = "a Unitree Go2 quadruped robot"
    max_linear_mps: float = Field(default=0.4, gt=0, le=1.0)  # at a fully confident +-1
    max_angular_rps: float = Field(default=0.3, gt=0, le=1.5)  # about 9 degrees a decision
    hold_s: float = Field(default=1.0, gt=0)  # each twist runs until replaced, or this long
    arrive_m: float = Field(default=0.5, gt=0)  # within this of the goal ends the run
    done_probability: float = Field(default=0.8, gt=0, le=1)
    max_steps: int = Field(default=1500, ge=1)
    history: int = Field(default=50, ge=0)  # past actions repeated in each prompt
    retries: int = Field(default=3, ge=0)
    min_period_s: float = Field(default=0.3, ge=0)
    request_timeout_s: float = Field(default=10.0, gt=0)
    frame_timeout_s: float = Field(default=60.0, gt=0)
    camera_height_m: float = Field(default=0.45, gt=0)  # Habitat's default Go2 camera mount
    map_scale: int = Field(default=2, ge=1)  # map pixels drawn per navmesh pixel
    map_radius_m: float = Field(default=6.0, gt=1)  # half the width of the robot-centred map


class DecisionsNavPolicy(Agent):
    """Map + front camera + goal bearing in, one base twist out, per step."""

    config: DecisionsNavConfig

    def preflight(self, environment: Environment) -> None:
        if not environment.provides_raw_robot:
            raise ValueError("DecisionsNavPolicy drives the raw robot bridge; use raw_bridge=True")
        if not os.environ.get("OPENAI_API_KEY"):
            raise RuntimeError("DecisionsNavPolicy needs OPENAI_API_KEY")

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        assert env.raw_endpoint is not None
        if "nav_map" not in env.artifacts:
            raise ValueError("DecisionsNavPolicy needs the environment's nav_map artifact")
        nav_map = NavMap(json.loads(Path(env.artifacts["nav_map"]).read_text()))
        cfg = self.config
        deadline = time.monotonic() + timeout_s
        raw_dir = run_dir / "raw"
        raw_dir.mkdir(parents=True, exist_ok=True)
        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model=cfg.model)
        robot = _RobotFeed()
        topics = RawTopics(env.raw_endpoint, listen=False)
        subs = [
            topics.subscribe("camera/jpeg", lambda data, _: robot.frame(data)),
            topics.subscribe("camera_info/json", lambda data, _: robot.intrinsics(data)),
            topics.subscribe("odom/json", lambda data, _: robot.odom(data)),
        ]
        step_s = min(cfg.hold_s, cfg.min_period_s + API_LATENCY_S)
        guidance = GUIDANCE.format(
            robot=cfg.robot,
            step_m=cfg.max_linear_mps * step_s,
            step_deg=math.degrees(cfg.max_angular_rps * step_s),
            map_m=2 * cfg.map_radius_m,
        )
        base = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1").rstrip("/")
        headers = {"Authorization": f"Bearer {os.environ['OPENAI_API_KEY']}"}
        ended_by: EndedBy = "max_steps"
        error = ""
        trail: list[tuple[float, float]] = []
        history: deque[str] = deque(maxlen=cfg.history)
        try:
            with httpx.Client(
                base_url=base, headers=headers, timeout=cfg.request_timeout_s
            ) as http:
                if not robot.wait(min(cfg.frame_timeout_s, timeout_s)):
                    raise TimeoutError("no camera frame, intrinsics and odometry from the bridge")
                for step in range(1, cfg.max_steps + 1):
                    if time.monotonic() >= deadline:
                        ended_by = "timeout"
                        break
                    tick = time.monotonic()
                    jpeg, k, pose = robot.latest()
                    x, y, z, yaw = pose
                    if not trail or math.dist(trail[-1], (x, y)) > 0.05:
                        trail.append((x, y))
                    distance, _ = nav_map.goal_relative(x, y, yaw)
                    if distance <= cfg.arrive_m:
                        ended_by = "answer"
                        break
                    map_png = nav_map.render(trail, yaw, cfg.map_scale, cfg.map_radius_m)
                    camera = mark_goal(jpeg, k, pose, nav_map.goal, cfg.camera_height_m)
                    text = (
                        f"{guidance}\n\n{past_actions(history)}\n\n"
                        f"Goal: {goal_words(distance)}\n\nTask: {inputs}"
                    )
                    request = nav_request(cfg.model, text, map_png, camera)
                    started = time.time()
                    body = post_decision(http, request, cfg.retries)
                    latency = time.time() - started
                    answers = {a["name"]: a for a in body.get("answers", [])}
                    if any(a.get("type") == "refusal" for a in answers.values()):
                        raise RuntimeError(f"the model refused: {body}")
                    twist = base_twist(answers, cfg.max_linear_mps, cfg.max_angular_rps)
                    done = float(answers.get("done", {}).get("probability", 0.0))
                    topics.put("cmd_vel/json", json.dumps({**twist, "t": cfg.hold_s}))
                    history.append(
                        f"forward{twist['vx'] / cfg.max_linear_mps:+.1f} "
                        f"turn{twist['wz'] / cfg.max_angular_rps:+.1f}"
                    )
                    (raw_dir / f"{step:04d}-map.png").write_bytes(map_png)
                    (raw_dir / f"{step:04d}-camera.jpg").write_bytes(camera)
                    trajectory.step(
                        message=json.dumps(
                            {
                                "twist": twist,
                                "done": done,
                                "pose": [round(x, 3), round(y, 3), round(yaw, 3)],
                                "goal_m": round(distance, 3),
                            }
                        ),
                        request=save_json(raw_dir / f"{step:04d}-request.json", _logged(request)),
                        response=save_json(raw_dir / f"{step:04d}-response.json", body),
                        tool_calls=(
                            ToolCall(
                                tool_call_id=f"step_{step}",
                                function_name="drive",
                                arguments={**twist},
                            ),
                        ),
                        metrics=decision_metrics(body),
                        latency_s=latency,
                        at=started,
                    )
                    if done >= cfg.done_probability:
                        ended_by = "answer"
                        break
                    time.sleep(max(0.0, cfg.min_period_s - (time.monotonic() - tick)))
        except (httpx.HTTPError, RuntimeError, TimeoutError, KeyError, ValueError) as exc:
            ended_by, error = "error", str(exc)
        finally:
            topics.put("cmd_vel/json", json.dumps({"t": cfg.hold_s}))  # zero: stop
            for sub in subs:
                sub.undeclare()
            topics.close()
        return trajectory.build(ended_by, error=error)


class NavMap:
    """The ground-truth navmesh image and its world mapping (ROS +x up, +y left)."""

    def __init__(self, spec: dict[str, Any]) -> None:
        self.mpp = float(spec["meters_per_pixel"])
        self.origin_x, self.origin_y = (float(v) for v in spec["origin_ros_xy"])
        self.goal = (float(spec["goal_ros"][0]), float(spec["goal_ros"][1]))
        png = base64.b64decode(spec["navmesh_png_base64"])
        drivable = np.asarray(Image.open(io.BytesIO(png))) > 0
        self.base = Image.fromarray(np.where(drivable, 255, 80).astype(np.uint8)).convert("RGB")

    def pixel(self, x: float, y: float, scale: int) -> tuple[float, float]:
        return ((self.origin_y - y) / self.mpp * scale, (self.origin_x - x) / self.mpp * scale)

    def goal_relative(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        """Distance in metres and bearing in radians (positive: to the robot's left)."""
        dx, dy = self.goal[0] - x, self.goal[1] - y
        bearing = math.atan2(dy, dx) - yaw
        return math.hypot(dx, dy), math.atan2(math.sin(bearing), math.cos(bearing))

    def render(
        self, trail: list[tuple[float, float]], yaw: float, scale: int, radius_m: float
    ) -> bytes:
        """The map around the robot, rotated so it faces up, with its trail and the goal."""
        px_per_m = scale / self.mpp
        half = int(radius_m * px_per_m)
        pad = int(half * 1.5)  # covers the crop's corners after any rotation
        x, y = trail[-1]
        cx, cy = (int(v) for v in self.pixel(x, y, scale))
        world = self.base.resize(
            (self.base.width * scale, self.base.height * scale), Image.Resampling.NEAREST
        )
        draw = ImageDraw.Draw(world)
        if len(trail) > 1:
            draw.line([self.pixel(tx, ty, scale) for tx, ty in trail], fill=(30, 120, 255), width=3)
        gx, gy = self.pixel(*self.goal, scale)
        draw.ellipse([gx - 9, gy - 9, gx + 9, gy + 9], fill=(220, 0, 0))
        window = Image.new("RGB", (2 * pad, 2 * pad), (80, 80, 80))  # off the map reads as wall
        window.paste(world, (pad - cx, pad - cy))
        # PIL turns counter-clockwise; on the map the heading points at yaw + 90 degrees.
        window = window.rotate(-math.degrees(yaw), resample=Image.Resampling.NEAREST)
        image = window.crop((pad - half, pad - half, pad + half, pad + half))
        draw = ImageDraw.Draw(image)
        _arrow(draw, (half, half + 14), (half, half - 18))
        ahead, left = _goal_side(self.goal, x, y, yaw)
        dx, dy = -left * px_per_m, -ahead * px_per_m  # goal offset in image pixels
        if max(abs(dx), abs(dy)) > half - 9:
            k = (half - 16) / max(abs(dx), abs(dy))
            tip = (half + k * dx, half + k * dy)
            length = math.hypot(k * dx, k * dy)
            tail = (tip[0] - 50 * k * dx / length, tip[1] - 50 * k * dy / length)
            _arrow(draw, tail, tip, color=(220, 0, 0))
        return _png(image)


def mark_goal(
    jpeg: bytes,
    k: list[float],
    pose: tuple[float, float, float, float],
    goal: tuple[float, float],
    camera_height_m: float,
) -> bytes:
    """Draw the goal as a pole on the camera image, or an edge arrow toward it; JPEG out."""
    image = Image.open(io.BytesIO(jpeg)).convert("RGB")
    draw = ImageDraw.Draw(image)
    x, y, z, yaw = pose
    camera = np.array([x, y, z + camera_height_m])
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]

    def project(point: np.ndarray) -> tuple[float, float, float]:
        d = point - camera
        depth = float(d @ forward)
        return fx * float(d @ right) / depth + cx, fy * float(-d[2]) / depth + cy, depth

    foot = project(np.array([goal[0], goal[1], z]))
    top = project(np.array([goal[0], goal[1], z + 1.0]))
    if foot[2] > 0.2 and 0 <= foot[0] < image.width:
        draw.line([foot[:2], top[:2]], fill=(255, 0, 0), width=6)
        draw.ellipse([top[0] - 10, top[1] - 10, top[0] + 10, top[1] + 10], fill=(255, 0, 0))
    else:
        _, left = _goal_side(goal, x, y, yaw)
        mid = image.height / 2
        tip, tail = (12, 70) if left > 0 else (image.width - 12, image.width - 70)
        _arrow(draw, (tail, mid), (tip, mid), color=(255, 0, 0))
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=90)
    return buffer.getvalue()


def goal_words(distance: float) -> str:
    """Straight-line distance to the goal, rounded to half a metre; no direction."""
    return f"the goal is about {max(0.5, round(distance * 2) / 2):.1f} m away in a straight line."


def nav_request(model: str, text: str, map_png: bytes, camera_jpeg: bytes) -> dict[str, Any]:
    questions: list[dict[str, Any]] = [
        {
            "type": "score",
            "name": "forward",
            "instructions": "Should the robot drive forward, stop, or back up now?",
            "levels": [
                {"label": "-1", "description": "back up: down on the map"},
                {"label": "0", "description": "do not drive: turn in place or wait"},
                {"label": "+1", "description": "drive forward: up on the map"},
            ],
        },
        {
            "type": "score",
            "name": "turn",
            "instructions": "Should the robot turn left, keep its heading, or turn right now?",
            "levels": [
                {
                    "label": "-1",
                    "description": "turn right: toward the right side of the map and camera",
                },
                {"label": "0", "description": "keep the current heading"},
                {
                    "label": "+1",
                    "description": "turn left: toward the left side of the map and camera",
                },
            ],
        },
        {
            "type": "predicate",
            "name": "done",
            "instructions": "The robot is at the red goal dot on the map.",
        },
    ]
    content = [
        {"type": "input_text", "text": text},
        {"type": "input_image", "image_url": _png_url(map_png)},
        {"type": "input_image", "image_url": jpeg_data_url(camera_jpeg)},
    ]
    return {"model": model, "input": [{"role": "user", "content": content}], "questions": questions}


def base_twist(
    answers: dict[str, dict[str, Any]], max_linear: float, max_angular: float
) -> dict[str, float]:
    """Full speed toward the most likely forward and turn levels, or not at all."""
    return {
        "vx": top_level(answers.get("forward")) * max_linear,
        "wz": top_level(answers.get("turn")) * max_angular,
    }


class _RobotFeed:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._jpeg: bytes | None = None
        self._k: list[float] | None = None
        self._pose: tuple[float, float, float, float] | None = None

    def frame(self, data: bytes) -> None:
        with self._lock:
            self._jpeg = data
            self._check()

    def intrinsics(self, data: bytes) -> None:
        with self._lock:
            self._k = [float(v) for v in json.loads(data)["K"]]
            self._check()

    def odom(self, data: bytes) -> None:
        o = json.loads(data)
        yaw = math.atan2(
            2 * (o["qw"] * o["qz"] + o["qx"] * o["qy"]), 1 - 2 * (o["qy"] ** 2 + o["qz"] ** 2)
        )
        with self._lock:
            self._pose = (float(o["x"]), float(o["y"]), float(o["z"]), yaw)
            self._check()

    def _check(self) -> None:
        if self._jpeg is not None and self._k is not None and self._pose is not None:
            self._ready.set()

    def wait(self, timeout: float) -> bool:
        return self._ready.wait(timeout)

    def latest(self) -> tuple[bytes, list[float], tuple[float, float, float, float]]:
        with self._lock:
            assert self._jpeg is not None and self._k is not None and self._pose is not None
            return self._jpeg, self._k, self._pose


def _goal_side(goal: tuple[float, float], x: float, y: float, yaw: float) -> tuple[float, float]:
    """The goal in the robot frame: (ahead, left)."""
    dx, dy = goal[0] - x, goal[1] - y
    return (dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw))


def _arrow(
    draw: ImageDraw.ImageDraw,
    tail: tuple[float, float],
    tip: tuple[float, float],
    color: tuple[int, int, int] = (0, 170, 0),
) -> None:
    draw.line([tail, tip], fill=color, width=5)
    angle = math.atan2(tip[1] - tail[1], tip[0] - tail[0])
    for side in (2.6, -2.6):
        draw.line(
            [tip, (tip[0] + 14 * math.cos(angle + side), tip[1] + 14 * math.sin(angle + side))],
            fill=color,
            width=5,
        )
    draw.ellipse([tail[0] - 7, tail[1] - 7, tail[0] + 7, tail[1] + 7], fill=color)


def _png(image: Image.Image) -> bytes:
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def _png_url(png: bytes) -> str:
    return "data:image/png;base64," + base64.b64encode(png).decode()


def _logged(request: dict[str, Any]) -> dict[str, Any]:
    """The request without its inline images, which are saved beside it."""
    content = request["input"][0]["content"]
    logged = [
        content[0],
        {"type": "input_image", "file": "map.png"},
        {"type": "input_image", "file": "camera.jpg"},
    ]
    return {**request, "input": [{"role": "user", "content": logged}]}
