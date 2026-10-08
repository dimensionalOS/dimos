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

"""A System-1 arm policy: OpenAI's Decisions API picks each step's motion from two images.

Every loop sends the wrist and overview frames with the task to ``/v1/decisions``
and asks independent questions: a 3-level score per axis (x, y, z), an
open/close/hold gripper choice and a done predicate. The most likely level of
each score becomes an end-effector twist on the raw robot bridge, held until the
next decision replaces it.

dimos evals run dimos.evals.suites.robosuite_twist --agent dimos.evals.agents.decisions
"""

from __future__ import annotations

import base64
from collections import deque
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

import httpx
import numpy as np
from pydantic import Field
from scipy.spatial.transform import Rotation

from dimos.evals.agents.base import Agent, ModelAgentConfig
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment
from dimos.evals.types import EndedBy, Metrics, RunningEnvironment, ToolCall, Trajectory
from dimos.robot.raw_robot_bridge import RawTopics

PRICE_PER_INPUT_TOKEN_USD = 0.10 / 1_000_000
API_LATENCY_S = 0.3  # typical /v1/decisions round trip with two images

# Robot base frame axis: (question, unit vector, negative motion, positive motion)
AXES: dict[str, tuple[str, tuple[float, float, float], str, str]] = {
    "x": (
        "Should the gripper move forward or back next?",
        (1.0, 0.0, 0.0),
        "back, toward the robot base",
        "forward, away from the robot base",
    ),
    "y": (
        "Should the gripper move to the robot's left or right next?",
        (0.0, 1.0, 0.0),
        "to the robot's right",
        "to the robot's left",
    ),
    "z": (
        "Should the gripper move up or down next?",
        (0.0, 0.0, 1.0),
        "down, toward the table",
        "up, away from the table",
    ),
}
GRIPPER_OPENING = {"open": 1.0, "close": 0.0}

GUIDANCE = (
    "You control {robot} one small step at a time, moving its gripper along the robot base's "
    "axes. Each decision moves the gripper at most about {step_cm:.0f} cm per axis.\n"
    "Image 1, wrist camera: mounted on the gripper, looking along it. The gripper body is "
    "at the bottom of the frame and the fingers close on whatever is at the centre of the "
    "image. Objects grow as the gripper approaches them. To grasp an object, centre it in "
    "this image, then lower until it sits between the fingers.\n"
    "Image 2, workspace camera: fixed, viewing the whole table and the robot arm.\n"
    "Each motion option says how it looks in both images. When the target is already "
    "centred along an axis, choose 0 for that axis. Close the gripper only when the object "
    "is between the fingers; keep it closed while carrying the object; open it to release."
)


class DecisionsPolicyConfig(ModelAgentConfig):
    model: str = "gpt-6-luna"
    max_speed_mps: float = Field(default=0.05, gt=0, le=0.1)  # at a fully confident +-1 axis
    robot: str = "an xArm7 robot arm with a two-finger parallel gripper"
    hold_s: float = Field(default=1.0, gt=0)  # each twist runs until replaced, or this long
    done_probability: float = Field(default=0.8, gt=0, le=1)
    max_steps: int = Field(default=300, ge=1)
    history: int = Field(default=50, ge=0)  # past actions repeated in each prompt
    request_timeout_s: float = Field(default=10.0, gt=0)
    retries: int = Field(default=3, ge=0)  # per decision, on 5xx, 429 and network errors
    min_period_s: float = Field(default=0.3, ge=0)  # at most one decision this often
    frame_timeout_s: float = Field(default=30.0, gt=0)


class DecisionsPolicy(Agent):
    """Wrist + overview images and the task in, one twist and gripper command out, per step."""

    config: DecisionsPolicyConfig

    def preflight(self, environment: Environment) -> None:
        if not environment.provides_raw_robot:
            raise ValueError("DecisionsPolicy drives the raw robot bridge; use raw_bridge=True")
        if not os.environ.get("OPENAI_API_KEY"):
            raise RuntimeError("DecisionsPolicy needs OPENAI_API_KEY")

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        assert env.raw_endpoint is not None
        deadline = time.monotonic() + timeout_s
        raw_dir = run_dir / "raw"
        raw_dir.mkdir(parents=True, exist_ok=True)
        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model=self.config.model)
        frames = _Frames()
        topics = RawTopics(env.raw_endpoint, listen=False)
        subs = [
            topics.subscribe("camera/jpeg", lambda data, _: frames.set("wrist", data)),
            topics.subscribe("overview/jpeg", lambda data, _: frames.set("overview", data)),
            topics.subscribe("camera_pose/json", lambda data, _: frames.pose("wrist", data)),
            topics.subscribe(
                "overview/camera_pose/json", lambda data, _: frames.pose("overview", data)
            ),
        ]
        base = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1").rstrip("/")
        headers = {"Authorization": f"Bearer {os.environ['OPENAI_API_KEY']}"}
        ended_by: EndedBy = "max_steps"
        error = ""
        gripper: str | None = None
        history: deque[str] = deque(maxlen=self.config.history)
        # One decision's twist runs until the next replaces it: the API call plus the pacing.
        step_s = min(self.config.hold_s, self.config.min_period_s + API_LATENCY_S)
        guidance = GUIDANCE.format(
            robot=self.config.robot, step_cm=self.config.max_speed_mps * step_s * 100
        )
        try:
            with httpx.Client(
                base_url=base, headers=headers, timeout=self.config.request_timeout_s
            ) as http:
                if not frames.wait(min(self.config.frame_timeout_s, timeout_s)):
                    raise TimeoutError(
                        "no wrist and overview frames and poses from the raw robot bridge"
                    )
                for step in range(1, self.config.max_steps + 1):
                    if time.monotonic() >= deadline:
                        ended_by = "timeout"
                        break
                    tick = time.monotonic()
                    wrist, overview, poses = frames.latest()
                    request = decision_request(
                        self.config.model,
                        inputs,
                        wrist,
                        overview,
                        axis_views(poses),
                        f"{guidance}\n\n{past_actions(history)}",
                    )
                    started = time.time()
                    body = post_decision(http, request, self.config.retries)
                    latency = time.time() - started
                    answers = {a["name"]: a for a in body.get("answers", [])}
                    if any(a.get("type") == "refusal" for a in answers.values()):
                        raise RuntimeError(f"the model refused: {body}")
                    twist = twist_from(answers, self.config.max_speed_mps)
                    choice = answer_choice(answers.get("gripper", {}))
                    done = float(answers.get("done", {}).get("probability", 0.0))
                    if choice in GRIPPER_OPENING and choice != gripper:
                        twist = dict.fromkeys(twist, 0.0)  # change the grip standing still
                        topics.put(
                            "arm/gripper/json", json.dumps({"opening": GRIPPER_OPENING[choice]})
                        )
                        gripper = choice
                    topics.put("arm/twist/json", json.dumps({**twist, "t": self.config.hold_s}))
                    leans = " ".join(
                        f"{axis}{twist[f'v{axis}'] / self.config.max_speed_mps:+.1f}"
                        for axis in AXES
                    )
                    history.append(f"{leans} {choice or 'hold'}")
                    trajectory.step(
                        message=json.dumps({"twist": twist, "gripper": choice, "done": done}),
                        request=_save_request(raw_dir, step, request, wrist, overview),
                        response=save_json(raw_dir / f"{step:03d}-response.json", body),
                        tool_calls=(_command_call(step, twist, choice),),
                        metrics=decision_metrics(body),
                        latency_s=latency,
                        at=started,
                    )
                    if done >= self.config.done_probability:
                        ended_by = "answer"
                        break
                    time.sleep(max(0.0, self.config.min_period_s - (time.monotonic() - tick)))
        except (httpx.HTTPError, RuntimeError, TimeoutError, KeyError, ValueError) as exc:
            ended_by, error = "error", str(exc)
        finally:
            topics.put("arm/twist/json", json.dumps({"t": self.config.hold_s}))  # zero: stop
            for sub in subs:
                sub.undeclare()
            topics.close()
        return trajectory.build(ended_by, error=error)


def past_actions(history: deque[str]) -> str:
    """The recent actions, oldest first, on the options' -1..+1 scale."""
    if not history:
        return "Your recent actions: none yet."
    return (
        f"Your last {len(history)} actions, oldest first, on the -1..+1 scale of the options: "
        + "; ".join(history)
        + "."
    )


def post_decision(http: httpx.Client, request: dict[str, Any], retries: int) -> dict[str, Any]:
    """POST one decision, retrying transient failures; the held twist expires meanwhile."""
    for attempt in range(retries + 1):
        try:
            response = http.post("/decisions", json=request)
            if response.status_code < 500 and response.status_code != 429:
                response.raise_for_status()
                body: dict[str, Any] = response.json()
                return body
            if attempt == retries:
                response.raise_for_status()
        except httpx.TransportError:
            if attempt == retries:
                raise
        time.sleep(0.5 * 2**attempt)
    raise AssertionError("unreachable")


def decision_request(
    model: str,
    task: str,
    wrist: bytes,
    overview: bytes,
    views: dict[str, tuple[str, str]],
    guidance: str,
) -> dict[str, Any]:
    """``views`` gives each axis's negative and positive motion as it appears in the images."""
    questions: list[dict[str, Any]] = [
        {
            "type": "score",
            "name": axis,
            "instructions": question,
            "levels": [
                {"label": "-1", "description": f"move {negative} ({views[axis][0]})"},
                {
                    "label": "0",
                    "description": "stay: the target is already centred along this axis",
                },
                {"label": "+1", "description": f"move {positive} ({views[axis][1]})"},
            ],
        }
        for axis, (question, _, negative, positive) in AXES.items()
    ]
    questions += [
        {
            "type": "choice",
            "name": "gripper",
            "instructions": "What should the gripper fingers do now?",
            "choices": [
                {"value": "open", "description": "open the fingers, or keep them open"},
                {"value": "close", "description": "close the fingers on the object"},
                {"value": "hold", "description": "leave the fingers as they are"},
            ],
        },
        {
            "type": "predicate",
            "name": "done",
            "instructions": f"This task is already complete in the images: {task}",
        },
    ]
    content = [
        {"type": "input_text", "text": f"{guidance}\n\nTask: {task}"},
        {"type": "input_image", "image_url": jpeg_data_url(wrist)},
        {"type": "input_image", "image_url": jpeg_data_url(overview)},
    ]
    return {"model": model, "input": [{"role": "user", "content": content}], "questions": questions}


def axis_views(poses: dict[str, list[float]]) -> dict[str, tuple[str, str]]:
    """How each base axis's negative and positive motion looks from each camera.

    ``poses`` maps "wrist" and "overview" to the optical frame's world quaternion
    (x, y, z, w); the robot base shares world's axes.
    """
    views = {}
    for axis, (_, vector, _, _) in AXES.items():
        sides = []
        for sign in (-1.0, 1.0):
            parts = [
                f"{label} image: {_image_direction(poses[camera], sign * np.array(vector))}"
                for camera, label in (("wrist", "wrist"), ("overview", "workspace"))
            ]
            sides.append("; ".join(parts))
        views[axis] = (sides[0], sides[1])
    return views


def _image_direction(quaternion_xyzw: list[float], world: np.ndarray) -> str:
    """Optical frame: +X image right, +Y image down, +Z away from the camera."""
    x, y, z = Rotation.from_quat(quaternion_xyzw).inv().apply(world)
    terms = [
        (abs(x), "toward the right" if x > 0 else "toward the left"),
        (abs(y), "toward the bottom" if y > 0 else "toward the top"),
        (abs(z), "away from the camera" if z > 0 else "toward the camera"),
    ]
    picked = [text for size, text in sorted(terms, reverse=True) if size >= 0.35]
    return " and ".join(picked)


def top_level(answer: dict[str, Any] | None) -> int:
    """The most likely level of a 3-level score as -1, 0 or +1; 0 when unanswered."""
    if not answer:
        return 0
    levels = answer.get("probabilities") or []
    if levels:
        best = max(levels, key=lambda level: float(level["probability"]))
        return int(best["value"]) - 1
    return max(-1, min(1, round(float(answer.get("score", 1.0))) - 1))


def twist_from(answers: dict[str, dict[str, Any]], max_speed_mps: float) -> dict[str, float]:
    """Each axis moves at full speed toward its most likely level, or not at all."""
    names = {"x": "vx", "y": "vy", "z": "vz"}
    return {field: top_level(answers.get(axis)) * max_speed_mps for axis, field in names.items()}


def answer_choice(answer: dict[str, Any]) -> str | None:
    value = answer.get("choice", answer.get("value"))
    return value if isinstance(value, str) else None


class _Frames:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._frames: dict[str, bytes] = {}
        self._poses: dict[str, list[float]] = {}

    def set(self, name: str, data: bytes) -> None:
        with self._lock:
            self._frames[name] = data
            self._check()

    def pose(self, name: str, data: bytes) -> None:
        with self._lock:
            self._poses[name] = json.loads(data)["quaternion_xyzw"]
            self._check()

    def _check(self) -> None:
        if len(self._frames) == 2 and len(self._poses) == 2:
            self._ready.set()

    def wait(self, timeout: float) -> bool:
        return self._ready.wait(timeout)

    def latest(self) -> tuple[bytes, bytes, dict[str, list[float]]]:
        with self._lock:
            return self._frames["wrist"], self._frames["overview"], dict(self._poses)


def jpeg_data_url(jpeg: bytes) -> str:
    return "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()


def save_json(path: Path, body: Any) -> Path:
    path.write_text(json.dumps(body, indent=2))
    return path


def _save_request(
    raw_dir: Path, step: int, request: dict[str, Any], wrist: bytes, overview: bytes
) -> Path:
    """The request as sent, with its images written beside it instead of inlined."""
    images = []
    for name, data in (("wrist", wrist), ("overview", overview)):
        path = raw_dir / f"{step:03d}-{name}.jpg"
        path.write_bytes(data)
        images.append(path.name)
    content = request["input"][0]["content"]
    logged = [content[0], *({"type": "input_image", "file": name} for name in images)]
    return save_json(
        raw_dir / f"{step:03d}-request.json",
        {**request, "input": [{"role": "user", "content": logged}]},
    )


def _command_call(step: int, twist: dict[str, float], gripper: str | None) -> ToolCall:
    return ToolCall(
        tool_call_id=f"step_{step}",
        function_name="arm_command",
        arguments={**twist, "gripper": gripper},
    )


def decision_metrics(body: dict[str, Any]) -> Metrics:
    tokens = int(body.get("usage", {}).get("input_tokens", 0))
    return Metrics(
        prompt_tokens=tokens, completion_tokens=0, cost_usd=tokens * PRICE_PER_INPUT_TOKEN_USD
    )
