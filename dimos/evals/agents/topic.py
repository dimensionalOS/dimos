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

"""Evaluate whatever is already running: send the instruction on one topic, wait on another.

The agent under test is a module in the environment's launch (``modules=``). This
adapter publishes the case instruction on ``send`` and returns when ``done`` carries
a truthy message. If the module traces its model calls, ``trace`` names its handle
and the trajectory is built from those request/response pairs; otherwise the run is
one step.

    # the TypeSafe reactive agent: instruction -> /human_input, done when /agent_idle
    dimos evals run <suite> --agent dimos.evals.agents.topic \\
        --set 'modules=["type-safe-agent"]' --set trace=TypeSafeAgent

    # the planner alone: the instruction's "(x, y)" -> /goal, done on /goal_reached
    dimos evals run <suite> --agent dimos.evals.agents.topic \\
        --set send=goal --set send_type=point --set done=goal_reached --set done_type=Bool
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import threading
import time
from typing import Any, Literal

from dimos.agents.llm_trace import list_llm_trace_pairs
from dimos.evals.agents.base import Agent, AgentConfig
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment
from dimos.evals.types import Metrics, RunningEnvironment, Trajectory


def point_in(text: str) -> tuple[float, float]:
    """The last ``(x, y)`` pair in *text*."""
    pairs = re.findall(r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)", text)
    if not pairs:
        raise ValueError(f"no (x, y) in instruction: {text!r}")
    return float(pairs[-1][0]), float(pairs[-1][1])


def floor_z(wait_s: float = 5.0) -> float:
    """The robot's current z from ``/odom``: goals go onto its floor, not to z = 0."""
    from dimos.core.transport_factory import make_transport
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    seen: list[float] = []
    got = threading.Event()

    def on_pose(m: Any, *_: Any) -> None:
        seen.append(m.z)
        got.set()

    odom = make_transport("/odom", PoseStamped)
    odom.start()
    try:
        odom.subscribe(on_pose)
        got.wait(wait_s)
    finally:
        odom.stop()
    return seen[0] if seen else 0.0


class _Stillness:
    """Sets *done* once the robot has moved at least 0.5 m and then held still."""

    def __init__(self, still_s: float, done: threading.Event) -> None:
        self.still_s, self.done = still_s, done
        self.origin: tuple[float, float] | None = None
        self.moved = False
        self.anchor: tuple[float, float] | None = None
        self.since = 0.0

    def on_pose(self, m: Any, *_: Any) -> None:
        x, y, now = float(m.x), float(m.y), time.monotonic()
        if self.origin is None:
            self.origin = (x, y)
        if not self.moved:
            self.moved = math.hypot(x - self.origin[0], y - self.origin[1]) > 0.5
            return
        if self.anchor is None or math.hypot(x - self.anchor[0], y - self.anchor[1]) > 0.05:
            self.anchor, self.since = (x, y), now
        elif now - self.since >= self.still_s:
            self.done.set()


class TopicAgentConfig(AgentConfig):
    send: str = "human_input"
    send_type: Literal["text", "point"] = "text"
    done: str = "agent_idle"
    done_type: Literal["python", "Bool"] = "python"
    frame_id: str = "world"
    # Module handle (class name) whose ``set_trace_dir`` records model calls, if any.
    trace: str | None = None
    # Also done once the robot has moved and then stood still for ``still_s`` (a planner
    # that arrived, or gave up, without publishing on ``done``).
    done_when_still: bool = False
    still_s: float = 8.0
    model: str = ""  # reported in the trajectory when the trace does not say


class TopicAgent(Agent):
    config: TopicAgentConfig

    def validate_tools(self) -> None:
        if self.config.no_dimos:
            raise ValueError("TopicAgent drives modules inside dimOS")
        super().validate_tools()

    def preflight(self, environment: Environment) -> None:
        if not environment.has_robot:
            raise RuntimeError(f"{type(environment).__name__} launches no modules to drive")

    def _stillness_watch(self, done: threading.Event) -> Any:
        """An ``/odom`` transport whose subscriber sets *done* after motion, then stillness."""
        from dimos.core.transport_factory import make_transport
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        watch = _Stillness(self.config.still_s, done)
        odom = make_transport("/odom", PoseStamped)
        odom.subscribe(watch.on_pose)
        return odom

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        from dimos_lcm.std_msgs import Bool

        from dimos.core.transport_factory import make_transport
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped

        raw = run_dir / "raw"
        raw.mkdir(parents=True, exist_ok=True)
        if self.config.trace:
            from dimos.porcelain.dimos import Dimos

            app = Dimos.connect()
            try:
                getattr(app, self.config.trace).set_trace_dir(str(raw))
            finally:
                app.stop()

        payload: Any = inputs
        if self.config.send_type == "point":
            x, y = point_in(inputs)
            payload = PointStamped(x, y, floor_z(), frame_id=self.config.frame_id)
        send_t = make_transport(
            f"/{self.config.send}", type(payload) if self.config.send_type == "point" else None
        )
        done_t = make_transport(
            f"/{self.config.done}", Bool if self.config.done_type == "Bool" else None
        )
        done = threading.Event()
        started = time.time()
        transports = [send_t, done_t]
        if self.config.done_when_still:
            transports.append(self._stillness_watch(done))
        for t in transports:
            t.start()
        try:
            done_t.subscribe(lambda m, *_: done.set() if getattr(m, "data", m) else None)
            send_t.publish(payload)
            finished = done.wait(timeout_s)
        finally:
            for t in transports:
                t.stop()

        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model=self.config.model)
        pairs = list_llm_trace_pairs(raw) if self.config.trace else []
        if not pairs:
            request, response = raw / "1-request.json", raw / "1-response.json"
            request.write_text(
                json.dumps(
                    {
                        "started_at": started,
                        "body": {"topic": self.config.send, "payload": str(payload)},
                    }
                )
            )
            response.write_text(
                json.dumps({"latency_s": time.time() - started, "body": {"done": finished}})
            )
            pairs = [(1, request, response)]
        for _, request, response in pairs:
            req, resp = json.loads(request.read_text()), json.loads(response.read_text())
            body = resp.get("body", {})
            usage = body.get("usage") or {}
            trajectory.step(
                message=json.dumps(body.get("answers", body))[:2000],
                request=request,
                response=response,
                model_name=str(body.get("model", "")),
                metrics=Metrics(
                    prompt_tokens=int(usage.get("input_tokens", usage.get("prompt_tokens", 0))),
                    completion_tokens=int(
                        usage.get("output_tokens", usage.get("completion_tokens", 0))
                    ),
                ),
                at=req.get("started_at"),
                latency_s=float(resp.get("latency_s", 0.0)),
            )
        return trajectory.build("answer" if finished else "timeout")
