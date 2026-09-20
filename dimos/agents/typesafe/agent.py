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
"""Reactive text-only agent: WorldState in, joystick-style cmd_vel out."""

from __future__ import annotations

from dataclasses import replace
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Generic, TypeVar

from dimos_lcm.std_msgs import Bool
from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.messages.base import BaseMessage
from reactivex.disposable import Disposable

from dimos.agents.typesafe.client import API_KEY_ENV, DEFAULT_MODEL, Answers, Question, SystemOne
from dimos.agents.typesafe.drive import Drive, decode, questions
from dimos.agents.typesafe.world_state import (
    Memory,
    RobotState,
    WorldState,
    build_world_state,
)
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

T = TypeVar("T")
Vec3 = tuple[float, float, float]
ZERO: Vec3 = (0.0, 0.0, 0.0)
PUBLISH_HZ = 10.0
# ponytail: fixed steering gains; make them config if a robot needs different ones.
SLOW_WITHIN_M = 1.5
TURN_FULL_AT_DEG = 45.0
TASK = (
    "You are a mobile ground robot indoors. Everything in this JSON is relative to you. "
    "`goal`: what to do. `robot`: your last motion and picks; `recent`: what "
    "you did over the last 8 seconds (`pattern` stuck: driving without moving). `objects`: first the target named in `goal` "
    "(`target: true`), then the nearest floor-level obstacles, each with `bearing` (8-way word; "
    "ahead means within 15 degrees), `distance_m` to its nearest edge and `width_m`; the target "
    "also has `bearing_deg` (positive is left) and a `distance` word. `way_to_target`: whether "
    "the straight line to the target is free (`state` clear / blocked). Clear: `room` is the "
    "room along that line, `narrowed_on` the side from which something beside the line narrows "
    "it. Blocked: `blocked_by` names what stands on it (obstacle: only the depth scan sees it), "
    "`blocked_at_m` how far away, "
    "and `open_sides` lists, left and right of that line, the nearest way past: kind doorway "
    "is an opening in a wall with a free straight line to it (`range_m`, `width_m`, "
    "`target_beyond`: the target is on the other side of that wall); kind open / corner is a "
    "direction with free length `clear_m` (kind free: nothing is open, only the longest). Each has its own `bearing` and `detour_deg` away "
    "from the line, agrees with what the depth scan sees or saw lately, and has `been_there` true when you have "
    "already driven where it leads; `going_around` is the side your own picks began steering to. "
    "`free_space`: the nearest obstacle in each direction (`clear_m`, `state` clear / tight / "
    "blocked, `by` what it is). You cannot drive through what blocks the line: while blocked, "
    "steer by the `bearing` of one of `open_sides` instead of the target's. The task is "
    "finished when the target's `distance` is touching, or near with the robot stopped as "
    "close as it can get and no wall on the line to it: then report finished."
)


def typesafe_api_key() -> str | None:
    if os.environ.get(API_KEY_ENV):
        return None
    return f"{API_KEY_ENV} is not set. Create a key at https://console.typesafe.ai/settings/keys"


class _Latest(Generic[T]):
    def __init__(self) -> None:
        self._item: tuple[float, T] | None = None

    def put(self, msg: T) -> None:
        self._item = (time.monotonic(), msg)

    def get(self, max_age_s: float) -> T | None:
        item = self._item
        return item[1] if item and time.monotonic() - item[0] <= max_age_s else None


class TypeSafeAgentConfig(ModuleConfig):
    model: str = DEFAULT_MODEL
    rate_hz: float = 2.0
    timeout_s: float = 5.0
    stale_s: float = 2.0
    deadman_s: float | None = None  # None: max(1, 2.5 / rate_hz), must outlast one slow inference
    linear_speed: float = 0.5
    angular_speed: float = 0.8
    linear_accel: float = 0.8
    angular_accel: float = 1.6
    stop_threshold: float = 0.7
    reached_m: float = 0.5
    give_up_s: float = 5.0  # goal clears after this long without motion
    image_size: tuple[int, int] = (1280, 720)
    lidar_band: tuple[float, float, float] = (-0.2, 0.8, 5.0)  # z_min, z_max, max_range
    trace_dir: Path | None = None


class TypeSafeAgent(Module):
    config: TypeSafeAgentConfig

    odom: In[PoseStamped]
    odometry: In[Odometry]  # same pose, nav_msgs flavour (habitat)
    detections_3d: In[Detection3DArray]
    detections_2d: In[Detection2DArray]
    lidar: In[PointCloud2]
    human_input: In[str]

    cmd_vel: Out[Twist]
    agent: Out[BaseMessage]
    agent_idle: Out[bool]
    finished: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._odom: _Latest[PoseStamped] = _Latest()
        self._det3d: _Latest[Detection3DArray] = _Latest()
        self._det2d: _Latest[Detection2DArray] = _Latest()
        self._lidar: _Latest[PointCloud2] = _Latest()
        self._client: SystemOne | None = None
        self._lock = threading.Lock()
        self._goal: str | None = None
        self._robot: RobotState = {"motion": "idle"}
        self._memory = Memory()
        self._target: Vec3 = ZERO
        self._current: Vec3 = ZERO
        self._decided_at = 0.0
        self._zero_since: float | None = None
        self._last_message = ""
        self._seq = 0
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []

    @rpc
    def start(self) -> None:
        super().start()
        self._client = SystemOne(
            os.environ.get(API_KEY_ENV, ""),
            model=self.config.model,
            timeout_s=self.config.timeout_s,
        )
        self.register_disposable(Disposable(self.odom.subscribe(self._odom.put)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.detections_3d.subscribe(self._det3d.put)))
        self.register_disposable(Disposable(self.detections_2d.subscribe(self._det2d.put)))
        self.register_disposable(Disposable(self.lidar.subscribe(self._lidar.put)))
        self.register_disposable(Disposable(self.human_input.subscribe(self.set_goal)))
        self._stop_event.clear()
        self._threads = [
            threading.Thread(target=self._infer_loop, name="TypeSafeAgent-infer", daemon=True),
            threading.Thread(target=self._publish_loop, name="TypeSafeAgent-publish", daemon=True),
        ]
        for t in self._threads:
            t.start()
        self.agent_idle.publish(True)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.cmd_vel.publish(Twist.zero())
        if self._client is not None:
            self._client.close()
        super().stop()

    def _on_odometry(self, o: Odometry) -> None:
        self._odom.put(
            PoseStamped(
                ts=o.ts, frame_id=o.frame_id, position=o.position, orientation=o.orientation
            )
        )

    @rpc
    def set_goal(self, goal: str | None) -> None:
        goal = (goal or "").strip() or None
        if goal:
            goal = goal.splitlines()[-1].strip()  # a briefing may precede the goal
        with self._lock:
            self._goal = goal
            self._target = self._current = ZERO
            self._zero_since = None
            self._robot = {"motion": "idle"}
        if goal is None:
            self.cmd_vel.publish(Twist.zero())
        else:
            self.agent.publish(HumanMessage(content=goal))
        self.agent_idle.publish(goal is None)

    @rpc
    def goal(self) -> str | None:
        return self._goal

    @rpc
    def set_trace_dir(self, path: str | None) -> None:
        """One request/response pair per model call under *path*; None turns it off."""
        with self._lock:
            self.config.trace_dir = Path(path) if path is not None else None
            self._seq = 0

    def _say(self, text: str) -> None:
        if text != self._last_message:
            self._last_message = text
            logger.info(text)
            self.agent.publish(AIMessage(content=text))

    def _infer_loop(self) -> None:
        period = 1.0 / self.config.rate_hz
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                self._tick()
            except Exception:
                logger.exception("TypeSafeAgent tick failed")
                self._set_target(ZERO)
            self._stop_event.wait(max(0.0, period - (time.monotonic() - t0)))

    def _tick(self) -> None:
        goal = self._goal
        if goal is None or self._client is None:
            return
        stale = self.config.stale_s
        pose, det3d, det2d = self._odom.get(stale), self._det3d.get(stale), self._det2d.get(stale)
        if pose is None or (det3d is None and det2d is None):
            self._set_target(ZERO)
            self._say("holding: no odom" if pose is None else "holding: no detections")
            return
        state = build_world_state(
            goal,
            pose,
            task=TASK,
            detections_3d=det3d,
            detections_2d=det2d,
            lidar=self._lidar.get(stale),
            robot=self._robot,
            image_size=self.config.image_size,
            lidar_band=self.config.lidar_band,
            memory=self._memory,
            now=time.monotonic(),
        )
        qs = questions(tuple(dict.fromkeys(o["label"] for o in state["objects"])))
        started, t0 = time.time(), time.monotonic()
        answers = self._client(state, qs)
        self._trace(state, qs, answers, started, time.monotonic() - t0)
        drive = decode(
            answers,
            stop_threshold=self.config.stop_threshold,
        )
        self._steer(state, drive)

    def _steer(self, state: WorldState, drive: Drive) -> None:
        """The model picked directions; geometry to the chosen target sets the magnitudes."""
        lin, ang = self.config.linear_speed, self.config.angular_speed
        target = next((o for o in state["objects"] if o["label"] == drive.target), None)
        if target is not None and "distance_m" in target:
            dist, err = target["distance_m"], abs(target.get("bearing_deg", 0.0))
            if dist <= self.config.reached_m:
                drive = replace(drive, x=0.0, y=0.0, yaw=0.0, stop=True)
            lin *= min(1.0, max(0.3, dist / SLOW_WITHIN_M))
            ang *= min(1.0, max(0.25, err / TURN_FULL_AT_DEG))
        self._set_target((drive.x * lin, drive.y * lin, drive.yaw * ang), immediate=drive.stop)
        with self._lock:
            self._robot = {
                "motion": "stopped" if drive.is_zero else "driving",
                "last_drive": dict(zip(("x", "y", "yaw"), drive.labels, strict=True)),
            }
            gave_up = (
                self._zero_since is not None
                and time.monotonic() - self._zero_since > self.config.give_up_s
            )
        self._say(
            f"drive {'/'.join(drive.labels)} stop={drive.stop} target={drive.target} confidence={drive.confidence:.2f}"
        )
        if drive.finished:
            self.finished.publish(Bool(True))
            self.set_goal(None)
            self._say(f"finished at the target {drive.target}")
        elif gave_up:
            self.set_goal(None)
            self._say("goal reached or unreachable; stopped")

    def _set_target(self, target: Vec3, *, immediate: bool = False) -> None:
        now = time.monotonic()
        with self._lock:
            self._target = target
            self._decided_at = now
            if immediate:
                self._current = ZERO
            zero = target == ZERO
            self._zero_since = (self._zero_since or now) if zero else None
        if immediate:
            self.cmd_vel.publish(Twist.zero())

    def _publish_loop(self) -> None:
        dt = 1.0 / PUBLISH_HZ
        steps = (
            self.config.linear_accel * dt,
            self.config.linear_accel * dt,
            self.config.angular_accel * dt,
        )
        deadman = self.config.deadman_s or max(1.0, 2.5 / self.config.rate_hz)
        while not self._stop_event.is_set():
            with self._lock:
                stale = time.monotonic() - self._decided_at > deadman
                target = ZERO if self._goal is None or stale else self._target
                cur = self._current
                nxt = tuple(
                    c + max(-s, min(s, t - c)) for c, t, s in zip(cur, target, steps, strict=True)
                )
                self._current = (nxt[0], nxt[1], nxt[2])
                publish = self._goal is not None or any(cur)
            if publish:
                self.cmd_vel.publish(
                    Twist(linear=(nxt[0], nxt[1], 0.0), angular=(0.0, 0.0, nxt[2]))
                )
            self._stop_event.wait(dt)

    def _trace(
        self,
        state: WorldState,
        qs: dict[str, Question],
        answers: Answers,
        started_at: float,
        latency_s: float,
    ) -> None:
        """Same layout as ``dimos.agents.llm_trace`` so eval adapters read both."""
        if self.config.trace_dir is None or self._client is None:
            return
        d = Path(self.config.trace_dir)
        d.mkdir(parents=True, exist_ok=True)
        self._seq += 1
        (d / f"{self._seq}-request.json").write_text(
            json.dumps({"started_at": started_at, "body": {"state": state, "questions": qs}})
        )
        (d / f"{self._seq}-response.json").write_text(
            json.dumps(
                {
                    "latency_s": latency_s,
                    "body": {
                        "model": self._client.last_model,
                        "answers": answers,
                        "usage": self._client.last_usage,
                    },
                }
            )
        )
