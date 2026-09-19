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
"""TypeSafe navigation agent: pose, detections and scan in; joystick-style cmd_vel and a
world-frame goal point out. The model picks directions; code sets magnitudes from the
goal's bearing and distance, ramps the Twist and zeroes it on a deadman.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.messages.base import BaseMessage
from reactivex import Observable, operators as ops
from reactivex.disposable import Disposable

from dimos.agents.typesafe.agent import TypeSafeAgent, TypeSafeAgentConfig
from dimos.agents.typesafe.constants import (
    ANGULAR_ACCEL,
    ANGULAR_SPEED,
    DEADMAN_MIN_S,
    DEADMAN_PERIODS,
    GIVE_UP_S,
    IMAGE_SIZE,
    LIDAR_BAND,
    LINEAR_ACCEL,
    LINEAR_SPEED,
    MIN_PROBABILITY,
    NAV_MAX_HZ,
    PUBLISH_HZ,
    REACHED_M,
    SLOW_WITHIN_M,
    STALE_S,
    STOP_THRESHOLD,
    TURN_FULL_AT_DEG,
)
from dimos.agents.typesafe.drive import Drive, decode, questions
from dimos.agents.typesafe.types import Answers, Question
from dimos.agents.typesafe.world_state import WorldState, build_world_state
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.types.timestamped import Timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import LatestReader, getter_streaming

logger = setup_logger()

Vec3 = tuple[float, float, float]
ZERO: Vec3 = (0.0, 0.0, 0.0)


class TypeSafeNavigationConfig(TypeSafeAgentConfig):
    max_hz: float | None = NAV_MAX_HZ
    stale_s: float = STALE_S
    deadman_s: float | None = None  # None: max(DEADMAN_MIN_S, DEADMAN_PERIODS / max_hz)
    linear_speed: float = LINEAR_SPEED
    angular_speed: float = ANGULAR_SPEED
    linear_accel: float = LINEAR_ACCEL
    angular_accel: float = ANGULAR_ACCEL
    min_probability: float = MIN_PROBABILITY
    stop_threshold: float = STOP_THRESHOLD
    reached_m: float = REACHED_M
    give_up_s: float = GIVE_UP_S
    image_size: tuple[int, int] = IMAGE_SIZE
    lidar_band: tuple[float, float, float] = LIDAR_BAND


class TypeSafeNavigationAgent(TypeSafeAgent):
    config: TypeSafeNavigationConfig

    odom: In[PoseStamped]
    odometry: In[Odometry]  # same pose, nav_msgs flavour (habitat)
    detections_3d: In[Detection3DArray]
    detections_2d: In[Detection2DArray]
    lidar: In[PointCloud2]
    human_input: In[str]

    cmd_vel: Out[Twist]
    goal: Out[PointStamped]  # world-frame XY the goal text resolved to
    agent: Out[BaseMessage]
    agent_idle: Out[bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._goal: str | None = None
        self._goal_gen = 0  # bumps on every set_goal; a late answer for an old goal is dropped
        self._goal_xy: tuple[float, float] | None = None
        self._motion = "idle"
        self._target: Vec3 = ZERO
        self._current: Vec3 = ZERO
        self._decided_at = 0.0
        self._zero_since: float | None = None
        self._last_said = ""
        self._stop_event = threading.Event()
        self._publisher: threading.Thread | None = None

    # ---- inputs ------------------------------------------------------------------
    def trigger(self) -> Observable[object]:
        pose: Observable[object] = self.odom.observable().pipe(
            ops.merge(self.odometry.observable().pipe(ops.map(self._pose_of)))
        )
        return pose

    @staticmethod
    def _pose_of(o: Odometry) -> PoseStamped:
        return PoseStamped(
            ts=o.ts, frame_id=o.frame_id, position=o.position, orientation=o.orientation
        )

    def _fresh(self, name: str) -> Any:
        """Latest message on `name`, or None when absent or older than `stale_s`."""
        try:
            msg: Timestamped = self._latest[name]()
        except TimeoutError:
            return None
        return msg if time.time() - msg.ts <= self.config.stale_s else None

    # ---- generic hooks -------------------------------------------------------------
    def state(self, trigger: object) -> WorldState | None:
        goal = self._goal
        if goal is None:
            return None
        assert isinstance(trigger, PoseStamped)
        pose, det3d, det2d = trigger, self._fresh("detections_3d"), self._fresh("detections_2d")
        if det3d is None and det2d is None and self._goal_xy is None:
            self._set_target(ZERO)
            self._say("holding: no detections")
            return None
        state = build_world_state(
            goal,
            pose,
            self._motion,
            detections_3d=det3d,
            detections_2d=det2d,
            lidar=self._fresh("lidar"),
            goal_xy=self._goal_xy,
            image_size=self.config.image_size,
            lidar_band=self.config.lidar_band,
        )
        self._state_gen = self._goal_gen
        return state

    def questions(self, state: object) -> dict[str, Question]:
        assert isinstance(state, dict)
        return questions(tuple(dict.fromkeys(o["label"] for o in state["objects"])))

    def on_answers(self, state: object, answers: Answers) -> None:
        assert isinstance(state, dict)
        if self._state_gen != self._goal_gen:
            return  # the goal changed while this request was in flight
        drive = decode(
            answers,
            min_probability=self.config.min_probability,
            stop_threshold=self.config.stop_threshold,
        )
        self._steer(state, drive)  # type: ignore[arg-type]

    # ---- lifecycle -----------------------------------------------------------------
    @rpc
    def start(self) -> None:
        self._latest: dict[str, LatestReader[Any]] = {
            "detections_3d": getter_streaming(
                self.detections_3d.observable(), timeout=0, nonblocking=True
            ),
            "detections_2d": getter_streaming(
                self.detections_2d.observable(), timeout=0, nonblocking=True
            ),
            "lidar": getter_streaming(self.lidar.observable(), timeout=0, nonblocking=True),
        }
        self._state_gen = 0
        super().start()
        self.register_disposable(Disposable(self.human_input.subscribe(self.set_goal)))
        self._stop_event.clear()
        self._publisher = threading.Thread(
            target=self._publish_loop, name="TypeSafeNav-publish", daemon=True
        )
        self._publisher.start()
        self.agent_idle.publish(True)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._publisher is not None:
            self._publisher.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        for reader in self._latest.values():
            reader.dispose()
        self.cmd_vel.publish(Twist.zero())
        super().stop()

    @rpc
    def set_goal(self, goal: str | None) -> None:
        goal = (goal or "").strip() or None
        with self._lock:
            self._goal = goal
            self._goal_gen += 1
            self._goal_xy = None
            self._target = self._current = ZERO
            self._zero_since = None
            self._motion = "idle"
        if goal is None:
            self.cmd_vel.publish(Twist.zero())
        else:
            self.agent.publish(HumanMessage(content=goal))
        self.agent_idle.publish(goal is None)

    @rpc
    def current_goal(self) -> str | None:
        return self._goal

    # ---- steering ------------------------------------------------------------------
    def _say(self, text: str) -> None:
        if text != self._last_said:
            self._last_said = text
            logger.info(text)
            self.agent.publish(AIMessage(content=text))

    def _steer(self, state: WorldState, drive: Drive) -> None:
        """The model picked directions; geometry to the goal point sets the magnitudes."""
        target = next((o for o in state["objects"] if o["label"] == drive.target), None)
        if target is not None and "position" in target:
            self._resolve_goal(target["position"]["x"], target["position"]["y"])
            dist, err = target.get("distance_m", 0.0), abs(target.get("bearing_deg", 0.0))
        elif "goal_point" in state:
            dist, err = state["goal_point"]["distance_m"], abs(state["goal_point"]["bearing_deg"])
        else:
            dist, err = None, 0.0
        lin, ang = self.config.linear_speed, self.config.angular_speed
        if dist is not None:
            if dist <= self.config.reached_m:
                drive = Drive(0.0, 0.0, 0.0, True, drive.confidence, drive.labels, drive.target)
            lin *= min(1.0, max(0.3, dist / SLOW_WITHIN_M))
            ang *= min(1.0, max(0.25, err / TURN_FULL_AT_DEG))
        self._set_target((drive.x * lin, drive.y * lin, drive.yaw * ang), immediate=drive.stop)
        with self._lock:
            self._motion = "stopped" if drive.is_zero else "driving"
            gave_up = (
                self._zero_since is not None
                and time.monotonic() - self._zero_since > self.config.give_up_s
            )
        self._say(
            f"drive {'/'.join(drive.labels)} stop={drive.stop} target={drive.target} confidence={drive.confidence:.2f}"
        )
        if gave_up:
            self.set_goal(None)
            self._say("goal reached or unreachable; stopped")

    def _resolve_goal(self, x: float, y: float) -> None:
        """Latch the target's world XY; publish it when it moves more than 10 cm."""
        with self._lock:
            prev = self._goal_xy
            self._goal_xy = (x, y)
        if prev is None or abs(prev[0] - x) > 0.1 or abs(prev[1] - y) > 0.1:
            self.goal.publish(PointStamped(x, y, 0.0, frame_id="world"))

    def _set_target(self, target: Vec3, *, immediate: bool = False) -> None:
        now = time.monotonic()
        with self._lock:
            self._target = target
            self._decided_at = now
            if immediate:
                self._current = ZERO
            self._zero_since = (self._zero_since or now) if target == ZERO else None
        if immediate:
            self.cmd_vel.publish(Twist.zero())

    def _publish_loop(self) -> None:
        dt = 1.0 / PUBLISH_HZ
        steps = (
            self.config.linear_accel * dt,
            self.config.linear_accel * dt,
            self.config.angular_accel * dt,
        )
        deadman = self.config.deadman_s or max(
            DEADMAN_MIN_S, DEADMAN_PERIODS / (self.config.max_hz or 1.0)
        )
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
