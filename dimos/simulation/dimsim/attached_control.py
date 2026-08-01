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

"""Attached DimSim control using its pinned teleport, cmd_vel, and odometry APIs."""

from __future__ import annotations

from collections.abc import Callable
import math
from threading import Condition
import time

from dimos.benchmark.dimsim.models import Pose2, SceneOracleView
from dimos.benchmark.dimsim.oracle import SceneClientOracleProvider
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.simulation.dimsim.evaluation import AuthoritativeBodySample
from dimos.simulation.dimsim.scene_client import SceneClient

_CMD_VEL_TOPIC = "/cmd_vel"
_ODOM_TOPIC = "/odom"
_SETTLE_TIME_S = 0.25


class AttachedDimSimControl:
    """Own only an attached control connection; never launch or stop DimSim."""

    def __init__(self, *, host: str, port: int, scene_timeout_s: float = 30.0) -> None:
        self._scene = SceneClient(host=host, port=port, timeout=scene_timeout_s)
        self._cmd_vel: LCMTransport[Twist] = LCMTransport(_CMD_VEL_TOPIC, Twist)
        self._odom: LCMTransport[PoseStamped] = LCMTransport(_ODOM_TOPIC, PoseStamped)
        self._condition = Condition()
        self._unsubscribe: Callable[[], None] | None = None
        self._latest: AuthoritativeBodySample | None = None
        self._previous_pose: Pose2 | None = None
        self._previous_timestamp_s: float | None = None
        self._samples_since_teleport = 0
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._scene.start()
        self._cmd_vel.start()
        self._odom.start()
        self._unsubscribe = self._odom.subscribe(self._on_odom)
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        self._odom.stop()
        self._cmd_vel.stop()
        self._scene.stop()
        self._started = False

    def clear_motion(self) -> None:
        self._require_started()
        self._cmd_vel.publish(Twist.zero())

    def settle_motion(self) -> None:
        self._require_started()
        deadline = time.monotonic() + _SETTLE_TIME_S
        while time.monotonic() < deadline:
            self._cmd_vel.publish(Twist.zero())
            time.sleep(min(0.05, deadline - time.monotonic()))

    def teleport(self, pose: Pose2) -> None:
        self._require_started()
        current = self._scene.get_agent_position()
        with self._condition:
            self._latest = None
            self._previous_pose = None
            self._previous_timestamp_s = None
            self._samples_since_teleport = 0
        self._scene.set_agent_position(
            x=pose.x_m,
            y=float(current["y"]),
            z=pose.z_m,
        )

    def wait_body_sample(self, timeout_s: float) -> AuthoritativeBodySample:
        self._require_started()
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._latest is None or self._samples_since_teleport < 2:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no settled post-reset odometry within {timeout_s:.3f}s")
                self._condition.wait(remaining)
            return self._latest

    def latest_body_sample(self, timeout_s: float) -> AuthoritativeBodySample:
        """Wait for any authoritative odometry sample for rubric polling."""
        self._require_started()
        deadline = time.monotonic() + timeout_s
        with self._condition:
            baseline = None if self._latest is None else self._latest.pose_timestamp_s
            while self._latest is None or self._latest.pose_timestamp_s == baseline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no fresh authoritative odometry within {timeout_s:.3f}s")
                self._condition.wait(remaining)
            return self._latest

    def oracle_view(self) -> SceneOracleView:
        """Capture one private coherent view through the owned control client."""
        self._require_started()
        return SceneClientOracleProvider(self._scene).get_scene_oracle_view()

    def _on_odom(self, odom: PoseStamped) -> None:
        pose = Pose2(x_m=odom.y, z_m=odom.x, yaw_rad=odom.yaw)
        with self._condition:
            if self._previous_timestamp_s is not None and odom.ts <= self._previous_timestamp_s:
                return
            linear_speed = 0.0
            angular_speed = 0.0
            if self._previous_pose is not None and self._previous_timestamp_s is not None:
                duration = odom.ts - self._previous_timestamp_s
                linear_speed = (
                    math.hypot(
                        pose.x_m - self._previous_pose.x_m,
                        pose.z_m - self._previous_pose.z_m,
                    )
                    / duration
                )
                angular_speed = (
                    abs(_yaw_delta(pose.yaw_rad, self._previous_pose.yaw_rad)) / duration
                )
            self._latest = AuthoritativeBodySample(
                pose=pose,
                linear_speed_m_s=linear_speed,
                angular_speed_rad_s=angular_speed,
                simulated_time_s=odom.ts,
                pose_timestamp_s=odom.ts,
            )
            self._previous_pose = pose
            self._previous_timestamp_s = odom.ts
            self._samples_since_teleport += 1
            self._condition.notify_all()

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("attached DimSim control is not started")


def _yaw_delta(first: float, second: float) -> float:
    return (first - second + math.pi) % (2 * math.pi) - math.pi
