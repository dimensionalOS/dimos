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

"""Internal Go2 device: the stock walking API backed by a resident sim2 robot."""

from pathlib import Path
from threading import Event, Lock, Thread
import time
from typing import Any

import numpy as np
from reactivex import empty
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.robot.unitree.go2.controllers.locomotion.policy import (
    COMMAND_SLEW,
    CONTROL_DT,
    Proprioception,
    load_policy,
    projected_gravity,
)
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import StreamDescriptor
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Go2SimDevice:
    def __init__(self, policy: Path | None = None) -> None:
        self._policy_path = policy
        self._client: SimRobotClient | None = None
        self._odom: Subject[PoseStamped] = Subject()
        self._stop = Event()
        self._lock = Lock()
        self._command = np.zeros(3)
        self._expires = 0.0
        self._thread: Thread | None = None
        self._error: str | None = None

    def bind(self, descriptor: StreamDescriptor) -> None:
        self._client = SimRobotClient(descriptor, writable=True)
        self._client.connect()

    def start(self) -> None:
        if self._client is None:
            raise RuntimeError("Go2 requires a collected sim2 world before starting")
        self._policy = load_policy(self._policy_path)
        names = self._client.descriptor.joint_names
        if set(names) != set(self._policy.joint_names):
            raise ValueError("Go2 policy joints do not match the physical robot")
        self._policy_order = np.array([names.index(name) for name in self._policy.joint_names])
        self._motor_order = np.argsort(self._policy_order)
        self._error = None
        self._stop.clear()
        self._thread = Thread(target=self._run, name="go2-locomotion", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise RuntimeError("Go2 policy did not stop")
        if self._client is not None:
            self._client.close()
            self._client = None
        self._odom.on_completed()

    def _run(self) -> None:
        assert self._client is not None
        policy = self._policy
        command = np.zeros(3)
        episode = -1
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                frame = self._client.read()
                v = frame.values
                if frame.metadata.episode_id != episode:
                    policy.reset()
                    command[:] = 0
                    if episode != -1:
                        self.stop_movement()
                    episode = frame.metadata.episode_id
                with self._lock:
                    desired = self._command if time.monotonic() < self._expires else np.zeros(3)
                    command += np.clip(desired - command, -COMMAND_SLEW, COMMAND_SLEW)
                target = policy.act(
                    Proprioception(
                        angular_velocity=v["imu_gyroscope"],
                        gravity=projected_gravity(v["imu_quaternion"]),
                        joint_position=v["position"][self._policy_order],
                        joint_velocity=v["velocity"][self._policy_order],
                    ),
                    command,
                )
                try:
                    self._client.write_motors(
                        MotorCommandArray(
                            q=target[self._motor_order].tolist(),
                            kp=policy.kp[self._motor_order].tolist(),
                            kd=policy.kd[self._motor_order].tolist(),
                            timestamp=float(v["wall_time"][0]),
                        ),
                        observed=frame,
                    )
                except RuntimeError:
                    # A reset during inference invalidates this result, not the policy loop.
                    if self._client.read().metadata.episode_id != episode:
                        continue
                    raise
                w, x, y, z = v["root_quaternion"]
                self._odom.on_next(
                    PoseStamped(
                        position=Vector3(*v["root_position"]),
                        orientation=Quaternion(x, y, z, w),
                        frame_id="world",
                        ts=float(v["wall_time"][0]),
                    )
                )
                deadline += CONTROL_DT
                self._stop.wait(max(0, deadline - time.monotonic()))
                if time.monotonic() - deadline > CONTROL_DT:
                    deadline = time.monotonic()
        except Exception as exc:
            self._error = str(exc)
            logger.exception("Go2 policy stopped; physics command watchdog will expire")

    def check_health(self) -> None:
        if self._error is not None:
            raise RuntimeError(f"Go2 policy stopped: {self._error}")
        if self._thread is None or not self._thread.is_alive():
            raise RuntimeError("Go2 policy is not running")

    def move(self, twist: Twist, duration: float = 0) -> bool:
        command = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        if duration < 0 or not np.isfinite([*command, duration]).all():
            raise ValueError("Go2 movement must be finite with a nonnegative duration")
        self.check_health()
        with self._lock:
            self._command = command
            self._expires = time.monotonic() + (duration or CMD_VEL_TIMEOUT)
        return True

    def stop_movement(self) -> None:
        with self._lock:
            self._command[:] = 0
            self._expires = 0

    def standup(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def balance_stand(self) -> bool:
        self.stop_movement()
        return self.standup()

    def liedown(self) -> bool:
        raise NotImplementedError("the Go2 locomotion policy does not implement lie-down")

    def sport_command(self, api_id: int) -> bool:
        raise NotImplementedError("vendor sport actions are not provided by the walking policy")

    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        return False

    def set_rage_mode(self, enable: bool) -> bool:
        return False

    def set_light(self, level: int) -> bool:
        return False

    def switch_joystick(self, enable: bool = True) -> bool:
        return False

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        raise NotImplementedError("vendor requests are not simulated")

    def odom_stream(self) -> Observable[PoseStamped]:
        return self._odom

    def lidar_stream(self) -> Observable[PointCloud2]:
        return empty()

    def video_stream(self) -> Observable[Image]:
        return empty()

    def lowstate_stream(self) -> Observable[LowStateMsg]:
        return empty()
