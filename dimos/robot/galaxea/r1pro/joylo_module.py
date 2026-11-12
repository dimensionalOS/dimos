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

"""Robot-side JoyLo authority and mapping, shared by simulation and hardware."""

import math
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.joylo import UPPER, JoyloConfig, JoyloController
from dimos.teleop.joycon.raw import decode_readings
from dimos.teleop.leader.module import GUARDED_JOINT_FRAME_ID


class R1ProJoylo(Module):
    config: JoyloConfig
    leader_readings: In[JointState]
    joycon_sample: In[Joy]
    robot_feedback: In[JointState]
    leader_joint_command: Out[JointState]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]
    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._guard = threading.RLock()
        self._controller: JoyloController | None = None
        self._stamps = {"leader": -math.inf, "joycon": -math.inf}
        self._done = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        limits = {}
        for joint in R1PRO_MODEL.load().joints:
            if joint.name in UPPER:
                assert joint.lower is not None and joint.upper is not None
                limits[joint.name] = (joint.lower, joint.upper)
        with self._guard:
            self._controller = JoyloController(limits, self.config)
            self._stamps = {"leader": -math.inf, "joycon": -math.inf}
        self._done.clear()
        super().start()
        self._thread = threading.Thread(target=self._run, name="r1pro-joylo", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise RuntimeError("JoyLo polling thread did not stop")
            self._thread = None
        self.cmd_vel.publish(Twist.zero())
        super().stop()

    @rpc
    def pause(self) -> None:
        """Latch motion off; enabling requires a fresh profile enable-button press."""
        with self._guard:
            if self._controller is not None:
                self._controller.pause("Paused by operator")

    @rpc
    def status(self) -> dict[str, Any]:
        with self._guard:
            c = self._controller
            ages = (
                {name: time.monotonic() - stamp for name, stamp in c.receipts.items()} if c else {}
            )
            ready = bool(
                c
                and all(age <= self.config.input_timeout_s for age in ages.values())
                and all(
                    c.readings[("left", "right").index(side)] is not None
                    for side in self.config.enabled_sides
                )
            )
            reason = c.reason if c else "Not started"
            if c is not None and ready and not c.active and reason.startswith("Waiting for fresh "):
                reason = f"Ready; release and press {c.enable_button} to align and enable"
            return {
                "active": c.active if c else False,
                "ready": ready,
                "reason": reason,
                "arm_locked": c.arm_locked.copy() if c else [False, False],
                "enabled_sides": self.config.enabled_sides,
                "arm_only": self.config.arm_only,
                "enable_button": c.enable_button if c else "",
                "input_age_s": ages,
            }

    def _fresh(self, source: str, stamp: float) -> bool:
        # Receipt time alone would give a queued, old command a new lease.
        age = time.time() - stamp
        if (
            not math.isfinite(stamp)
            or not -0.05 <= age <= self.config.input_timeout_s
            or stamp <= self._stamps[source]
        ):
            return False
        self._stamps[source] = stamp
        return True

    async def handle_leader_readings(self, sample: JointState) -> None:
        with self._guard:
            if self._controller is None or not self._fresh("leader", sample.ts):
                return
            if len(sample.name) != len(sample.position) or len(set(sample.name)) != len(
                sample.name
            ):
                self._controller.pause("Malformed leader sample")
                return
            self._controller.set_leader(
                dict(zip(sample.name, sample.position, strict=True)), time.monotonic()
            )

    async def handle_joycon_sample(self, sample: Joy) -> None:
        with self._guard:
            if self._controller is None or not self._fresh("joycon", sample.ts):
                return
            try:
                readings = decode_readings(sample)
            except ValueError:
                self._controller.pause("Malformed Joy-Con sample")
                self._controller.set_joycon(None, None, time.monotonic())
                return
            self._controller.set_joycon(readings["left"], readings["right"], time.monotonic())

    async def handle_robot_feedback(self, sample: JointState) -> None:
        with self._guard:
            if self._controller is not None and len(sample.name) == len(sample.position):
                self._controller.set_feedback(
                    dict(zip(sample.name, sample.position, strict=True)), time.monotonic()
                )

    def tick(self, now: float, dt: float) -> None:
        with self._guard:
            c = self._controller
            output = c.step(now, dt) if c else None
            if output is None:
                self.cmd_vel.publish(Twist.zero())
                return
            self.leader_joint_command.publish(
                JointState(
                    name=list(output.positions),
                    position=list(output.positions.values()),
                    frame_id=GUARDED_JOINT_FRAME_ID,
                )
            )
            self.left_gripper_command.publish(Float32(data=output.openings[0]))
            self.right_gripper_command.publish(Float32(data=output.openings[1]))
            x, y, yaw = output.velocity
            self.cmd_vel.publish(Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw)))

    def _run(self) -> None:
        previous = time.monotonic()
        deadline = previous
        while not self._done.is_set():
            now = time.monotonic()
            self.tick(now, now - previous)
            previous = now
            deadline = max(deadline + self.config.tick_period_s, now)
            self._done.wait(max(0.0, deadline - time.monotonic()))
