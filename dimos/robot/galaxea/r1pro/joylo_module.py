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
from dimos.robot.galaxea.r1pro.joylo import JAWS, UPPER, JoyloConfig, JoyloController
from dimos.teleop.joycon.raw import decode_readings
from dimos.teleop.leader.module import GUARDED_JOINT_FRAME_ID
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1ProJoylo(Module):
    config: JoyloConfig
    leader_readings: In[JointState]
    joycon_sample: In[Joy]
    robot_feedback: In[JointState]
    arm_tracking_speed: In[Float32]
    torso_tracking_speed: In[Float32]
    joycon_drive_speed: In[Float32]
    joycon_turn_speed: In[Float32]
    leader_joint_command: Out[JointState]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]
    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._guard = threading.RLock()
        self._controller: JoyloController | None = None
        self._logged_pause_count = 0
        self._stamps = {"leader": -math.inf, "joycon": -math.inf}
        self._feedback: dict[str, tuple[float, float]] = {}
        self._done = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        limits = {}
        for joint in self.config.model.load().joints:
            if joint.name in UPPER:
                assert joint.lower is not None and joint.upper is not None
                limits[joint.name] = (joint.lower, joint.upper)
        with self._guard:
            self._controller = JoyloController(limits, self.config)
            self._logged_pause_count = 0
            self._stamps = {"leader": -math.inf, "joycon": -math.inf}
            self._feedback.clear()
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
    def set_motion_speeds(
        self,
        arm_rate: float | None = None,
        torso_rate: float | None = None,
        base_speed: float | None = None,
        yaw_speed: float | None = None,
    ) -> dict[str, float]:
        """Change teleop rates without moving or enabling the robot.

        Args:
            arm_rate: Arm tracking in rad/s, up to this deployment's arm ceiling.
            torso_rate: Torso tracking in rad/s, up to its deployment ceiling.
            base_speed: Joy-Con drive speed in m/s (0..0.5; stick-click doubles it).
            yaw_speed: Joy-Con turn speed in rad/s (0..2).
        """
        with self._guard:
            values = dict(
                arm_rate=arm_rate, torso_rate=torso_rate, base_speed=base_speed, yaw_speed=yaw_speed
            )
            limits = dict(
                arm_rate=self.config.max_arm_rate,
                torso_rate=self.config.max_torso_rate,
                base_speed=0.5,
                yaw_speed=2.0,
            )
            for name, value in values.items():
                if value is not None and (
                    not math.isfinite(value) or not 0 <= value <= limits[name]
                ):
                    raise ValueError(f"{name} must be finite and between 0 and {limits[name]}")
            # Validate the entire request before changing any rate.
            for name, value in values.items():
                if value is not None:
                    setattr(self.config, name, value)
            return {name: float(getattr(self.config, name)) for name in values}

    async def handle_arm_tracking_speed(self, sample: Float32) -> None:
        """Desktop SET: arm tracking rate in rad/s."""
        self.set_motion_speeds(arm_rate=sample.data)

    async def handle_torso_tracking_speed(self, sample: Float32) -> None:
        """Desktop SET: torso tracking rate in rad/s."""
        self.set_motion_speeds(torso_rate=sample.data)

    async def handle_joycon_drive_speed(self, sample: Float32) -> None:
        """Desktop SET: Joy-Con translation speed in m/s (stick-click doubles it)."""
        self.set_motion_speeds(base_speed=sample.data)

    async def handle_joycon_turn_speed(self, sample: Float32) -> None:
        """Desktop SET: Joy-Con yaw rate in rad/s."""
        self.set_motion_speeds(yaw_speed=sample.data)

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
                reason = f"Ready; release and press {c.enable_button} to enable tracking"
            return {
                "active": c.active if c else False,
                "arm_speed_rad_s": self.config.arm_rate,
                "torso_speed_rad_s": self.config.torso_rate,
                "joycon_speed_m_s": self.config.base_speed,
                "joycon_turn_rad_s": self.config.yaw_speed,
                "ready": ready,
                "reason": reason,
                "pause_count": c.pause_count if c else 0,
                "last_pause_reason": c.last_pause_reason if c else None,
                "arm_locked": c.arm_locked.copy() if c else [False, False],
                "enabled_sides": self.config.enabled_sides,
                "arm_only": self.config.arm_only,
                "enable_button": c.enable_button if c else "",
                "input_age_s": ages,
                "leader_positions": c.leader.copy() if c else {},
                "arm_targets": {name: c.target[name] for name in c.arm_names if name in c.target}
                if c
                else {},
                "arm_feedback": {
                    name: c.feedback[name] for name in c.arm_names if name in c.feedback
                }
                if c
                else {},
                "joint_limit_clamps": {
                    name: {
                        "requested": value,
                        "limited": min(c.limits[name][1], max(c.limits[name][0], value)),
                    }
                    for name, value in c.leader.items()
                    if not c.limits[name][0] <= value <= c.limits[name][1]
                }
                if c
                else {},
                "locked_joints": sorted(c.locked) if c else [],
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
            wall = time.time()
            if (
                self._controller is None
                or not math.isfinite(sample.ts)
                or not -0.05 <= wall - sample.ts <= self.config.input_timeout_s
                or len(sample.name) != len(sample.position)
                or len(set(sample.name)) != len(sample.name)
                or not all(math.isfinite(value) for value in sample.position)
            ):
                return
            # Hardware sends upper body and jaws separately (and body rates with
            # no positions). Merge by joint, preserving each source timestamp:
            # fresh jaws must never renew stale arm feedback, nor vice versa.
            required = (*UPPER, *JAWS)
            for name, value in zip(sample.name, sample.position, strict=True):
                if name in required and sample.ts > self._feedback.get(name, (0, -math.inf))[1]:
                    self._feedback[name] = (value, sample.ts)
            if all(name in self._feedback for name in required):
                oldest = min(stamp for _, stamp in self._feedback.values())
                self._controller.set_feedback(
                    {name: value for name, (value, _) in self._feedback.items()},
                    time.monotonic() - max(0.0, wall - oldest),
                )

    def tick(self, now: float, dt: float) -> None:
        with self._guard:
            c = self._controller
            output = c.step(now, dt) if c else None
            if c is not None and c.pause_count != self._logged_pause_count:
                logger.warning(
                    "JoyLo following paused",
                    reason=c.last_pause_reason,
                    pause_count=c.pause_count,
                    input_age_s={name: now - stamp for name, stamp in c.receipts.items()},
                )
                self._logged_pause_count = c.pause_count
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
