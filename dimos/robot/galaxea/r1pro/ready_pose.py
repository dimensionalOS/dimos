#!/usr/bin/env python3
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

"""Move the R1 Pro to a teleoperation-ready pose once, at startup.

Torso upright and both elbows at ninety degrees, so the operator starts from
the posture they are most likely to be standing in themselves. Measured by
forward kinematics on the vendor URDF, that puts each gripper 0.34 m in front
of the base at 1.14 m, 0.42 m apart, forearms level -- a tray-carrying pose.
"""

from __future__ import annotations

import math
import threading
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Elbow flexion is negative on both arms (joint4, axis +Y, range -2.06..0.31).
# Every other upper-body joint stays at its URDF zero, which is torso upright
# and the upper arms hanging.
_ELBOW = -math.pi / 2
READY_POSE: dict[str, float] = {
    **{coordinator_name(joint): 0.0 for joint in UPPER_BODY_JOINTS},
    coordinator_name("left_arm_joint4"): _ELBOW,
    coordinator_name("right_arm_joint4"): _ELBOW,
}


class R1ProReadyPoseConfig(ModuleConfig):
    enabled: bool = True
    # Let the connection publish real joint state before commanding a move, so
    # the trajectory is planned from where the arms actually are.
    settle_s: float = 5.0


class R1ProReadyPoseModule(Module):
    """Publish the ready pose once, as a single velocity-bounded goal.

    One JointState on ``joint_command`` becomes a one-point trajectory that the
    coordinator's joint-trajectory task interpolates from the measured
    position, so this never steps the arms.
    """

    config: R1ProReadyPoseConfig

    joint_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._timer: threading.Timer | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.enabled:
            logger.info("ready pose disabled")
            return
        self._timer = threading.Timer(self.config.settle_s, self._send)
        self._timer.daemon = True
        self._timer.start()

    @rpc
    def stop(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        super().stop()

    def _send(self) -> None:
        names = list(READY_POSE)
        try:
            self.joint_command.publish(
                JointState(name=names, position=[READY_POSE[n] for n in names])
            )
        except Exception:
            logger.warning("ready pose publish failed", exc_info=True)
            return
        logger.info("commanded the teleoperation ready pose", joints=len(names))
