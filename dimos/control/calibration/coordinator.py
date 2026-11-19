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

"""The control coordinator of a calibration blueprint: the calibration's targets, as they come.

:class:`CalibrationCoordinator` is the Damiao coordinator plus the port the
calibration module publishes its targets on, ``calibration_joint_command``,
bound to a pass-through task (:mod:`dimos.control.tasks.passthrough_task`,
:func:`calibration_passthrough_task`): a target reaches the connection on the
next tick, unchanged, so a step stays a step. While the task is stale (no
target for ``stale_after_s``, or ``hold`` called) the robot is sent nothing
and its holds are dropped: the connection's deadman then halts the arm where
it is, which ends the config session. A calibration module that stops
publishing lets go of the arm within the stale time plus the deadman. Its
``stop`` waits while targets come, as the connection's waits for the config
session, so a park the calibration module runs while the run stops finishes.
"""

from __future__ import annotations

import time
from typing import ClassVar

from dimos.control.connection import connection_module
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.passthrough_task.passthrough_task import (
    DEFAULT_STALE_AFTER_S,
    PASSTHROUGH_PRIORITY,
    PassthroughTask,
)
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

CALIBRATION_PASSTHROUGH_TASK = "calibration_passthrough"
CALIBRATION_COMMAND_PORT = "calibration_joint_command"
PARK_POLL_S = 0.02


def calibration_passthrough_task(joint_names: list[str]) -> TaskConfig:
    """The pass-through task over ``joint_names``, fed by ``calibration_joint_command``."""
    return TaskConfig(
        name=CALIBRATION_PASSTHROUGH_TASK,
        type="passthrough",
        joint_names=list(joint_names),
        priority=PASSTHROUGH_PRIORITY,
        params={"stale_after_s": DEFAULT_STALE_AFTER_S},
        stream_bind={"joint_command": CALIBRATION_COMMAND_PORT},
    )


class CalibrationCoordinator(DamiaoCoordinator):
    """Sends the arm the calibration's targets while they come, and nothing once they stop."""

    dedicated_worker: ClassVar[bool] = True

    calibration_joint_command: In[JointState]

    def _passthrough(self) -> PassthroughTask | None:
        task = self._tasks.get(CALIBRATION_PASSTHROUGH_TASK)
        return task if isinstance(task, PassthroughTask) else None

    def _publish_command(self, robot: str, values: dict[str, float]) -> None:
        """Nothing for the robot, and its holds dropped, unless the pass-through is fresh."""
        task = self._passthrough()
        if task is None or task.stale:
            self._connections[robot].drop_holds()
            return
        super()._publish_command(robot, values)

    @rpc
    def stop(self) -> None:
        """Wait for the calibration module to stop sending targets, then stop."""
        self._await_park()
        super().stop()

    def _await_park(self) -> None:
        """Wait while targets come, and ``CONFIG_PARK_GRACE_S`` after for a park about to begin.

        At most ``CONFIG_PARK_WAIT_S`` in all, the limits a connection's stop
        waits for its config session by.
        """
        loop = self._tick_loop
        if loop is None or not loop.is_running or self._passthrough() is None:
            return
        started = time.monotonic()
        deadline = started + connection_module.CONFIG_PARK_WAIT_S
        fresh_at = started
        logged = False
        while True:
            now = time.monotonic()
            task = self._passthrough()
            if task is not None and not task.stale:
                fresh_at = now
                if not logged:
                    logger.warning(
                        f"stopping; waiting up to {connection_module.CONFIG_PARK_WAIT_S:g} s for "
                        "the calibration module to park the arm"
                    )
                    logged = True
            elif now - fresh_at >= connection_module.CONFIG_PARK_GRACE_S:
                return
            if now >= deadline:
                logger.error(
                    "the calibration module still sent targets after "
                    f"{connection_module.CONFIG_PARK_WAIT_S:g} s; stopping with the arm where it is"
                )
                return
            time.sleep(PARK_POLL_S)
