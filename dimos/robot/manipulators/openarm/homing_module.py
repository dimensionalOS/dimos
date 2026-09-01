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

"""Quest-triggered return to the OpenArm home pose.

Pressing the configured button while teleop is disengaged sends both arms
to the hanging rest pose through the coordinator's trajectory task, so
every recording episode starts from the same configuration. The motion is
a slow joint-space interpolation from wherever the arms are; clear the
workspace obstacles by teleop first, release the deadman, then press the
button.
"""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.openarm.config import OPENARM_ARM_JOINTS
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Measured collection-ready pose (elbows bent forward), captured on the
# Alfred-mounted rig 2026-09-01.
OPENARM_HOME_POSITIONS: dict[str, float] = {
    "openarm_left_joint1": 0.006,
    "openarm_left_joint2": 0.002,
    "openarm_left_joint3": 0.005,
    "openarm_left_joint4": 1.554,
    "openarm_left_joint5": -0.002,
    "openarm_left_joint6": -0.005,
    "openarm_left_joint7": 0.000,
    "openarm_right_joint1": -0.008,
    "openarm_right_joint2": -0.003,
    "openarm_right_joint3": 0.097,
    "openarm_right_joint4": 1.564,
    "openarm_right_joint5": -0.013,
    "openarm_right_joint6": 0.046,
    "openarm_right_joint7": -0.022,
}


class OpenArmHomingModuleConfig(ModuleConfig):
    button: str = "right_thumbstick"
    max_joint_speed_rad_s: float = 0.15
    min_duration_s: float = 5.0
    max_duration_s: float = 15.0
    home_positions: dict[str, float] | None = None


class OpenArmHomingModule(Module):
    """Send the arms home on a button press while the deadman is released."""

    config: OpenArmHomingModuleConfig

    teleop_buttons: In[Buttons]
    _control_coordinator: ControlCoordinator

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._button_was_down = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))

    def _home_positions(self) -> dict[str, float]:
        return self.config.home_positions or OPENARM_HOME_POSITIONS

    def _on_buttons(self, msg: Buttons) -> None:
        bits = int(msg.data)
        button_down = bool((bits >> Buttons.BITS[self.config.button]) & 1)
        deadman_held = bool((bits >> Buttons.BITS["left_primary"]) & 1) or bool(
            (bits >> Buttons.BITS["right_primary"]) & 1
        )
        rising_edge = button_down and not self._button_was_down
        self._button_was_down = button_down
        if not rising_edge:
            return
        if deadman_held:
            logger.warning("Ignoring home request while a teleop deadman button is held")
            return
        self.go_home()

    @rpc
    def go_home(self) -> bool:
        """Move both arms to the home pose through the trajectory task."""
        current = self._control_coordinator.get_joint_positions()
        missing = [name for name in OPENARM_ARM_JOINTS if name not in current]
        if missing:
            logger.error("Cannot home, current positions missing joints: %s", missing)
            return False

        home = self._home_positions()
        start = [current[name] for name in OPENARM_ARM_JOINTS]
        goal = [home[name] for name in OPENARM_ARM_JOINTS]
        max_delta = max(abs(g - s) for g, s in zip(goal, start, strict=True))
        duration = min(
            max(max_delta / self.config.max_joint_speed_rad_s, self.config.min_duration_s),
            self.config.max_duration_s,
        )
        # Smoothstep easing: velocity rises from zero and settles back to
        # zero, instead of the abrupt constant-velocity ramp of two points.
        steps = max(int(duration * 20), 2)
        points = []
        for i in range(steps + 1):
            f = i / steps
            eased = f * f * (3.0 - 2.0 * f)
            points.append(
                TrajectoryPoint(
                    time_from_start=duration * f,
                    positions=[s + (g - s) * eased for s, g in zip(start, goal, strict=True)],
                )
            )
        trajectory = JointTrajectory(
            joint_names=list(OPENARM_ARM_JOINTS),
            points=points,
        )
        result = self._control_coordinator.execute_trajectory(trajectory)
        accepted = result.status is TrajectoryExecutionStatus.ACCEPTED
        log = logger.info if accepted else logger.error
        log(
            "Homing over %.1fs (max delta %.2f rad): %s %s",
            duration,
            max_delta,
            result.status,
            result.message,
        )
        return accepted
