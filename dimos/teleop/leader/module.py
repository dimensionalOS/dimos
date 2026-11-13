# Copyright 2025-2026 Dimensional Inc.
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

"""Turn leader arm readings into follower joint targets and gripper openings.

The leader arms' own connection (for Feetech leaders, a ``FeetechConnection``)
publishes their joint angles. This module maps each reading onto the follower
with the robot's ``SideMapping``, drops readings that jump, low-pass filters
the targets and publishes them for the follower coordinator's trajectory and
gripper tasks.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.leader.mapping import (
    LEADER_SIDES,
    LeaderSide,
    SideCommand,
    SideMapping,
    combine_side_commands,
    low_pass_alpha,
    map_side_readings,
    smooth_positions,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
# The trajectory task treats a JointState in this frame as a streamed jog that
# expires shortly after the last message, so the leader's claim ends when it stops.
GUARDED_JOINT_FRAME_ID = "dimos/guarded-joint"


class LeaderTeleopModuleConfig(ModuleConfig):
    """Which follower joints each leader side drives, and how the targets are smoothed.

    Attributes:
        mappings: What each leader side drives; the robot folder supplies these.
        reading_period_s: How often the leader connection publishes, in
            seconds. Sets the filter's blend; a wrong value changes the cutoff.
        max_joint_jump_radians: Largest change in a follower target between
            two readings, in radians; a reading that jumps further is dropped.
        command_filter_cutoff_hz: Cutoff of the low-pass on the arm targets,
            in Hz. None passes the readings straight through.
        guarded: Publish targets as guarded jogs, which the trajectory task
            releases shortly after the leader goes quiet.
    """

    mappings: dict[LeaderSide, SideMapping]
    reading_period_s: float = Field(default=0.01, gt=0.0)
    max_joint_jump_radians: float = Field(default=0.75, gt=0.0)
    command_filter_cutoff_hz: float | None = Field(default=None, gt=0.0)
    guarded: bool = True


class LeaderTeleopModule(Module):
    """Follower joint targets and gripper openings (1 = open) from leader arm readings."""

    config: LeaderTeleopModuleConfig

    leader_joint_state: In[JointState]
    leader_joint_command: Out[JointState]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._previous_by_side: dict[LeaderSide, dict[str, float]] = {}
        self._filtered_by_joint: dict[str, float] = {}
        self._last_error: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.leader_joint_state.subscribe(self.on_reading)))

    def commands_for(self, reading: JointState) -> list[SideCommand] | None:
        """Mapped targets for every side the reading covers; None when it is dropped.

        A side whose leader joints are all missing from the reading is not
        connected and is skipped; one with only some of them is an error.
        """
        readings = dict(zip(reading.name, reading.position, strict=True))
        commands: list[SideCommand] = []
        previous: dict[LeaderSide, dict[str, float]] = {}
        try:
            for side in LEADER_SIDES:
                mapping = self.config.mappings.get(side)
                if mapping is None or not set(mapping.leader_joint_names()) & set(readings):
                    continue
                command = map_side_readings(
                    side,
                    mapping,
                    readings,
                    previous_positions_by_joint=self._previous_by_side.get(side),
                    max_joint_jump_radians=self.config.max_joint_jump_radians,
                )
                commands.append(command)
                previous[side] = command.positions_by_joint
        except ValueError as exc:
            message = str(exc)
            if message != self._last_error:
                logger.warning("leader reading dropped: %s", message)
                self._last_error = message
            return None
        self._last_error = None
        self._previous_by_side = previous
        return self._filtered(commands)

    def on_reading(self, reading: JointState) -> None:
        """Publish the follower targets for one leader reading."""
        commands = self.commands_for(reading)
        if not commands:
            return
        frame_id = GUARDED_JOINT_FRAME_ID if self.config.guarded else ""
        self.leader_joint_command.publish(combine_side_commands(commands, frame_id=frame_id))
        outputs: Mapping[LeaderSide, Out[Float32]] = {
            "left": self.left_gripper_command,
            "right": self.right_gripper_command,
        }
        for command in commands:
            if command.gripper_opening is not None:
                outputs[command.side].publish(Float32(data=command.gripper_opening))

    def _filtered(self, commands: list[SideCommand]) -> list[SideCommand]:
        """Low-pass the mapped arm targets; the jump guard already saw the raw values."""
        cutoff = self.config.command_filter_cutoff_hz
        if cutoff is None:
            return commands
        alpha = low_pass_alpha(cutoff, self.config.reading_period_s)
        filtered: list[SideCommand] = []
        for command in commands:
            positions = smooth_positions(self._filtered_by_joint, command.positions_by_joint, alpha)
            self._filtered_by_joint.update(positions)
            filtered.append(replace(command, positions_by_joint=positions))
        return filtered
