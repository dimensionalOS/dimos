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

"""Robot feedback and actions, separate from the pose-only scene stream."""

import struct

from dimos_lcm.sim_msgs import Action, Actions, JointCommand
import numpy as np

from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.sim2.physics.interface import JointTargets
from dimos.sim2.state import Frame, Layout


def feedback(run_id: str, layout: Layout, frame: Frame) -> ControlValues:
    state = frame.state
    names, values = ["sim/time"], [state.sim_time]
    for index, joint in enumerate(layout.joints):
        for field, array in (
            ("position", state.position),
            ("velocity", state.velocity),
            ("effort", state.effort),
        ):
            names.append(joint + "/" + field)
            values.append(float(array[index]))
    for index, name in enumerate(layout.imu_frames):
        pose = state.poses[layout.bodies.index(name)]
        for field, value in zip(
            ("qx", "qy", "qz", "qw", "ax", "ay", "az", "wx", "wy", "wz"),
            (*pose[3:], *frame.imu[index]),
            strict=True,
        ):
            names.append(name + "/" + field)
            values.append(float(value))
    return ControlValues(run_id, state.wall_time, state.epoch, state.tick, names, values)


def joint_targets(message: Actions, run_id: str, layout: Layout) -> JointTargets:
    """Flatten and validate a whole action list before the owner applies any motor."""
    if message.run_id != run_id or message.model_revision != layout.revision:
        raise ValueError("action belongs to a different simulation/model")
    if message.actions_length != len(message.actions) or not message.actions or message.tick < -1:
        raise ValueError("invalid action count")
    names: list[str] = []
    columns: list[list[float]] = [[] for _ in range(5)]
    for action in message.actions:
        if action.kind != Action.JOINT_COMMAND or action.data_length != len(action.data):
            raise ValueError("unsupported action or invalid payload length")
        try:
            command = JointCommand.lcm_decode(bytes(action.data))
        except (ValueError, struct.error, UnicodeDecodeError) as error:
            raise ValueError("invalid encoded joint command") from error
        rows = (command.position, command.velocity, command.kp, command.kd, command.effort)
        if (
            command.n <= 0
            or command.n != len(command.names)
            or any(len(row) != command.n for row in rows)
        ):
            raise ValueError("joint command arrays must have equal nonzero lengths")
        names.extend(command.names)
        for column, row in zip(columns, rows, strict=True):
            column.extend(row)
    if len(names) != len(set(names)) or not np.isfinite(columns).all():
        raise ValueError("joint commands require unique names and finite values")
    return JointTargets(
        epoch=message.epoch,
        joints=tuple(names),
        positions=tuple(columns[0]),
        velocities=tuple(columns[1]),
        kp=tuple(columns[2]),
        kd=tuple(columns[3]),
        feedforward=tuple(columns[4]),
        ttl=message.ttl,
        tick=None if message.tick == -1 else message.tick,
    )
