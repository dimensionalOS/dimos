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

from __future__ import annotations

from collections.abc import Sequence
import time
from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def joint_state_to_positions(joint_state: JointState, ordered_names: Sequence[str]) -> list[float]:
    name_to_value = dict(zip(joint_state.name, joint_state.position, strict=False))
    missing = [name for name in ordered_names if name not in name_to_value]
    if missing:
        raise ValueError(f"JointState missing required joints: {missing}")
    return [float(name_to_value[name]) for name in ordered_names]


def joint_state_to_velocities(joint_state: JointState, ordered_names: Sequence[str]) -> list[float]:
    if not joint_state.velocity:
        return []
    name_to_value = dict(zip(joint_state.name, joint_state.velocity, strict=False))
    return [float(name_to_value[name]) for name in ordered_names if name in name_to_value]


def make_joint_state(
    names: Sequence[str],
    positions: Sequence[float] | None = None,
    velocities: Sequence[float] | None = None,
) -> JointState:
    state = JointState.__new__(JointState)
    state.ts = time.time()
    state.frame_id = ""
    state.name = list(names)
    state.position = [float(value) for value in positions] if positions is not None else []
    state.velocity = [float(value) for value in velocities] if velocities is not None else []
    state.effort = []
    return state


def transform_from_pose(pose: PoseStamped) -> np.ndarray:
    rotation = pose.orientation.to_rotation_matrix()
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return np.asfortranarray(transform)


def pose_from_transform(transform: np.ndarray, frame_id: str) -> PoseStamped:
    return PoseStamped(
        frame_id=frame_id,
        position=Vector3(float(transform[0, 3]), float(transform[1, 3]), float(transform[2, 3])),
        orientation=Quaternion.from_rotation_matrix(transform[:3, :3]),
    )


def dimos_path_from_roboplan(
    path: Any,
    public_joint_names: Sequence[str],
    active_joint_names: Sequence[str] | None = None,
    reference_positions: Sequence[float] | None = None,
) -> list[JointState]:
    path_names = list(getattr(path, "joint_names", [])) or list(
        active_joint_names or public_joint_names
    )
    states: list[JointState] = []
    for positions in getattr(path, "positions", []):
        ordered = _expand_positions(
            path_names,
            [float(value) for value in positions],
            public_joint_names,
            reference_positions,
        )
        states.append(make_joint_state(public_joint_names, ordered))
    return states


def _expand_positions(
    source_names: Sequence[str],
    source_positions: Sequence[float],
    public_joint_names: Sequence[str],
    reference_positions: Sequence[float] | None,
) -> list[float]:
    source = dict(zip(source_names, source_positions, strict=False))
    if reference_positions is None:
        missing = [name for name in public_joint_names if name not in source]
        if missing:
            raise ValueError(
                f"RoboPlan path missing public joints and no reference was provided: {missing}"
            )
        return [float(source[name]) for name in public_joint_names]
    reference = dict(zip(public_joint_names, reference_positions, strict=False))
    reference.update(source)
    return [float(reference[name]) for name in public_joint_names]


def dimos_trajectory_from_roboplan(
    trajectory: Any, public_joint_names: Sequence[str]
) -> JointTrajectory:
    traj_names = list(getattr(trajectory, "joint_names", [])) or list(public_joint_names)
    times = [float(value) for value in getattr(trajectory, "times", [])]
    positions = list(getattr(trajectory, "positions", []))
    velocities = list(getattr(trajectory, "velocities", []))
    points: list[TrajectoryPoint] = []
    for index, values in enumerate(positions):
        state = make_joint_state(traj_names, [float(value) for value in values])
        ordered = joint_state_to_positions(state, public_joint_names)
        velocity = [0.0] * len(ordered)
        if index < len(velocities):
            velocity_state = make_joint_state(
                traj_names,
                ordered,
                [float(value) for value in velocities[index]],
            )
            maybe_velocity = joint_state_to_velocities(velocity_state, public_joint_names)
            if len(maybe_velocity) == len(ordered):
                velocity = maybe_velocity
        points.append(
            TrajectoryPoint(
                time_from_start=times[index] if index < len(times) else float(index),
                positions=ordered,
                velocities=velocity,
            )
        )
    return JointTrajectory(joint_names=list(public_joint_names), points=points)
