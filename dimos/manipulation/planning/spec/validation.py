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

"""Validation for shared manipulation planning specifications."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.robot.assets.model import JointDescription, LoadedRobotModel

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import Obstacle

_EXPECTED_DIMENSIONS = {
    ObstacleType.BOX: 3,
    ObstacleType.SPHERE: 1,
    ObstacleType.CYLINDER: 2,
}

# An octree obstacle carries one point per occupied cell across worker RPC, where
# every point is a pickled tuple. A wrist workspace at 5 cm sits in the low
# thousands; this is the ceiling that keeps a misaimed room-scale map from
# quietly stalling the pipe.
MAX_OCTREE_POINTS = 200_000
_SUPPORTED_CONTROLLED_JOINT_TYPES = {"continuous", "prismatic", "revolute"}


@dataclass(frozen=True)
class PreparedRobotModel:
    """Materialized, validated model and its compiled canonical joint space."""

    config: RobotModelConfig
    description: LoadedRobotModel
    joint_space: JointSpace
    planning_groups: tuple[PlanningGroup, ...]


def prepare_robot_model(config: RobotModelConfig) -> PreparedRobotModel:
    """Materialize and validate one robot model exactly once."""
    if not config.joint_names:
        raise ValueError("RobotModelConfig contains no controllable joints")
    try:
        model = config.model.load()
        root = ET.fromstring(model.xml)
        model_joints = model.joints
    except (ET.ParseError, OSError, ValueError) as exc:
        raise ValueError(f"RobotModelConfig has an invalid model asset: {exc}") from exc

    if config.srdf_path is not None:
        _validate_srdf(config.srdf_path)

    duplicate_model_joints = _duplicates(joint.name for joint in model_joints)
    if duplicate_model_joints:
        raise ValueError(
            f"RobotModelConfig model contains duplicate joint names: {duplicate_model_joints}"
        )
    model_links = [name for link in root.findall("link") if (name := link.get("name")) is not None]
    duplicate_links = _duplicates(model_links)
    if duplicate_links:
        raise ValueError(f"RobotModelConfig model contains duplicate link names: {duplicate_links}")

    model_joint_names = {joint.name for joint in model_joints}
    missing_joints = sorted(set(config.joint_names) - model_joint_names)
    if missing_joints:
        raise ValueError(
            f"RobotModelConfig configured joints are missing from the model: {missing_joints}"
        )
    joints_by_name = {joint.name: joint for joint in model_joints}
    unsupported_joints = {
        name: joints_by_name[name].type
        for name in config.joint_names
        if joints_by_name[name].type not in _SUPPORTED_CONTROLLED_JOINT_TYPES
    }
    if unsupported_joints:
        raise ValueError(
            "RobotModelConfig controlled joints must be one-DoF revolute, continuous, "
            f"or prismatic joints; unsupported joints: {unsupported_joints}. Use "
            "RobotModel.with_planar_base() for a floor-constrained mobile base."
        )
    model_link_names = set(model_links)
    if config.base_link not in model_link_names:
        raise ValueError(f"RobotModelConfig base link '{config.base_link}' is missing")
    planar_base = config.model.planar_base
    if planar_base is not None:
        if config.base_link != planar_base.root_link:
            raise ValueError(
                f"Planar robot base_link must be '{planar_base.root_link}', "
                f"got '{config.base_link}'"
            )
        missing_base_joints = sorted(set(planar_base.joint_names) - set(config.joint_names))
        if missing_base_joints:
            raise ValueError(f"Planar robot controllable joints are missing: {missing_base_joints}")

    duplicate_group_names = _duplicates(group.name for group in config.planning_groups)
    if duplicate_group_names:
        raise ValueError(
            f"RobotModelConfig contains duplicate planning groups: {duplicate_group_names}"
        )
    controllable = set(config.joint_names)
    for group in config.planning_groups:
        if not group.name:
            raise ValueError("RobotModelConfig contains an empty planning-group name")
        if not group.joint_names:
            raise ValueError(f"Planning group '{group.name}' contains no joints")
        duplicate_group_joints = _duplicates(group.joint_names)
        if duplicate_group_joints:
            raise ValueError(
                f"Planning group '{group.name}' contains duplicate joints: {duplicate_group_joints}"
            )
        unknown_group_joints = sorted(set(group.joint_names) - controllable)
        if unknown_group_joints:
            raise ValueError(
                f"Planning group '{group.name}' references joints outside "
                f"the controllable model set: {unknown_group_joints}"
            )
        for role, link_name in (("base", group.base_link), ("tip", group.tip_link)):
            if link_name is not None and link_name not in model_link_names:
                raise ValueError(
                    f"Planning group '{group.name}' has missing {role} link '{link_name}'"
                )
    coordinates = tuple(_compile_joint(joints_by_name[name]) for name in config.joint_names)
    groups = PlanningGroupRegistry(config.planning_groups).list()
    return PreparedRobotModel(
        config=config,
        description=model,
        joint_space=JointSpace(coordinates),
        planning_groups=groups,
    )


def _compile_joint(joint: JointDescription) -> JointCoordinate:
    lower, upper = joint.lower, joint.upper
    if (lower is None) != (upper is None):
        raise ValueError(f"Joint '{joint.name}' must define both lower and upper limits or neither")

    if joint.type == "continuous":
        if lower is not None:
            raise ValueError(f"Continuous joint '{joint.name}' must not define position limits")
        topology = CoordinateTopology.CIRCLE
    elif joint.type == "prismatic":
        topology = CoordinateTopology.LINE if lower is None else CoordinateTopology.INTERVAL
    elif joint.type == "revolute":
        if lower is None:
            raise ValueError(
                f"Revolute joint '{joint.name}' requires finite position limits; "
                "use URDF type='continuous' for a periodic coordinate"
            )
        topology = CoordinateTopology.INTERVAL
    else:
        raise ValueError(f"Unsupported controlled joint type: {joint.type}")

    if joint.velocity is None:
        raise ValueError(f"Joint '{joint.name}' is missing a velocity limit")
    if joint.acceleration is None:
        raise ValueError(
            f"Joint '{joint.name}' is missing an acceleration limit; apply "
            "RobotModel.with_default_joint_acceleration_limit() explicitly"
        )
    return JointCoordinate(
        name=joint.name,
        mechanism_type=joint.type,
        topology=topology,
        lower=lower,
        upper=upper,
        max_velocity=joint.velocity,
        max_acceleration=joint.acceleration,
    )


def _validate_srdf(srdf_path: Path) -> None:
    if not srdf_path.exists():
        raise ValueError(f"RobotModelConfig SRDF file is missing: {srdf_path}")
    try:
        root = ET.parse(srdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"RobotModelConfig has an invalid SRDF: {exc}") from exc
    if root.tag != "robot":
        raise ValueError(f"RobotModelConfig SRDF root must be <robot>, got <{root.tag}>")


def _duplicates(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for value in values:
        if value in seen:
            duplicates.add(value)
        seen.add(value)
    return sorted(duplicates)


def validate_obstacle(
    obstacle: Obstacle,
    pose_matrix: NDArray[np.float64],
    *,
    allow_empty_name: bool = False,
) -> None:
    """Validate obstacle name, dimensions, color and pose. Raises ValueError.

    The pose matrix is supplied by the caller because each backend builds it
    from its own transform utilities.
    """
    if not obstacle.name and not allow_empty_name:
        raise ValueError("Obstacle name must be non-empty")
    if obstacle.obstacle_type in _EXPECTED_DIMENSIONS:
        expected = _EXPECTED_DIMENSIONS[obstacle.obstacle_type]
        if len(obstacle.dimensions) != expected:
            raise ValueError(
                f"{obstacle.obstacle_type.name} obstacle requires {expected} dimensions, "
                f"got {len(obstacle.dimensions)}"
            )
        dimensions = np.asarray(obstacle.dimensions, dtype=np.float64)
        if not np.isfinite(dimensions).all() or np.any(dimensions <= 0.0):
            raise ValueError("Obstacle dimensions must be finite and positive")
    elif obstacle.obstacle_type == ObstacleType.MESH:
        if not obstacle.mesh_path:
            raise ValueError("MESH obstacle requires mesh_path")
    elif obstacle.obstacle_type == ObstacleType.OCTREE:
        _validate_octree(obstacle)
    else:
        raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")
    color = np.asarray(obstacle.color, dtype=np.float64)
    if color.shape != (4,) or not np.isfinite(color).all():
        raise ValueError("Obstacle color must contain four finite values")
    if not np.isfinite(pose_matrix).all():
        raise ValueError("Obstacle pose must contain only finite values")


def _validate_octree(obstacle: Obstacle) -> None:
    if obstacle.octree_resolution is None or not obstacle.octree_resolution > 0.0:
        raise ValueError("OCTREE obstacle requires a positive octree_resolution")
    if not obstacle.points:
        raise ValueError("OCTREE obstacle requires at least one point")
    if len(obstacle.points) > MAX_OCTREE_POINTS:
        raise ValueError(
            f"OCTREE obstacle has {len(obstacle.points)} points, over the "
            f"{MAX_OCTREE_POINTS} limit. Each one crosses worker RPC as a pickled "
            f"tuple, so an unbounded map stalls the pipe."
        )
    points = np.asarray(obstacle.points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("OCTREE obstacle points must each hold three coordinates")
    if not np.isfinite(points).all():
        raise ValueError("OCTREE obstacle points must be finite")
