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

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Literal

from dimos.manipulation.planning.spec.config import RobotModelConfig

RetimingMode = Literal["dimos", "toppra", "none"]


@dataclass(frozen=True)
class RoboPlanRRTConfig:
    max_nodes: int = 1000
    max_connection_distance: float = 3.0
    collision_check_step_size: float = 0.05
    collision_check_use_bisection: bool = False
    goal_biasing_probability: float = 0.15
    max_planning_time: float = 0.0
    rrt_connect: bool = True
    seed: int | None = None


@dataclass(frozen=True)
class RoboPlanIKConfig:
    max_iters: int = 100
    max_time: float = 0.05
    max_restarts: int = 2
    step_size: float = 0.25
    damping: float = 0.001
    max_linear_error_norm: float = 0.001
    max_angular_error_norm: float = 0.001
    check_collisions: bool = True
    fast_return: bool = True


@dataclass(frozen=True)
class RoboPlanTOPPRAConfig:
    dt: float = 0.02
    mode: str = "Hermite"
    velocity_scale: float = 1.0
    acceleration_scale: float = 1.0
    max_adaptive_iterations: int = 10
    max_adaptive_step_size: float = 0.05


@dataclass(frozen=True)
class RoboPlanSceneConfig:
    primitive_obstacles: bool = True
    mesh_obstacles: bool = False
    pointcloud_layers: bool = False
    attached_objects: bool = False


@dataclass(frozen=True)
class RoboPlanBackendConfig:
    model_path: Path
    srdf_path: Path
    package_paths: list[Path]
    planning_group: str
    active_joint_names: list[str]
    base_frame: str
    end_effector_frame: str
    retiming: RetimingMode = "dimos"
    rrt: RoboPlanRRTConfig = field(default_factory=RoboPlanRRTConfig)
    ik: RoboPlanIKConfig = field(default_factory=RoboPlanIKConfig)
    toppra: RoboPlanTOPPRAConfig = field(default_factory=RoboPlanTOPPRAConfig)
    scene: RoboPlanSceneConfig = field(default_factory=RoboPlanSceneConfig)


def extract_roboplan_options(options: dict[str, Any] | None) -> dict[str, Any]:
    raw = dict(options or {})
    nested = raw.get("roboplan")
    return dict(nested) if isinstance(nested, dict) else raw


def parse_backend_config(robot: RobotModelConfig, options: dict[str, Any]) -> RoboPlanBackendConfig:
    model_path = _model_path(robot, options)
    srdf_path = _required_path(options, "srdf_path")
    package_paths = _package_paths(robot, options)
    planning_group = str(options.get("planning_group") or options.get("group_name") or robot.name)
    active_joint_names = [
        str(name) for name in options.get("active_joint_names", robot.joint_names)
    ]
    base_frame = str(options.get("base_frame", robot.base_link))
    end_effector_frame = str(options.get("end_effector_frame", robot.end_effector_link))
    retiming = _retiming_mode(options.get("retiming", "dimos"))

    _validate_robot_config(
        robot, model_path, srdf_path, active_joint_names, base_frame, end_effector_frame
    )

    return RoboPlanBackendConfig(
        model_path=model_path,
        srdf_path=srdf_path,
        package_paths=package_paths,
        planning_group=planning_group,
        active_joint_names=active_joint_names,
        base_frame=base_frame,
        end_effector_frame=end_effector_frame,
        retiming=retiming,
        rrt=RoboPlanRRTConfig(**_known(options.get("rrt", {}), RoboPlanRRTConfig)),
        ik=RoboPlanIKConfig(**_known(options.get("ik", {}), RoboPlanIKConfig)),
        toppra=RoboPlanTOPPRAConfig(**_known(options.get("toppra", {}), RoboPlanTOPPRAConfig)),
        scene=RoboPlanSceneConfig(**_known(options.get("scene", {}), RoboPlanSceneConfig)),
    )


def _required_path(options: dict[str, Any], key: str) -> Path:
    value = options.get(key)
    if not value:
        raise ValueError(f"RoboPlan backend requires planning_backend_options['{key}']")
    path = Path(value)
    if not path.exists():
        raise ValueError(f"RoboPlan backend {key} does not exist: {path}")
    return path


def _model_path(robot: RobotModelConfig, options: dict[str, Any]) -> Path:
    value = options.get("model_path", robot.model_path)
    path = Path(value)
    if not path.exists():
        raise ValueError(f"RoboPlan backend model_path does not exist: {path}")
    return path


def _package_paths(robot: RobotModelConfig, options: dict[str, Any]) -> list[Path]:
    configured = options.get("package_paths")
    if configured is None:
        return list(robot.package_paths.values())
    if isinstance(configured, dict):
        return [Path(value) for value in configured.values()]
    return [Path(value) for value in configured]


def _validate_robot_config(
    robot: RobotModelConfig,
    model_path: Path,
    srdf_path: Path,
    active_joint_names: list[str],
    base_frame: str,
    end_effector_frame: str,
) -> None:
    if not model_path.exists():
        raise ValueError(f"RoboPlan backend robot model_path does not exist: {model_path}")
    if not srdf_path.exists():
        raise ValueError(f"RoboPlan backend srdf_path does not exist: {srdf_path}")
    if not robot.joint_names:
        raise ValueError("RoboPlan backend requires RobotModelConfig.joint_names")
    if not active_joint_names:
        raise ValueError("RoboPlan backend requires at least one active joint")
    unknown = [name for name in active_joint_names if name not in robot.joint_names]
    if unknown:
        raise ValueError(f"RoboPlan active_joint_names not present in RobotModelConfig: {unknown}")
    if not base_frame:
        raise ValueError("RoboPlan backend requires a base_frame/base_link")
    if not end_effector_frame:
        raise ValueError("RoboPlan backend requires an end_effector_frame/end_effector_link")
    if robot.joint_limits_lower is None or robot.joint_limits_upper is None:
        raise ValueError("RoboPlan backend requires joint_limits_lower and joint_limits_upper")
    if len(robot.joint_limits_lower) != len(robot.joint_names):
        raise ValueError("RoboPlan joint_limits_lower length must match joint_names")
    if len(robot.joint_limits_upper) != len(robot.joint_names):
        raise ValueError("RoboPlan joint_limits_upper length must match joint_names")


def _known(raw: Any, cls: type) -> dict[str, Any]:
    if not isinstance(raw, dict):
        return {}
    known_fields = {item.name for item in fields(cls)}
    return {key: value for key, value in raw.items() if key in known_fields}


def _retiming_mode(value: Any) -> RetimingMode:
    mode = str(value).lower()
    if mode == "dimos":
        return "dimos"
    if mode == "toppra":
        return "toppra"
    if mode == "none":
        return "none"
    raise ValueError("RoboPlan retiming must be one of: 'dimos', 'toppra', 'none'")
