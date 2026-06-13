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

"""Robot-scoped MuJoCo model bindings.

Whole-body policy sims must not infer robot state from global MuJoCo model
order. Scene entities, extra freejoints, or attached robots can change global
joint ordering. A ``RobotSimBinding`` resolves the robot a policy controls and
gives downstream code explicit qpos, actuator, joint, and sensor indices.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco

from dimos.simulation.utils.xml_parser import JointMapping

_MJOBJ_BODY = int(mujoco.mjtObj.mjOBJ_BODY)
_MJOBJ_JOINT = int(mujoco.mjtObj.mjOBJ_JOINT)
_MJOBJ_ACTUATOR = int(mujoco.mjtObj.mjOBJ_ACTUATOR)
_MJOBJ_SENSOR = int(mujoco.mjtObj.mjOBJ_SENSOR)  # type: ignore[attr-defined]
_MJJNT_FREE = int(mujoco.mjtJoint.mjJNT_FREE)  # type: ignore[attr-defined]


@dataclass(frozen=True)
class RobotSimSpec:
    """Robot contract requested by a blueprint before starting policy sim."""

    robot_id: str
    hardware_joints: tuple[str, ...]
    root_body_names: tuple[str, ...] = ()
    root_joint_names: tuple[str, ...] = ()
    require_floating_base: bool = False
    model_joint_names: tuple[str, ...] | None = None
    model_actuator_names: tuple[str, ...] | None = None
    imu_quat_names: tuple[str, ...] = ()
    imu_gyro_names: tuple[str, ...] = ()
    imu_accel_names: tuple[str, ...] = ()
    imu_linvel_names: tuple[str, ...] = ()
    model_prefix: str | None = None
    require_imu: bool = False


@dataclass(frozen=True)
class RobotSimBinding:
    """Resolved robot indices inside one compiled MuJoCo model."""

    robot_id: str
    model_prefix: str | None
    hardware_joints: tuple[str, ...]
    joint_ids: tuple[int, ...]
    joint_qpos_adrs: tuple[int, ...]
    joint_qvel_adrs: tuple[int, ...]
    actuator_ids: tuple[int, ...]
    joint_mappings: tuple[JointMapping, ...]
    root_body_id: int | None
    root_joint_id: int | None
    root_qpos_adr: int | None
    root_qvel_adr: int | None
    imu_quat_slice: slice | None
    imu_gyro_slice: slice | None
    imu_accel_slice: slice | None
    imu_linvel_slice: slice | None


def mjcf_joint_names_from_hardware(hardware_joints: tuple[str, ...]) -> tuple[str, ...]:
    """Convert ``robot/joint`` hardware names to common MJCF joint names."""

    names: list[str] = []
    for hardware_joint in hardware_joints:
        short = hardware_joint.split("/", 1)[-1]
        names.append(short if short.endswith("_joint") else f"{short}_joint")
    return tuple(names)


def resolve_robot_sim_binding(
    model: mujoco.MjModel,
    spec: RobotSimSpec,
    joint_mappings: list[JointMapping],
) -> RobotSimBinding:
    """Resolve and validate one robot's control contract in ``model``."""

    root_body_id: int | None = None
    root_joint_id: int | None = None
    root_qpos_adr: int | None = None
    root_qvel_adr: int | None = None

    if spec.root_body_names:
        root_body_id = _find_unique_id(
            model,
            _MJOBJ_BODY,
            spec.root_body_names,
            spec.model_prefix,
            "root body",
        )

    if spec.root_joint_names:
        root_joint_id = _find_unique_id(
            model,
            _MJOBJ_JOINT,
            spec.root_joint_names,
            spec.model_prefix,
            "root joint",
        )
        if int(model.jnt_type[root_joint_id]) != _MJJNT_FREE:
            root_name = _name(model, _MJOBJ_JOINT, root_joint_id)
            raise ValueError(f"Robot '{spec.robot_id}' root joint '{root_name}' is not a freejoint")
        if root_body_id is not None and int(model.jnt_bodyid[root_joint_id]) != root_body_id:
            root_joint_name = _name(model, _MJOBJ_JOINT, root_joint_id)
            root_body_name = _name(model, _MJOBJ_BODY, root_body_id)
            raise ValueError(
                f"Robot '{spec.robot_id}' root joint '{root_joint_name}' is not on "
                f"root body '{root_body_name}'"
            )
        root_qpos_adr = int(model.jnt_qposadr[root_joint_id])
        root_qvel_adr = int(model.jnt_dofadr[root_joint_id])
    elif spec.require_floating_base:
        raise ValueError(f"Robot '{spec.robot_id}' requires a configured floating-base root")

    model_joint_names = spec.model_joint_names or mjcf_joint_names_from_hardware(
        spec.hardware_joints
    )
    if len(model_joint_names) != len(spec.hardware_joints):
        raise ValueError(
            f"Robot '{spec.robot_id}' has {len(spec.hardware_joints)} hardware joints "
            f"but {len(model_joint_names)} MJCF joint names"
        )
    model_actuator_names = spec.model_actuator_names
    if model_actuator_names is not None and len(model_actuator_names) != len(spec.hardware_joints):
        raise ValueError(
            f"Robot '{spec.robot_id}' has {len(spec.hardware_joints)} hardware joints "
            f"but {len(model_actuator_names)} MJCF actuator names"
        )

    mappings_by_joint_id = {
        mapping.joint_id: mapping for mapping in joint_mappings if mapping.joint_id is not None
    }
    ordered_mappings: list[JointMapping] = []
    joint_ids: list[int] = []
    joint_qpos_adrs: list[int] = []
    joint_qvel_adrs: list[int] = []
    actuator_ids: list[int] = []

    for index, model_joint_name in enumerate(model_joint_names):
        joint_id = _find_unique_id(
            model,
            _MJOBJ_JOINT,
            (model_joint_name,),
            spec.model_prefix,
            f"policy joint {index}",
        )
        if int(model.jnt_type[joint_id]) == _MJJNT_FREE:
            joint_name = _name(model, _MJOBJ_JOINT, joint_id)
            raise ValueError(f"Robot '{spec.robot_id}' policy joint '{joint_name}' is free")
        mapping = mappings_by_joint_id.get(joint_id)
        if mapping is None:
            joint_name = _name(model, _MJOBJ_JOINT, joint_id)
            raise ValueError(
                f"Robot '{spec.robot_id}' policy joint '{joint_name}' has no actuator mapping"
            )
        actuator_id = mapping.actuator_id
        if actuator_id is None:
            joint_name = _name(model, _MJOBJ_JOINT, joint_id)
            raise ValueError(f"Robot '{spec.robot_id}' policy joint '{joint_name}' has no actuator")
        if model_actuator_names is not None:
            expected_actuator_id = _find_unique_id(
                model,
                _MJOBJ_ACTUATOR,
                (model_actuator_names[index],),
                spec.model_prefix,
                f"policy actuator {index}",
            )
            if actuator_id != expected_actuator_id:
                joint_name = _name(model, _MJOBJ_JOINT, joint_id)
                raise ValueError(
                    f"Robot '{spec.robot_id}' joint '{joint_name}' actuator mismatch: "
                    f"mapping={actuator_id}, expected={expected_actuator_id}"
                )
        if mapping.qpos_adr is None or mapping.dof_adr is None:
            joint_name = _name(model, _MJOBJ_JOINT, joint_id)
            raise ValueError(f"Robot '{spec.robot_id}' policy joint '{joint_name}' has no qpos")

        ordered_mappings.append(mapping)
        joint_ids.append(joint_id)
        joint_qpos_adrs.append(mapping.qpos_adr)
        joint_qvel_adrs.append(mapping.dof_adr)
        actuator_ids.append(actuator_id)

    imu_quat_slice = _find_sensor_slice(model, spec.imu_quat_names, spec.model_prefix, dim=4)
    imu_gyro_slice = _find_sensor_slice(model, spec.imu_gyro_names, spec.model_prefix, dim=3)
    imu_accel_slice = _find_sensor_slice(model, spec.imu_accel_names, spec.model_prefix, dim=3)
    imu_linvel_slice = _find_sensor_slice(model, spec.imu_linvel_names, spec.model_prefix, dim=3)
    if spec.require_imu and (
        (imu_quat_slice is None and root_qpos_adr is None)
        or imu_gyro_slice is None
        or imu_accel_slice is None
    ):
        raise ValueError(
            f"Robot '{spec.robot_id}' requires IMU orientation+gyro+accel, "
            f"got quat_or_root={imu_quat_slice is not None or root_qpos_adr is not None}, "
            f"gyro={imu_gyro_slice is not None}, accel={imu_accel_slice is not None}"
        )

    return RobotSimBinding(
        robot_id=spec.robot_id,
        model_prefix=spec.model_prefix,
        hardware_joints=spec.hardware_joints,
        joint_ids=tuple(joint_ids),
        joint_qpos_adrs=tuple(joint_qpos_adrs),
        joint_qvel_adrs=tuple(joint_qvel_adrs),
        actuator_ids=tuple(actuator_ids),
        joint_mappings=tuple(ordered_mappings),
        root_body_id=root_body_id,
        root_joint_id=root_joint_id,
        root_qpos_adr=root_qpos_adr,
        root_qvel_adr=root_qvel_adr,
        imu_quat_slice=imu_quat_slice,
        imu_gyro_slice=imu_gyro_slice,
        imu_accel_slice=imu_accel_slice,
        imu_linvel_slice=imu_linvel_slice,
    )


def _find_sensor_slice(
    model: mujoco.MjModel,
    names: tuple[str, ...],
    model_prefix: str | None,
    *,
    dim: int,
) -> slice | None:
    if not names:
        return None
    sensor_id = _find_unique_id_or_none(model, _MJOBJ_SENSOR, names, model_prefix, "sensor")
    if sensor_id is None:
        return None
    sensor_dim = int(model.sensor_dim[sensor_id])
    if sensor_dim != dim:
        sensor_name = _name(model, _MJOBJ_SENSOR, sensor_id)
        raise ValueError(f"MuJoCo sensor '{sensor_name}' has dim {sensor_dim}, expected {dim}")
    adr = int(model.sensor_adr[sensor_id])
    return slice(adr, adr + dim)


def _find_unique_id(
    model: mujoco.MjModel,
    obj_type: int,
    names: tuple[str, ...],
    model_prefix: str | None,
    label: str,
) -> int:
    obj_id = _find_unique_id_or_none(model, obj_type, names, model_prefix, label)
    if obj_id is None:
        raise ValueError(f"Could not find MuJoCo {label}; tried {names}")
    return obj_id


def _find_unique_id_or_none(
    model: mujoco.MjModel,
    obj_type: int,
    names: tuple[str, ...],
    model_prefix: str | None,
    label: str,
) -> int | None:
    for name in names:
        candidates: list[int] = []
        for candidate in _candidate_names(name, model_prefix):
            obj_id = mujoco.mj_name2id(model, obj_type, candidate)
            if obj_id >= 0 and obj_id not in candidates:
                candidates.append(int(obj_id))
        if candidates:
            if len(candidates) > 1:
                matched = [_name(model, obj_type, obj_id) for obj_id in candidates]
                raise ValueError(f"Ambiguous MuJoCo {label}; tried {name}, matched {matched}")
            return candidates[0]
    return None


def _candidate_names(name: str, model_prefix: str | None) -> tuple[str, ...]:
    raw = name.lstrip("/")
    candidates = [name, f"/{raw}"]
    if model_prefix:
        prefix = model_prefix.strip("/")
        candidates.extend(
            [
                f"{prefix}{raw}",
                f"{prefix}-{raw}",
                f"{prefix}/{raw}",
                f"/{prefix}{raw}",
                f"/{prefix}-{raw}",
                f"/{prefix}/{raw}",
            ]
        )
    return tuple(dict.fromkeys(candidates))


def _name(model: mujoco.MjModel, obj_type: int, obj_id: int) -> str:
    return mujoco.mj_id2name(model, obj_type, obj_id) or f"<unnamed:{obj_id}>"


__all__ = [
    "RobotSimBinding",
    "RobotSimSpec",
    "mjcf_joint_names_from_hardware",
    "resolve_robot_sim_binding",
]
