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

"""MuJoCo XML parsing helpers for joint/actuator metadata."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

import mujoco

if TYPE_CHECKING:
    from pathlib import Path


@dataclass(frozen=True)
class JointMapping:
    name: str
    joint_id: int | None
    actuator_id: int | None
    qpos_adr: int | None
    dof_adr: int | None
    tendon_qpos_adrs: tuple[int, ...]
    tendon_dof_adrs: tuple[int, ...]


@dataclass(frozen=True)
class _ActuatorSpec:
    name: str
    joint: str | None
    tendon: str | None


def build_joint_mappings(xml_path: Path, model: mujoco.MjModel) -> list[JointMapping]:
    specs = _parse_actuator_specs(xml_path)
    if specs:
        return _build_joint_mappings_from_specs(specs, model)
    if int(model.nu) > 0:
        return _build_joint_mappings_from_actuators(model)
    return _build_joint_mappings_from_model(model)


def _parse_actuator_specs(xml_path: Path) -> list[_ActuatorSpec]:
    return _collect_actuator_specs(xml_path.resolve(), seen=set())


def _collect_actuator_specs(xml_path: Path, seen: set[Path]) -> list[_ActuatorSpec]:
    if xml_path in seen:
        return []
    seen.add(xml_path)

    root = ET.parse(xml_path).getroot()
    base_dir = xml_path.parent
    specs: list[_ActuatorSpec] = []

    def walk(node: ET.Element) -> None:
        for child in node:
            if child.tag == "include":
                include_file = child.attrib.get("file")
                if include_file:
                    include_path = (base_dir / include_file).resolve()
                    specs.extend(_collect_actuator_specs(include_path, seen))
                continue
            if child.tag == "actuator":
                specs.extend(_parse_actuator_block(child))
                continue
            walk(child)

    walk(root)
    return specs


def _parse_actuator_block(actuator_elem: ET.Element) -> list[_ActuatorSpec]:
    specs: list[_ActuatorSpec] = []
    for child in actuator_elem:
        joint = child.attrib.get("joint")
        tendon = child.attrib.get("tendon")
        if not joint and not tendon:
            continue
        name = child.attrib.get("name") or joint or tendon or "actuator"
        specs.append(_ActuatorSpec(name=name, joint=joint, tendon=tendon))
    return specs


def _build_joint_mappings_from_specs(
    specs: list[_ActuatorSpec],
    model: mujoco.MjModel,
) -> list[JointMapping]:
    mappings: list[JointMapping] = []
    for spec in specs:
        if spec.joint:
            mappings.append(_mapping_for_joint(spec, model))
        elif spec.tendon:
            mappings.append(_mapping_for_tendon(spec, model))
    return mappings


def _mapping_for_joint(spec: _ActuatorSpec, model: mujoco.MjModel) -> JointMapping:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spec.joint)
    if joint_id < 0:
        raise ValueError(f"Unknown joint '{spec.joint}' in MuJoCo model")
    actuator_id = _find_actuator_id_for_joint(model, joint_id, spec.name)
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or spec.name
    return JointMapping(
        name=joint_name,
        joint_id=joint_id,
        actuator_id=actuator_id,
        qpos_adr=int(model.jnt_qposadr[joint_id]),
        dof_adr=int(model.jnt_dofadr[joint_id]),
        tendon_qpos_adrs=(),
        tendon_dof_adrs=(),
    )


def _mapping_for_tendon(spec: _ActuatorSpec, model: mujoco.MjModel) -> JointMapping:
    name = spec.name or spec.tendon
    if not name:
        raise ValueError("Tendon actuator is missing a name and tendon reference")
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, spec.tendon)
    if tendon_id < 0:
        raise ValueError(f"Unknown tendon '{spec.tendon}' in MuJoCo model")
    actuator_id = _find_actuator_id_for_tendon(model, tendon_id, spec.name)
    joint_ids = _tendon_joint_ids(model, tendon_id)
    return JointMapping(
        name=name,
        joint_id=None,
        actuator_id=actuator_id,
        qpos_adr=None,
        dof_adr=None,
        tendon_qpos_adrs=tuple(int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids),
        tendon_dof_adrs=tuple(int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids),
    )


def _find_actuator_id_for_joint(
    model: mujoco.MjModel,
    joint_id: int,
    actuator_name: str | None,
) -> int | None:
    if actuator_name:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if act_id >= 0:
            return int(act_id)
    for act_id in range(int(model.nu)):
        trn_type = int(model.actuator_trntype[act_id])
        if trn_type != int(mujoco.mjtTrn.mjTRN_JOINT):
            continue
        if int(model.actuator_trnid[act_id, 0]) == joint_id:
            return act_id
    return None


def _find_actuator_id_for_tendon(
    model: mujoco.MjModel,
    tendon_id: int,
    actuator_name: str | None,
) -> int | None:
    if actuator_name:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if act_id >= 0:
            return int(act_id)
    for act_id in range(int(model.nu)):
        trn_type = int(model.actuator_trntype[act_id])
        if trn_type != int(mujoco.mjtTrn.mjTRN_TENDON):
            continue
        if int(model.actuator_trnid[act_id, 0]) == tendon_id:
            return act_id
    return None


def _tendon_joint_ids(model: mujoco.MjModel, tendon_id: int) -> tuple[int, ...]:
    adr = int(model.tendon_adr[tendon_id])
    num = int(model.tendon_num[tendon_id])
    joint_ids: list[int] = []
    for wrap_id in range(adr, adr + num):
        wrap_type = int(model.wrap_type[wrap_id])
        if wrap_type == int(mujoco.mjtWrap.mjWRAP_JOINT):
            joint_ids.append(int(model.wrap_objid[wrap_id]))
    return tuple(joint_ids)


def _build_joint_mappings_from_actuators(model: mujoco.MjModel) -> list[JointMapping]:
    mappings: list[JointMapping] = []
    for actuator_id in range(int(model.nu)):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        name = actuator_name or f"actuator{actuator_id}"
        trn_type = int(model.actuator_trntype[actuator_id])
        if trn_type == int(mujoco.mjtTrn.mjTRN_JOINT):
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            mappings.append(
                JointMapping(
                    name=joint_name or name,
                    joint_id=joint_id,
                    actuator_id=actuator_id,
                    qpos_adr=int(model.jnt_qposadr[joint_id]),
                    dof_adr=int(model.jnt_dofadr[joint_id]),
                    tendon_qpos_adrs=(),
                    tendon_dof_adrs=(),
                )
            )
            continue

        if trn_type == int(mujoco.mjtTrn.mjTRN_TENDON):
            tendon_id = int(model.actuator_trnid[actuator_id, 0])
            tendon_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id)
            if not actuator_name and tendon_name:
                name = tendon_name
            joint_ids = _tendon_joint_ids(model, tendon_id)
            mappings.append(
                JointMapping(
                    name=name,
                    joint_id=None,
                    actuator_id=actuator_id,
                    qpos_adr=None,
                    dof_adr=None,
                    tendon_qpos_adrs=tuple(
                        int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids
                    ),
                    tendon_dof_adrs=tuple(
                        int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids
                    ),
                )
            )
            continue

        mappings.append(
            JointMapping(
                name=name,
                joint_id=None,
                actuator_id=actuator_id,
                qpos_adr=None,
                dof_adr=None,
                tendon_qpos_adrs=(),
                tendon_dof_adrs=(),
            )
        )

    return mappings


def _build_joint_mappings_from_model(model: mujoco.MjModel) -> list[JointMapping]:
    mappings: list[JointMapping] = []
    for joint_id in range(int(model.njnt)):
        jnt_type = int(model.jnt_type[joint_id])
        if jnt_type not in (
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        ):
            continue
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        name = joint_name or f"joint{joint_id}"
        mappings.append(
            JointMapping(
                name=name,
                joint_id=joint_id,
                actuator_id=None,
                qpos_adr=int(model.jnt_qposadr[joint_id]),
                dof_adr=int(model.jnt_dofadr[joint_id]),
                tendon_qpos_adrs=(),
                tendon_dof_adrs=(),
            )
        )
    return mappings
