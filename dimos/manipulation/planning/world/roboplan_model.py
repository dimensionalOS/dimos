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

"""Private composite-model construction for :mod:`roboplan_world`."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Protocol
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.robot.assets.model import LoadedRobotModel
from dimos.utils.transform_utils import pose_to_matrix

ROBOPLAN_WORLD_FRAME = "dimos_world"

_ROOT_LINK = ROBOPLAN_WORLD_FRAME
_ROOT_JOINT = "dimos_world_joint"
_FREE_ROOTS = {"world", "map", _ROOT_LINK}
# TODO: Remove this global fallback when formal per-joint acceleration overrides are available.
_DEFAULT_ACCELERATION_LIMIT = 2.0
_REFERENCE_ATTRIBUTES = (
    "reference",
    "frame",
    "frame_id",
    "parent_frame",
    "child_frame",
    "parent_frame_id",
    "child_frame_id",
)
_MODEL_KEY = "model"
_MODEL_NAME = "dimos_model"


class _BuildRobot(Protocol):
    config: RobotModelConfig


class _SceneFactory(Protocol):
    def __call__(
        self,
        *,
        name: str,
        urdf: str,
        srdf: str,
        package_paths: Sequence[str],
    ) -> Any: ...


@dataclass(frozen=True)
class RoboPlanGroup:
    """One backend-private group in canonical public order."""

    group_ids: tuple[PlanningGroupID, ...]
    name: str
    native_names: tuple[str, ...]
    public_names: tuple[str, ...]
    output_names: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.output_names:
            object.__setattr__(self, "output_names", self.public_names)


@dataclass(frozen=True)
class RoboPlanModel:
    """The scene and small amount of mapping state needed by its adapter."""

    scene: Any
    groups: Mapping[frozenset[PlanningGroupID], RoboPlanGroup]
    native_joints: Mapping[str, str]
    native_links: Mapping[str, str]
    all_group: RoboPlanGroup

    def native_joint(self, canonical_name: str) -> str:
        return self.native_joints[canonical_name]

    def native_link(self, canonical_name: str) -> str:
        return self.native_links[canonical_name]


@dataclass(frozen=True)
class _NameMap:
    links: Mapping[str, str]
    joints: Mapping[str, str]
    materials: Mapping[str, str]
    frames: Mapping[str, str]


@dataclass(frozen=True)
class _Composed:
    xml: str
    maps: Mapping[str, _NameMap]
    adjacent_links: tuple[tuple[str, str], ...]


def build_roboplan_model(
    robot: _BuildRobot,
    registry: PlanningGroupRegistry,
    scene_factory: _SceneFactory,
) -> RoboPlanModel:
    """Build one composite scene transactionally."""
    prepared = [
        (
            robot,
            prepare_urdf_for_drake(
                robot.config.model.load(),
                convert_meshes=robot.config.auto_convert_meshes,
            ),
        )
    ]
    composed = _compose(prepared, False)
    groups, all_group = _groups(robot, registry, composed.maps)
    srdf = _srdf(_MODEL_NAME, [robot], groups, composed)
    package_paths = [str(path) for path in prepared[0][1].package_paths.values()]
    scene = scene_factory(
        name=_MODEL_NAME,
        urdf=composed.xml,
        srdf=srdf,
        package_paths=package_paths,
    )
    groups = _validate_group_order(scene, groups)
    all_group = groups[frozenset(all_group.group_ids)]
    _apply_collision_exclusions(scene, srdf)
    mapping = composed.maps[_MODEL_KEY]
    return RoboPlanModel(
        scene=scene,
        groups=groups,
        native_joints={name: mapping.joints[name] for name in robot.config.joint_names},
        native_links=mapping.links,
        all_group=all_group,
    )


def _compose(
    prepared: Sequence[tuple[_BuildRobot, LoadedRobotModel]], composite: bool
) -> _Composed:
    result = ET.Element(
        "robot",
        {"name": "dimos_composite" if composite else _MODEL_NAME},
    )
    ET.SubElement(result, "link", {"name": _ROOT_LINK})
    maps: dict[str, _NameMap] = {}
    used_names: set[str] = {_ROOT_LINK}
    for robot, description in prepared:
        config = robot.config
        root = ET.fromstring(description.xml)
        if _tag(root.tag) != "robot":
            raise ValueError("Prepared model is not a URDF robot")
        _add_missing_acceleration_limits(root)
        mapping = _name_map(root, _MODEL_KEY, composite)
        mapped_names = {
            value
            for table in (mapping.links, mapping.joints, mapping.materials, mapping.frames)
            for value in table.values()
        }
        duplicates = used_names & mapped_names
        if duplicates:
            raise ValueError(f"Duplicate composed model names: {sorted(duplicates)}")
        used_names.update(mapped_names)
        if config.base_link not in mapping.links:
            raise ValueError(f"Model base link '{config.base_link}' is missing")
        missing_joints = set(config.joint_names) - set(mapping.joints)
        if missing_joints:
            raise ValueError(f"Configured model joints are missing: {sorted(missing_joints)}")
        authored_root = _authored_root(root, config.base_link, _MODEL_KEY)
        authored_parent = (
            _joint_link(authored_root, "parent") if authored_root is not None else None
        )
        for element in list(root):
            if element is authored_root:
                continue
            if (
                authored_parent
                and _tag(element.tag) == "link"
                and element.get("name") == authored_parent
            ):
                continue
            copied = ET.fromstring(ET.tostring(element, encoding="unicode"))
            _rewrite(copied, mapping)
            result.append(copied)
        attachment_name = _qualified(_MODEL_KEY, _ROOT_JOINT) if composite else _ROOT_JOINT
        if attachment_name in used_names:
            raise ValueError("Model collides with its synthetic attachment name")
        used_names.add(attachment_name)
        joint = ET.SubElement(result, "joint", {"name": attachment_name, "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": _ROOT_LINK})
        ET.SubElement(joint, "child", {"link": mapping.links[config.base_link]})
        ET.SubElement(joint, "origin", _pose_attributes(config.base_pose))
        maps[_MODEL_KEY] = mapping
    adjacent: list[tuple[str, str]] = []
    for joint in result:
        if _tag(joint.tag) != "joint":
            continue
        parent, child = _joint_link(joint, "parent"), _joint_link(joint, "child")
        if parent and child and _ROOT_LINK not in (parent, child):
            adjacent.append((parent, child))
    return _Composed(
        ET.tostring(result, encoding="unicode", xml_declaration=True),
        maps,
        tuple(adjacent),
    )


def _add_missing_acceleration_limits(root: ET.Element) -> None:
    for joint in root.iter():
        if _tag(joint.tag) != "joint" or joint.get("type") == "fixed":
            continue
        limit = next((child for child in joint if _tag(child.tag) == "limit"), None)
        if limit is not None and limit.get("acceleration") is None:
            limit.set("acceleration", str(_DEFAULT_ACCELERATION_LIMIT))


def _name_map(root: ET.Element, model_key: str, prefix: bool) -> _NameMap:
    def names(tag: str) -> dict[str, str]:
        return {
            name: _qualified(model_key, name) if prefix else name
            for element in root.iter()
            if _tag(element.tag) == tag
            if (name := element.get("name"))
        }

    return _NameMap(names("link"), names("joint"), names("material"), names("frame"))


def _authored_root(root: ET.Element, base_link: str, model_key: str) -> ET.Element | None:
    links = {element.get("name") for element in root if _tag(element.tag) == "link"}
    roots = {"world", "map", root.get("name", "")} & links | {"world", "map"}
    matches = [
        joint
        for joint in root
        if _tag(joint.tag) == "joint"
        if _joint_link(joint, "parent") in roots
        if _joint_link(joint, "child") == base_link
    ]
    if len(matches) > 1:
        raise ValueError(f"Model '{model_key}' has ambiguous world attachment")
    if not matches:
        return None
    if matches[0].get("type") != "fixed":
        raise ValueError(f"Model '{model_key}' world attachment must be fixed")
    return matches[0]


def _rewrite(element: ET.Element, mapping: _NameMap) -> None:
    element.tag = _tag(element.tag)
    tables = {
        "link": mapping.links,
        "joint": mapping.joints,
        "material": mapping.materials,
        "frame": mapping.frames,
    }
    name = element.get("name")
    if name and element.tag in tables and name in tables[element.tag]:
        element.set("name", tables[element.tag][name])
    for attribute, table in (("link", mapping.links), ("joint", mapping.joints)):
        value = element.get(attribute)
        if value is not None and value in table:
            element.set(attribute, table[value])
        elif value is not None and value not in table.values() and value not in _FREE_ROOTS:
            raise ValueError(f"Unresolved URDF {attribute} reference: {value}")
    references = {**mapping.links, **mapping.joints, **mapping.frames}
    for attribute in _REFERENCE_ATTRIBUTES:
        value = element.get(attribute)
        if value in references:
            element.set(attribute, references[value])
        elif value is not None and value not in references.values() and value not in _FREE_ROOTS:
            raise ValueError(f"Unresolved URDF reference '{attribute}': {value}")
    for child in element:
        _rewrite(child, mapping)


def _groups(
    robot: _BuildRobot,
    registry: PlanningGroupRegistry,
    maps: Mapping[str, _NameMap],
) -> tuple[
    dict[frozenset[PlanningGroupID], RoboPlanGroup],
    RoboPlanGroup,
]:
    groups: dict[frozenset[PlanningGroupID], RoboPlanGroup] = {}
    configured = registry.list()
    for group in configured:
        layout = _group_layout((group,), maps)
        groups[frozenset(layout.group_ids)] = layout
    all_id = "__dimos_all_configured__"
    config = robot.config
    all_group = RoboPlanGroup(
        (all_id,),
        all_id,
        tuple(maps[_MODEL_KEY].joints[name] for name in config.joint_names),
        tuple(config.joint_names),
    )
    groups[frozenset(all_group.group_ids)] = all_group
    names = [group.name for group in groups.values()]
    if len(names) != len(set(names)):
        raise ValueError("Generated RoboPlan planning-group names are not unique")
    return groups, all_group


def _group_layout(
    selected: Sequence[PlanningGroup],
    maps: Mapping[str, _NameMap],
) -> RoboPlanGroup:
    ids = tuple(group.id for group in selected)
    return RoboPlanGroup(
        ids,
        _composite_group_name(ids) if len(selected) > 1 else selected[0].id,
        tuple(maps[_MODEL_KEY].joints[name] for group in selected for name in group.joint_names),
        tuple(name for group in selected for name in group.joint_names),
    )


def _srdf(
    model_name: str,
    robots: Sequence[_BuildRobot],
    groups: Mapping[frozenset[PlanningGroupID], RoboPlanGroup],
    composed: _Composed,
) -> str:
    lines = [f'<robot name="{escape(model_name)}">']
    for group in groups.values():
        lines.append(f'  <group name="{escape(group.name)}">')
        lines.extend(f'    <joint name="{escape(name)}"/>' for name in group.native_names)
        lines.append("  </group>")
    pairs = {tuple(sorted(pair)) for pair in composed.adjacent_links if _ROOT_LINK not in pair}
    for robot in robots:
        config = robot.config
        mapping = composed.maps[_MODEL_KEY]
        configured = list(config.collision_exclusion_pairs)
        if config.srdf_path is not None:
            configured.extend(_source_exclusions(config.srdf_path))
        for first, second in configured:
            first_exists = first in mapping.links
            second_exists = second in mapping.links
            if not first_exists and not second_exists:
                continue
            if not first_exists or not second_exists:
                raise ValueError(
                    f"Model collision exclusion references unknown links: {first} <-> {second}"
                )
            pairs.add(tuple(sorted((mapping.links[first], mapping.links[second]))))
    lines.extend(
        f'  <disable_collisions link1="{escape(first)}" link2="{escape(second)}" '
        'reason="DimOS configured"/>'
        for first, second in sorted(pairs)
    )
    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def _validate_group_order(
    scene: Any,
    groups: Mapping[frozenset[PlanningGroupID], RoboPlanGroup],
) -> dict[frozenset[PlanningGroupID], RoboPlanGroup]:
    validated: dict[frozenset[PlanningGroupID], RoboPlanGroup] = {}
    for key, group in groups.items():
        reported = tuple(scene.getJointGroupInfo(group.name).joint_names)
        if len(reported) != len(set(reported)) or set(reported) != set(group.native_names):
            raise ValueError(f"RoboPlan group '{group.name}' does not match the composed model")
        public_by_native = dict(zip(group.native_names, group.public_names, strict=True))
        validated[key] = replace(
            group,
            native_names=reported,
            public_names=tuple(public_by_native[name] for name in reported),
            output_names=group.output_names,
        )
    return validated


def _source_exclusions(path: Path) -> list[tuple[str, str]]:
    return _exclusions(ET.parse(path).getroot())


def _exclusions(root: ET.Element) -> list[tuple[str, str]]:
    return [
        (first, second)
        for element in root.iter()
        if _tag(element.tag) == "disable_collisions"
        if (first := element.get("link1")) is not None
        if (second := element.get("link2")) is not None
    ]


def _apply_collision_exclusions(scene: Any, srdf: str) -> None:
    for first, second in _exclusions(ET.fromstring(srdf)):
        scene.setCollisions(first, second, False)


def _pose_attributes(pose: Any) -> dict[str, str]:
    matrix = pose_to_matrix(pose)
    sy = float(np.hypot(matrix[0, 0], matrix[1, 0]))
    if sy > 1e-9:
        rpy = (
            np.arctan2(matrix[2, 1], matrix[2, 2]),
            np.arctan2(-matrix[2, 0], sy),
            np.arctan2(matrix[1, 0], matrix[0, 0]),
        )
    else:
        rpy = (
            np.arctan2(-matrix[1, 2], matrix[1, 1]),
            np.arctan2(-matrix[2, 0], sy),
            0.0,
        )
    xyz = matrix[:3, 3]
    return {
        "xyz": " ".join(f"{float(value):.17g}" for value in xyz),
        "rpy": " ".join(f"{float(value):.17g}" for value in rpy),
    }


def _joint_link(joint: ET.Element | None, tag: str) -> str | None:
    if joint is None:
        return None
    return next(
        (child.get("link") for child in joint if _tag(child.tag) == tag),
        None,
    )


def _qualified(model_key: str, local_name: str) -> str:
    return f"{_safe(model_key)}__{_safe(local_name)}"


def _safe(value: str) -> str:
    return value.replace("/", "_").replace(":", "_").replace(" ", "_")


def _composite_group_name(group_ids: Sequence[PlanningGroupID]) -> str:
    return "_dimos_composite__" + "__".join(_safe(value) for value in group_ids)


def _tag(value: str) -> str:
    return value.rsplit("}", 1)[-1]
