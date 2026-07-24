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

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from itertools import combinations
from pathlib import Path
import tempfile
from typing import Any, Protocol
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.utils.transform_utils import pose_to_matrix

_MAX_COMPOSITE_GROUPS = 64
_ROOT_LINK = "dimos_world"
_ROOT_JOINT = "dimos_world_joint"
_FREE_ROOTS = {"world", "map", _ROOT_LINK}
_REFERENCE_ATTRIBUTES = (
    "reference",
    "frame",
    "frame_id",
    "parent_frame",
    "child_frame",
    "parent_frame_id",
    "child_frame_id",
)


class _BuildRobot(Protocol):
    config: RobotModelConfig


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
    owner: tempfile.TemporaryDirectory[str]
    groups: Mapping[frozenset[PlanningGroupID], RoboPlanGroup]
    legacy_group_ids: Mapping[RobotName, PlanningGroupID]
    native_joint_by_global: Mapping[str, str]
    native_link_by_robot: Mapping[RobotName, Mapping[str, str]]
    all_group: RoboPlanGroup

    def native_joint(self, robot_name: RobotName, local_name: str) -> str:
        return self.native_joint_by_global[f"{robot_name}/{local_name}"]

    def native_link(self, robot_name: RobotName, local_name: str) -> str:
        return self.native_link_by_robot[robot_name][local_name]


@dataclass(frozen=True)
class _NameMap:
    links: Mapping[str, str]
    joints: Mapping[str, str]
    materials: Mapping[str, str]
    frames: Mapping[str, str]


@dataclass(frozen=True)
class _Composed:
    xml: str
    maps: Mapping[RobotName, _NameMap]
    adjacent_links: tuple[tuple[str, str], ...]


def build_roboplan_model(
    robots: Sequence[_BuildRobot],
    registry: PlanningGroupRegistry,
    scene_factory: Callable[[str, str, str, list[str]], Any],
) -> RoboPlanModel:
    """Build one composite scene transactionally."""
    if not robots:
        raise ValueError("RoboPlanWorld requires at least one robot")
    owner = tempfile.TemporaryDirectory(prefix="dimos_roboplan_")
    try:
        prepared = [
            (
                robot,
                Path(
                    prepare_urdf_for_drake(
                        robot.config.model_path,
                        package_paths=robot.config.package_paths,
                        xacro_args=robot.config.xacro_args,
                        convert_meshes=robot.config.auto_convert_meshes,
                    )
                ),
            )
            for robot in robots
        ]
        composite = len(robots) > 1
        composed = _compose(prepared, composite)
        groups, legacy_ids, all_group = _groups(robots, registry, composed.maps, composite)
        model_name = "dimos_composite" if composite else robots[0].config.name
        cache = Path(owner.name)
        urdf_path = cache / f"{model_name}.urdf"
        srdf_path = cache / f"{model_name}.srdf"
        urdf_path.write_text(composed.xml)
        srdf_path.write_text(_srdf(model_name, robots, groups, composed))
        package_paths = list(
            dict.fromkeys(
                str(path) for robot in robots for path in robot.config.package_paths.values()
            )
        )
        scene = scene_factory(model_name, str(urdf_path), str(srdf_path), package_paths)
        groups = _validate_group_order(scene, groups)
        all_group = groups[frozenset(all_group.group_ids)]
        _apply_collision_exclusions(scene, srdf_path)
        native_joint_by_global = {
            f"{robot.config.name}/{local}": composed.maps[robot.config.name].joints[local]
            for robot in robots
            for local in robot.config.joint_names
        }
        return RoboPlanModel(
            scene=scene,
            owner=owner,
            groups=groups,
            legacy_group_ids=legacy_ids,
            native_joint_by_global=native_joint_by_global,
            native_link_by_robot={name: mapping.links for name, mapping in composed.maps.items()},
            all_group=all_group,
        )
    except BaseException:
        owner.cleanup()
        raise


def _compose(prepared: Sequence[tuple[_BuildRobot, Path]], composite: bool) -> _Composed:
    result = ET.Element(
        "robot",
        {"name": "dimos_composite" if composite else prepared[0][0].config.name},
    )
    ET.SubElement(result, "link", {"name": _ROOT_LINK})
    maps: dict[RobotName, _NameMap] = {}
    used_names: set[str] = {_ROOT_LINK}
    for robot, path in prepared:
        config = robot.config
        root = ET.parse(path).getroot()
        if _tag(root.tag) != "robot":
            raise ValueError(f"Prepared model for '{config.name}' is not a URDF robot")
        mapping = _name_map(root, config.name, composite)
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
            raise ValueError(f"Robot '{config.name}' base link '{config.base_link}' is missing")
        missing_joints = set(config.joint_names) - set(mapping.joints)
        if missing_joints:
            raise ValueError(
                f"Robot '{config.name}' configured joints are missing: {sorted(missing_joints)}"
            )
        authored_root = _authored_root(root, config.base_link, config.name)
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
        attachment_name = _qualified(config.name, _ROOT_JOINT) if composite else _ROOT_JOINT
        if attachment_name in used_names:
            raise ValueError(f"Robot '{config.name}' collides with its synthetic attachment name")
        used_names.add(attachment_name)
        joint = ET.SubElement(result, "joint", {"name": attachment_name, "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": _ROOT_LINK})
        ET.SubElement(joint, "child", {"link": mapping.links[config.base_link]})
        ET.SubElement(joint, "origin", _pose_attributes(config.base_pose))
        maps[config.name] = mapping
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


def _name_map(root: ET.Element, robot_name: RobotName, prefix: bool) -> _NameMap:
    def names(tag: str) -> dict[str, str]:
        return {
            name: _qualified(robot_name, name) if prefix else name
            for element in root.iter()
            if _tag(element.tag) == tag
            if (name := element.get("name"))
        }

    return _NameMap(names("link"), names("joint"), names("material"), names("frame"))


def _authored_root(root: ET.Element, base_link: str, robot_name: RobotName) -> ET.Element | None:
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
        raise ValueError(f"Robot '{robot_name}' has ambiguous world attachment")
    if not matches:
        return None
    if matches[0].get("type") != "fixed":
        raise ValueError(f"Robot '{robot_name}' world attachment must be fixed")
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
    robots: Sequence[_BuildRobot],
    registry: PlanningGroupRegistry,
    maps: Mapping[RobotName, _NameMap],
    composite: bool,
) -> tuple[
    dict[frozenset[PlanningGroupID], RoboPlanGroup],
    dict[RobotName, PlanningGroupID],
    RoboPlanGroup,
]:
    groups: dict[frozenset[PlanningGroupID], RoboPlanGroup] = {}
    legacy_ids: dict[RobotName, PlanningGroupID] = {}
    for robot in robots:
        config = robot.config
        group_id = f"{config.name}/__roboplan_legacy__"
        legacy_ids[config.name] = group_id
        legacy_group = RoboPlanGroup(
            (group_id,),
            f"_dimos_legacy__{_safe(config.name)}" if composite else config.name,
            tuple(maps[config.name].joints[name] for name in config.joint_names),
            tuple(config.joint_names),
        )
        groups[frozenset(legacy_group.group_ids)] = legacy_group
    configured = registry.list()
    for group in configured:
        layout = _group_layout((group,), maps, composite)
        groups[frozenset(layout.group_ids)] = layout
    generated = 0
    for size in range(2, len(configured) + 1):
        for selected in combinations(configured, size):
            if len({group.robot_name for group in selected}) < 2:
                continue
            if len({name for group in selected for name in group.joint_names}) != sum(
                len(group.joint_names) for group in selected
            ):
                continue
            generated += 1
            if generated > _MAX_COMPOSITE_GROUPS:
                raise ValueError(
                    f"RoboPlan composite planning groups exceed {_MAX_COMPOSITE_GROUPS}"
                )
            layout = _group_layout(selected, maps, True)
            groups[frozenset(layout.group_ids)] = layout
    all_id = "__dimos_all_configured__"
    all_group = RoboPlanGroup(
        (all_id,),
        all_id,
        tuple(
            maps[robot.config.name].joints[name]
            for robot in robots
            for name in robot.config.joint_names
        ),
        tuple(
            f"{robot.config.name}/{name}" for robot in robots for name in robot.config.joint_names
        ),
    )
    groups[frozenset(all_group.group_ids)] = all_group
    names = [group.name for group in groups.values()]
    if len(names) != len(set(names)):
        raise ValueError("Generated RoboPlan planning-group names are not unique")
    return groups, legacy_ids, all_group


def _group_layout(
    selected: Sequence[PlanningGroup],
    maps: Mapping[RobotName, _NameMap],
    composite: bool,
) -> RoboPlanGroup:
    ids = tuple(group.id for group in selected)
    return RoboPlanGroup(
        ids,
        _composite_group_name(ids) if composite else selected[0].group_name,
        tuple(
            maps[group.robot_name].joints[local]
            for group in selected
            for local in group.local_joint_names
        ),
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
        mapping = composed.maps[config.name]
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
                    f"Robot '{config.name}' collision exclusion references unknown links: "
                    f"{first} <-> {second}"
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
    root = ET.parse(path).getroot()
    return [
        (first, second)
        for element in root.iter()
        if _tag(element.tag) == "disable_collisions"
        if (first := element.get("link1")) is not None
        if (second := element.get("link2")) is not None
    ]


def _apply_collision_exclusions(scene: Any, srdf_path: Path) -> None:
    for first, second in _source_exclusions(srdf_path):
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


def _qualified(robot_name: RobotName, local_name: str) -> str:
    return f"{_safe(robot_name)}__{_safe(local_name)}"


def _safe(value: str) -> str:
    return value.replace("/", "_").replace(":", "_").replace(" ", "_")


def _composite_group_name(group_ids: Sequence[PlanningGroupID]) -> str:
    return "_dimos_composite__" + "__".join(_safe(value) for value in group_ids)


def _tag(value: str) -> str:
    return value.rsplit("}", 1)[-1]
