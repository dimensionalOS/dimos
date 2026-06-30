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

"""Planning group discovery from SRDF or conservative model fallback."""

from __future__ import annotations

import itertools
from pathlib import Path
import warnings
import xml.etree.ElementTree as ET

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.robot.model_parser import JointDescription, ModelDescription
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FALLBACK_PLANNING_GROUP_NAME = "manipulator"


class PlanningGroupDiscoveryError(ValueError):
    """Raised when planning groups cannot be discovered for a model."""


def _warn(message: str) -> None:
    logger.warning(message)
    warnings.warn(message, UserWarning, stacklevel=2)


def discover_planning_group_definitions(
    *,
    robot_name: str,
    model_path: Path,
    model: ModelDescription,
    controllable_joint_names: list[str],
    srdf_path: Path | None = None,
) -> list[PlanningGroupDefinition]:
    """Discover planning groups from SRDF or fallback generation.

    Precedence is explicit SRDF path, conservative auto-discovery with warning,
    then fallback generation from the controllable joint set.
    """
    resolved_srdf_path = _resolve_srdf_path(model_path, srdf_path)
    if resolved_srdf_path is not None:
        groups = parse_srdf_planning_groups(
            resolved_srdf_path,
            model=model,
            controllable_joint_names=controllable_joint_names,
        )
        if groups:
            return groups
        _warn(
            f"No supported planning groups found in SRDF {resolved_srdf_path} "
            f"for robot {robot_name}; trying fallback generation"
        )

    return [
        generate_fallback_planning_group(
            model=model,
            controllable_joint_names=controllable_joint_names,
        )
    ]


def parse_srdf_planning_groups(
    srdf_path: Path,
    *,
    model: ModelDescription,
    controllable_joint_names: list[str],
) -> list[PlanningGroupDefinition]:
    """Extract supported SRDF planning group definitions.

    Supported forms are a single ``<chain base_link="..." tip_link="..."/>``
    child or an ordered list of ``<joint name="..."/>`` children. Other forms,
    including SRDF ``<end_effector>`` metadata, are ignored for planning group
    extraction. This is intentionally a minimal SRDF group extractor rather
    than a full SRDF parser; adopting a ROS/MoveIt parser such as srdfdom would
    add substantial dependency overhead for this narrow subset.
    """
    root = ET.parse(srdf_path).getroot()
    groups: list[PlanningGroupDefinition] = []
    for group_elem in root.findall("group"):
        group_name = group_elem.get("name")
        if not group_name:
            _warn(f"Skipping SRDF group without a name in {srdf_path}")
            continue

        children = [child for child in list(group_elem) if isinstance(child.tag, str)]
        chain_children = [child for child in children if child.tag == "chain"]
        joint_children = [child for child in children if child.tag == "joint"]
        unsupported_children = [child for child in children if child.tag not in {"chain", "joint"}]

        if len(chain_children) == 1 and not joint_children and not unsupported_children:
            definition = _parse_chain_group(
                group_name,
                chain_children[0],
                model=model,
                controllable_joint_names=controllable_joint_names,
                srdf_path=srdf_path,
            )
        elif joint_children and len(joint_children) == len(children):
            definition = _parse_joint_list_group(
                group_name,
                joint_children,
                model=model,
                controllable_joint_names=controllable_joint_names,
                srdf_path=srdf_path,
            )
        else:
            child_tags = [child.tag for child in children]
            _warn(
                f"Skipping unsupported SRDF planning group {group_name} in "
                f"{srdf_path} with child tags {child_tags}"
            )
            definition = None

        if definition is not None:
            groups.append(definition)

    return groups


def generate_fallback_planning_group(
    *,
    model: ModelDescription,
    controllable_joint_names: list[str],
) -> PlanningGroupDefinition:
    """Generate one conservative fallback planning group named ``manipulator``."""
    ordered_joints = _validate_and_order_serial_joints(model, controllable_joint_names)
    while ordered_joints and ordered_joints[-1].type == "prismatic":
        removed = ordered_joints.pop()
        _warn(
            f"Excluding terminal prismatic joint {removed.name} from "
            f"fallback planning group {FALLBACK_PLANNING_GROUP_NAME}"
        )

    if not ordered_joints:
        raise PlanningGroupDiscoveryError(
            "Fallback planning group generation removed all candidate joints; provide SRDF"
        )

    return PlanningGroupDefinition(
        name=FALLBACK_PLANNING_GROUP_NAME,
        joint_names=tuple(joint.name for joint in ordered_joints),
        base_link=ordered_joints[0].parent_link,
        tip_link=ordered_joints[-1].child_link,
        source="fallback",
    )


def _resolve_srdf_path(model_path: Path, srdf_path: Path | None) -> Path | None:
    if srdf_path is not None:
        if srdf_path.exists():
            return srdf_path
        raise FileNotFoundError(f"SRDF file not found: {srdf_path}")

    for candidate in _srdf_auto_discovery_candidates(model_path):
        if candidate.exists():
            _warn(f"Auto-discovered SRDF at {candidate}")
            return candidate
    return None


def _srdf_auto_discovery_candidates(model_path: Path) -> list[Path]:
    candidates: list[Path] = []
    name = model_path.name
    if name.endswith(".urdf.xacro"):
        candidates.append(model_path.with_name(name.removesuffix(".urdf.xacro") + ".srdf"))
    elif model_path.suffix:
        candidates.append(model_path.with_suffix(".srdf"))
    candidates.append(model_path.parent / "config" / "robot.srdf")
    candidates.append(model_path.parent.parent / "config" / "robot.srdf")
    return list(dict.fromkeys(candidates))


def _parse_chain_group(
    group_name: str,
    chain_elem: ET.Element,
    *,
    model: ModelDescription,
    controllable_joint_names: list[str],
    srdf_path: Path,
) -> PlanningGroupDefinition | None:
    base_link = chain_elem.get("base_link")
    tip_link = chain_elem.get("tip_link")
    if not base_link or not tip_link:
        _warn(
            f"Skipping SRDF chain group {group_name} in {srdf_path} because "
            "base_link or tip_link is missing"
        )
        return None

    try:
        ordered_joints = _ordered_joints_between_links(model, base_link, tip_link)
        controlled_joints = [joint for joint in ordered_joints if joint.type != "fixed"]
        _validate_controllable(group_name, controlled_joints, controllable_joint_names)
    except PlanningGroupDiscoveryError as exc:
        _warn(f"Skipping SRDF chain group {group_name} in {srdf_path}: {exc}")
        return None

    return PlanningGroupDefinition(
        name=group_name,
        joint_names=tuple(joint.name for joint in controlled_joints),
        base_link=base_link,
        tip_link=tip_link,
        source="srdf",
    )


def _parse_joint_list_group(
    group_name: str,
    joint_children: list[ET.Element],
    *,
    model: ModelDescription,
    controllable_joint_names: list[str],
    srdf_path: Path,
) -> PlanningGroupDefinition | None:
    joint_names = [child.get("name", "") for child in joint_children]
    if any(not name for name in joint_names):
        _warn(f"Skipping SRDF joint-list group {group_name} in {srdf_path} with empty joint name")
        return None
    try:
        ordered_joints = _validate_ordered_serial_joints(model, joint_names)
        _validate_controllable(group_name, ordered_joints, controllable_joint_names)
    except PlanningGroupDiscoveryError as exc:
        _warn(f"Skipping SRDF joint-list group {group_name} in {srdf_path}: {exc}")
        return None

    return PlanningGroupDefinition(
        name=group_name,
        joint_names=tuple(joint.name for joint in ordered_joints),
        base_link=ordered_joints[0].parent_link,
        tip_link=ordered_joints[-1].child_link,
        source="srdf",
    )


def _ordered_joints_between_links(
    model: ModelDescription,
    base_link: str,
    tip_link: str,
) -> list[JointDescription]:
    joints_by_parent: dict[str, list[JointDescription]] = {}
    for joint in model.joints:
        joints_by_parent.setdefault(joint.parent_link, []).append(joint)

    ordered_joints: list[JointDescription] = []
    current_link = base_link
    visited_links = {base_link}
    while current_link != tip_link:
        children = joints_by_parent.get(current_link, [])
        if len(children) != 1:
            raise PlanningGroupDiscoveryError(
                f"chain from {base_link} to {tip_link} is branching or disconnected at {current_link}"
            )
        joint = children[0]
        ordered_joints.append(joint)
        current_link = joint.child_link
        if current_link in visited_links:
            raise PlanningGroupDiscoveryError("chain contains a cycle")
        visited_links.add(current_link)

    return ordered_joints


def _validate_ordered_serial_joints(
    model: ModelDescription,
    joint_names: list[str],
) -> list[JointDescription]:
    ordered_joints: list[JointDescription] = []
    for joint_name in joint_names:
        joint = model.get_joint(joint_name)
        if joint is None:
            raise PlanningGroupDiscoveryError(f"joint {joint_name} does not exist in model")
        if joint.type == "fixed":
            raise PlanningGroupDiscoveryError(f"joint {joint_name} is fixed")
        ordered_joints.append(joint)

    if not ordered_joints:
        raise PlanningGroupDiscoveryError("planning group contains no joints")

    for previous, current in itertools.pairwise(ordered_joints):
        if previous.child_link != current.parent_link:
            raise PlanningGroupDiscoveryError(
                f"joints {previous.name} and {current.name} are not adjacent in a serial chain"
            )
    return ordered_joints


def _validate_and_order_serial_joints(
    model: ModelDescription,
    joint_names: list[str],
) -> list[JointDescription]:
    if not joint_names:
        raise PlanningGroupDiscoveryError("fallback requires at least one controllable joint")

    joints: list[JointDescription] = []
    for joint_name in joint_names:
        joint = model.get_joint(joint_name)
        if joint is None:
            raise PlanningGroupDiscoveryError(f"joint {joint_name} does not exist in model")
        if joint.type == "fixed":
            raise PlanningGroupDiscoveryError(f"joint {joint_name} is fixed")
        joints.append(joint)

    by_parent = {joint.parent_link: joint for joint in joints}
    by_child = {joint.child_link: joint for joint in joints}
    if len(by_parent) != len(joints) or len(by_child) != len(joints):
        raise PlanningGroupDiscoveryError("controllable joints branch or merge; provide SRDF")

    starts = [joint for joint in joints if joint.parent_link not in by_child]
    ends = [joint for joint in joints if joint.child_link not in by_parent]
    if len(starts) != 1 or len(ends) != 1:
        raise PlanningGroupDiscoveryError(
            "controllable joints are disconnected or cyclic; provide SRDF"
        )

    ordered_joints: list[JointDescription] = []
    current = starts[0]
    while True:
        ordered_joints.append(current)
        next_joint = by_parent.get(current.child_link)
        if next_joint is None:
            break
        current = next_joint

    if len(ordered_joints) != len(joints):
        raise PlanningGroupDiscoveryError("controllable joints are disconnected; provide SRDF")
    return ordered_joints


def _validate_controllable(
    group_name: str,
    joints: list[JointDescription],
    controllable_joint_names: list[str],
) -> None:
    if not joints:
        raise PlanningGroupDiscoveryError(
            f"planning group {group_name} contains no controllable joints"
        )
    controllable = set(controllable_joint_names)
    missing = [joint.name for joint in joints if joint.name not in controllable]
    if missing:
        raise PlanningGroupDiscoveryError(
            f"planning group {group_name} includes joints outside controllable set: {missing}"
        )
