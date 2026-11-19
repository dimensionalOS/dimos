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

"""Prepare the fixed-base xArm grasp model for CuRobo V2."""

from __future__ import annotations

import atexit
from copy import deepcopy
from math import isfinite
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

import numpy as np

from dimos.manipulation.planning.world.collision_exclusions import collision_exclusion_pairs
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.validation import PreparedRobotModel


def build_curobo_model(prepared: PreparedRobotModel) -> dict[str, Any]:
    """Return the robot dict for ``MotionPlannerCfg.create(robot=...)``.

    This supports a single fixed-base arm group covering the canonical joints,
    with one TCP and a gripper locked at its configured open position (including
    URDF mimic followers). The planning frame is the physical base frame; the
    xArm grasp configuration places that base at the planning world's origin.
    Sphere fitting happens once here, not when the planner is rebuilt. The
    materialized URDF file remains available for the life of the process.
    """
    config = prepared.config
    names = prepared.joint_space.names
    if len(prepared.planning_groups) != 1:
        raise ValueError("CuRobo requires exactly one fixed-base arm planning group")
    group = prepared.planning_groups[0]
    tip = group.tip_link
    if not tip or group.base_link != config.base_link or set(group.joint_names) != set(names):
        raise ValueError("CuRobo requires one base-to-TCP group covering all canonical joints")
    if not np.allclose(pose_to_matrix(config.base_pose), np.eye(4), rtol=0.0, atol=1e-12):
        raise ValueError("CuRobo requires the robot base at the planning-world origin")
    if not prepared.joint_space.is_interval_only:
        raise ValueError("CuRobo requires finite interval joints for the arm")

    description = prepared.description
    root = ET.fromstring(description.xml)
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = description.joints
    by_child = {joint.child_link: joint for joint in joints}
    if config.base_link not in links or tip not in links or tip == config.base_link:
        raise ValueError("CuRobo base or TCP is not a valid link in the prepared URDF")

    retained = {config.base_link}
    while (
        new_links := {joint.child_link for joint in joints if joint.parent_link in retained}
        - retained
    ):
        retained.update(new_links)
    outlying_links = set(links) - retained
    attachment = by_child.get(config.base_link)
    if outlying_links or attachment is not None:
        # An authored empty world link and its weld are not robot geometry.
        if (
            attachment is None
            or attachment.type != "fixed"
            or attachment.parent_link != description.root_link
            or outlying_links != {description.root_link}
            or links[description.root_link].find("collision") is not None
        ):
            raise ValueError("CuRobo requires one fixed-base robot subtree")
        root.remove(links[description.root_link])
        root.remove(next(j for j in root.findall("joint") if j.get("name") == attachment.name))
    if tip not in retained:
        raise ValueError(f"CuRobo TCP '{tip}' is not attached to base '{config.base_link}'")
    arm_chain = set()
    child = tip
    while child != config.base_link:
        joint = by_child[child]
        if joint.type != "fixed":
            arm_chain.add(joint.name)
        child = joint.parent_link
    if arm_chain != set(names):
        raise ValueError("CuRobo TCP chain does not match the canonical arm joints")

    finger_joints = config.gripper.finger_joints if config.gripper is not None else {}
    locked = {name: opened for name, (opened, _) in finger_joints.items()}
    independent = {
        joint.name: joint
        for joint in joints
        if joint.child_link in retained and joint.child_link != config.base_link
        if joint.type != "fixed" and joint.mimic is None
    }
    if set(names) & locked.keys() or independent.keys() != set(names) | locked.keys():
        raise ValueError("CuRobo URDF joints do not match the canonical arm and gripper joints")
    for name, value in locked.items():
        joint = independent[name]
        if (
            joint.type not in {"revolute", "prismatic"}
            or joint.lower is None
            or joint.upper is None
            or not joint.lower <= value <= joint.upper
        ):
            raise ValueError(f"CuRobo gripper joint '{name}' has an invalid open position")
    for joint in joints:
        if joint.mimic is not None and joint.mimic[0] not in locked:
            raise ValueError(
                f"CuRobo mimic joint '{joint.name}' must follow a locked gripper joint"
            )

    adjacent = (
        (joint.parent_link, joint.child_link)
        for joint in joints
        if joint.parent_link in retained and joint.child_link in retained
    )
    exclusions = collision_exclusion_pairs(config, retained, adjacent)
    collision_links = {name for name in retained if links[name].find("collision") is not None}
    if not collision_links:
        raise ValueError("CuRobo requires URDF collision geometry to fit robot spheres")
    if config.gripper is not None and (
        missing_pads := set(config.gripper.pad_links) - collision_links
    ):
        raise ValueError(f"CuRobo gripper pads have no collision geometry: {sorted(missing_pads)}")

    coordinates = prepared.joint_space.coordinates
    home = config.home_joints
    if home is None:
        home = [
            max(c.lower, min(0.0, c.upper))
            for c in coordinates
            if c.lower is not None and c.upper is not None
        ]
    if len(home) != len(names) or any(
        c.lower is None or c.upper is None or not c.lower <= value <= c.upper
        for c, value in zip(coordinates, home, strict=True)
    ):
        raise ValueError("CuRobo home position does not match the prepared joint limits")

    try:
        from curobo.robot_builder import RobotBuilder
    except ImportError as exc:
        raise ImportError("The CuRobo model backend requires NVIDIA CuRobo V2") from exc

    directory = TemporaryDirectory(prefix="dimos-curobo-model-")
    try:
        urdf = Path(directory.name) / "robot.urdf"
        fitting_urdf = urdf.with_name("collision.urdf")
        ET.ElementTree(root).write(urdf, encoding="utf-8", xml_declaration=True)
        _write_fitting_urdf(root, fitting_urdf)
        builder = RobotBuilder(urdf_path=str(fitting_urdf), tool_frames=[tip])
        spheres = builder.fit_collision_spheres(use_collision_mesh=True)
        fitting_urdf.unlink()
        _check_spheres(spheres, collision_links)
    except BaseException:
        directory.cleanup()
        raise

    # Solver creation (and rebuilds) may read this path after the builder returns.
    atexit.register(directory.cleanup)
    ignore: dict[str, list[str]] = {}
    for first, second in exclusions:
        if first in collision_links and second in collision_links:
            # V2 interprets each entry bidirectionally.
            ignore.setdefault(first, []).append(second)

    return {
        "kinematics": {
            "format_version": 2.0,
            "urdf_path": str(urdf),
            "base_link": config.base_link,
            "tool_frames": [tip],
            "lock_joints": locked,
            "collision_link_names": list(spheres),
            "collision_spheres": spheres,
            "self_collision_ignore": ignore,
            "self_collision_buffer": {},
            "cspace": {
                "joint_names": list(names),
                "default_joint_position": list(home),
                "cspace_distance_weight": [1.0] * len(names),
                "null_space_weight": [1.0] * len(names),
                "max_acceleration": list(prepared.joint_space.acceleration_limits),
            },
        },
    }


def _write_fitting_urdf(root: ET.Element, path: Path) -> None:
    """Make the builder enumerate collision-only links; V2 discovers visuals first."""
    fitting = deepcopy(root)
    for link in fitting.findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)
        for collision in link.findall("collision"):
            view = deepcopy(collision)
            view.tag = "visual"
            link.append(view)
            mesh = collision.find("geometry/mesh")
            if mesh is not None:
                filename = Path(mesh.get("filename", ""))
                if not filename.is_absolute() or not filename.is_file():
                    raise ValueError(f"CuRobo collision mesh is not a resolved file: {filename}")
    ET.ElementTree(fitting).write(path, encoding="utf-8", xml_declaration=True)


def _check_spheres(spheres: dict[str, list[dict[str, Any]]], links: set[str]) -> None:
    if spheres.keys() != links or any(not fitted for fitted in spheres.values()):
        missing = links - {name for name, fitted in spheres.items() if fitted}
        unexpected = spheres.keys() - links
        raise ValueError(
            f"CuRobo collision sphere coverage mismatch: missing={sorted(missing)}, "
            f"unexpected={sorted(unexpected)}"
        )
    for name, fitted in spheres.items():
        for sphere in fitted:
            center, radius = sphere["center"], sphere["radius"]
            if (
                len(center) != 3
                or not all(isfinite(v) for v in center)
                or not (0 < radius < float("inf"))
            ):
                raise ValueError(f"CuRobo fitted an invalid collision sphere on '{name}'")
