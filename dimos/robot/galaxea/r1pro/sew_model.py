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

"""Validate R1 Pro's kinematic assumptions without loading meshes or hardware."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.manipulation.planning.kinematics.sew_retargeting import SewArmSolver


def required(element: ET.Element, tag: str) -> ET.Element:
    child = element.find(tag)
    if child is None:
        raise ValueError(f"Missing URDF {tag} in {element.attrib.get('name')}")
    return child


class R1SewModel:
    def __init__(self, path: Path) -> None:
        self.root = ET.parse(path).getroot()
        self.joints = {j.attrib["name"]: j for j in self.root.findall("joint")}
        self.names = [
            f"{part}_joint{i}"
            for part, n in [("torso", 4), ("left_arm", 7), ("right_arm", 7)]
            for i in range(1, n + 1)
        ]
        self.lower = np.array(
            [float(required(self.joints[n], "limit").attrib["lower"]) for n in self.names]
        )
        self.upper = np.array(
            [float(required(self.joints[n], "limit").attrib["upper"]) for n in self.names]
        )
        self.velocity = np.array(
            [float(required(self.joints[n], "limit").attrib["velocity"]) for n in self.names]
        )
        self.solvers = {
            side: SewArmSolver(
                self.lower[4 + 7 * i : 11 + 7 * i], self.upper[4 + 7 * i : 11 + 7 * i]
            )
            for i, side in enumerate(["left", "right"])
        }
        for side, solver in self.solvers.items():
            for i, axis in enumerate(solver.axes):
                joint = self.joints[f"{side}_arm_joint{i + 1}"]
                rpy = np.fromstring(required(joint, "origin").attrib.get("rpy", "0 0 0"), sep=" ")
                actual = np.fromstring(required(joint, "axis").attrib["xyz"], sep=" ")
                if (
                    joint.attrib["type"] != "revolute"
                    or not np.allclose(rpy, 0, atol=1e-7)
                    or not np.allclose(actual, axis)
                ):
                    raise ValueError(f"Unsupported SEW geometry: {side} joint {i + 1}")
            if not (-np.pi / 2 < solver.lower[5] < solver.upper[5] < np.pi / 2):
                raise ValueError("Wrist pitch limits must exclude gimbal lock")
        for i, axis in enumerate([[0, 1, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1]]):
            j = self.joints[f"torso_joint{i + 1}"]
            if not np.allclose(np.fromstring(required(j, "axis").attrib["xyz"], sep=" "), axis):
                raise ValueError("Unsupported torso axes")

    def fk(self, q: np.ndarray) -> dict[str, np.ndarray]:
        values = dict(zip(self.names, q, strict=True))
        frames = {"base_link": np.eye(4)}
        pending = list(self.joints.values())
        while pending:
            remaining = []
            for joint in pending:
                parent = required(joint, "parent").attrib["link"]
                if parent not in frames:
                    remaining.append(joint)
                    continue
                origin = joint.find("origin")
                xyz = (
                    np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ")
                    if origin is not None
                    else np.zeros(3)
                )
                rpy = (
                    np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ")
                    if origin is not None
                    else np.zeros(3)
                )
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
                transform[:3, 3] = xyz
                if joint.attrib["type"] == "revolute":
                    axis = np.fromstring(required(joint, "axis").attrib["xyz"], sep=" ")
                    transform[:3, :3] = (
                        transform[:3, :3]
                        @ Rotation.from_rotvec(
                            axis * values.get(joint.attrib["name"], 0.0)
                        ).as_matrix()
                    )
                frames[required(joint, "child").attrib["link"]] = frames[parent] @ transform
            if len(remaining) == len(pending):
                raise ValueError("Disconnected URDF tree")
            pending = remaining
        return frames
