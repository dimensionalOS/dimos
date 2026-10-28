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

"""Lazy composition of two copies of the pinned official Metal model."""

from copy import deepcopy
from hashlib import sha256
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.dual_makermods_metal.config import ALL_ARM_JOINTS, ARM_JOINTS, SIDES
from dimos.robot.manipulators.makermods_metal.model import make_metal_model_config

Vector3 = tuple[float, float, float]
# Example bench layout only; no published official bimanual mount transform was found.
DEFAULT_LEFT_XYZ: Vector3 = (0.0, 0.3, 0.0)
DEFAULT_RIGHT_XYZ: Vector3 = (0.0, -0.3, 0.0)
ZERO_RPY: Vector3 = (0.0, 0.0, 0.0)


class DualMetalDescription(os.PathLike[str]):
    """Materialize a content-addressed combined URDF only when its path is used."""

    def __init__(
        self, left_xyz: Vector3, right_xyz: Vector3, left_rpy: Vector3, right_rpy: Vector3
    ) -> None:
        self.mounts = ((left_xyz, left_rpy), (right_xyz, right_rpy))
        if not all(math.isfinite(v) for mount in self.mounts for vector in mount for v in vector):
            raise ValueError("Metal mount xyz/rpy values must be finite")
        if left_xyz == right_xyz:
            raise ValueError("Metal arm bases must have distinct positions")

    def __fspath__(self) -> str:
        single = ET.fromstring(make_metal_model_config().model.load().xml)
        combined = ET.Element("robot", name="dual_makermods_metal")
        ET.SubElement(combined, "link", name="dual_metal_base")
        for side, (xyz, rpy) in zip(SIDES, self.mounts, strict=True):
            for child in single:
                node = deepcopy(child)
                for element in node.iter():
                    for attr in ("name", "link", "joint"):
                        value = element.get(attr)
                        if value:
                            element.set(attr, f"{side}_{value}")
                combined.append(node)
            mount = ET.SubElement(combined, "joint", name=f"{side}_mount", type="fixed")
            ET.SubElement(mount, "parent", link="dual_metal_base")
            ET.SubElement(mount, "child", link=f"{side}_base_link")
            ET.SubElement(mount, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
        data = ET.tostring(combined, encoding="unicode")
        folder = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "derived" / "dual_makermods_metal"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{sha256(data.encode()).hexdigest()}.urdf"
        if not path.exists():
            with tempfile.NamedTemporaryFile(
                mode="w", dir=folder, suffix=".urdf", delete=False
            ) as stream:
                temporary = Path(stream.name)
                stream.write(data)
            try:
                temporary.replace(path)
            finally:
                temporary.unlink(missing_ok=True)
        return str(path)


def dual_metal_model_config(
    *,
    left_xyz: Vector3 = DEFAULT_LEFT_XYZ,
    right_xyz: Vector3 = DEFAULT_RIGHT_XYZ,
    left_rpy: Vector3 = ZERO_RPY,
    right_rpy: Vector3 = ZERO_RPY,
) -> RobotModelConfig:
    """A single collision world and independently selectable pose groups for both arms."""
    return RobotModelConfig(
        model=RobotModel.from_file(DualMetalDescription(left_xyz, right_xyz, left_rpy, right_rpy)),
        joint_names=list(ALL_ARM_JOINTS),
        base_link="dual_metal_base",
        planning_groups=[
            PlanningGroupDefinition(
                name=f"{side}_arm",
                joint_names=ARM_JOINTS[side],
                base_link="dual_metal_base",
                tip_link=f"{side}_gripper_base",
            )
            for side in SIDES
        ],
        gripper_hardware_ids={f"{side}_arm": f"{side}_arm" for side in SIDES},
        auto_convert_meshes=True,
    )
