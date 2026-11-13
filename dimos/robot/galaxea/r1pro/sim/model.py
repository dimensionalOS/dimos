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

"""R1 Pro 2025 physical model, matching WIM's on-robot r1_pro.urdf.

Ledger: metres/kg/radians, X forward/Y left/Z up. Preserve the 2025 vendor
joint frames, axes, limits, masses, centres of mass and inertia tensors exactly.
In particular the torso folding links are 0.40/0.30 m, not the 2026 0.35/0.35 m.
WIM d2594a1's URDF is identical to this pinned public vendor description.

The vendor URDF repository omits the 2025 meshes. Arm, torso, chassis and jaw
meshes come from Galaxea's pinned older simulator. They are visual/collision
approximations: that simulator has small chassis revisions and different
sensor housings. Missing sensor/head meshes are omitted, retaining their mass
and frames; no 2026 geometry is silently substituted. MuJoCo uses convex hulls
for mesh collisions, with structural intersections excluded in scene config.
Wheel collision cylinders retain the 70 mm radius and existing contact model.
Wheel effort/speed and servo gains remain simulation assumptions, not measured
motor characteristics. The 2025 fingers retain vendor +/-50 mm travel, 0.25 m/s
speed and 100 N effort limits, with equal/opposite commands (no mimic).
"""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from dimos.constants import CACHE_DIR
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.galaxea.r1pro.config import R1PRO_DESCRIPTION_SOURCE

MODEL_PATH = CACHE_DIR / "sim2/r1pro-joylo-2025/r1pro.urdf"
VENDOR_MODEL_PATH = R1PRO_DESCRIPTION_SOURCE / "R1Pro" / "urdf_r1pro_2025.urdf"
MESH_SOURCE = RobotDescriptionSource(
    "https://github.com/OpenGalaxea/GalaxeaManipSim",
    "abe7f5161eeaa150e6eaffdf443af5df7f23f356",
)
MESH_ROOT = MESH_SOURCE / "galaxea_sim" / "assets" / "r1_pro" / "meshes"
# These sensor/head meshes are absent from the older public simulator. Keep
# their physical inertias and transforms; never silently omit a required mesh.
MESHLESS_LINKS = frozenset(
    (
        "camera_chassis_front_left_link",
        "camera_chassis_front_right_link",
        "camera_chassis_left_link",
        "camera_chassis_right_link",
        "camera_chassis_rear_link",
        "lidar_chassis_left_link",
        "imu_chassis_link",
        "imu_torso_link",
        "head_link",
        "camera_head_left_link",
        "camera_head_right_link",
        "left_D405_link",
        "right_D405_link",
    )
)
TORSO_LINK_LENGTHS = (0.4, 0.3)
WHEEL_RADIUS = 0.070
WHEEL_WIDTH = 0.08005
WHEEL_CENTER_Y = 0.002175
ASSUMED_WHEEL_EFFORT = 30.0
ASSUMED_WHEEL_SPEED = 20.0
STEER_JOINTS = tuple(f"r1pro/steer_motor_joint{i}" for i in range(1, 4))
WHEEL_JOINTS = tuple(f"r1pro/wheel_motor_joint{i}" for i in range(1, 4))
FINGER_TRAVEL_M = 0.05
FINGER_SPEED_M_S = 0.25
FINGER_JOINTS = tuple(
    f"r1pro/{side}_gripper_finger_joint{i}" for side in ("left", "right") for i in (1, 2)
)


def gen_urdf() -> str:
    root = ET.fromstring(Path(VENDOR_MODEL_PATH).read_text())
    for source_link in root.findall("link"):
        for tag in ("visual", "collision"):
            for element in source_link.findall(tag):
                mesh = element.find("geometry/mesh")
                if mesh is None:
                    continue
                path = Path(MESH_ROOT / Path(mesh.attrib["filename"]).name)
                if source_link.attrib["name"] in MESHLESS_LINKS:
                    source_link.remove(element)
                elif path.is_file():
                    mesh.set("filename", str(path))
                else:
                    raise FileNotFoundError(f"Required R1 Pro mesh missing: {path}")
    # Prefix after resolving vendor package:// paths explicitly.
    root = ET.fromstring(
        RobotModel.from_urdf_string(ET.tostring(root, encoding="unicode"))
        .with_prefix("r1pro/")
        .load()
        .xml
    )
    for name in WHEEL_JOINTS:
        joint = root.find(f"joint[@name='{name}']")
        assert joint is not None
        ET.SubElement(
            joint, "limit", effort=str(ASSUMED_WHEEL_EFFORT), velocity=str(ASSUMED_WHEEL_SPEED)
        )
    for i in range(1, 4):
        link = root.find(f"link[@name='r1pro/wheel_motor_link{i}']")
        assert link is not None
        for collision in link.findall("collision"):
            link.remove(collision)
        collision = ET.SubElement(link, "collision")
        ET.SubElement(
            collision, "origin", xyz=f"0 {WHEEL_CENTER_Y} 0", rpy="1.5707963267948966 0 0"
        )
        ET.SubElement(
            ET.SubElement(collision, "geometry"),
            "cylinder",
            radius=str(WHEEL_RADIUS),
            length=str(WHEEL_WIDTH),
        )
    return ET.tostring(root, encoding="unicode")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=MODEL_PATH)
    args = parser.parse_args()
    xml = gen_urdf()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(xml)
    print(args.output)


if __name__ == "__main__":
    main()
