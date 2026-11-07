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

"""Simulation adaptation of the pinned vendor R1 Pro description.

Ledger: preserve all vendor link frames, joint axes, visual meshes and inertias
in metres/kg/radians (X forward, Y left, Z up). The three wheels turn about +Y;
positive wheel speed rolls toward +X at zero steering. Steering is about +Z.
Wheel collision cylinders use the measured vendor mesh bounds: radius .070 m,
Y extent [-.03785, .04220] m. Visuals remain the vendor meshes. Other mesh
collisions are MuJoCo convex hulls, with known structural overlaps excluded by
scene configuration. The G1Z fingers keep the vendor origins and +Y axes:
finger 1 opens toward +Y, finger 2 toward -Y (verified on the fingertip meshes).
Zero is closed. Replace the vendor's zero limits with symmetric 52 mm travel
per finger: G1Z's published aperture is 104 mm and aperture speed 108 mm/s.
Source: https://galaxea-dynamics.com/products/galaxea-g1z-gripper
Use two driven fingers with equal/opposite targets, not an unsupported mimic.
The 30 N per-finger effort cap and PD gains are conservative simulation tuning,
not a calibrated model of the published 150 N clamping force.
The vendor omits wheel effort/speed limits; the demo assumptions below are NOT
hardware specifications. Servo gains likewise are simulation tuning only.
Wrist variant: retain the vendor D405 mount, mesh and inertia unchanged. The
co-located alternative GMSL fisheye cameras are absent on this depth-camera
configuration: their links remain frame-only (no visual, collision or inertia).
This prevents the alternative housing from occluding the D405 lens. The D405
link origin is used as an ideal optical center (+X right, +Y down, +Z forward);
mesh ray checks place the lens at z=0 with the housing behind it. Factory
optical-center corrections and distortion remain unavailable.
"""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from dimos.constants import CACHE_DIR
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL_PATH, R1PRO_PACKAGE_ROOT

MODEL_PATH = CACHE_DIR / "sim2/r1pro/r1pro.urdf"
WHEEL_RADIUS = 0.070
WHEEL_WIDTH = 0.08005
WHEEL_CENTER_Y = 0.002175
ASSUMED_WHEEL_EFFORT = 30.0
ASSUMED_WHEEL_SPEED = 20.0
STEER_JOINTS = tuple(f"r1pro/steer_motor_joint{i}" for i in range(1, 4))
WHEEL_JOINTS = tuple(f"r1pro/wheel_motor_joint{i}" for i in range(1, 4))
FINGER_TRAVEL_M = 0.052
FINGER_SPEED_M_S = 0.054
ASSUMED_FINGER_EFFORT_N = 30.0
FINGER_JOINTS = tuple(
    f"r1pro/{side}_gripper_finger_joint{i}" for side in ("left", "right") for i in (1, 2)
)


def gen_urdf() -> str:
    source = (
        RobotModel.from_file(R1PRO_MODEL_PATH, package_paths={"r1pro_urdf": R1PRO_PACKAGE_ROOT})
        .with_prefix("r1pro/")
        .load()
    )
    root = ET.fromstring(source.xml)
    for index, name in enumerate(FINGER_JOINTS):
        joint = root.find(f"joint[@name='{name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        limit.attrib.update(
            lower=str(0 if index % 2 == 0 else -FINGER_TRAVEL_M),
            upper=str(FINGER_TRAVEL_M if index % 2 == 0 else 0),
            effort=str(ASSUMED_FINGER_EFFORT_N),
            velocity=str(FINGER_SPEED_M_S),
        )
    for side in ("left", "right"):
        link = root.find(f"link[@name='r1pro/{side}_gmsl_link']")
        assert link is not None
        for element in list(link):
            link.remove(element)
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
