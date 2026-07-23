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

"""Extract standalone per-arm URDFs from the robot's captured description.

The R1 Lite arms are A1X units whose kinematics differ from the standalone
A1Z models (link lengths, limits, zero convention), so per-arm IK must use
the chain captured from the robot itself. This produces one 6-DOF URDF per
arm rooted at that arm's base link, geometry stripped (the captured file
references vendor mesh paths that do not resolve outside the robot).

    python scripts/r1lite_test/extract_arm_urdfs.py \
        scripts/r1lite_test/r1lite_from_robot.urdf data/r1lite_description
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

ARM_DOF = 6


def _load_robot(path: Path) -> ET.Element:
    text = path.read_text()
    # The capture is a topic echo; drop anything after the document element.
    text = text[: text.index("</robot>") + len("</robot>")]
    return ET.fromstring(text)


def _strip_geometry(elem: ET.Element) -> None:
    for tag in ("visual", "collision"):
        for child in elem.findall(tag):
            elem.remove(child)


def extract_arm(root: ET.Element, side: str) -> ET.Element:
    joints = {j.get("name"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}

    chain_joints = [f"{side}_arm_joint{i}" for i in range(1, ARM_DOF + 1)]
    missing = [n for n in chain_joints if n not in joints]
    if missing:
        raise SystemExit(f"missing joints in capture: {missing}")

    out = ET.Element("robot", {"name": f"r1lite_{side}_arm"})
    base_link = joints[chain_joints[0]].find("parent").get("link")
    chain_links = [base_link] + [joints[name].find("child").get("link") for name in chain_joints]
    for link_name in chain_links:
        link = copy.deepcopy(links[link_name])
        _strip_geometry(link)
        out.append(link)
    for name in chain_joints:
        out.append(copy.deepcopy(joints[name]))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("out_dir", type=Path)
    args = parser.parse_args()

    root = _load_robot(args.capture)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for side in ("left", "right"):
        arm = extract_arm(root, side)
        ET.indent(arm)
        out_path = args.out_dir / f"r1lite_{side}_arm.urdf"
        out_path.write_bytes(ET.tostring(arm, xml_declaration=True))
        print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
