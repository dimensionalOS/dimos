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

"""Download the pinned ABC assets and build the three-bottle demo scene."""

import argparse
from copy import deepcopy
from pathlib import Path
import shutil
import tarfile
import tempfile
from typing import BinaryIO, cast
import xml.etree.ElementTree as ET

import requests

from dimos.constants import DIMOS_PROJECT_ROOT

ABC_REVISION = "6bc6586721cf0c409ccee80f675a28de9b9b2f5e"
ABC_ARCHIVE_URL = f"https://codeload.github.com/amazon-far/abc/tar.gz/{ABC_REVISION}"
SIM_ASSET_DIR = DIMOS_PROJECT_ROOT / "data" / "dual_openyam_sim"
DEMO_BOTTLES = ("bottle_1", "bottle_4", "bottle_6")


def write_demo_scene(source: Path, destination: Path) -> None:
    """Keep three separated bottles and remove the upstream malformed keyframe."""
    tree = ET.parse(source)
    root = tree.getroot()
    for keyframe in root.findall("keyframe"):
        root.remove(keyframe)
    # Two tilted bottles need more room than the upstream single-bottle bin.
    # Local Y is world Z after the bin's quarter-turn, so preserve its depth.
    for mesh in root.findall("./asset/mesh"):
        if "garbage_can/" in mesh.get("file", ""):
            x, y, z = map(float, mesh.get("scale", "1 1 1").split())
            mesh.set("scale", f"{1.5 * x} {y} {1.5 * z}")
    world = root.find("worldbody")
    if world is None:
        raise ValueError("ABC scene has no worldbody")
    right = world.find("./body[@name='bottle_1']")
    left = world.find("./body[@name='bottle_4']")
    if right is None or left is None:
        raise ValueError("ABC scene is missing the two demo targets")
    # Use the same object on each side so the bimanual task tests arm choice
    # and placement, with one calibrated grasp geometry.
    mirrored = deepcopy(right)
    for element in mirrored.iter():
        if "name" in element.attrib:
            element.set("name", f"demo_left_{element.attrib['name']}")
    mirrored.set("name", "bottle_4")
    mirrored.set("pos", left.attrib["pos"])
    world.remove(left)
    world.append(mirrored)
    for body in list(world.findall("body")):
        name = body.get("name", "")
        if name.startswith("bottle_") and name not in DEMO_BOTTLES:
            world.remove(body)
        elif name == "bottle_6":
            body.set("pos", "0.9 0.0 0.754")
    with destination.open("x") as stream:
        tree.write(stream, encoding="unicode")


def download_assets(destination: Path) -> None:
    """Extract only the task assets and their licenses from the pinned archive."""
    prefix = f"abc-{ABC_REVISION}/assets/put_bottles/"
    with tempfile.TemporaryDirectory(prefix="openyam-assets-") as scratch:
        stage = Path(scratch) / "scene"
        stage.mkdir()
        with requests.get(ABC_ARCHIVE_URL, stream=True, timeout=(15, 120)) as response:
            response.raise_for_status()
            with tarfile.open(fileobj=cast("BinaryIO", response.raw), mode="r|gz") as archive:
                for member in archive:
                    relative = None
                    if member.name.startswith(prefix):
                        relative = Path(member.name.removeprefix(prefix))
                    elif member.name == f"abc-{ABC_REVISION}/LICENSE":
                        relative = Path("ABC_LICENSE")
                    if relative is None or not member.isfile():
                        continue
                    if relative.is_absolute() or ".." in relative.parts:
                        raise ValueError("Unsafe archive member")
                    target = stage / relative
                    target.parent.mkdir(parents=True, exist_ok=True)
                    source = archive.extractfile(member)
                    if source is None:
                        raise ValueError(f"Unreadable archive member: {member.name}")
                    with source, target.open("wb") as output:
                        shutil.copyfileobj(source, output)
        if not (stage / "put_bottle.xml").is_file():
            raise ValueError("Pinned archive does not contain the task scene")
        write_demo_scene(stage / "put_bottle.xml", stage / "demo.xml")
        (stage / "SOURCE.md").write_text(
            f"Source: https://github.com/amazon-far/abc @ {ABC_REVISION}\n"
            "Path: assets/put_bottles/. ABC_LICENSE: Apache-2.0; "
            "assets/i2rt_yam/LICENSE: MIT.\n"
            "demo.xml removes the invalid keyframe and keeps bottle_1, bottle_4, "
            "bottle_6 (the last moved to x=0.9,y=0), and widens the bin by 1.5x "
            "horizontally to hold two bottles. Both targets use bottle_1's geometry. "
            "Robot and contact parameters are unchanged.\n"
        )
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(stage, destination)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=SIM_ASSET_DIR)
    parser.add_argument(
        "--existing-assets",
        action="store_true",
        help="Build demo.xml from an already downloaded put_bottle.xml",
    )
    args = parser.parse_args()
    if args.existing_assets:
        write_demo_scene(args.destination / "put_bottle.xml", args.destination / "demo.xml")
    elif args.destination.exists():
        parser.error("destination exists; use --existing-assets or choose a new directory")
    else:
        download_assets(args.destination)
    print(args.destination / "demo.xml")


if __name__ == "__main__":
    main()
