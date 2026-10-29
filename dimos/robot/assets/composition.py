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

"""URDF composition and materialization used by the RobotModel API."""

from collections.abc import Sequence
from hashlib import sha256
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT

Vector3 = tuple[float, float, float]


def compose_urdf(
    instances: Sequence[tuple[str, str, Vector3, Vector3]], *, name: str, root_link: str
) -> str:
    """Join core URDF trees with namespaced identifiers and fixed root mounts.

    Each instance supplies a prefix, already resolved XML, xyz in meters, and
    extrinsic xyz roll/pitch/yaw in radians. Controller and simulator extensions
    are rejected rather than carrying unrewritten references into the assembly.
    """
    combined = ET.Element("robot", name=name)
    ET.SubElement(combined, "link", name=root_link)
    for prefix, xml, xyz, rpy in instances:
        source = ET.fromstring(xml)
        if source.tag != "robot":
            raise ValueError("Composition inputs must be URDF robot elements")
        unsupported = {child.tag for child in source} - {"link", "joint", "material"}
        if unsupported:
            raise ValueError(f"Unsupported URDF composition elements: {sorted(unsupported)}")
        links = {node.get("name", "") for node in source.findall("link")}
        children: set[str] = set()
        edges: list[tuple[str, str]] = []
        for joint in source.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                raise ValueError("Composition joints must declare parent and child links")
            if parent.get("link") not in links or child.get("link") not in links:
                raise ValueError("Composition joint references an unknown link")
            child_link = child.get("link", "")
            if child_link in children:
                raise ValueError("Composition links must have at most one parent")
            children.add(child_link)
            edges.append((parent.get("link", ""), child_link))
        roots = links - children
        if len(roots) != 1:
            raise ValueError("Composition inputs must have exactly one root link")
        source_root = next(iter(roots))
        # Check reachability as well as the root count: a disconnected cycle
        # can coexist with an otherwise valid root.
        reached = {source_root}
        while True:
            expanded = reached | {child for parent, child in edges if parent in reached}
            if expanded == reached:
                break
            reached = expanded
        if reached != links:
            raise ValueError("Composition input must be one connected tree")
        for child in source:
            for element in child.iter():
                if element.tag in {"link", "joint", "material"} and element.get("name"):
                    element.set("name", prefix + element.attrib["name"])
                if element.tag in {"parent", "child"} and element.get("link"):
                    element.set("link", prefix + element.attrib["link"])
                if element.tag == "mimic" and element.get("joint"):
                    element.set("joint", prefix + element.attrib["joint"])
                if element.tag in {"mesh", "texture"}:
                    filename = element.get("filename", "")
                    if filename.startswith("package://") or (
                        filename and not Path(filename).is_absolute() and "://" not in filename
                    ):
                        raise ValueError(f"Unresolved asset in composition input: {filename!r}")
            combined.append(child)
        mount = ET.SubElement(combined, "joint", name=f"{prefix}mount", type="fixed")
        ET.SubElement(mount, "parent", link=root_link)
        ET.SubElement(mount, "child", link=f"{prefix}{source_root}")
        ET.SubElement(mount, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
    for tag in ("link", "joint", "material"):
        names = [element.get("name", "") for element in combined.findall(tag)]
        if any(not item for item in names) or len(names) != len(set(names)):
            raise ValueError(f"Composed {tag} names must be non-empty and unique")
    joints = {element.attrib["name"] for element in combined.findall("joint")}
    for mimic in combined.findall("joint/mimic"):
        if mimic.get("joint") not in joints:
            raise ValueError("Composition mimic references an unknown joint")
    return ET.tostring(combined, encoding="unicode")


def materialize_composed_urdf(xml: str) -> Path:
    """Atomically cache an assembled description by the complete XML content."""
    folder = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "derived" / "composed_models"
    path = folder / f"{sha256(xml.encode()).hexdigest()}.urdf"
    if not path.exists():
        folder.mkdir(parents=True, exist_ok=True)
        temporary: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", dir=folder, suffix=".urdf", delete=False
            ) as stream:
                temporary = Path(stream.name)
                stream.write(xml)
            temporary.replace(path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return path
