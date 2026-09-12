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

"""Seeded rigid-object layouts and local MuJoCo overlays for ACT collection."""

from dataclasses import asdict, dataclass, replace
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from dimos.robot.galaxea.r1pro.grasping_sim import TABLE_Z
from dimos.robot.galaxea.r1pro.object_packing import (
    MAX_OBJECTS as MAX_OBJECTS,
    SHAPES as SHAPES,
    ObjectShape,
)
from dimos.robot.galaxea.r1pro.packing import OccupiedFootprint, empty_slots
from dimos.robot.galaxea.r1pro.tray_sim import prepare_tray_delivery_scene

OBJECT_TRAY_XY = (0.34, -0.04)
OBJECT_TRAY_HALF_SIZE = (0.145, 0.145)
# Reserve room along Y for opening fingers below the rim of shorter objects.
OBJECT_SLOT_BOUNDS = (0.145, 0.105)


@dataclass(frozen=True)
class PackingObject:
    name: str
    shape: ObjectShape
    half_size: tuple[float, float, float]
    mass: float
    rgba: tuple[float, float, float, float]
    position: tuple[float, float, float]
    yaw: float
    in_tray: bool = False

    @property
    def joint(self) -> str:
        return self.name + "_free"

    @property
    def radius(self) -> float:
        x, y, _ = self.half_size
        return math.hypot(x, y) if self.shape == "box" else x


@dataclass(frozen=True)
class ObjectLayout:
    seed: int
    objects: tuple[PackingObject, ...]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _sample_layout(seed: int, *, count: int | None = None, occupied: int = 0) -> ObjectLayout:
    """Generate supported, separated objects; IDs do not encode pose or shape."""
    rng = np.random.default_rng(seed)
    count = int(rng.integers(4, 6)) if count is None else count
    if count not in (4, 5) or not 0 <= occupied < count:
        raise ValueError("Use four or five objects with at least one remaining source")
    placed: list[PackingObject] = []
    footprints: list[OccupiedFootprint] = []
    aboard = set(map(int, rng.permutation(count)[:occupied]))
    for i in range(count):
        shape = SHAPES[int(rng.integers(len(SHAPES)))]
        radius = float(rng.uniform(0.018, 0.024))
        half_z = float(rng.uniform(0.052, 0.076))
        angle = float(rng.uniform(0.6, 0.97))
        half_xy = (
            (radius * math.cos(angle), radius * math.sin(angle))
            if shape == "box"
            else (radius, radius)
        )
        yaw = float(rng.uniform(-math.pi, math.pi))
        if i in aboard:
            slots = empty_slots(radius, tuple(footprints), inner_half_size=OBJECT_SLOT_BOUNDS)
            if not slots:
                raise RuntimeError("Sampled objects do not fit the initial tray occupancy")
            x, y = slots[0]
            footprints.append(OccupiedFootprint(x, y, radius))
            position = (OBJECT_TRAY_XY[0] + x, OBJECT_TRAY_XY[1] + y, TABLE_Z + 0.016 + half_z)
        else:
            for _ in range(1000):
                x = float(rng.uniform(0.235, 0.51))
                y = float(rng.uniform(-0.53, -0.335))
                # Open fingers extend along Y. Space is checked geometrically,
                # independently of the requested order and object identity.
                if all(
                    o.in_tray
                    or abs(x - o.position[0]) > radius + o.radius + 0.026
                    or abs(y - o.position[1]) > radius + o.radius + 0.090
                    for o in placed
                ):
                    break
            else:
                raise RuntimeError("Could not sample separated grasp approaches")
            position = (x, y, TABLE_Z + half_z + 0.001)
        placed.append(
            PackingObject(
                f"task_object_{i + 1}",
                shape,
                (*half_xy, half_z),
                float(rng.uniform(0.07, 0.15)),
                (
                    float(rng.uniform(0.12, 0.9)),
                    float(rng.uniform(0.12, 0.9)),
                    float(rng.uniform(0.12, 0.9)),
                    1.0,
                ),
                position,
                yaw,
                i in aboard,
            )
        )
    return ObjectLayout(seed, tuple(placed))


def sample_layout(seed: int, *, count: int | None = None, occupied: int = 0) -> ObjectLayout:
    """Deterministic bounded restarts avoid a final object blocked by earlier samples."""
    for attempt in range(32):
        try:
            candidate = _sample_layout(seed * 32 + attempt, count=count, occupied=occupied)
            return ObjectLayout(seed, candidate.objects)
        except RuntimeError:
            continue
    raise RuntimeError("Could not generate a separated layout after 32 attempts")


def perturb_tray_occupants(layout: ObjectLayout, distance: float) -> ObjectLayout:
    """Model imperfect earlier placements while keeping source geometry identical."""
    if not 0 <= distance <= 0.02:
        raise ValueError("Tray perturbations must be between zero and two centimetres")
    if distance == 0:
        return layout
    rng = np.random.default_rng(layout.seed + 701)
    objects = list(layout.objects)
    for i, obj in enumerate(objects):
        if not obj.in_tray:
            continue
        for _ in range(100):
            xy = np.asarray(obj.position[:2]) + rng.uniform(-distance, distance, 2)
            relative = xy - OBJECT_TRAY_XY
            if np.any(np.abs(relative) + obj.radius + 0.005 >= OBJECT_TRAY_HALF_SIZE):
                continue
            if any(
                other.in_tray
                and j != i
                and np.linalg.norm(xy - other.position[:2]) <= obj.radius + other.radius + 0.015
                for j, other in enumerate(objects)
            ):
                continue
            objects[i] = replace(obj, position=(float(xy[0]), float(xy[1]), obj.position[2]))
            break
        else:
            raise RuntimeError("Could not perturb a tray occupant without overlap")
    return ObjectLayout(layout.seed, tuple(objects))


def prepare_object_scene(
    output: Path,
    layout: ObjectLayout,
    *,
    scene_package: Path | None = None,
    template: Path | None = None,
) -> Path:
    """Compile physical objects into an overlay without modifying house assets."""
    output.parent.mkdir(parents=True, exist_ok=True)
    if template is None:
        template = prepare_tray_delivery_scene(
            output.parent / "object-template.xml", scene_package=scene_package
        )
    tree = ET.parse(template)
    root = tree.getroot()
    world = root.find("worldbody")
    assert world is not None
    old = world.find('body[@name="task_bottle"]')
    assert old is not None
    world.remove(old)
    tray = world.find('body[@name="task_bin"]')
    assert tray is not None
    tray.set("pos", f"{OBJECT_TRAY_XY[0]} {OBJECT_TRAY_XY[1]} {TABLE_Z + 0.001}")
    for geom in tray.findall("geom"):
        name = geom.get("name", "")
        if name == "bin_floor":
            geom.set("size", "0.155 0.155 0.0075")
        elif name.startswith("bin_wall_"):
            position = np.fromstring(geom.get("pos", ""), sep=" ")
            size = np.fromstring(geom.get("size", ""), sep=" ")
            axis = int(np.argmax(np.abs(position[:2])))
            position[axis] = float(np.sign(position[axis])) * 0.15
            size[1 - axis] = 0.155
            geom.set("pos", " ".join(map(str, position)))
            geom.set("size", " ".join(map(str, size)))
        elif "handle" in name:
            sign = 1 if "left" in name else -1
            geom.set("pos", f"0 {sign * 0.215} 0.075")
        elif "strut" in name:
            points = np.fromstring(geom.get("fromto", ""), sep=" ")
            sign = 1 if "left" in name else -1
            points[1], points[4] = sign * 0.15, sign * 0.215
            geom.set("fromto", " ".join(map(str, points)))
    for site in tray.findall("site"):
        sign = 1 if "left" in site.get("name", "") else -1
        site.set("pos", f"0 {sign * 0.215} 0.075")
    for index in range(1, 5):
        actuator = root.find(f'.//actuator/general[@name="r1pro/torso_joint{index}"]')
        assert actuator is not None
        actuator.set("biasprm", "0 -250 -80")
    for obj in layout.objects:
        x, y, h = obj.half_size
        body = ET.SubElement(
            world,
            "body",
            name=obj.name,
            pos=" ".join(map(str, obj.position)),
            quat=f"{math.cos(obj.yaw / 2)} 0 0 {math.sin(obj.yaw / 2)}",
        )
        ET.SubElement(body, "freejoint", name=obj.joint)
        common = dict(
            rgba=" ".join(map(str, obj.rgba)),
            friction="1.0 0.02 0.001",
            condim="4",
            solref="0.005 1",
        )
        if obj.shape == "box":
            ET.SubElement(
                body,
                "geom",
                name=obj.name + "_body",
                type="box",
                size=f"{x} {y} {h}",
                mass=str(obj.mass),
                attrib=common,
            )
        elif obj.shape == "cylinder":
            ET.SubElement(
                body,
                "geom",
                name=obj.name + "_body",
                type="cylinder",
                size=f"{x} {h}",
                mass=str(obj.mass),
                attrib=common,
            )
        else:
            ET.SubElement(
                body,
                "geom",
                name=obj.name + "_body",
                type="cylinder",
                size=f"{x} {h * 0.8}",
                pos=f"0 0 {-h * 0.2}",
                mass=str(obj.mass * 0.9),
                attrib=common,
            )
            ET.SubElement(
                body,
                "geom",
                name=obj.name + "_neck",
                type="cylinder",
                size=f"{x * 0.52} {h * 0.2}",
                pos=f"0 0 {h * 0.8}",
                mass=str(obj.mass * 0.1),
                attrib=common,
            )
    ET.indent(root)
    tree.write(output, encoding="unicode")
    return output
