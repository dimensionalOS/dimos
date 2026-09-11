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

"""Rebuild the locker rooms and side benchmark wing without changing match rules."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OFFSET = (8.3, -0.825)
# Keep the pitch edge fixed. Each locker room is 2.55 x 1.625 metres.
SHIFT = 1.625
SPAWN_Y = -0.25 + SHIFT
DOOR_Y = 1.25


def values(items):
    return " ".join(f"{v:.6g}" for v in items)


def build():
    source = ET.parse(ROOT / "assets/scenes/benchmark/original.xml").getroot()
    for inc in list(source.findall("include")):
        source.remove(inc)
    source.set("model", "benchmark_wing")
    body = source.find("worldbody")
    for name in ("wall_north_west", "wall_north_east", "wall_west"):
        body.remove(body.find(f"geom[@name='{name}']"))

    def oldwall(name, size, pos):
        ET.SubElement(
            body,
            "geom",
            name=name,
            type="box",
            size=values(size),
            pos=values(pos),
            material="wallmat",
            group="0",
        )

    oldwall("wall_north_west", [2.1, 0.05, 0.25], [0, 2.05, 0.25])
    # Door at local y=.25..1.05 joins the narrow corridor without hitting a divider.
    oldwall("wall_west", [0.05, 1.125, 0.25], [-2.05, -0.875, 0.25])
    oldwall("wall_west_upper", [0.05, 0.475, 0.25], [-2.05, 1.525, 0.25])
    group = ET.Element("body", name="benchmark_wing", pos=values([*OFFSET, 0]))
    for element in list(body):
        body.remove(element)
        group.append(element)
    body.append(group)
    ET.indent(source)
    ET.ElementTree(source).write(ROOT / "assets/scenes/benchmark/scene.xml", encoding="unicode")
    root = ET.Element("mujoco", model="microduck_football_club")
    ET.SubElement(root, "include", file="../benchmark/scene.xml")
    ET.SubElement(root, "include", file="../football/field.xml")
    world = ET.SubElement(root, "worldbody")

    def box(name, size, pos, color, visual=False):
        return ET.SubElement(
            world,
            "geom",
            name=name,
            type="box",
            size=values(size),
            pos=values(pos),
            rgba=color,
            group="1" if visual else "0",
            **({"contype": "0", "conaffinity": "0"} if visual else {}),
        )

    wall = "0.78 0.82 0.83 1"
    dark = "0.13 0.18 0.22 1"
    # The pitch floor starts at y=2.1. Extend to its edge while retaining the
    # club floor's south edge at y=0.275; ending at the wall centre leaves a gap.
    box("club_floor", [3.35, 0.9125, 0.05], [0, 1.1875, -0.05], "0.58 0.64 0.65 1")
    box("tunnel_floor", [0.6, 0.45, 0.05], [0, -0.125, -0.05], "0.58 0.64 0.65 1")
    box("corridor_floor", [2.8, 0.45, 0.05], [3.4, -0.125, -0.05], "0.57 0.64 0.65 1")
    box("corridor_south", [3.45, 0.05, 0.3], [2.85, -0.625, 0.3], wall)
    box("corridor_north", [1.45, 0.05, 0.3], [4.85, 0.375, 0.3], wall)
    box("tunnel_west", [0.05, 0.5, 0.3], [-0.65, -0.125, 0.3], wall)
    for team, sign, color in [("red", -1, "0.89 0.30 0.31 1"), ("blue", 1, "0.25 0.56 0.91 1")]:
        box(f"{team}_outer_wall", [0.05, 0.8375, 0.42], [sign * 3.3, 1.1625, 0.42], wall)
        box(f"{team}_south_wall", [1.325, 0.05, 0.42], [sign * 1.975, 0.325, 0.42], wall)
        # Door into the central passage is y=.75..1.75, one metre wide.
        box(f"{team}_inner_south", [0.05, 0.2125, 0.42], [sign * 0.65, 0.5375, 0.42], wall)
        box(f"{team}_inner_north", [0.05, 0.15, 0.42], [sign * 0.65, 1.9, 0.42], wall)
        box(
            f"floor_{team}_lockers",
            [1.25, 0.7875, 0.001],
            [sign * 1.95, 1.1625, 0.001],
            color,
            True,
        )
        box(f"{team}_door_header", [0.06, 0.50, 0.045], [sign * 0.65, DOOR_Y, 0.88], color)
        # Three low lockers and benches face the individual standing bays.
        for number in range(1, 4):
            x = sign * (2.65 - (number - 1) * 0.75)
            box(f"{team}_locker_{number}", [0.29, 0.16, 0.33], [x, -1.08 + SHIFT, 0.33], color)
            box(
                f"{team}_locker_inset_{number}",
                [0.25, 0.007, 0.26],
                [x, -0.911 + SHIFT, 0.33],
                dark,
                True,
            )
            box(
                f"{team}_locker_handle_{number}",
                [0.015, 0.015, 0.045],
                [x + 0.16, -0.895 + SHIFT, 0.35],
                "0.75 0.80 0.82 1",
                True,
            )
            for slat in range(3):
                box(
                    f"{team}_vent_{number}_{slat}",
                    [0.15, 0.009, 0.006],
                    [x, -0.900 + SHIFT, 0.48 + slat * 0.035],
                    color,
                    True,
                )
            box(
                f"{team}_bench_{number}",
                [0.28, 0.115, 0.022],
                [x, -0.67 + SHIFT, 0.14],
                "0.66 0.47 0.29 1",
            )
            for dx in (-0.20, 0.20):
                box(
                    f"{team}_bench_leg_{number}_{dx}",
                    [0.018, 0.075, 0.059],
                    [x + dx, -0.67 + SHIFT, 0.059],
                    dark,
                )
            for edge in (-1, 1):
                box(
                    f"{team}_bay_{number}_x{edge}",
                    [0.24, 0.008, 0.001],
                    [x, SPAWN_Y + edge * 0.23, 0.002],
                    color,
                    True,
                )
                box(
                    f"{team}_bay_{number}_y{edge}",
                    [0.008, 0.23, 0.001],
                    [x + edge * 0.24, SPAWN_Y, 0.002],
                    color,
                    True,
                )
    # Direction markers are paint only, with no floor collision bumps.
    for y in (-0.3, 0.2, 0.7, 1.2, 1.7):
        box(f"tunnel_mark_{y}", [0.055, 0.11, 0.001], [0, y, 0.002], "0.94 0.93 0.77 1", True)
    ET.indent(root)
    ET.ElementTree(root).write(ROOT / "assets/scenes/apartment/scene.xml", encoding="unicode")
    # Rebuild metadata from the immutable pre-move benchmark reference.
    old = json.loads((ROOT / "assets/scenes/benchmark/places.json").read_text())
    rooms = {}
    for name, r in old["rooms"].items():
        if name == "football":
            continue
        x0, x1, y0, y1 = r["bounds"]
        x, y, yaw = r["target"]
        rooms[name] = {
            **r,
            "bounds": [x0 + OFFSET[0], x1 + OFFSET[0], y0 + OFFSET[1], y1 + OFFSET[1]],
            "target": [x + OFFSET[0], y + OFFSET[1], yaw],
        }

    def room(name, aliases, bounds, target):
        rooms[name] = {"name": name, "aliases": aliases, "bounds": bounds, "target": target}

    room(
        "football",
        ["football field", "pitch", "soccer"],
        [-3.25, 3.25, 2.1, 6.5],
        [0, 3.05, 1.5707963268],
    )
    room(
        "red_lockers",
        ["red locker room", "red changing room"],
        [-3.25, -0.7, 0.375, 2],
        [-1.6, DOOR_Y, 0],
    )
    room(
        "blue_lockers",
        ["blue locker room", "blue changing room"],
        [0.7, 3.25, 0.375, 2],
        [1.6, DOOR_Y, 3.1415926536],
    )
    room(
        "player_tunnel",
        ["player entrance", "tunnel"],
        [-0.6, 0.6, -0.575, 2],
        [0, DOOR_Y, 1.5707963268],
    )
    room(
        "benchmark_corridor",
        ["benchmark corridor", "side corridor"],
        [0.6, 6.2, -0.575, 0.325],
        [4, -0.175, 0],
    )
    objects = {name: [x + OFFSET[0], y + OFFSET[1]] for name, (x, y) in old["objects"].items()}
    places = {
        "id": "football-club-v3",
        "spawn_xy": [-0.6, 3.25],
        "rooms": rooms,
        "objects": objects,
    }
    (ROOT / "assets/scenes/apartment/places.json").write_text(json.dumps(places, indent=2) + "\n")
    appearance = json.loads((ROOT / "assets/scenes/apartment/viewer.json").read_text())
    appearance["camera"] = {"position": [6, -5, 8], "target": [0, 3.2, 0.12]}
    appearance["colors"].update({"floor_red_lockers": "#bd6163", "floor_blue_lockers": "#628ebd"})
    appearance["views"].update(
        {
            "lockers": {"position": [4, -5, 5], "target": [0, 1.15, 0.1], "extent": [6.7, 1.8, 1]},
            "benchmark": {
                "position": [13, -8, 6],
                "target": [*OFFSET, 0.1],
                "extent": [4.2, 4.2, 0.8],
            },
        }
    )
    (ROOT / "assets/scenes/apartment/viewer.json").write_text(
        json.dumps(appearance, indent=2) + "\n"
    )


if __name__ == "__main__":
    build()
