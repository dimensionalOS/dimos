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

"""Rebuild the project-owned football room MJCF; all dimensions are metres."""

from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from microduck_world.ball_physics import BALL_RADIUS, BALL_SPAWN_HEIGHT
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
GREEN = "0.176 0.467 0.345 1"
IVORY = "0.93 0.95 0.88 1"
BLUE = "0.20 0.53 0.75 1"
CORAL = "0.88 0.37 0.27 1"


def values(items):
    return " ".join(f"{v:.6g}" for v in items)


def build():
    root = ET.Element("mujoco", model="microduck_football_club")
    world = ET.SubElement(root, "worldbody")

    def geom(name, kind, size, pos=None, color=IVORY, decorative=False, **kwargs):
        attrs = dict(
            name=name, type=kind, size=values(size), rgba=color, group="1" if decorative else "0"
        )
        if pos is not None:
            attrs["pos"] = values(pos)
        if decorative:
            attrs.update(contype="0", conaffinity="0")
        attrs.update(kwargs)
        return ET.SubElement(world, "geom", attrs)

    # All pitch paint is one surface texture, shared by MuJoCo and Three.js.
    width, height = 2048, 1344
    surface = Image.new("RGB", (width, height), (71, 89, 87))
    draw = ImageDraw.Draw(surface)

    def pixel(point):
        point = (point[0] * 0.5, 4.3 + (point[1] - 4.3) * 0.5)
        return ((point[0] + 3.35) / 6.7 * width, (6.5 - point[1]) / 4.4 * height)

    def rectangle(a, b, color):
        p, q = pixel(a), pixel(b)
        draw.rectangle(
            [min(p[0], q[0]), min(p[1], q[1]), max(p[0], q[0]), max(p[1], q[1])], fill=color
        )

    def line(name, a, b, color=IVORY, width=0.012):
        rgb = tuple(round(float(c) * 255) for c in color.split()[:3])
        draw.line([pixel(a), pixel(b)], fill=rgb, width=max(1, round(width / 6.7 * 2048)))

    rectangle((-2.85, 2.47), (2.85, 6.13), (45, 119, 88))
    for i in range(10):
        rectangle(
            (-2.6 + i * 0.52, 2.7),
            (-2.6 + (i + 1) * 0.52, 5.9),
            (45, 119, 88) if i % 2 else (51, 130, 97),
        )
    # A faint deterministic grain gives the turf a material finish without new geometry.
    pixels = np.asarray(surface).astype(np.int16)
    noise = (
        np.random.default_rng(7)
        .integers(-2, 3, (168, 256), dtype=np.int16)
        .repeat(8, 0)
        .repeat(8, 1)
    )
    surface = Image.fromarray(np.clip(pixels + noise[:, :, None], 0, 255).astype(np.uint8))
    draw = ImageDraw.Draw(surface)
    for a, b in [
        ((-2.6, 2.7), (2.6, 2.7)),
        ((-2.6, 5.9), (2.6, 5.9)),
        ((-2.6, 2.7), (-2.6, 5.9)),
        ((2.6, 2.7), (2.6, 5.9)),
        ((0, 2.7), (0, 5.9)),
    ]:
        line("", a, b)
    center = pixel((0, 4.3))
    rx = 0.215 / 6.7 * 2048
    ry = 0.215 / 4.4 * 1344
    draw.ellipse(
        [center[0] - rx, center[1] - ry, center[0] + rx, center[1] + ry],
        outline=(237, 242, 224),
        width=4,
    )

    def spot(x, y, r):
        c = pixel((x, y))
        rx = r * 0.5 / 6.7 * 2048
        ry = r * 0.5 / 4.4 * 1344
        draw.ellipse([c[0] - rx, c[1] - ry, c[0] + rx, c[1] + ry], fill=(237, 242, 224))

    spot(0, 4.3, 0.025)
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset, "texture", name="football_surface", type="2d", file="../football/surface.png"
    )
    ET.SubElement(
        asset,
        "material",
        name="football_surface",
        texture="football_surface",
        texrepeat="1 1",
        texuniform="false",
        specular="0.05",
    )
    # Level with the apartment floor; the boxes meet at y=2.1 with no step.
    geom(
        "football_floor",
        "box",
        [3.35, 2.2, 0.05],
        [0, 4.3, -0.05],
        "1 1 1 1",
        material="football_surface",
    )
    for sign, team, color in [(-1, "blue", BLUE), (1, "coral", CORAL)]:
        x, y = sign * 1.3, 4.3
        for k, a, b in [
            ("front", (sign * 1.98, 3.43), (sign * 1.98, 5.17)),
            ("low", (sign * 2.6, 3.43), (sign * 1.98, 3.43)),
            ("high", (sign * 2.6, 5.17), (sign * 1.98, 5.17)),
        ]:
            line(f"football_penalty_{team}_{k}", a, b)
        spot(sign * 1.55, y, 0.02)
        # Site size declares clear half-width, clear height, and line half-width.
        # Its local +X points from the field into the goal.
        ET.SubElement(
            world,
            "site",
            name="football_goal_" + team,
            type="box",
            pos=values([sign * 1.306, y, 0]),
            size=".5 .48 .006",
            quat="0 0 0 1" if sign < 0 else "1 0 0 0",
            rgba="0 0 0 0",
            group="5",
        )
        for side in (-1, 1):
            geom(
                f"football_{team}_post_{side}",
                "capsule",
                [0.025],
                color=IVORY,
                fromto=values([x, y + side * 0.525, 0.025, x, y + side * 0.525, 0.505]),
            )
        geom(
            f"football_{team}_crossbar",
            "capsule",
            [0.025],
            color=IVORY,
            fromto=values([x, y - 0.525, 0.505, x, y + 0.525, 0.505]),
        )
        back = x + sign * 0.4
        # Fixed woven cords approximate an anchored net. No invisible collision box.
        segments = []
        for i in range(22):
            v = y - 0.525 + i * 0.05
            segments.append(([back, v, 0.005], [back, v, 0.505]))
            segments.append(([x, v, 0.505], [back, v, 0.505]))
        for i in range(11):
            z = 0.005 + i * 0.05
            segments.append(([back, y - 0.525, z], [back, y + 0.525, z]))
            for side in (-1, 1):
                segments.append(([x, y + side * 0.525, z], [back, y + side * 0.525, z]))
        for side in (-1, 1):
            for i in range(1, 8):
                v = x + sign * i * 0.05
                segments.append(([v, y + side * 0.525, 0.005], [v, y + side * 0.525, 0.505]))
        for i, (a, b) in enumerate(segments):
            geom(
                f"football_{team}_net_{i}",
                "capsule",
                [0.0025],
                color="0.73 0.81 0.79 1",
                fromto=values([*a, *b]),
                friction=".4 .005 .0001",
                solref=".03 1",
            )
        rectangle(
            (sign * 3.12 - 0.08, 2.65),
            (sign * 3.12 + 0.08, 5.95),
            (51, 135, 191) if sign < 0 else (224, 94, 69),
        )
    surface.save(ROOT / "assets/scenes/football/surface.png", optimize=True)
    # Central player entrance is x=-.6..+.6, y=2.05.
    for side in (-1, 1):
        geom(
            f"football_south_wing_{side}",
            "box",
            [1.325, 0.05, 0.42],
            [side * 1.975, 2.05, 0.42],
            "0.79 0.84 0.81 1",
        )
        geom(
            f"football_side_wall_{side}",
            "box",
            [0.05, 2.2, 0.25],
            [side * 3.3, 4.3, 0.25],
            "0.79 0.84 0.81 1",
        )
        geom(
            f"football_side_rail_{side}",
            "box",
            [0.052, 2.2, 0.018],
            [side * 3.3, 4.3, 0.518],
            "0.17 0.26 0.26 1",
            True,
        )
    geom("football_back_wall", "box", [3.35, 0.05, 0.66], [0, 6.55, 0.66], "0.79 0.84 0.81 1")
    geom(
        "football_back_wainscot",
        "box",
        [3.25, 0.006, 0.18],
        [0, 6.494, 0.18],
        "0.19 0.29 0.29 1",
        True,
    )
    geom(
        "football_back_coping",
        "box",
        [3.35, 0.06, 0.025],
        [0, 6.55, 1.345],
        "0.17 0.26 0.26 1",
        True,
    )
    for i in range(13):
        geom(
            f"football_wall_rib_{i}",
            "box",
            [0.015, 0.014, 0.45],
            [-3 + i * 0.5, 6.485, 0.86],
            "0.72 0.78 0.75 1",
            True,
        )
    for sign, color in [(-1, BLUE), (1, CORAL)]:
        geom(f"football_bench_{sign}", "box", [0.55, 0.12, 0.025], [sign * 2.0, 6.26, 0.18], color)
        for dx in (-0.43, 0.43):
            geom(
                f"football_bench_leg_{sign}_{dx}",
                "box",
                [0.025, 0.09, 0.0775],
                [sign * 2.0 + dx, 6.26, 0.0775],
                "0.24 0.3 0.3 1",
            )
    # A physical board with seven-segment lamps; all native/client views share lamp IDs.
    geom("football_scoreboard_case", "box", [0.89, 0.045, 0.27], [0, 6.45, 0.96], "0.1 0.16 0.17 1")
    geom(
        "football_scoreboard_face",
        "box",
        [0.85, 0.006, 0.235],
        [0, 6.398, 0.96],
        "0.025 0.04 0.045 1",
        True,
    )
    for team, center, color in [("blue", -0.44, BLUE), ("coral", 0.44, CORAL)]:
        geom(
            f"football_scoreboard_{team}_stripe",
            "box",
            [0.33, 0.003, 0.012],
            [center, 6.389, 1.155],
            color,
            True,
        )
        for digit in range(3):
            x, z = center + (digit - 1) * 0.17, 0.96
            for segment, dx, dz, sx, sz in [
                ("a", 0, 0.125, 0.053, 0.009),
                ("b", 0.062, 0.0625, 0.009, 0.048),
                ("c", 0.062, -0.0625, 0.009, 0.048),
                ("d", 0, -0.125, 0.053, 0.009),
                ("e", -0.062, -0.0625, 0.009, 0.048),
                ("f", -0.062, 0.0625, 0.009, 0.048),
                ("g", 0, 0, 0.053, 0.009),
            ]:
                geom(
                    f"score_{team}_{digit}_{segment}",
                    "box",
                    [sx, 0.003, sz],
                    [x + dx, 6.389, z + dz],
                    color,
                    True,
                )
        for axis in (0, 1):
            geom(
                f"score_{team}_overflow_{axis}",
                "box",
                [0.021 if axis == 0 else 0.005, 0.003, 0.005 if axis == 0 else 0.021],
                [center + 0.31, 6.389, 0.81],
                color,
                True,
            )
    for z in (0.92, 1.0):
        geom(
            f"football_scoreboard_separator_{z}",
            "box",
            [0.011, 0.003, 0.011],
            [0, 6.389, z],
            IVORY,
            True,
        )
    for i, (x, y) in enumerate([(-0.7, 3.9), (0.75, 4.65), (0, 5.45)], 1):
        ET.SubElement(
            world,
            "site",
            name=f"football_spawn_{i}",
            pos=values([x * 0.5, 4.3 + (y - 4.3) * 0.5, BALL_SPAWN_HEIGHT]),
            size=values([BALL_RADIUS]),
            rgba="0 0 0 0",
            group="5",
        )
    ET.indent(root)
    (ROOT / "assets/scenes/football/field.xml").write_text(
        ET.tostring(root, encoding="unicode") + "\n"
    )


if __name__ == "__main__":
    build()
