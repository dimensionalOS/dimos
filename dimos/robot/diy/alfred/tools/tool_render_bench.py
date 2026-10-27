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

"""Render one frame of alfred-manipulation-sim's bench from the mast camera, and check it.

    python -m dimos.robot.diy.alfred.tools.tool_render_bench --out /tmp/bench
    python -m dimos.robot.diy.alfred.tools.tool_render_bench --base 0.12 0 0   # driven 12 cm in

Run it from the checkout's root; it needs only ``mujoco`` and ``numpy`` (the dimos venv, or a
throwaway one). It renders ``camera_front`` at the D455's 848x480 exactly as MujocoSimModule
does, writes ``color.png`` and ``depth.png`` (16-bit millimetres, the D455's encoding), and
checks every object on the bench: seen, in its own colour, at a depth the cloud keeps (under
3 m) and where the scene puts it. It exits 1 when a check fails.

MuJoCo takes its GL backend from ``MUJOCO_GL`` (see ``dimos.simulation.mujoco.headless_gl``):
nothing to set on macOS; on Linux with no display it defaults to ``egl`` here, and
``MUJOCO_GL=osmesa`` renders in software where EGL is missing.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import struct
import sys
import zlib

from dimos.simulation.mujoco.headless_gl import default_mujoco_gl

default_mujoco_gl()

import mujoco
import numpy as np
from numpy.typing import NDArray

SCENE = Path(__file__).resolve().parents[1] / "blueprints" / "alfred_bench_scene.xml"
CAMERA = "camera_front"
BASE_BODY = "base_link"
WIDTH, HEIGHT = 848, 480
# The bench stack drops cloud points further than this (DEPTH_TRUNC_M), and nothing on the
# bench is nearer than the camera's own mount.
DEPTH_RANGE_M = (0.3, 3.0)
MIN_PIXELS = 150
# How far an object's median depth may sit in front of its centre: its own front half.
DEPTH_TOLERANCE_M = 0.1
COLOUR_COSINE = 0.95
BENCH_OBJECTS = ("soda_can", "cardboard_box", "cucumber", "tennis_ball", "water_bottle")


@dataclass(frozen=True)
class Seen:
    name: str
    pixels: int
    colour: tuple[float, float, float]
    expected_colour: tuple[float, float, float]
    depth_m: float
    expected_depth_m: float

    @property
    def colour_cosine(self) -> float:
        seen, wanted = np.asarray(self.colour), np.asarray(self.expected_colour)
        return float(seen @ wanted / (np.linalg.norm(seen) * np.linalg.norm(wanted) + 1e-12))

    def problems(self) -> list[str]:
        if self.pixels < MIN_PIXELS:
            return [f"{self.name}: {self.pixels} pixels, not in view"]
        found = []
        same_hue = np.argmax(self.colour) == np.argmax(self.expected_colour)
        if self.colour_cosine < COLOUR_COSINE or not same_hue:
            found.append(f"{self.name}: colour {self.colour} is not {self.expected_colour}")
        low, high = DEPTH_RANGE_M
        if not low <= self.depth_m <= high:
            found.append(f"{self.name}: depth {self.depth_m:.3f} m outside {DEPTH_RANGE_M}")
        if not 0.0 <= self.expected_depth_m - self.depth_m <= DEPTH_TOLERANCE_M:
            found.append(
                f"{self.name}: depth {self.depth_m:.3f} m, its centre is at "
                f"{self.expected_depth_m:.3f} m"
            )
        return found


@dataclass(frozen=True)
class Frame:
    rgb: NDArray[np.uint8]
    depth: NDArray[np.float32]
    seen: list[Seen]
    camera_position: tuple[float, float, float]
    camera_pitch_deg: float


def render(base: tuple[float, float, float] = (0.0, 0.0, 0.0), scene: Path = SCENE) -> Frame:
    """One frame from the mast camera with base_link at ``base`` (x, y, yaw in the world)."""
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mocap = int(model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)])
    x, y, yaw = base
    data.mocap_pos[mocap] = (x, y, 0.0)
    data.mocap_quat[mocap] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    mujoco.mj_forward(model, data)
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=camera)
        depth = renderer.render().astype(np.float32)
        renderer.disable_depth_rendering()  # type: ignore[attr-defined]
        renderer.enable_segmentation_rendering()  # type: ignore[attr-defined]
        renderer.update_scene(data, camera=camera)
        segmentation = renderer.render().copy()
    finally:
        renderer.close()

    geoms = np.where(segmentation[..., 1] == mujoco.mjtObj.mjOBJ_GEOM, segmentation[..., 0], -1)
    bodies = np.where(geoms >= 0, model.geom_bodyid[np.clip(geoms, 0, None)], -1)
    cam_position = data.cam_xpos[camera].copy()
    cam_rotation = data.cam_xmat[camera].reshape(3, 3).copy()
    seen = []
    for name in BENCH_OBJECTS:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        main_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        on_body = bodies == body
        on_main = geoms == main_geom
        expected_rgb = model.mat_rgba[model.geom_matid[main_geom], :3]
        mean_rgb = rgb[on_main].mean(axis=0) / 255.0 if on_main.any() else np.zeros(3)
        # MuJoCo cameras look along their -z: depth is the centre's distance along it.
        centre = cam_rotation.T @ (data.geom_xpos[main_geom] - cam_position)
        seen.append(
            Seen(
                name=name,
                pixels=int(on_body.sum()),
                colour=(float(mean_rgb[0]), float(mean_rgb[1]), float(mean_rgb[2])),
                expected_colour=(
                    float(expected_rgb[0]),
                    float(expected_rgb[1]),
                    float(expected_rgb[2]),
                ),
                depth_m=float(np.median(depth[on_body])) if on_body.any() else math.inf,
                expected_depth_m=float(-centre[2]),
            )
        )
    look = -cam_rotation[:, 2]
    return Frame(
        rgb=rgb,
        depth=depth,
        seen=seen,
        camera_position=(
            float(cam_position[0]),
            float(cam_position[1]),
            float(cam_position[2]),
        ),
        camera_pitch_deg=math.degrees(math.asin(-look[2])),
    )


def write_png(path: Path, image: NDArray[np.uint8] | NDArray[np.uint16]) -> None:
    """A PNG with nothing but zlib: 8-bit RGB, or 16-bit grey."""
    height, width = image.shape[:2]
    if image.dtype == np.uint16:
        bit_depth, colour_type, rows = 16, 0, image.astype(">u2").tobytes()
    else:
        bit_depth, colour_type, rows = 8, 2, np.ascontiguousarray(image).tobytes()
    stride = len(rows) // height
    raw = b"".join(b"\x00" + rows[r * stride : (r + 1) * stride] for r in range(height))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))

    header = struct.pack(">IIBBBBB", width, height, bit_depth, colour_type, 0, 0, 0)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--out", type=Path, default=Path("."), help="where the PNGs go")
    parser.add_argument(
        "--base",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "YAW"),
        help="base_link in the world (odom): where the robot has driven to",
    )
    args = parser.parse_args(argv)
    frame = render(tuple(args.base))
    args.out.mkdir(parents=True, exist_ok=True)
    write_png(args.out / "color.png", frame.rgb)
    millimetres = np.where(frame.depth < DEPTH_RANGE_M[1], frame.depth * 1000.0, 0.0)
    write_png(args.out / "depth.png", millimetres.round().astype(np.uint16))

    print(f"MUJOCO_GL={default_mujoco_gl() or '(mujoco default)'}  scene={SCENE.name}")
    x, y, z = frame.camera_position
    print(
        f"camera at ({x:.3f}, {y:.3f}, {z:.3f}) m, {frame.camera_pitch_deg:.1f} deg down, "
        f"{WIDTH}x{HEIGHT}"
    )
    print(f"{'object':<14}{'pixels':>8}  {'colour (rgb)':<20}{'depth':>8}{'centre':>8}")
    problems = []
    for seen in frame.seen:
        r, g, b = seen.colour
        print(
            f"{seen.name:<14}{seen.pixels:>8}  ({r:.2f}, {g:.2f}, {b:.2f})    "
            f"{seen.depth_m:>7.3f}m{seen.expected_depth_m:>7.3f}m"
        )
        problems += seen.problems()
    print(f"wrote {args.out / 'color.png'} and {args.out / 'depth.png'}")
    for problem in problems:
        print(f"FAIL {problem}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
