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

"""A probe's sample room seen from above one corner, as MuJoCo renders it."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys

import cv2
import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.navigation.bench.suite import Case, Manifest, Point
from dimos.simulation.go2_legged.robot import BASE_QUAT
from dimos.simulation.go2_sim.world import build_model
from dimos.simulation.scenes.mjcf import geom_name
from dimos.simulation.scenes.procedural import WALL_THICKNESS, Box, Rect, Scene

SNAPSHOT_PX = (600, 480)
ELEVATION_DEG = 55.0
FOVY_DEG = 45.0
CORNERS_DEG = (45.0, 135.0, 225.0, 315.0)
FRAME_MARGIN = 0.94
CUT_HEIGHT_M = 0.08
FADED_ALPHA = 0.3
GOAL_RADIUS_M = 0.2
DISC_HEIGHT_M = 0.01
PIN_HEIGHT_M = 1.0
PIN_RADIUS_M = 0.025
PIN_HEAD_M = 0.08
GOAL_RGBA = (0.1, 0.8, 0.2, 1.0)
FEATURE_RGBA = (0.95, 0.15, 0.1, 1.0)
SUN = (0.45, 0.3, -1.0)
HEADLIGHT = 0.2
FIT_PASSES = 3
BACKGROUND = 255
EPS = 1e-6


@dataclass(frozen=True)
class View:
    """A camera above one corner of the room, the near walls it cuts low and the boxes it fades."""

    azimuth: float
    distance: float
    lookat: Point
    cut: tuple[int, ...]
    faded: tuple[int, ...]

    @property
    def forward(self) -> NDArray[np.float64]:
        azimuth, elevation = math.radians(self.azimuth), math.radians(-ELEVATION_DEG)
        return np.array(
            [
                math.cos(elevation) * math.cos(azimuth),
                math.cos(elevation) * math.sin(azimuth),
                math.sin(elevation),
            ]
        )

    @property
    def position(self) -> NDArray[np.float64]:
        return np.array(self.lookat) - self.distance * self.forward


def sample(cases: list[Case]) -> Case | None:
    """The lowest seed's case at the middle level of the swept variable, if any is swept."""
    swept = [(c.level.value, c) for c in cases if c.level is not None and not c.level.control]
    if not swept:
        return None
    values = sorted({value for value, _ in swept})
    middle = values[len(values) // 2]
    return min((c for value, c in swept if value == middle), key=lambda c: c.seed)


def snapshot_path(run_dir: Path, case: Case) -> Path:
    return run_dir / f"snapshot-{case.id}.png"


def bounds(scene: Scene) -> Rect:
    """The rectangle of ground the scene covers, edge to edge."""
    lo, hi = scene.bounds()
    return float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])


def key_points(case: Case, scene: Scene) -> list[Point]:
    """The start, the goal and the feature on the floor, which the picture has to show."""
    z = scene.start[2]
    feature = [] if case.feature is None else [(*case.feature, z)]
    return [(*case.start[:2], z), (*case.goal[:2], z), *feature]


def view(case: Case, scene: Scene) -> View:
    """The corner that hides the fewest key points, preferring to look from the start toward the goal."""
    toward = np.subtract(case.goal[:2], case.start[:2])
    views = [_corner(case, scene, azimuth) for azimuth in CORNERS_DEG]
    return min(views, key=lambda v: (len(v.faded), -float(v.forward[:2] @ toward)))


def _corner(case: Case, scene: Scene, azimuth: float) -> View:
    x0, y0, x1, y1 = bounds(scene)
    top = float(scene.bounds()[1][2])
    floor = scene.start[2]
    look = View(azimuth, 0.0, ((x0 + x1) / 2, (y0 + y1) / 2, floor), (), ())
    near = (x0 if look.forward[0] > 0 else x1, y0 if look.forward[1] > 0 else y1)
    cut = tuple(
        i
        for i, box in enumerate(scene.boxes)
        if box.kind == "wall"
        and any(
            abs(box.center[axis] - near[axis]) + box.half[axis] <= WALL_THICKNESS + EPS
            for axis in (0, 1)
        )
    )
    corners = [(x, y, floor) for x in (x0, x1) for y in (y0, y1)]
    corners += [(x, y, top) for x, y, _ in corners if (x, y) != near]
    forward = look.forward
    right = np.array([forward[1], -forward[0], 0.0]) / math.hypot(forward[0], forward[1])
    up = np.cross(right, forward)
    reach = math.tan(math.radians(FOVY_DEG) / 2) * FRAME_MARGIN
    aspect = SNAPSHOT_PX[0] / SNAPSHOT_PX[1]
    center, distance = np.array(look.lookat), 0.0
    for _ in range(FIT_PASSES):
        offsets = np.array(corners) - center
        across, upward, depth = offsets @ right, offsets @ up, offsets @ forward
        fit = np.maximum(np.abs(upward), np.abs(across) / aspect) / reach - depth
        distance = float(fit.max())
        across, upward = across / (depth + distance), upward / (depth + distance)
        center += distance * (
            (across.max() + across.min()) / 2 * right + (upward.max() + upward.min()) / 2 * up
        )
    lookat = (float(center[0]), float(center[1]), float(center[2]))
    placed = View(azimuth, distance, lookat, cut, ())
    faded = tuple(
        i
        for i, box in enumerate(scene.boxes)
        if box.kind != "floor"
        and i not in cut
        and any(_crosses(placed.position, np.array(p), box) for p in key_points(case, scene))
    )
    return View(azimuth, distance, lookat, cut, faded)


def _crosses(eye: NDArray[np.float64], point: NDArray[np.float64], box: Box) -> bool:
    """Whether the sight line from the eye to the point passes through the box."""
    ray = point - eye
    with np.errstate(divide="ignore", invalid="ignore"):
        a = (np.subtract(box.center, box.half) - eye) / ray
        b = (np.add(box.center, box.half) - eye) / ray
    enter, leave = np.fmin(a, b).max(), np.fmax(a, b).min()
    return bool(enter < leave and leave > 0.0 and enter < 1.0 - EPS)


def _mark(
    drawn: mujoco.MjvScene, shape: int, size: Point, position: Point, rgba: tuple[float, ...]
) -> None:
    """A shape the picture shows and the model never holds."""
    mujoco.mjv_initGeom(
        drawn.geoms[drawn.ngeom],
        shape,
        np.array(size),
        np.array(position),
        np.eye(3).ravel(),
        np.array(rgba, dtype=np.float32),
    )
    drawn.ngeom += 1


def _pin(drawn: mujoco.MjvScene, x: float, y: float, z: float, rgba: tuple[float, ...]) -> None:
    half = PIN_HEIGHT_M / 2
    cylinder, sphere = mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE
    _mark(drawn, cylinder, (PIN_RADIUS_M, half, 0.0), (x, y, z + half), rgba)
    _mark(drawn, sphere, (PIN_HEAD_M,) * 3, (x, y, z + PIN_HEIGHT_M), rgba)


def render(case: Case) -> NDArray[np.uint8]:
    """The case's scene with the robot at its start, a green disc at the goal and a red pin on the feature."""
    scene = case.scene()
    seen = view(case, scene)
    floor = scene.start[2]
    model = build_model(scene)
    model.vis.global_.offwidth, model.vis.global_.offheight = SNAPSHOT_PX
    model.vis.global_.fovy = FOVY_DEG
    model.vis.headlight.diffuse[:] = HEADLIGHT
    model.light_dir[0] = np.array(SUN) / np.linalg.norm(SUN)
    for i in seen.cut:
        geom = model.geom(geom_name(i, scene.boxes[i]))
        geom.size[2], geom.pos[2] = CUT_HEIGHT_M / 2, floor + CUT_HEIGHT_M / 2
    for i in seen.faded:
        model.geom(geom_name(i, scene.boxes[i])).rgba[3] = FADED_ALPHA
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    x, y, yaw = case.start
    data.qpos[0:3] = [x, y, data.qpos[2] + floor]
    data.qpos[BASE_QUAT] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = seen.lookat
    camera.distance, camera.azimuth, camera.elevation = seen.distance, seen.azimuth, -ELEVATION_DEG
    with mujoco.Renderer(model, SNAPSHOT_PX[1], SNAPSHOT_PX[0]) as renderer:
        renderer.update_scene(data, camera)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        disc = (GOAL_RADIUS_M, DISC_HEIGHT_M, 0.0)
        on_floor = (*case.goal[:2], floor + DISC_HEIGHT_M)
        _mark(renderer.scene, mujoco.mjtGeom.mjGEOM_CYLINDER, disc, on_floor, GOAL_RGBA)
        _pin(renderer.scene, *case.goal[:2], floor, GOAL_RGBA)
        if case.feature is not None:
            _pin(renderer.scene, *case.feature, floor, FEATURE_RGBA)
        image: NDArray[np.uint8] = renderer.render()
        renderer.enable_segmentation_rendering()
        image[renderer.render()[:, :, 1] < 0] = BACKGROUND
    return image


if __name__ == "__main__":
    suite, case_id, out = sys.argv[1:]
    picture = render(Manifest.load(Path(suite)).case(case_id))
    cv2.imwrite(out, cv2.cvtColor(picture, cv2.COLOR_RGB2BGR))
