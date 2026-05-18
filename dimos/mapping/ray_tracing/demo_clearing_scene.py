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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Synthetic test scene for the ray-tracing voxel module — Rerun preview.

Scene layout (all in world frame, meters):

    Sensor at (0, 0, 1.0), looking down +x.

    Floor   : z = 0,  x ∈ [0.5, 8],  y ∈ [-3, 3]
    Wall    : x = 6,  y ∈ [-3, 3],   z ∈ [0, 2.5]
    Box     : axis-aligned, x ∈ [4.0, 4.5], y ∈ [0.3, 1.1], z ∈ [0, 0.5]
              — a static obstacle on the floor, sitting between the person
              and the back wall. The person walks past it and partially
              occludes it for several frames; the ray tracer must NOT
              erase it during occlusion.
    Person  : a thin vertical "wall of points" at x = 3,
              y ∈ [person_y ± 0.3],  z ∈ [0, 1.8]

Per frame:
    * The person walks across the field of view in the +y direction.
    * The "lidar return" is the union of all voxel-grid surface points
      that are not occluded by a closer surface from the sensor.

This is a no-dimos sanity check — once the input geometry looks right
in Rerun, the same generator will be wrapped to feed PointCloud2 +
Odometry into RayTracingVoxelMap.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from dataclasses import dataclass
import time

import numpy as np
import rerun as rr

VOXEL_SIZE = 0.1  # meters per voxel edge

SENSOR_ORIGIN = np.array([0.0, 0.0, 1.0], dtype=np.float32)

WALL_X = 6.0
WALL_Y = (-3.0, 3.0)
WALL_Z = (0.0, 2.5)

# Floor stops at the wall — we never observe floor behind a wall from a
# sensor in front of it, so simulating it would just create ghost lidar
# returns past the wall.
FLOOR_X = (0.5, WALL_X)
FLOOR_Y = (-3.0, 3.0)
FLOOR_Z = 0.0

BOX_X = (4.0, 4.5)
BOX_Y = (0.3, 1.1)
BOX_Z = (0.0, 0.5)

PERSON_X = 3.0
PERSON_HALF_WIDTH = 0.3
PERSON_HEIGHT = 1.8
PERSON_Y_START = -2.0
PERSON_Y_END = 2.0


@dataclass
class Frame:
    index: int
    timestamp_s: float
    sensor_origin: np.ndarray  # (3,) float32
    points: np.ndarray  # (N, 3) float32, world-frame
    person_y: float | None  # None on frame 0 (no person yet)


def _grid_axis(lo: float, hi: float, step: float) -> np.ndarray:
    """Voxel-center positions covering [lo, hi)."""
    return np.arange(lo, hi, step, dtype=np.float32) + np.float32(step / 2)


def _floor_points() -> np.ndarray:
    xs = _grid_axis(FLOOR_X[0], FLOOR_X[1], VOXEL_SIZE)
    ys = _grid_axis(FLOOR_Y[0], FLOOR_Y[1], VOXEL_SIZE)
    grid_x, grid_y = np.meshgrid(xs, ys, indexing="ij")
    z = np.full_like(grid_x, FLOOR_Z)
    return np.stack([grid_x.ravel(), grid_y.ravel(), z.ravel()], axis=-1)


def _wall_points() -> np.ndarray:
    ys = _grid_axis(WALL_Y[0], WALL_Y[1], VOXEL_SIZE)
    zs = _grid_axis(WALL_Z[0], WALL_Z[1], VOXEL_SIZE)
    grid_y, grid_z = np.meshgrid(ys, zs, indexing="ij")
    x = np.full_like(grid_y, WALL_X)
    return np.stack([x.ravel(), grid_y.ravel(), grid_z.ravel()], axis=-1)


def _person_points(person_y: float) -> np.ndarray:
    ys = _grid_axis(person_y - PERSON_HALF_WIDTH, person_y + PERSON_HALF_WIDTH, VOXEL_SIZE)
    zs = _grid_axis(0.0, PERSON_HEIGHT, VOXEL_SIZE)
    grid_y, grid_z = np.meshgrid(ys, zs, indexing="ij")
    x = np.full_like(grid_y, PERSON_X)
    return np.stack([x.ravel(), grid_y.ravel(), grid_z.ravel()], axis=-1)


def _box_visible_face_points() -> np.ndarray:
    """Three sensor-facing faces of the box (front, top, near side).

    From a sensor at (0, 0, +z), only the -x face, the +z face, and the
    -y face are visible. The other three faces are hidden behind the box
    itself, so we don't generate them — that way no self-occlusion check
    is needed for the box.
    """
    # Front face: x = BOX_X[0], spans y × z
    ys = _grid_axis(BOX_Y[0], BOX_Y[1], VOXEL_SIZE)
    zs = _grid_axis(BOX_Z[0], BOX_Z[1], VOXEL_SIZE)
    gy, gz = np.meshgrid(ys, zs, indexing="ij")
    front = np.stack([np.full_like(gy, BOX_X[0]).ravel(), gy.ravel(), gz.ravel()], axis=-1)

    # Top face: z = BOX_Z[1], spans x × y
    xs = _grid_axis(BOX_X[0], BOX_X[1], VOXEL_SIZE)
    ys = _grid_axis(BOX_Y[0], BOX_Y[1], VOXEL_SIZE)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    top = np.stack([gx.ravel(), gy.ravel(), np.full_like(gx, BOX_Z[1]).ravel()], axis=-1)

    # Near side: y = BOX_Y[0], spans x × z
    xs = _grid_axis(BOX_X[0], BOX_X[1], VOXEL_SIZE)
    zs = _grid_axis(BOX_Z[0], BOX_Z[1], VOXEL_SIZE)
    gx, gz = np.meshgrid(xs, zs, indexing="ij")
    near = np.stack([gx.ravel(), np.full_like(gx, BOX_Y[0]).ravel(), gz.ravel()], axis=-1)

    return np.concatenate([front, top, near], axis=0)


def _occluded_by_box(origin: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """Boolean mask, True for targets whose ray from `origin` is blocked
    by the box AABB. Vectorized AABB slab test.

    Tolerance bands at t≈0 and t≈1 mean points sitting exactly on the box
    surface (or behind it past the back face) aren't flagged as
    self-occluding.
    """
    deltas = targets - origin  # (N, 3)
    box_min = np.array([BOX_X[0], BOX_Y[0], BOX_Z[0]], dtype=np.float32)
    box_max = np.array([BOX_X[1], BOX_Y[1], BOX_Z[1]], dtype=np.float32)

    eps = 1e-9
    safe_d = np.where(np.abs(deltas) < eps, eps, deltas)
    t1 = (box_min - origin) / safe_d
    t2 = (box_max - origin) / safe_d
    t_min = np.minimum(t1, t2)
    t_max = np.maximum(t1, t2)
    t_enter = t_min.max(axis=1)
    t_exit = t_max.min(axis=1)

    hits_box = t_enter <= t_exit
    # The box is in front of the target if it enters at t < 1, after the
    # origin (t > 0). A 1e-4 margin keeps box-surface points themselves
    # from being flagged as occluded.
    return hits_box & (t_enter > 1e-4) & (t_enter < 1.0 - 1e-4)  # type: ignore[no-any-return]


def _occluded_by_person(origin: np.ndarray, targets: np.ndarray, person_y: float) -> np.ndarray:
    """Boolean mask, True for targets whose ray from `origin` is blocked
    by the person standing at `person_y`. Vectorized over targets.

    A target is occluded if the ray origin→target crosses the person's
    front plane (x = PERSON_X) at a (y, z) point inside the person's
    rectangle, AND the crossing happens before the target itself.
    """
    deltas = targets - origin  # (N, 3)
    dx = deltas[:, 0]
    # Rays moving in -x or staying still can't be blocked by a +x plane.
    forward = dx > 0
    safe_dx = np.where(forward, dx, 1.0)
    t_p = (PERSON_X - origin[0]) / safe_dx  # parametric distance to person plane
    crosses_in_front = forward & (t_p > 0.0) & (t_p < 1.0)
    y_at = origin[1] + t_p * deltas[:, 1]
    z_at = origin[2] + t_p * deltas[:, 2]
    inside_person = (
        (np.abs(y_at - person_y) < PERSON_HALF_WIDTH) & (z_at >= 0.0) & (z_at < PERSON_HEIGHT)
    )
    return crosses_in_front & inside_person  # type: ignore[no-any-return]


def _visible_points(person_y: float | None) -> np.ndarray:
    """Lidar return from SENSOR_ORIGIN: floor + wall + box + (optional)
    person. Floor / wall / box points are dropped if a closer surface
    (the box itself, for floor/wall; or the person, for any of them)
    blocks their ray from the sensor.
    """
    floor = _floor_points()
    wall = _wall_points()
    box = _box_visible_face_points()

    box_occ_floor = _occluded_by_box(SENSOR_ORIGIN, floor)
    box_occ_wall = _occluded_by_box(SENSOR_ORIGIN, wall)

    if person_y is None:
        return np.concatenate(  # type: ignore[no-any-return]
            [floor[~box_occ_floor], wall[~box_occ_wall], box], axis=0
        ).astype(np.float32)

    person_occ_floor = _occluded_by_person(SENSOR_ORIGIN, floor, person_y)
    person_occ_wall = _occluded_by_person(SENSOR_ORIGIN, wall, person_y)
    person_occ_box = _occluded_by_person(SENSOR_ORIGIN, box, person_y)

    person = _person_points(person_y)
    return np.concatenate(  # type: ignore[no-any-return]
        [
            floor[~(box_occ_floor | person_occ_floor)],
            wall[~(box_occ_wall | person_occ_wall)],
            box[~person_occ_box],
            person,
        ],
        axis=0,
    ).astype(np.float32)


def synthetic_scene(num_frames: int = 60, frame_dt: float = 0.1) -> Iterator[Frame]:
    """Yield frames one at a time.

    Frame 0: empty scene (floor + back wall only, no person).
    Frames 1..num_frames-1: person walks from PERSON_Y_START to PERSON_Y_END.
    """
    yield Frame(
        index=0,
        timestamp_s=0.0,
        sensor_origin=SENSOR_ORIGIN.copy(),
        points=_visible_points(person_y=None),
        person_y=None,
    )

    if num_frames < 2:
        return
    walking_frames = num_frames - 1
    for i in range(walking_frames):
        t = i / max(walking_frames - 1, 1)
        person_y = PERSON_Y_START + t * (PERSON_Y_END - PERSON_Y_START)
        frame_idx = i + 1
        yield Frame(
            index=frame_idx,
            timestamp_s=frame_idx * frame_dt,
            sensor_origin=SENSOR_ORIGIN.copy(),
            points=_visible_points(person_y=person_y),
            person_y=person_y,
        )


def _classify_points(points: np.ndarray, person_y: float | None) -> np.ndarray:
    """Per-point class id: 0=floor, 1=wall, 2=person, 3=box. Coloring only.

    Classification by which surface generated the point — floor/wall both
    have voxels at z=0.05 (lowest row) so we can't disambiguate by z alone.
    """
    is_wall = np.abs(points[:, 0] - WALL_X) < 1e-3
    is_floor = np.abs(points[:, 2] - FLOOR_Z) < 1e-3
    in_box = (
        (points[:, 0] >= BOX_X[0] - 1e-3)
        & (points[:, 0] <= BOX_X[1] + 1e-3)
        & (points[:, 1] >= BOX_Y[0] - 1e-3)
        & (points[:, 1] <= BOX_Y[1] + 1e-3)
        & (points[:, 2] >= BOX_Z[0] - 1e-3)
        & (points[:, 2] <= BOX_Z[1] + 1e-3)
    )

    classes = np.empty(len(points), dtype=np.uint8)
    classes[:] = 1  # default to wall
    classes[is_floor & ~is_wall] = 0  # floor (but a wall point at z=0.05 stays wall)
    classes[in_box] = 3  # box overrides

    if person_y is not None:
        is_person = (np.abs(points[:, 0] - PERSON_X) < 1e-3) & (
            np.abs(points[:, 1] - person_y) < PERSON_HALF_WIDTH + 1e-3
        )
        classes[is_person] = 2  # person overrides everything else

    return classes


CLASS_COLORS = np.array(
    [
        [120, 120, 120],  # floor — gray
        [80, 160, 255],  # wall  — blue
        [255, 80, 80],  # person — red
        [255, 180, 60],  # box   — orange
    ],
    dtype=np.uint8,
)


def log_to_rerun(num_frames: int, frame_dt: float, realtime: bool = True) -> None:
    rr.log(
        "world/sensor",
        rr.Points3D(
            positions=SENSOR_ORIGIN.reshape(1, 3),
            colors=np.array([[0, 255, 0]], dtype=np.uint8),
            radii=0.08,
            labels=["sensor"],
        ),
        static=True,
    )
    # Show the un-occluded reference scene as a faint backdrop so the user
    # can see what the back-wall "should" look like behind the person.
    reference_floor = _floor_points().astype(np.float32)
    reference_wall = _wall_points().astype(np.float32)
    rr.log(
        "world/reference/floor",
        rr.Points3D(
            positions=reference_floor,
            colors=np.tile(np.array([[60, 60, 60]], dtype=np.uint8), (len(reference_floor), 1)),
            radii=VOXEL_SIZE / 2 * 0.6,
        ),
        static=True,
    )
    rr.log(
        "world/reference/wall",
        rr.Points3D(
            positions=reference_wall,
            colors=np.tile(np.array([[40, 60, 90]], dtype=np.uint8), (len(reference_wall), 1)),
            radii=VOXEL_SIZE / 2 * 0.6,
        ),
        static=True,
    )
    reference_box = _box_visible_face_points().astype(np.float32)
    rr.log(
        "world/reference/box",
        rr.Points3D(
            positions=reference_box,
            colors=np.tile(np.array([[90, 70, 40]], dtype=np.uint8), (len(reference_box), 1)),
            radii=VOXEL_SIZE / 2 * 0.6,
        ),
        static=True,
    )

    for frame in synthetic_scene(num_frames=num_frames, frame_dt=frame_dt):
        # Single timeline in seconds — viewer plays it at 1× wall-clock by
        # default, so 60 frames @ dt=0.1s plays as a 6-second video.
        rr.set_time("time", duration=frame.timestamp_s)

        classes = _classify_points(frame.points, frame.person_y)
        colors = CLASS_COLORS[classes]

        rr.log(
            "world/lidar_return",
            rr.Points3D(
                positions=frame.points,
                colors=colors,
                radii=VOXEL_SIZE / 2,
            ),
        )

        # Visualize a few sample rays from the sensor toward the back wall
        # to make occlusion easy to read at a glance.
        sample_ys = np.linspace(WALL_Y[0] + 0.2, WALL_Y[1] - 0.2, 9, dtype=np.float32)
        sample_targets = np.stack(
            [
                np.full_like(sample_ys, WALL_X),
                sample_ys,
                np.full_like(sample_ys, 1.2),
            ],
            axis=-1,
        )
        ray_origins = np.tile(SENSOR_ORIGIN, (len(sample_targets), 1))
        ray_strips = np.stack([ray_origins, sample_targets], axis=1)  # (R, 2, 3)
        rr.log(
            "world/sample_rays",
            rr.LineStrips3D(
                strips=list(ray_strips),
                colors=np.tile(
                    np.array([[200, 200, 80, 60]], dtype=np.uint8), (len(ray_strips), 1)
                ),
                radii=0.005,
            ),
        )

        # Stream live: sleep between frames so the viewer renders them
        # in order at real-time pace. Skipped for offline .rrd captures.
        if realtime:
            time.sleep(frame_dt)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--frames", type=int, default=60, help="total frames (incl. frame 0)")
    parser.add_argument("--dt", type=float, default=0.1, help="seconds per frame")
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="if set, save to this .rrd path instead of spawning the viewer",
    )
    args = parser.parse_args()

    if args.save:
        rr.init("ray_tracing_clearing_scene", spawn=False)
        rr.save(args.save)
        log_to_rerun(num_frames=args.frames, frame_dt=args.dt, realtime=False)
    else:
        rr.init("ray_tracing_clearing_scene", spawn=True)
        # Give the spawned viewer a moment to connect before we start
        # streaming, otherwise the first few frames can be missed.
        time.sleep(1.0)
        log_to_rerun(num_frames=args.frames, frame_dt=args.dt, realtime=True)


if __name__ == "__main__":
    main()
