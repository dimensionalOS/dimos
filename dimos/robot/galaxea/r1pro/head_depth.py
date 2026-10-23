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

"""The R1 Pro's head depth: Depth Anything on the left head camera, calibrated to the chassis lidar, with a floor prior."""

from __future__ import annotations

import numpy as np

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.depth2depth_cloud.module import Depth2DepthCloud
from dimos.robot.galaxea.r1pro.config import R1PRO_CHASSIS_WIDTH_M
from dimos.robot.galaxea.r1pro.lio import R1ProLioConfig

BASE_FRAME = R1ProLioConfig.model_fields["base_frame"].default


class HeadFloorPriorConfig(ModuleConfig):
    # Its origin is where the wheels touch the floor, so the floor is its z = 0 however the torso and head move.
    floor_frame: str = BASE_FRAME
    tf_tolerance_s: float = 0.1
    # A head point no higher than this above the floor (and no lower than min_height_m) is floor.
    floor_tolerance_m: float = 0.05
    # Kept from just under the floor to just above the chassis lidar's plane, what the lidar sees worst; a drop deeper
    # than min_height_m stays unseen rather than becoming floor.
    min_height_m: float = -0.15
    max_height_m: float = 0.35
    # The floor under the robot, which it stands on and no sensor sees, as points this far apart; 0 radius adds none.
    footprint_radius_m: float = 0.0
    footprint_spacing_m: float = 0.05
    # Unseen floor past the footprint, per bearing out to its nearest head point within view_fill_reach_m if that is floor.
    view_fill_reach_m: float = 0.0
    view_fill_bin_deg: float = 5.0


class HeadFloorPrior(Module):
    """Head depth onto the floor the robot stands on (floor_frame's z = 0; the chassis lidar never sees it): a point just
    above or under the floor slides along its ray onto it, and the floor under the robot's own footprint is added. Every
    pose comes from tf at the cloud's stamp, so the torso and base may move."""

    config: HeadFloorPriorConfig

    depth_cloud: In[PointCloud2]
    tf: In[TFMessage]
    head_cloud: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    async def handle_depth_cloud(self, cloud: PointCloud2) -> None:
        floor_from_camera = self._lookup(self.config.floor_frame, cloud.frame_id, cloud.ts)
        if floor_from_camera is None:
            return
        points = onto_floor(cloud.points_f32().astype(np.float64), floor_from_camera, self.config)
        in_floor = _apply(floor_from_camera, points)
        keep = (in_floor[:, 2] >= self.config.min_height_m) & (
            in_floor[:, 2] <= self.config.max_height_m
        )
        points = points[keep]
        if self.config.view_fill_reach_m > 0:
            unseen = view_floor(
                _apply(floor_from_camera, points),
                self.config.footprint_radius_m,
                self.config.view_fill_reach_m,
                self.config.view_fill_bin_deg,
                self.config.footprint_spacing_m,
            )
            points = np.concatenate([points, _apply(np.linalg.inv(floor_from_camera), unseen)])
        if self.config.footprint_radius_m > 0:
            under = footprint_floor(self.config.footprint_radius_m, self.config.footprint_spacing_m)
            points = np.concatenate([points, _apply(np.linalg.inv(floor_from_camera), under)])
        self.head_cloud.publish(
            PointCloud2.from_numpy(
                points.astype(np.float32), frame_id=cloud.frame_id, timestamp=cloud.ts
            )
        )

    def _lookup(self, target: str, source: str, stamp: float) -> np.ndarray | None:
        transform = self.tfbuffer.get(
            target,
            source,
            time_point=stamp,
            time_tolerance=self.config.tf_tolerance_s,
            forward_tolerance=self.config.tf_tolerance_s,
            warn=False,
        )
        return None if transform is None else transform.to_matrix()


def onto_floor(
    points: np.ndarray, floor_from_camera: np.ndarray, config: HeadFloorPriorConfig
) -> np.ndarray:
    """Camera-frame points from min_height_m to floor_tolerance_m about floor_frame's z = 0 slid along their rays onto it."""
    points = points.copy()
    in_floor = _apply(floor_from_camera, points)
    camera_z = floor_from_camera[2, 3]
    low = np.flatnonzero(
        (in_floor[:, 2] < camera_z - 1e-3)
        & (in_floor[:, 2] >= config.min_height_m)
        & (in_floor[:, 2] <= config.floor_tolerance_m)
    )
    points[low] *= (-camera_z / (in_floor[low, 2] - camera_z))[:, None]
    return points


def footprint_floor(radius: float, spacing: float) -> np.ndarray:
    """Floor points in floor_frame on a `spacing` grid over the disc of `radius` around its origin, at z = 0."""
    grid = np.arange(-radius, radius + 1e-9, spacing)
    xx, yy = np.meshgrid(grid, grid)
    inside = np.hypot(xx, yy) <= radius
    points: np.ndarray = np.c_[xx[inside], yy[inside], np.zeros(int(inside.sum()))]
    return points


def view_floor(
    in_floor: np.ndarray, inner: float, reach: float, bin_deg: float, spacing: float
) -> np.ndarray:
    """Floor points in floor_frame on a `spacing` grid from radius `inner` out to, per `bin_deg` bearing, the nearest of
    `in_floor` (floor_frame points) within `reach`, on the bearings where that nearest point is on the floor."""
    bins = int(np.ceil(360.0 / bin_deg))
    radius = np.hypot(in_floor[:, 0], in_floor[:, 1])
    near = (radius > inner) & (radius < reach)
    bearing = _bin(in_floor[near, 0], in_floor[near, 1], bin_deg)
    order = np.lexsort((radius[near], bearing))
    first = order[np.unique(bearing[order], return_index=True)[1]]
    nearest = np.zeros(bins)
    floor_bearings = first[np.abs(in_floor[near][first, 2]) < 1e-3]
    nearest[bearing[floor_bearings]] = radius[near][floor_bearings]
    grid = np.arange(-reach, reach + 1e-9, spacing)
    xx, yy = np.meshgrid(grid, grid)
    xx, yy = xx.ravel(), yy.ravel()
    cell_radius = np.hypot(xx, yy)
    fill = (cell_radius > inner) & (cell_radius < nearest[_bin(xx, yy, bin_deg)])
    points: np.ndarray = np.c_[xx[fill], yy[fill], np.zeros(int(fill.sum()))]
    return points


def _bin(x: np.ndarray, y: np.ndarray, bin_deg: float) -> np.ndarray:
    bearing: np.ndarray = (np.degrees(np.arctan2(y, x)) % 360.0 // bin_deg).astype(int)
    return bearing


def _apply(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    moved: np.ndarray = points @ matrix[:3, :3].T + matrix[:3, 3]
    return moved


def r1pro_head_depth(tf_tolerance_s: float, **overrides: object) -> Blueprint:
    """Head depth anchored on Point-LIO's scans, tuned for the Orin, then put onto the floor by HeadFloorPrior;
    keywords override the Depth2DepthCloud config."""
    return autoconnect(
        Depth2DepthCloud.blueprint(
            **{
                "tf_tolerance_s": tf_tolerance_s,
                # Past this the per-pixel calibration has few lidar anchors to lean on.
                "max_range_m": 6.0,
                # lidar_history_s stays at d2d's 2 s (20 sweeps): at 6 s a person who walked off stayed in the anchors, a 6 s ghost.
                # The lidar never sees the floor (base_link z = 0): unanchored, the near field read 2-5x short.
                "height_frame": BASE_FRAME,
                "floor_tolerance_m": HeadFloorPriorConfig.model_fields["floor_tolerance_m"].default,
                # One point per 8x8 pixels, at most 3000: replaying r1_live_20260930_1332 on the R1, floor near the robot
                # rose from ~45% of the map at 11x11 / 1500 to 51%.
                "decimation": 8,
                "max_points": 3000,
                # One floor probe per cloud pixel.
                "floor_anchor_step": 8,
                # JPEG decoded at 1/4 size: calibration then works on a quarter of the pixels (3 -> 9.5 fps on the Orin).
                "decode_scale": 4,
                # One lidar anchor per 4x4 px block: ~31k anchors per frame was most of the calibration time.
                "anchor_step": 4,
                **overrides,
            }
        ).remappings(
            [
                (Depth2DepthCloud, "image", "head_left_color"),
                (Depth2DepthCloud, "camera_info", "head_left_info"),
                (Depth2DepthCloud, "depth_cloud", "head_depth_raw"),
            ]
        ),
        HeadFloorPrior.blueprint(
            tf_tolerance_s=tf_tolerance_s,
            footprint_radius_m=R1PRO_CHASSIS_WIDTH_M / 2,
            # The head's lowest ray meets the floor ~0.74 m out (farther with the head raised); unfilled, the start is an island.
            view_fill_reach_m=1.5,
        ).remappings([(HeadFloorPrior, "depth_cloud", "head_depth_raw")]),
    )
