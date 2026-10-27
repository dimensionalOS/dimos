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

"""Leave the points inside named boxes, or outside a crop, out of a point cloud."""

from __future__ import annotations

import asyncio
from collections.abc import Iterable
from dataclasses import dataclass
import itertools
import math
from threading import RLock
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

logger = setup_logger()

# A missing transform drops every cloud until it shows up: say so, not at the clouds' rate.
MISSING_TRANSFORM_WARN_INTERVAL_S = 5.0

# Where a voxel is sampled when it is re-rasterized, in edges from its centre: 2 x 2 x 2 points a
# quarter edge in. Every point of the voxel has one within sqrt(3)/4 of an edge (0.43 < 0.5), so
# at any rotation a target cell whose centre lies in the voxel holds a sample. 3 x 3 x 3 gives the
# same guarantee at three times the cost (11 ms against 4 ms a 3600-voxel map on the Orin) and
# covers more cells a voxel only clips (2026-10-25).
VOXEL_LATTICE = np.array(list(itertools.product((-0.25, 0.25), repeat=3)), dtype=np.float64)
# Cell indices pack three to an int64, 21 bits each, so one sort deduplicates the cells.
_KEY_BITS = 21
_KEY_OFFSET = 1 << (_KEY_BITS - 1)
_KEY_MASK = (1 << _KEY_BITS) - 1


class PointCloudBoxFilterConfig(ModuleConfig):
    # The frame the filtered cloud is sent in. None passes each cloud on in its own
    # frame; a frame re-expresses the cloud through tf at the cloud's stamp, so a map
    # built in a fixed frame reaches a consumer that plans in a moving one. A box with
    # no frame of its own is then in this frame.
    target_frame: str | None = None
    # How far from the cloud's stamp the transform into target_frame may be, and how
    # long to wait for one still in flight; a cloud with no transform is dropped.
    tf_tolerance_s: float = Field(default=0.1, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.2, ge=0.0)
    # The edge of the grid the consumer bins the cloud on, for a cloud of voxel centres of that
    # edge (a voxel map) moved into target_frame. Moved one point per voxel into a turned frame,
    # some of the consumer's cells get two points and some none: holes. Set, each voxel is
    # re-rasterized onto target_frame's grid instead (cells at multiples of this edge from its
    # origin, as an octree bins) and the cloud carries one point per covered cell, at its centre.
    # None moves the points as they are.
    target_voxel_m: float | None = Field(default=None, gt=0.0)
    # An inclusion crop: the corners of an axis-aligned box in the frame the cloud is sent in
    # (target_frame, or the cloud's own), metres. Only the points inside it go on; with
    # target_voxel_m it picks the voxels by their centres before they are re-rasterized, so a
    # voxel inside may cover a cell up to one edge past it. None for both keeps the whole cloud.
    crop_min: tuple[float, float, float] | None = None
    crop_max: tuple[float, float, float] | None = None

    @model_validator(mode="after")
    def _crop_is_a_box(self) -> PointCloudBoxFilterConfig:
        if (self.crop_min is None) != (self.crop_max is None):
            raise ValueError("crop_min and crop_max are set together or not at all")
        if self.crop_min is not None and self.crop_max is not None:
            if not all(lo < hi for lo, hi in zip(self.crop_min, self.crop_max, strict=True)):
                raise ValueError("crop_min must be below crop_max on every axis")
        return self


@dataclass(frozen=True)
class Box:
    """A box to leave out of a cloud.

    Args:
        pose: The box's centre and orientation, in ``frame_id``.
        size: Full edge lengths along the box's own axes, metres.
        frame_id: The frame the box is fixed in, or None for the cloud's own
            frame. A moving frame, such as a robot's tool tip, carries the box.
    """

    pose: Pose
    size: Vector3
    frame_id: str | None = None


def inside_boxes(
    points: NDArray[np.floating[Any]],
    boxes: Iterable[tuple[NDArray[np.float64], NDArray[np.float64]]],
) -> NDArray[np.bool_]:
    """Which points lie inside any of the boxes.

    Args:
        points: N x 3 positions, metres.
        boxes: Each box as (its 4x4 pose in the points' frame, its full edge
            lengths in metres).
    """
    inside = np.zeros(len(points), dtype=bool)
    for pose, size in boxes:
        # Each point in the box's own axes, R^T (p - t), written for row vectors.
        local = (np.asarray(points, dtype=np.float64) - pose[:3, 3]) @ pose[:3, :3]
        inside |= np.all(np.abs(local) <= size / 2.0, axis=1)
    return inside


def inside_crop(
    points: NDArray[np.floating[Any]],
    crop_min: tuple[float, float, float],
    crop_max: tuple[float, float, float],
) -> NDArray[np.bool_]:
    """Which points lie inside an axis-aligned box, its faces included.

    Args:
        points: N x 3 positions, metres.
        crop_min: The box's lowest corner in the points' frame, metres.
        crop_max: Its highest corner, metres.
    """
    inside: NDArray[np.bool_] = np.all((points >= crop_min) & (points <= crop_max), axis=1)
    return inside


def rasterize_voxels(
    centres: NDArray[np.floating[Any]], transform: NDArray[np.float64], voxel_m: float
) -> NDArray[np.float32]:
    """The cells of another frame's grid that a set of voxels covers, one centre per cell.

    Each voxel is a cube of edge ``voxel_m`` around its centre, axis-aligned in the points'
    frame. It is sampled on ``VOXEL_LATTICE``, the samples are moved by ``transform`` and binned
    on the target grid, cell k spanning [k, k + 1) edges from the origin on each axis, the way
    an octomap octree of that resolution bins a point. Every cell whose centre lies inside a
    voxel is covered, whatever the rotation, so a turned surface keeps no holes; a cell a voxel
    only clips may be covered too. A non-finite point, or one 2^20 edges out, covers nothing.

    Args:
        centres: N x 3 voxel centres, metres.
        transform: The 4x4 pose of the points' frame in the target frame.
        voxel_m: The voxels' edge, which is the target grid's, metres.

    Returns:
        M x 3 cell centres in the target frame, each cell once, in key order.
    """
    if not len(centres):
        return np.empty((0, 3), dtype=np.float32)
    rotation = transform[:3, :3]
    moved = np.asarray(centres, dtype=np.float64) @ rotation.T + transform[:3, 3]
    # The lattice turns with its voxel: a sample is the moved centre plus a turned offset.
    offsets = (VOXEL_LATTICE * voxel_m) @ rotation.T
    keys = np.floor((moved[:, None, :] + offsets).reshape(-1, 3) / voxel_m)
    indexable = np.all(np.abs(keys) < _KEY_OFFSET, axis=1)  # False for NaN too
    if not indexable.all():
        keys = keys[indexable]
    biased = keys.astype(np.int64) + _KEY_OFFSET
    packed = np.unique(
        (biased[:, 0] << (2 * _KEY_BITS)) | (biased[:, 1] << _KEY_BITS) | biased[:, 2]
    )
    cells = np.stack(
        (packed >> (2 * _KEY_BITS), (packed >> _KEY_BITS) & _KEY_MASK, packed & _KEY_MASK),
        axis=1,
    )
    return np.asarray((cells - _KEY_OFFSET + 0.5) * voxel_m, dtype=np.float32)


class PointCloudBoxFilter(Module):
    """Pass a point cloud on with the points inside named boxes left out.

    A box is fixed in the cloud's frame or rides on another frame from tf.
    Changing the boxes re-sends the last cloud at once, so a consumer sees
    the change before the next cloud arrives. With a ``target_frame`` the
    cloud is re-expressed in that frame first, and the boxes are placed in it; with a
    ``target_voxel_m`` too, a voxel map is re-rasterized onto that frame's grid
    (``rasterize_voxels``) and the boxes leave out the cells whose centres they hold.
    A ``crop_min`` / ``crop_max`` box in that frame keeps only what lies inside it, cropped
    before the re-rasterization, so a map that grows without bound reaches the consumer
    bounded.
    """

    config: PointCloudBoxFilterConfig

    cloud: In[PointCloud2]
    tf: In[TFMessage]
    filtered_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._boxes: dict[str, Box] = {}
        self._last_cloud: PointCloud2 | None = None
        self._dropped_since_warning = 0
        self._missing_warned_at: float | None = None

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    async def handle_cloud(self, cloud: PointCloud2) -> None:
        await asyncio.to_thread(self._on_cloud, cloud)

    @rpc
    def set_box(self, name: str, pose: Pose, size: Vector3, frame_id: str | None = None) -> bool:
        """Leave the points inside a box out. Setting a name again replaces that box.

        Args:
            name: The caller's label for the box; clear it with the same name.
            pose: The box's centre and orientation in ``frame_id``, metres.
            size: Full edge lengths along the box's own axes, metres.
            frame_id: The frame the box is fixed in, or None for the cloud's frame.
        """
        if not name:
            raise ValueError("Box name must not be empty")
        if any(edge <= 0.0 or not math.isfinite(edge) for edge in size.to_tuple()):
            raise ValueError("Box size must have three positive, finite edges")
        with self._lock:
            self._boxes[name] = Box(pose, size, frame_id)
            self._resend()
        return True

    @rpc
    def clear_box(self, name: str) -> bool:
        """Let the points inside a named box through again. False if there is no such box."""
        with self._lock:
            removed = self._boxes.pop(name, None) is not None
            if removed:
                self._resend()
        return removed

    def _on_cloud(self, cloud: PointCloud2) -> None:
        with self._lock:
            self._last_cloud = cloud
            self._send(cloud)

    def _resend(self) -> None:
        if self._last_cloud is not None:
            self._send(self._last_cloud)

    def _send(self, cloud: PointCloud2) -> None:
        filtered = self._filtered(cloud)
        if filtered is not None:
            self.filtered_cloud.publish(filtered)

    def _filtered(self, cloud: PointCloud2) -> PointCloud2 | None:
        """The cloud in the target frame without the boxed points; None when it cannot be placed."""
        expressed = self._expressed(cloud)
        if expressed is None:
            return None
        frame_id, points, cropped, rasterized = expressed
        boxes = [placed for box in self._boxes.values() if (placed := self._placed(box, frame_id))]
        if frame_id == cloud.frame_id and cropped is None and (not boxes or not len(points)):
            return cloud
        keep = ~inside_boxes(points, boxes) if boxes and len(points) else None
        filtered = PointCloud2.from_numpy(
            points if keep is None else points[keep], frame_id=frame_id, timestamp=cloud.ts
        )
        filtered.seq = cloud.seq
        # A cell is none of the cloud's points: their other attributes stay behind.
        for name, values in cloud.pointcloud_tensor.point.items():
            if name != "positions" and not rasterized:
                kept = values if cropped is None else values[cropped]
                filtered.pointcloud_tensor.point[name] = kept if keep is None else kept[keep]
        return filtered

    def _expressed(
        self, cloud: PointCloud2
    ) -> tuple[str, NDArray[np.float32], NDArray[np.bool_] | None, bool] | None:
        """The cloud's frame and positions in the target frame, which of its points the crop
        kept (None without a crop), and whether the positions are cells re-rasterized from its
        voxels rather than its own points; None with no transform."""
        points = cloud.points_f32()
        target = self.config.target_frame
        if target is None or target == cloud.frame_id:
            cropped = self._cropped(points)
            return cloud.frame_id, _kept(points, cropped), cropped, False
        if not len(points):
            # Nothing to move: an empty cloud is empty in every frame.
            return target, points, None, False
        transform = self.tfbuffer.get(
            target,
            cloud.frame_id,
            cloud.ts,
            self.config.tf_tolerance_s,
            forward_tolerance=self.config.tf_forward_tolerance_s,
            warn=False,
        )
        if transform is None:
            self._dropped(cloud, target)
            return None
        matrix = transform.to_matrix()
        voxel_m = self.config.target_voxel_m
        if voxel_m is not None and self.config.crop_min is None:
            return target, rasterize_voxels(points, matrix, voxel_m), None, True
        moved = np.asarray(points @ matrix[:3, :3].T + matrix[:3, 3], dtype=np.float32)
        cropped = self._cropped(moved)
        if voxel_m is not None:
            # The voxels whose centres the crop holds, rasterized from where they are.
            return target, rasterize_voxels(_kept(points, cropped), matrix, voxel_m), cropped, True
        return target, _kept(moved, cropped), cropped, False

    def _cropped(self, points: NDArray[np.float32]) -> NDArray[np.bool_] | None:
        """Which points the crop keeps, in the frame the cloud is sent in; None without one."""
        if self.config.crop_min is None or self.config.crop_max is None:
            return None
        return inside_crop(points, self.config.crop_min, self.config.crop_max)

    def _dropped(self, cloud: PointCloud2, target: str) -> None:
        self._dropped_since_warning += 1
        now = time.monotonic()
        if (
            self._missing_warned_at is not None
            and now - self._missing_warned_at < MISSING_TRANSFORM_WARN_INTERVAL_S
        ):
            return
        logger.warning(
            "No transform from %s to %s within %.2f s of stamp %.3f; dropped %d cloud(s)",
            target,
            cloud.frame_id,
            self.config.tf_tolerance_s,
            cloud.ts,
            self._dropped_since_warning,
        )
        self._missing_warned_at = now
        self._dropped_since_warning = 0

    def _placed(
        self, box: Box, frame_id: str
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        """The box's pose in ``frame_id`` and its edge lengths; None while the box's frame is unknown."""
        pose = pose_to_matrix(box.pose)
        if box.frame_id is not None and box.frame_id != frame_id:
            transform = self.tfbuffer.get(frame_id, box.frame_id, warn=False)
            if transform is None:
                logger.warning(
                    "No transform from %s to %s yet; box points stay in the cloud",
                    frame_id,
                    box.frame_id,
                )
                return None
            pose = transform.to_matrix() @ pose
        return pose, np.asarray(box.size.to_tuple(), dtype=np.float64)


def _kept(points: NDArray[np.float32], mask: NDArray[np.bool_] | None) -> NDArray[np.float32]:
    return points if mask is None else points[mask]
