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

"""Project stored 2D detection streams into 3D."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory.transform import Transformer
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.memory.type.observation import Observation
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.perception.detection.type.detection3d.pointcloud_filters import PointCloudFilter
    from dimos.protocol.tf.tf import TFLookup


def _world_to_optical(
    obs: Observation[Any],
    world_frame: str,
    tf: TFLookup | None,
    base_to_optical: Transform | None,
    optical_frame: str,
    time_tolerance: float,
) -> Transform | None:
    """Resolve the world→optical transform for an observation."""
    if tf is not None:
        return tf.get(optical_frame, world_frame, obs.ts, time_tolerance)
    pose = obs.pose_stamped
    if base_to_optical is None or pose is None:
        return None
    return -(Transform.from_pose("base_link", pose) + base_to_optical)


class ProjectDepthTo3D(Transformer["ImageDetections2D", ImageDetections3DPC]):
    """Lift 2D detections into 3D by unprojecting their aligned depth pixels.

    ``depth`` resolves the depth frame for each observation (e.g. a temporal
    join against a depth stream). No world map needed — the depth image is the
    geometry, so only the detected pixels ever become points.
    """

    def __init__(
        self,
        depth: Callable[[Observation[ImageDetections2D]], Image | None],
        camera_info: CameraInfo,
        tf: TFLookup | None = None,
        base_to_optical: Transform | None = None,
        world_frame: str = "world",
        optical_frame: str = "camera_optical",
        time_tolerance: float = 5.0,
        filters: list[PointCloudFilter] | None = None,
    ) -> None:
        if tf is None and base_to_optical is None:
            raise ValueError("ProjectDepthTo3D needs either tf or base_to_optical")
        self.depth = depth
        self.camera_info = camera_info
        self.tf = tf
        self.base_to_optical = base_to_optical
        self.world_frame = world_frame
        self.optical_frame = optical_frame
        self.time_tolerance = time_tolerance
        self.filters = filters

    def __call__(
        self, upstream: Iterator[Observation[ImageDetections2D]]
    ) -> Iterator[Observation[ImageDetections3DPC]]:
        for obs in upstream:
            depth = self.depth(obs)
            if depth is None:
                continue
            transform = _world_to_optical(
                obs,
                self.world_frame,
                self.tf,
                self.base_to_optical,
                self.optical_frame,
                self.time_tolerance,
            )
            if transform is None:
                continue
            yield obs.derive(
                data=ImageDetections3DPC.from_depth(
                    obs.data,
                    depth,
                    self.camera_info,
                    transform,
                    self.filters,
                )
            )


def sees(
    point: Any,
    camera_info: CameraInfo,
    tf: TFLookup | None = None,
    base_to_optical: Transform | None = None,
    world_frame: str = "world",
    optical_frame: str = "camera_optical",
    time_tolerance: float = 5.0,
    margin: float = 0.0,
    max_range: float | None = None,
    extent: Any | None = None,
    min_fraction: float = 1.0,
    depth: Callable[[Observation[Any]], Image | None] | None = None,
    occlusion_tolerance: float = 0.08,
) -> Callable[[Observation[Any]], bool]:
    """Predicate for ``Stream.filter()``: keep frames whose camera sees the world point.

    Pair with ``.near()`` so the store prefilters spatially:

        images.near(det.pose, radius=6.0).filter(sees(det.pose, camera_info, ...))

    ``margin`` requires projections that many pixels inside the image edges;
    ``max_range`` drops frames further than that from the point.

    ``extent`` turns the point into a support box: the center plus the eight
    box corners are projected, and the frame passes when at least
    ``min_fraction`` of those samples land inside the image. A half-framed
    object is then no longer treated like a fully framed one.

    ``depth`` enables a cheap occlusion test: measured depth at the visible
    sample pixels is compared against each sample's expected range, and the
    frame is rejected when a majority of samples sit behind nearer geometry
    by more than ``occlusion_tolerance`` meters. Invalid depth (0) counts as
    unknown, not as occluding.
    """
    if tf is None and base_to_optical is None:
        raise ValueError("sees needs either tf or base_to_optical")

    target = np.array([point.x, point.y, point.z]) if hasattr(point, "x") else np.asarray(point)
    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]
    width, height = float(camera_info.width), float(camera_info.height)

    if extent is not None:
        half = 0.5 * (
            np.array([extent.x, extent.y, extent.z])
            if hasattr(extent, "x")
            else np.asarray(extent, dtype=float)
        )
        corners = np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float
        )
        samples = np.vstack([target, target + corners * half])
    else:
        samples = target.reshape(1, 3)

    def _sees(obs: Observation[Any]) -> bool:
        transform = _world_to_optical(
            obs, world_frame, tf, base_to_optical, optical_frame, time_tolerance
        )
        if transform is None:
            return False
        matrix = transform.to_matrix()
        pts = (matrix @ np.column_stack([samples, np.ones(len(samples))]).T).T[:, :3]
        in_front = pts[:, 2] > 0
        if not in_front.any():
            return False
        if max_range is not None and float(np.linalg.norm(pts[0])) > max_range:
            return False
        u = fx * pts[:, 0] / np.where(in_front, pts[:, 2], 1.0) + cx
        v = fy * pts[:, 1] / np.where(in_front, pts[:, 2], 1.0) + cy
        visible = (
            in_front & (u >= margin) & (u < width - margin) & (v >= margin) & (v < height - margin)
        )
        if float(visible.mean()) < min_fraction:
            return False

        if depth is not None and visible.any():
            depth_frame = depth(obs)
            if depth_frame is not None:
                depth_m = np.asarray(depth_frame.data, dtype=np.float32)
                if depth_frame.data.dtype == np.uint16:
                    depth_m = depth_m * 0.001
                rows = np.clip(v[visible].astype(int), 0, depth_m.shape[0] - 1)
                cols = np.clip(u[visible].astype(int), 0, depth_m.shape[1] - 1)
                measured = depth_m[rows, cols]
                expected = pts[visible][:, 2]
                known = measured > 0
                if known.any():
                    occluded = measured[known] + occlusion_tolerance < expected[known]
                    if float(occluded.mean()) > 0.5:
                        return False

        return True

    return _sees


class ProjectTo3D(Transformer["ImageDetections2D", ImageDetections3DPC]):
    """Lift 2D detections into 3D by projecting a world pointcloud through the camera.

    The pointcloud is either a fixed world map or a callable resolving one per
    observation (e.g. a local voxel map around the observation pose). The
    world→optical transform comes from ``tf`` (e.g. ``StreamTF.from_store(store)``);
    for recordings without a tf stream, pass ``base_to_optical`` to derive it
    from each observation's pose instead.
    """

    def __init__(
        self,
        pointcloud: PointCloud2 | Callable[[Observation[ImageDetections2D]], PointCloud2 | None],
        camera_info: CameraInfo,
        tf: TFLookup | None = None,
        base_to_optical: Transform | None = None,
        optical_frame: str = "camera_optical",
        time_tolerance: float = 5.0,
        filters: list[PointCloudFilter] | None = None,
    ) -> None:
        if tf is None and base_to_optical is None:
            raise ValueError("ProjectTo3D needs either tf or base_to_optical")
        self.pointcloud = pointcloud
        self.camera_info = camera_info
        self.tf = tf
        self.base_to_optical = base_to_optical
        self.optical_frame = optical_frame
        self.time_tolerance = time_tolerance
        self.filters = filters

    def __call__(
        self, upstream: Iterator[Observation[ImageDetections2D]]
    ) -> Iterator[Observation[ImageDetections3DPC]]:
        for obs in upstream:
            pointcloud = self.pointcloud(obs) if callable(self.pointcloud) else self.pointcloud
            if pointcloud is None:
                continue
            transform = _world_to_optical(
                obs,
                pointcloud.frame_id,
                self.tf,
                self.base_to_optical,
                self.optical_frame,
                self.time_tolerance,
            )
            if transform is None:
                continue
            yield obs.derive(
                data=ImageDetections3DPC.from_2d(
                    obs.data,
                    pointcloud,
                    self.camera_info,
                    transform,
                    self.filters,
                )
            )
