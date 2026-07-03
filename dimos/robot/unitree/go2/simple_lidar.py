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

"""GO2Connection variant that re-expresses the Go2's onboard lidar in base_link.

Drop-in replacement for ``GO2Connection`` — it inherits the whole connection
(movement, odom, camera, tf, skills) and changes only how the lidar cloud is
published. The Go2's onboard stack transforms its lidar into the odom/world
frame (``pointcloud2_from_webrtc_lidar`` stamps it ``world``) and accumulates
scans. This undoes both: it applies the inverse of the robot's current world
pose so points land back in ``base_link``, and with ``un_accumulate`` on it
subtracts the prior cloud (in the stable world frame, where accumulated points
keep identical coordinates) so only the new points are published.

An optional rigid ``transform`` (row-major 4x4, 16 floats) is applied to the
base_link cloud before publishing — mirrors PointLio's ``transform`` and lets
the cloud be re-expressed as if it came from another sensor (e.g. mid360_link).
"""

from __future__ import annotations

import numpy as np
from pydantic import field_validator

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection


def _rows_not_in(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Rows of ``points`` (Nx3) that aren't byte-identical rows of ``reference``."""
    if reference.size == 0:
        return points
    row_dtype = np.dtype([("", points.dtype)] * points.shape[1])
    points_view = np.ascontiguousarray(points).view(row_dtype).ravel()
    reference_view = np.ascontiguousarray(reference).view(row_dtype).ravel()
    return points[~np.isin(points_view, reference_view)]


class SimpleLidarConfig(ConnectionConfig):
    # Optional rigid transform applied to the base_link cloud before publishing:
    # row-major 4x4 (16 floats), None = identity. Mirrors PointLio's transform.
    transform: list[float] | None = None
    # Frame the transformed cloud is stamped with. None keeps the body frame; set
    # it to re-express the cloud as if it came from another sensor (e.g. mid360_link).
    output_frame: str | None = None
    # Subtract the previous (accumulated) cloud and publish only the new points.
    un_accumulate: bool = True

    @field_validator("transform")
    @classmethod
    def _validate_transform(cls, value: list[float] | None) -> list[float] | None:
        if value is not None and len(value) != 16:
            raise ValueError(f"transform must be a row-major 4x4 (16 floats), got {len(value)}")
        return value


class SimpleLidar(GO2Connection):
    """GO2Connection that publishes the onboard lidar back in the base_link frame."""

    config: SimpleLidarConfig

    _latest_pose: PoseStamped | None = None
    _transform: Transform | None = None
    _previous_points: np.ndarray | None = None

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        if self.config.transform is not None:
            matrix = np.array(self.config.transform, dtype=float).reshape(4, 4)
            output_frame = self.config.output_frame or self.frame_mapping["body"]
            self._transform = Transform.from_matrix(
                matrix, frame_id=output_frame, child_frame_id=output_frame
            )

    def _publish_tf(self, msg: PoseStamped) -> None:
        self._latest_pose = msg
        super()._publish_tf(msg)

    def _publish_lidar(self, cloud: PointCloud2) -> None:
        pose = self._latest_pose
        if pose is None:
            return
        if self.config.un_accumulate:
            cloud = self._only_new_points(cloud)
        base_frame = self.frame_mapping["body"]
        # from_pose gives base_link's pose in world; its inverse maps the world
        # cloud back into base_link.
        world_to_base = Transform.from_pose(base_frame, pose).inverse()
        base_cloud = cloud.transform(world_to_base)
        if self._transform is not None:
            base_cloud = base_cloud.transform(self._transform)
        self.lidar.publish(base_cloud)

    def _only_new_points(self, cloud: PointCloud2) -> PointCloud2:
        points = cloud.points_f32()
        previous = self._previous_points
        self._previous_points = points
        if previous is None:
            return cloud
        new_points = _rows_not_in(points, previous)
        return PointCloud2.from_numpy(new_points, frame_id=cloud.frame_id, timestamp=cloud.ts)
