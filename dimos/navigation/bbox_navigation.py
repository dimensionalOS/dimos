# Copyright 2025 Dimensional Inc.
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

import logging

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.DEBUG)


class BBoxNavigationModule(Module):
    """Minimal module that converts 2D bbox center to navigation goals."""

    detection2d: In[Detection2DArray]
    camera_info: In[CameraInfo]
    goal_request: Out[PoseStamped]

    def __init__(self, goal_distance: float = 1.0) -> None:
        super().__init__()
        self.goal_distance = goal_distance
        self.camera_intrinsics = None

    @rpc
    def start(self) -> None:
        unsub = self.camera_info.subscribe(
            lambda msg: setattr(self, "camera_intrinsics", [msg.K[0], msg.K[4], msg.K[2], msg.K[5]])
        )
        self._disposables.add(Disposable(unsub))

        unsub = self.detection2d.subscribe(self._on_detection)
        self._disposables.add(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_detection(self, det: Detection2DArray) -> None:
        if det.detections_length == 0 or not self.camera_intrinsics:
            return
        fx, fy, cx, cy = self.camera_intrinsics
        center_x, center_y = (
            det.detections[0].bbox.center.position.x,
            det.detections[0].bbox.center.position.y,
        )
        x, y, z = (
            (center_x - cx) / fx * self.goal_distance,
            (center_y - cy) / fy * self.goal_distance,
            self.goal_distance,
        )
        goal = PoseStamped(
            position=Vector3(z, -x, -y),
            orientation=Quaternion(0, 0, 0, 1),
            frame_id=det.header.frame_id,
        )
        logger.debug(
            f"BBox center: ({center_x:.1f}, {center_y:.1f}) → "
            f"Goal pose: ({z:.2f}, {-x:.2f}, {-y:.2f}) in frame '{det.header.frame_id}'"
        )
        self.goal_request.publish(goal)
