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


from typing import Any

from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.type import ImageDetections2D
from dimos.types.timestamped import align_timestamped
from dimos.utils.reactive import backpressure


class PersonTracker(Module):
    detections: In[Detection2DArray]
    image: In[Image]
    target: Out[PoseStamped]

    camera_info: CameraInfo

    def __init__(self, cameraInfo: CameraInfo, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.camera_info = cameraInfo

    def center_to_3d(
        self,
        pixel: tuple[float, float],
        camera_info: CameraInfo,
        assumed_depth: float = 1.0,
    ) -> Vector3:
        """Unproject 2D pixel coordinates to 3D position in camera_link frame.

        Args:
            camera_info: Camera calibration information
            assumed_depth: Assumed depth in meters (default 1.0m from camera)

        Returns:
            Vector3 position in camera_link frame coordinates (Z up, X forward)
        """
        # Extract camera intrinsics
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]

        # Unproject pixel to normalized camera coordinates
        x_norm = (pixel[0] - cx) / fx
        y_norm = (pixel[1] - cy) / fy

        # Create 3D point at assumed depth in camera optical frame
        # Camera optical frame: X right, Y down, Z forward
        x_optical = x_norm * assumed_depth
        y_optical = y_norm * assumed_depth
        z_optical = assumed_depth

        # Transform from camera optical frame to camera_link frame
        # Optical: X right, Y down, Z forward
        # Link: X forward, Y left, Z up
        # Transformation: x_link = z_optical, y_link = -x_optical, z_link = -y_optical
        return Vector3(z_optical, -x_optical, -y_optical)

    def detections_stream(self) -> Observable[ImageDetections2D]:
        return backpressure(
            align_timestamped(
                self.image.pure_observable(),
                self.detections.pure_observable().pipe(
                    ops.filter(lambda d: d.detections_length > 0)  # type: ignore[attr-defined]
                ),
                match_tolerance=0.0,
                buffer_size=2.0,
            ).pipe(
                ops.map(
                    lambda pair: ImageDetections2D.from_ros_detection2d_array(
                        *pair  # type: ignore[misc]
                    )
                )
            )
        )

    @rpc
    def start(self) -> None:
        self.detections_stream().subscribe(self.track)

    @rpc
    def stop(self) -> None:
        super().stop()

    def track(self, detections2D: ImageDetections2D) -> None:
        if len(detections2D) == 0:
            return

        target = max(detections2D.detections, key=lambda det: det.bbox_2d_volume())  # type: ignore[attr-defined]
        vector = self.center_to_3d(target.center_bbox, self.camera_info, 2.0)  # type: ignore[attr-defined]

        pose_in_camera = PoseStamped(
            ts=detections2D.ts,
            position=vector,
            frame_id="camera_link",
        )

        tf_world_to_camera = self.tf.get("world", "camera_link", detections2D.ts, 5.0)
        if not tf_world_to_camera:
            return

        tf_camera_to_target = Transform.from_pose("target", pose_in_camera)
        tf_world_to_target = tf_world_to_camera + tf_camera_to_target
        pose_in_world = tf_world_to_target.to_pose(ts=detections2D.ts)

        self.target.publish(pose_in_world)


person_tracker_module = PersonTracker.blueprint

__all__ = ["PersonTracker", "person_tracker_module"]
