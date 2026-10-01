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


from typing import Any

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_generated.vision_msgs.msg import Detection2DArray
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry import compose_transforms, pose_from_transform, transform_from_pose
from dimos.msgs.time import to_seconds
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.types.timestamped import TimestampedData, align_timestamped
from dimos.utils.reactive import backpressure


def _timed_image(message: Image) -> TimestampedData[Image]:
    return TimestampedData(message, to_seconds(message.header.stamp))


def _timed_detections(message: Detection2DArray) -> TimestampedData[Detection2DArray]:
    return TimestampedData(message, to_seconds(message.header.stamp))


def _has_detections(message: Detection2DArray) -> bool:
    return len(message.detections) > 0


def _paired_detections(
    pair: tuple[TimestampedData[Image], TimestampedData[Detection2DArray]],
) -> ImageDetections2D:
    return ImageDetections2D.from_ros_detection2d_array(pair[0].value, pair[1].value)


class PersonTracker(Module):
    detections: In[Detection2DArray]
    color_image: In[Image]
    target: Out[PoseStamped]
    tf: In[TFMessage]

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
        fx, fy = camera_info.k[0], camera_info.k[4]
        cx, cy = camera_info.k[2], camera_info.k[5]

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
        return Vector3(x=z_optical, y=-x_optical, z=-y_optical)

    def detections_stream(self) -> Observable[ImageDetections2D]:
        return backpressure(
            align_timestamped(
                self.color_image.pure_observable().pipe(ops.map(_timed_image)),
                self.detections.pure_observable().pipe(
                    ops.filter(_has_detections), ops.map(_timed_detections)
                ),
                match_tolerance=0.0,
                buffer_size=2.0,
            ).pipe(ops.map(_paired_detections))
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

        target = max(detections2D.detections, key=lambda det: det.bbox_2d_volume())
        vector = self.center_to_3d(target.center_bbox, self.camera_info, 2.0)

        pose_in_camera = PoseStamped(
            header=Header(frame_id="camera_link", stamp=detections2D.image.header.stamp),
            pose=Pose(
                position=Point(x=vector.x, y=vector.y, z=vector.z), orientation=Quaternion(w=1)
            ),
        )

        tf_world_to_camera = self.tfbuffer.get("world", "camera_link", detections2D.ts, 5.0)
        if not tf_world_to_camera:
            return

        tf_camera_to_target = transform_from_pose(pose_in_camera, child_frame_id="target")
        tf_world_to_target = compose_transforms(tf_world_to_camera, tf_camera_to_target)
        pose_in_world = pose_from_transform(tf_world_to_target)

        self.target.publish(pose_in_world)
