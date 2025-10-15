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
from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from dimos_lcm.sensor_msgs import CameraInfo
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Transform, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import sharpness_barrier
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.detectors import Detector
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.type import (
    ImageDetections2D,
)
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure


@dataclass
class Config(ModuleConfig):
    max_freq: float = 10
    detector: Optional[Callable[[Any], Detector]] = YoloPersonDetector
    camera_info: CameraInfo = CameraInfo()


class Detection2DModule(Module):
    default_config = Config
    config: Config
    detector: Detector

    image: In[Image] = None  # type: ignore

    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    detected_image_0: Out[Image] = None  # type: ignore
    detected_image_1: Out[Image] = None  # type: ignore
    detected_image_2: Out[Image] = None  # type: ignore

    cnt: int = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config: Config = Config(**kwargs)
        self.detector = self.config.detector()
        self.vlm_detections_subject = Subject()
        self.previous_detection_count = 0

    def process_image_frame(self, image: Image) -> ImageDetections2D:
        return self.detector.process_image(image)

    @simple_mcache
    def sharp_image_stream(self) -> Observable[Image]:
        return backpressure(
            self.image.pure_observable().pipe(
                sharpness_barrier(self.config.max_freq),
            )
        )

    @simple_mcache
    def detection_stream_2d(self) -> Observable[ImageDetections2D]:
        return backpressure(self.image.observable().pipe(ops.map(self.process_image_frame)))

    def pixel_to_3d(
        self,
        pixel: Tuple[int, int],
        camera_info: CameraInfo,
        assumed_depth: float = 1.0,
    ) -> Vector3:
        """Unproject 2D pixel coordinates to 3D position in camera optical frame.

        Args:
            camera_info: Camera calibration information
            assumed_depth: Assumed depth in meters (default 1.0m from camera)

        Returns:
            Vector3 position in camera optical frame coordinates
        """
        # Extract camera intrinsics
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]

        # Unproject pixel to normalized camera coordinates
        x_norm = (pixel[0] - cx) / fx
        y_norm = (pixel[1] - cy) / fy

        # Create 3D point at assumed depth in camera optical frame
        # Camera optical frame: X right, Y down, Z forward
        return Vector3(x_norm * assumed_depth, y_norm * assumed_depth, assumed_depth)

    def track(self, detections: ImageDetections2D):
        sensor_frame = self.tf.get("sensor", "camera_optical", detections.image.ts, 5.0)

        if not sensor_frame:
            return

        if not detections.detections:
            return

        sensor_frame.child_frame_id = "sensor_frame"
        transforms = [sensor_frame]

        current_count = len(detections.detections)
        max_count = max(current_count, self.previous_detection_count)

        # Publish transforms for all detection slots up to max_count
        for index in range(max_count):
            if index < current_count:
                # Active detection - compute real position
                detection = detections.detections[index]
                position_3d = self.pixel_to_3d(
                    detection.center_bbox, self.config.camera_info, assumed_depth=1.0
                )
            else:
                # No detection at this index - publish zero transform
                position_3d = Vector3(0.0, 0.0, 0.0)

            transforms.append(
                Transform(
                    frame_id=sensor_frame.child_frame_id,
                    child_frame_id=f"det_{index}",
                    ts=detections.image.ts,
                    translation=position_3d,
                )
            )

        self.previous_detection_count = current_count
        self.tf.publish(*transforms)

    @rpc
    def start(self):
        self.detection_stream_2d().subscribe(self.track)

        self.detection_stream_2d().subscribe(
            lambda det: self.detections.publish(det.to_ros_detection2d_array())
        )

        self.detection_stream_2d().subscribe(
            lambda det: self.annotations.publish(det.to_foxglove_annotations())
        )

        def publish_cropped_images(detections: ImageDetections2D):
            for index, detection in enumerate(detections[:3]):
                image_topic = getattr(self, "detected_image_" + str(index))
                image_topic.publish(detection.cropped_image())

        self.detection_stream_2d().subscribe(publish_cropped_images)

    @rpc
    def stop(self): ...
