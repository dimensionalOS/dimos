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

"""Image + pointcloud 3D detection as a memory2 fan-I/O StreamModule.

Two ``In`` ports (``color_image``, ``pointcloud``) feed one pipeline that
detects in 2D, aligns each detection frame with the nearest pointcloud, and
fuses to 3D. One pass emits four kinds of output - a ``Detection2DArray``, a
``Detection3DArray``, and the top-3 per-object pointclouds / cropped images for
visualization - all scattered from a single ``Bundle`` per tick. This replaces
the previous Rx ``align_timestamped`` wiring; cross-port alignment is now
``Stream.align(..., tolerance=0.25)`` inside ``pipeline()``.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos.agents.annotation import skill
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.memory2.fanio import Bundle
from dimos.memory2.module import StreamModule
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow, Transformer
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.module2D import Config
from dimos.perception.detection.type.detection2d.base import Filter2D
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.spec.perception import Camera, Pointcloud

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.core.rpc_client import ModuleProxy
    from dimos.memory2.type.observation import Observation


class Detect2D(Transformer[Image, ImageDetections2D]):
    """Run the 2D detector on each image, optionally filtering the results.

    Mirrors the old ``Detection2DModule.process_image_frame`` step as a memory2
    transform so it can sit inside a ``Stream`` pipeline ahead of ``.align()``.
    """

    def __init__(self, detector: Detector, filters: Sequence[Filter2D] = ()) -> None:
        self._detector = detector
        self._filters = tuple(filters)

    def __call__(
        self, upstream: Iterator[Observation[Image]]
    ) -> Iterator[Observation[ImageDetections2D]]:
        for obs in upstream:
            detections = self._detector.process_image(obs.data)
            if self._filters:
                detections = detections.filter(*self._filters)
            yield obs.derive(data=detections)


@dataclass
class FusedDetections:
    """Per-frame fusion result, before it is split across output ports.

    Carries both the original 2D detections and the projected 3D detections so a
    single fused tick can feed every ``Out`` (2D array, 3D array, per-object
    pointclouds and cropped images) without recomputing anything.
    """

    detections_2d: ImageDetections2D
    detections_3d: ImageDetections3DPC


class Detection3DModule(StreamModule):
    """Fuse 2D detections with a pointcloud and publish 2D + 3D detections.

    Fan-I/O shape: ``color_image`` drives the pipeline, ``pointcloud`` is the
    aligned sibling reached via ``self.streams.pointcloud``. The world->camera
    transform is resolved from TF at the image timestamp inside :meth:`_fuse`
    (no pose ``In`` port needed), matching the previous module's behavior.
    """

    config: Config
    detector: Detector

    color_image: In[Image]
    pointcloud: In[PointCloud2]

    detections_2d: Out[Detection2DArray]
    detections_3d: Out[Detection3DArray]

    # Visualization only: top-3 per-object pointclouds and cropped detection
    # images from the same fused frame.
    detected_pointcloud_0: Out[PointCloud2]
    detected_pointcloud_1: Out[PointCloud2]
    detected_pointcloud_2: Out[PointCloud2]
    detected_image_0: Out[Image]
    detected_image_1: Out[Image]
    detected_image_2: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.detector = self.config.detector()  # type: ignore[call-arg, misc]

    def pipeline(self, image: Stream[Image]) -> Stream[Bundle]:
        return self.fused_detections(image).map_data(self._to_bundle)

    def fused_detections(self, image: Stream[Image]) -> Stream[FusedDetections]:
        """2D-detect, align with the nearest pointcloud, then project to 3D.

        Split from :meth:`pipeline` so subclasses (e.g. the object database) can
        tap the fused stream before it is collapsed into a port ``Bundle``.
        """
        return (
            image.transform(
                QualityWindow(lambda img: img.sharpness, window=1.0 / self.config.max_freq)
            )
            .transform(Detect2D(self.detector, self.config.filter))
            .align(self.streams.pointcloud, tolerance=0.25)
            .map_data(self._fuse)
        )

    def _fuse(self, obs: Observation[Any]) -> FusedDetections:
        # obs.data is the AlignedPair; its fields are named after the backing
        # streams (the In ports), so color_image/pointcloud regardless of how the
        # primary payload changed through Detect2D.
        pair = obs.data
        detections_2d: ImageDetections2D = pair.color_image.data
        pointcloud: PointCloud2 = pair.pointcloud.data
        transform = self.tf.get("camera_optical", pointcloud.frame_id, detections_2d.image.ts, 5.0)
        detections_3d = self.process_frame(detections_2d, pointcloud, transform)
        return FusedDetections(detections_2d=detections_2d, detections_3d=detections_3d)

    def _to_bundle(self, obs: Observation[FusedDetections]) -> Bundle:
        fused = obs.data
        values: dict[str, Any] = {
            "detections_2d": fused.detections_2d.to_ros_detection2d_array(),
            "detections_3d": fused.detections_3d.to_ros_detection3d_array(),
        }
        for index, detection in enumerate(fused.detections_3d[:3]):
            values[f"detected_pointcloud_{index}"] = detection.pointcloud
        if self.config.publish_detection_images:
            for index, detection in enumerate(fused.detections_2d[:3]):
                values[f"detected_image_{index}"] = detection.cropped_image()
        return Bundle(values)

    def process_frame(
        self,
        detections: ImageDetections2D,
        pointcloud: PointCloud2,
        transform: Transform,
    ) -> ImageDetections3DPC:
        if not transform:
            return ImageDetections3DPC(detections.image, [])

        detection3d_list: list[Detection3DPC] = []
        for detection in detections:
            detection3d = Detection3DPC.from_2d(
                detection,
                world_pointcloud=pointcloud,
                camera_info=self.config.camera_info,
                world_to_optical_transform=transform,
            )
            if detection3d is not None:
                detection3d_list.append(detection3d)

        return ImageDetections3DPC(detections.image, detection3d_list)

    def pixel_to_3d(
        self,
        pixel: tuple[int, int],
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
        fx, fy = self.config.camera_info.K[0], self.config.camera_info.K[4]
        cx, cy = self.config.camera_info.K[2], self.config.camera_info.K[5]

        # Unproject pixel to normalized camera coordinates
        x_norm = (pixel[0] - cx) / fx
        y_norm = (pixel[1] - cy) / fy

        # Create 3D point at assumed depth in camera optical frame
        # Camera optical frame: X right, Y down, Z forward
        return Vector3(x_norm * assumed_depth, y_norm * assumed_depth, assumed_depth)

    @skill
    def ask_vlm(self, question: str) -> str:
        """asks a visual model about the view of the robot, for example
        is the bannana in the trunk?
        """
        from dimos.models.vl.qwen import QwenVlModel

        model = QwenVlModel()
        image = self.color_image.get_next()
        return model.query(image, question)

    # @skill
    @rpc
    def nav_vlm(self, question: str) -> str:
        """
        query visual model about the view in front of the camera
        you can ask to mark objects like:

        "red cup on the table left of the pencil"
        "laptop on the desk"
        "a person wearing a red shirt"
        """
        from dimos.models.vl.qwen import QwenVlModel

        model = QwenVlModel()
        image = self.color_image.get_next()
        result = model.query_detections(image, question)

        print("VLM result:", result, "for", image, "and question", question)

        if isinstance(result, str) or not result or not len(result):
            return None  # type: ignore[return-value]

        detections: ImageDetections2D = result

        print(detections)
        if not len(detections):
            print("No 2d detections")
            return None  # type: ignore[return-value]

        pc = self.pointcloud.get_next()
        transform = self.tf.get("camera_optical", pc.frame_id, detections.image.ts, 5.0)

        detections3d = self.process_frame(detections, pc, transform)

        if len(detections3d):
            return detections3d[0].pose  # type: ignore[no-any-return]
        print("No 3d detections, projecting 2d")

        center = detections[0].get_bbox_center()
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        return PoseStamped(
            ts=detections.image.ts,
            frame_id="world",
            position=self.pixel_to_3d(center, assumed_depth=1.5),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )


def deploy(  # type: ignore[no-untyped-def]
    dimos: ModuleCoordinator,
    lidar: Pointcloud,
    camera: Camera,
    prefix: str = "/detector3d",
    **kwargs,
) -> "ModuleProxy":
    detector = dimos.deploy(Detection3DModule, camera_info=camera.hardware_camera_info, **kwargs)  # type: ignore[attr-defined]

    detector.color_image.connect(camera.color_image)
    detector.pointcloud.connect(lidar.pointcloud)

    detector.detections_2d.transport = LCMTransport(f"{prefix}/detections", Detection2DArray)
    detector.detections_3d.transport = LCMTransport(f"{prefix}/detections_3d", Detection3DArray)

    detector.detected_image_0.transport = LCMTransport(f"{prefix}/image/0", Image)
    detector.detected_image_1.transport = LCMTransport(f"{prefix}/image/1", Image)
    detector.detected_image_2.transport = LCMTransport(f"{prefix}/image/2", Image)

    detector.detected_pointcloud_0.transport = LCMTransport(f"{prefix}/pointcloud/0", PointCloud2)
    detector.detected_pointcloud_1.transport = LCMTransport(f"{prefix}/pointcloud/1", PointCloud2)
    detector.detected_pointcloud_2.transport = LCMTransport(f"{prefix}/pointcloud/2", PointCloud2)

    detector.start()

    return detector
