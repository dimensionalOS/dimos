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


from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    ImageAnnotations,
)
from lcm_msgs.foxglove_msgs import SceneUpdate  # type: ignore[import-not-found]
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos import spec
from dimos.agents import skill  # type: ignore[attr-defined]
from dimos.core import DimosCluster, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.detection.type import (
    ImageDetections2D,
    ImageDetections3DPC,
)
from dimos.perception.detection.type.detection3d import Detection3DPC
from dimos.types.timestamped import align_timestamped
from dimos.utils.reactive import backpressure


class Detection3DModule(Detection2DModule):
    image: In[Image]
    pointcloud: In[PointCloud2]

    detections: Out[Detection2DArray]
    annotations: Out[ImageAnnotations]
    scene_update: Out[SceneUpdate]

    # just for visualization,
    # emits latest pointclouds of detected objects in a frame
    detected_pointcloud_0: Out[PointCloud2]
    detected_pointcloud_1: Out[PointCloud2]
    detected_pointcloud_2: Out[PointCloud2]

    # just for visualization, emits latest top 3 detections in a frame
    detected_image_0: Out[Image]
    detected_image_1: Out[Image]
    detected_image_2: Out[Image]

    detection_3d_stream: Observable[ImageDetections3DPC] | None = None

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

    @skill()
    def ask_vlm(self, question: str) -> str:
        """asks a visual model about the view of the robot, for example
        is the bannana in the trunk?
        """
        from dimos.models.vl.qwen import QwenVlModel

        model = QwenVlModel()
        image = self.image.get_next()
        return model.query(image, question)

    # @skill  # type: ignore[arg-type]
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
        image = self.image.get_next()
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
        return PoseStamped(
            ts=detections.image.ts,
            frame_id="world",
            position=self.pixel_to_3d(center, assumed_depth=1.5),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )

    @rpc
    def start(self) -> None:
        super().start()

        def detection2d_to_3d(args):  # type: ignore[no-untyped-def]
            detections, pc = args
            transform = self.tf.get("camera_optical", pc.frame_id, detections.image.ts, 5.0)
            return self.process_frame(detections, pc, transform)

        self.detection_stream_3d = align_timestamped(
            backpressure(self.detection_stream_2d()),
            self.pointcloud.observable(),  # type: ignore[no-untyped-call]
            match_tolerance=0.25,
            buffer_size=20.0,
        ).pipe(ops.map(detection2d_to_3d))

        self.detection_stream_3d.subscribe(self._publish_detections)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _publish_detections(self, detections: ImageDetections3DPC) -> None:
        if not detections:
            return

        for index, detection in enumerate(detections[:3]):
            pointcloud_topic = getattr(self, "detected_pointcloud_" + str(index))
            pointcloud_topic.publish(detection.pointcloud)

        self.scene_update.publish(detections.to_foxglove_scene_update())


def deploy(  # type: ignore[no-untyped-def]
    dimos: DimosCluster,
    lidar: spec.Pointcloud,
    camera: spec.Camera,
    prefix: str = "/detector3d",
    **kwargs,
) -> Detection3DModule:
    from dimos.core import LCMTransport

    detector = dimos.deploy(Detection3DModule, camera_info=camera.hardware_camera_info, **kwargs)  # type: ignore[attr-defined]

    detector.image.connect(camera.color_image)
    detector.pointcloud.connect(lidar.pointcloud)

    detector.annotations.transport = LCMTransport(f"{prefix}/annotations", ImageAnnotations)
    detector.detections.transport = LCMTransport(f"{prefix}/detections", Detection2DArray)
    detector.scene_update.transport = LCMTransport(f"{prefix}/scene_update", SceneUpdate)

    detector.detected_image_0.transport = LCMTransport(f"{prefix}/image/0", Image)
    detector.detected_image_1.transport = LCMTransport(f"{prefix}/image/1", Image)
    detector.detected_image_2.transport = LCMTransport(f"{prefix}/image/2", Image)

    detector.detected_pointcloud_0.transport = LCMTransport(f"{prefix}/pointcloud/0", PointCloud2)
    detector.detected_pointcloud_1.transport = LCMTransport(f"{prefix}/pointcloud/1", PointCloud2)
    detector.detected_pointcloud_2.transport = LCMTransport(f"{prefix}/pointcloud/2", PointCloud2)

    detector.start()

    return detector  # type: ignore[no-any-return]


detection3d_module = Detection3DModule.blueprint

__all__ = ["Detection3DModule", "deploy", "detection3d_module"]
