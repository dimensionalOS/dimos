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

import base64
from threading import Event, RLock, Thread
import time
from typing import Any

from langchain_core.messages import HumanMessage
import numpy as np
from reactivex.disposable import Disposable
from turbojpeg import TurboJPEG

from dimos.agents.agent_spec import AgentSpec
from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.qwen.bbox import BBox
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.models.vl.base import VlModel
from dimos.models.vl.create import create
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.patrolling.patrolling_module_spec import PatrollingModuleSpec
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.navigation.visual_servoing.detection_navigation import DetectionNavigation
from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Config(ModuleConfig):
    camera_info: CameraInfo
    use_3d_navigation: bool = False


class PersonFollowSkillContainer(Module):
    """Skill container for following a person.

    This skill uses:
    - A VL model (QwenVlModel) to initially detect a person from a text description.
    - EdgeTAM for continuous tracking across frames.
    - Visual servoing OR 3D navigation to control robot movement towards the person.
    - Does not do obstacle avoidance; assumes a clear path.
    """

    config: Config

    color_image: In[Image]
    global_map: In[PointCloud2]
    cmd_vel: Out[Twist]

    _agent_spec: AgentSpec
    _frequency: float = 20.0  # Hz - control loop frequency
    _max_lost_frames: int = 15  # number of frames to wait before declaring person lost
    _patrolling_module_spec: PatrollingModuleSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_image: Image | None = None
        self._latest_pointcloud: PointCloud2 | None = None
        self._vl_model: VlModel = create("qwen")
        self._tracker: EdgeTAMProcessor | None = None
        self._thread: Thread | None = None
        self._should_stop: Event = Event()
        self._lock = RLock()

        # Use simulator camera intrinsics in simulation mode
        camera_info = self.config.camera_info
        if self.config.g.simulation == "mujoco":
            from dimos.robot.unitree.mujoco_connection import MujocoConnection

            camera_info = MujocoConnection.camera_info_static
        elif self.config.g.simulation == "dimsim":
            from dimos.robot.unitree.dimsim_connection import DimSimConnection

            camera_info = DimSimConnection.camera_info_static

        self._visual_servo = VisualServoing2D(camera_info, bool(self.config.g.simulation))
        self._detection_navigation = DetectionNavigation(self.tf, camera_info)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))
        if self.config.use_3d_navigation:
            self.register_disposable(Disposable(self.global_map.subscribe(self._on_pointcloud)))

    @rpc
    def stop(self) -> None:
        self._stop_following()

        with self._lock:
            if self._tracker is not None:
                self._tracker.stop()
                self._tracker = None

        self._vl_model.stop()
        super().stop()

    @skill
    def follow_person(
        self,
        query: str,
        initial_bbox: list[float] | None = None,
        initial_image: str | None = None,
    ) -> str:
        """Follow a person matching the given description using visual servoing.

        The robot will continuously track and follow the person, while keeping
        them centered in the camera view.

        Args:
            query: Description of the person to follow (e.g., "man with blue shirt")
            initial_bbox: Optional pre-computed bounding box [x1, y1, x2, y2].
                If provided, skips the initial VL model detection step. This is
                used by the continuation system to pass detection data directly
                from look_out_for, avoiding a redundant detection.
            initial_image: Optional base64-encoded JPEG of the frame on which
                initial_bbox was detected.

        Returns:
            Status message indicating the result of the following action.

        Example:
            follow_person("man with blue shirt")
            follow_person("person in the doorway")
        """

        self._stop_following()

        self._should_stop.clear()

        with self._lock:
            latest_image = self._latest_image

        if latest_image is None:
            return "No image available to detect person."

        detection_image: Image | None = None
        if initial_bbox is not None:
            bbox: BBox = (
                initial_bbox[0],
                initial_bbox[1],
                initial_bbox[2],
                initial_bbox[3],
            )
            if initial_image is not None:
                detection_image = _decode_base64_image(initial_image)
        else:
            detected = get_object_bbox_from_image(
                self._vl_model,
                latest_image,
                query,
            )
            if detected is None:
                return f"Could not find '{query}' in the current view."
            bbox = detected

        return self._follow_person(query, bbox, detection_image)

    @skill
    def stop_following(self) -> str:
        """Stop following the current person.

        Returns:
            Confirmation message.
        """
        self._stop_following()

        self.cmd_vel.publish(Twist.zero())

        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None

        return "Stopped following."

    def _on_color_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    def _on_pointcloud(self, pointcloud: PointCloud2) -> None:
        with self._lock:
            self._latest_pointcloud = pointcloud

    def _follow_person(
        self, query: str, initial_bbox: BBox, detection_image: Image | None = None
    ) -> str:
        x1, y1, x2, y2 = initial_bbox
        box = np.array([x1, y1, x2, y2], dtype=np.float32)

        with self._lock:
            if self._tracker is None:
                # Here to prevent unwanted imports in the file.
                from dimos.models.segmentation.edge_tam import EdgeTAMProcessor

                self._tracker = EdgeTAMProcessor()
            tracker = self._tracker
            latest_image = self._latest_image
            if latest_image is None:
                return "No image available to start tracking."

        # Use the detection frame for tracker init when available, so the bbox
        # matches the image it was computed on.
        init_image = detection_image if detection_image is not None else latest_image
        initial_detections = tracker.init_track(
            image=init_image,
            box=box,
            obj_id=1,
        )

        if len(initial_detections) == 0:
            self.cmd_vel.publish(Twist.zero())
            return f"EdgeTAM failed to segment '{query}'."

        logger.info(f"EdgeTAM initialized with {len(initial_detections)} detections")

        self._thread = Thread(target=self._follow_loop, args=(tracker, query), daemon=True)
        self._thread.start()

        message = (
            "Found the person. Starting to follow. You can stop following by calling "
            "the 'stop_following' tool."
        )

        if self._patrolling_module_spec.is_patrolling():
            message += (
                " Note: since the robot was patrolling, this has been stopped automatically "
                "(the equivalent of calling the `stop_patrol` tool call) so you don't have "
                "to do it. "
            )
            self._patrolling_module_spec.stop_patrol()

        return message

    def _follow_loop(self, tracker: "EdgeTAMProcessor", query: str) -> None:
        lost_count = 0
        period = 1.0 / self._frequency
        next_time = time.monotonic()

        while not self._should_stop.is_set():
            next_time += period

            with self._lock:
                latest_image = self._latest_image
                assert latest_image is not None

            detections = tracker.process_image(latest_image)

            if len(detections) == 0:
                self.cmd_vel.publish(Twist.zero())

                lost_count += 1
                if lost_count > self._max_lost_frames:
                    self._send_stop_reason(query, "lost track of the person")
                    return
            else:
                lost_count = 0
                best_detection = max(detections.detections, key=lambda d: d.bbox_2d_volume())

                if self.config.use_3d_navigation:
                    with self._lock:
                        pointcloud = self._latest_pointcloud
                    if pointcloud is None:
                        self._send_stop_reason(query, "no pointcloud available for 3D navigation")
                        return
                    twist = self._detection_navigation.compute_twist_for_detection_3d(
                        pointcloud,
                        best_detection,
                        latest_image,
                    )
                    if twist is None:
                        self._send_stop_reason(query, "3D navigation failed")
                        return
                else:
                    twist = self._visual_servo.compute_twist(
                        best_detection.bbox,
                        latest_image.width,
                    )
                self.cmd_vel.publish(twist)

            now = time.monotonic()
            sleep_duration = next_time - now
            if sleep_duration > 0:
                time.sleep(sleep_duration)

        self._send_stop_reason(query, "it was requested to stop following")

    def _stop_following(self) -> None:
        self._should_stop.set()

    def _send_stop_reason(self, query: str, reason: str) -> None:
        self.cmd_vel.publish(Twist.zero())
        message = f"Person follow stopped for '{query}'. Reason: {reason}."
        self._agent_spec.add_message(HumanMessage(message))
        logger.info("Person follow stopped", query=query, reason=reason)


def _decode_base64_image(b64: str) -> Image:
    bgr_array = TurboJPEG().decode(base64.b64decode(b64))
    return Image(data=bgr_array, format=ImageFormat.BGR)
