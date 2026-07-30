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

"""DimSim-specific continuous visual detection semantics."""

from collections import deque
import json
import os
from typing import Any

import cv2
from dimos_lcm.std_msgs import Bool
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.perceive_loop_skill import (
    PerceiveLoopSkill,
    _write_debug_image,
    logger,
)
from dimos.simulation.dimsim.spatial_memory import DimSimSpatialMemorySpec

_ODOM_HISTORY_SIZE = 2048
_MAX_FRAME_POSE_DELTA_SEC = 0.5
_MIN_VISIBLE_LUMA = 24
_MIN_VISIBLE_PIXEL_FRACTION = 0.3


class DimSimPerceiveLoopSkill(PerceiveLoopSkill):
    """Pass individual object descriptions to the configured detector.

    ``look_out_for`` accepts a list because one lookout can watch for several
    things. Moondream's detection API accepts one object description at a
    time. The upstream implementation serializes the complete Python tuple as
    JSON and sends text such as ``["bathtub"]`` as the object name. That query
    is ambiguous and produced false matches during natural DimSim searches.

    This replacement preserves the existing agent-visible tools and
    notification behavior while adapting the list-valued tool input to the
    detector's scalar query contract.
    """

    odom: In[PoseStamped]
    stop_movement: Out[Bool]

    _spatial_memory: DimSimSpatialMemorySpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._odom_history: deque[PoseStamped] = deque(maxlen=_ODOM_HISTORY_SIZE)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odom.subscribe(self._on_odom)),
        )

    def _on_odom(self, pose: PoseStamped) -> None:
        with self._lock:
            if self._odom_history and pose.ts <= self._odom_history[-1].ts:
                return
            self._odom_history.append(pose)

    def _pose_for_frame(self, frame_ts: float) -> PoseStamped | None:
        with self._lock:
            if not self._odom_history:
                return None
            pose = min(
                self._odom_history,
                key=lambda candidate: abs(candidate.ts - frame_ts),
            )
        if abs(pose.ts - frame_ts) > _MAX_FRAME_POSE_DELTA_SEC:
            return None
        return pose

    def _query_active_lookout(
        self,
        image: Image,
        descriptions: tuple[str, ...],
    ) -> ImageDetections2D[Detection2DBBox]:
        combined: ImageDetections2D[Detection2DBBox] = ImageDetections2D(image)
        for description in descriptions:
            result = self._vl_model.query_detections(image, description)
            combined.detections.extend(result.detections)
        return combined

    @staticmethod
    def _visible_pixel_fraction(image: Image) -> float:
        frame = image.to_opencv()
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if gray.size == 0:
            return 0.0
        visible = np.count_nonzero(gray > _MIN_VISIBLE_LUMA)
        return visible / gray.size

    def _on_image(self, image: Image) -> None:
        with self._lock:
            if not self._active_lookout:
                return
            active_lookout = self._active_lookout
            active_lookout_str = json.dumps(active_lookout)

        # The Go2 camera is mounted ahead of the robot's body collider. When
        # the body stops close to a thin wall, the camera near plane can cross
        # that wall and render a mostly black frame. Open-vocabulary detectors
        # can hallucinate confident boxes on such invalid input. Keep the
        # lookout active and wait for a usable view instead of turning a render
        # artifact into an object discovery.
        visible_fraction = self._visible_pixel_fraction(image)
        if visible_fraction < _MIN_VISIBLE_PIXEL_FRACTION:
            logger.info(
                "Skipping low-visibility lookout frame",
                lookout=active_lookout_str,
                frame_ts=image.ts,
                visible_fraction=visible_fraction,
            )
            return

        detections = self._query_active_lookout(image, active_lookout)
        if not detections:
            return

        capture_pose = self._pose_for_frame(image.ts)
        if os.environ.get("DEBUG"):
            _write_debug_image(image, detections)

        with self._lock:
            if not self._active_lookout:
                return
            if self._lookout_subscription is not None:
                self._lookout_subscription.dispose()
                self._lookout_subscription = None
            self._active_lookout = ()
            then = self._then
            self._then = None
            self._vl_model.stop()
            self._model_started = False

        # The detector can take several seconds while frontier exploration
        # keeps moving. Stop all navigation immediately, then remember the
        # odometry pose belonging to the matched frame rather than the pose at
        # detector completion. A later navigate_with_text call can therefore
        # return to the real observed viewpoint.
        self.stop_movement.publish(Bool(data=True))
        if capture_pose is not None:
            self._spatial_memory.record_detection_viewpoint(
                list(active_lookout),
                capture_pose,
                image.ts,
            )
            logger.info(
                "Recorded lookout capture viewpoint",
                lookout=active_lookout_str,
                frame_ts=image.ts,
                pose=capture_pose,
            )
        else:
            logger.warning(
                "Lookout matched without correlated odometry",
                lookout=active_lookout_str,
                frame_ts=image.ts,
            )

        if then is None:
            self.tool_update(
                "look_out_for",
                f"Found a match for {active_lookout_str}. Please announce audibly.",
            )
            self.stop_tool("look_out_for")
            return

        self.stop_tool("look_out_for")

        best = max(detections.detections, key=lambda detection: detection.bbox_2d_volume())
        continuation_context: dict[str, Any] = {
            "bbox": list(best.bbox),
            "label": best.name,
            "image": image.to_base64(quality=70),
        }
        logger.info(
            "Lookout matched, dispatching continuation",
            lookout=active_lookout_str,
            continuation=then,
            detection=continuation_context,
        )
        self._agent_spec.dispatch_continuation(then, continuation_context)
