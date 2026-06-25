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

"""Marker detection as a :class:`~dimos.memory2.puremodule.PureModule`.

Experimental parallel of :class:`MarkerDetectionStreamModule` — the
original is untouched; this file is the same job re-expressed in the pure
declaration language:

- **Camera pose is an input, not a TF lookup.** The stream module calls
  ``self.tf.get(world, optical, time_point=ts, tolerance=0.5)`` per frame;
  here that is ``camera_pose: In[PoseStamped] = interpolate(tolerance=0.5)``
  — the alignment runtime produces the camera-in-world pose *at the
  frame's capture time*, live or from a recording, and the detection logic
  never knows TF exists.
- **Frame gating is composition, not configuration.** ``QualityWindow`` /
  ``SpeedLimit`` knobs from the stream module's config are not knobs here:
  gate upstream instead, e.g.
  ``module.over(color_image=imgs.transform(QualityWindow(...)), ...)``
  offline, or chain a gating module in front when deployed.
- **Empty frames need no sentinel plumbing.** The stream module threads an
  ``emit_empty_frames`` flag plus ``MarkersPerFrame`` bookkeeping to emit
  one array per processed frame; here ``step`` simply returns the
  (possibly empty) ``Detection3DArray``.

Knowingly out of scope: ``smoothing_window`` pose averaging — that is
recurrent state (per-marker sliding buffers + track ids) and belongs in an
explicit Mealy ``state`` parameter; left out of this experiment to keep
the core comparable. The intrinsics→OpenCV calibration cache kept on the
instance is memoization of config, not behavioral state.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.stream import In, Out
from dimos.memory2.puremodule import PureModule, PureModuleConfig, interpolate, tick
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.imageDetections3D import ImageDetections3D
from dimos.perception.fiducial.marker_detect import detect_markers_in_image
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    camera_optical_frame_id,
    create_aruco_detector,
    is_fisheye_model,
)
from dimos.perception.fiducial.marker_transformer import _camera_info_key
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MarkerDetectionPureModuleConfig(PureModuleConfig):
    """Configuration for :class:`MarkerDetectionPureModule`."""

    world_frame: str = "world"
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    marker_length_m: float = Field(
        ..., gt=0.0, description="Physical square marker edge length in meters."
    )
    camera_info: CameraInfo | None = None


class MarkerDetectionPureModule(PureModule):
    """One marker-detection pass per camera frame, posed via aligned inputs."""

    config: MarkerDetectionPureModuleConfig

    color_image: In[Image] = tick()
    camera_pose: In[PoseStamped] = interpolate(tolerance=0.5)
    """World ← camera-optical pose; interpolated to each frame's capture time."""

    detections: Out[Detection3DArray]

    # Lazy per-intrinsics calibration cache (class defaults: `offline()`
    # constructs instances without running __init__).
    _calibration: tuple[Any, np.ndarray, np.ndarray] | None = None
    _calibration_key: tuple[Any, ...] | None = None
    _warned_distortion: bool = False

    def _resolve_calibration(self, info: CameraInfo) -> tuple[Any, np.ndarray, np.ndarray]:
        key = _camera_info_key(info)
        if key != self._calibration_key or self._calibration is None:
            model = (info.distortion_model or "").strip().lower()
            if model not in ("", "plumb_bob") and not is_fisheye_model(model):
                if not self._warned_distortion:
                    logger.warning(
                        "MarkerDetectionPureModule: distortion_model=%r may be unsupported; "
                        "using D as-is.",
                        info.distortion_model,
                    )
                    self._warned_distortion = True
            camera_matrix, dist_coeffs = camera_info_to_cv_matrices(info)
            detector = create_aruco_detector(self.config.aruco_dictionary)
            self._calibration = (detector, camera_matrix, dist_coeffs)
            self._calibration_key = key
        return self._calibration

    def step(
        self, color_image: Image, camera_pose: PoseStamped, ts: float
    ) -> Detection3DArray | None:
        info = self.config.camera_info
        if info is None:
            logger.debug("MarkerDetectionPureModule: no CameraInfo configured; skipping frame")
            return None
        if (
            info.width
            and info.height
            and (color_image.width != info.width or color_image.height != info.height)
        ):
            logger.debug(
                "MarkerDetectionPureModule: image %sx%s != CameraInfo %sx%s; skipping frame",
                color_image.width,
                color_image.height,
                info.width,
                info.height,
            )
            return None

        detector, camera_matrix, dist_coeffs = self._resolve_calibration(info)
        world_t_optical = Transform(
            translation=Vector3(
                camera_pose.position.x, camera_pose.position.y, camera_pose.position.z
            ),
            rotation=Quaternion(
                camera_pose.orientation.x,
                camera_pose.orientation.y,
                camera_pose.orientation.z,
                camera_pose.orientation.w,
            ),
            frame_id=self.config.world_frame,
            child_frame_id=camera_optical_frame_id(color_image, info),
            ts=ts,
        )

        markers = detect_markers_in_image(
            color_image,
            camera_info=info,
            world_T_optical=world_t_optical,
            marker_length_m=self.config.marker_length_m,
            aruco_dictionary=self.config.aruco_dictionary,
            world_frame=self.config.world_frame,
            detector=detector,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )
        return ImageDetections3D(color_image, markers).to_ros_detection3d_array(
            frame_id=self.config.world_frame
        )
