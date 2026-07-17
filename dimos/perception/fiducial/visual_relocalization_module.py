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

"""A camera swapped in for lidar; publishes ``world -> map``, mirroring RelocalizationModule."""

from __future__ import annotations

from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.fiducial.marker_pose import create_aruco_detector
from dimos.perception.fiducial.visual_relocalization import (
    MAP_FRAME,
    OPTICAL_FRAME,
    LocalizationConfig,
    detect_markers,
    load_marker_map,
    localize_from_detections,
)
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

WORLD_FRAME = "world"  # mirrors RelocalizationModule.FRAME_WORLD


class VisualRelocalizationModuleConfig(ModuleConfig):
    marker_map_file: str | None = None  # via `resolve_named_path`, RelocalizationModule convention
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    marker_length_m: float = Field(..., gt=0.0)
    min_tags: int = Field(1, ge=1)  # tags that must clear the gate before a fix is trusted
    max_reprojection_error_px: float = Field(3.0, gt=0.0)
    # best-vs-runner-up IPPE candidate reprojection ratio below which a tag's
    # view is mirror-ambiguous and rejected; 1.0 disables (see LocalizationConfig)
    ambiguity_ratio_min: float = Field(2.0, ge=1.0)
    camera_info: CameraInfo | None = None  # static, MarkerDetectionStreamModule convention
    camera_optical_frame: str = OPTICAL_FRAME
    # target frame for the published correction TF (world -> map_frame) and the pose output's
    # frame_id; unchanged default keeps this module's publish identical to RelocalizationModule's
    # world -> map. Override (e.g. "map_marker") to run this module in shadow mode alongside
    # RelocalizationModule (which owns "map") for side-by-side evaluation of the two correctors.
    map_frame: str = MAP_FRAME


class VisualRelocalizationModule(Module):
    config: VisualRelocalizationModuleConfig
    color_image: In[Image]
    # The same world->map fix that goes to TF, as a typed stream — so
    # RelocalizationModule's FiducialPrior (`use_fiducial_prior`) can consume
    # it by name/type autoconnect without scraping the TF tree.
    world_map_fix: Out[Transform]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._marker_map: dict[int, Transform] = {}
        self._detector = create_aruco_detector(self.config.aruco_dictionary)
        c = self.config
        self._cfg = LocalizationConfig(
            c.marker_length_m,
            min_tags=c.min_tags,
            max_reprojection_error_px=c.max_reprojection_error_px,
            ambiguity_ratio_min=c.ambiguity_ratio_min,
        )

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.marker_map_file:
            return
        self._marker_map = load_marker_map(resolve_named_path(self.config.marker_map_file, ".yaml"))

    async def handle_color_image(self, msg: Image) -> None:
        info = self.config.camera_info
        if not self._marker_map or info is None:
            return
        detections = detect_markers(msg.to_grayscale().as_numpy(), self._detector)
        if not detections:
            return  # no tags in view: the normal case, not worth logging
        pose = localize_from_detections(detections, self._marker_map, info, self._cfg, msg.ts)
        if pose is None:
            logger.warning(
                f"VisualRelocalizationModule: gate rejected ({len(detections)} tags seen)"
            )
            return
        pose.frame_id = self.config.map_frame  # retarget map_T_optical before inverting/publishing
        optical = self.config.camera_optical_frame
        if (world_T_optical := self.tf.get(WORLD_FRAME, optical, time_point=msg.ts)) is None:
            logger.warning(
                f"VisualRelocalizationModule: no TF {WORLD_FRAME} -> {optical} at {msg.ts}; "
                "check camera_optical_frame against the camera's static TF chain"
            )
            return
        fix = (world_T_optical + pose.inverse()).now()
        self.tf.publish(fix)
        self.world_map_fix.publish(fix)
