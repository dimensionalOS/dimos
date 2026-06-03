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

"""TF publication for fiducial marker detections.

Publishes ``world -> markers`` (identity) and ``markers -> marker_{id}`` from
incoming ``Detection3DArray`` messages. Marker pose estimation is owned by the
marker detector; this module only mirrors detection bbox poses into TF.

Compose with a marker detector via matching ``detections_3d`` streams::

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.perception.fiducial.marker_module import MarkerModule
    from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
    from dimos.robot.unitree.go2.connection import GO2Connection

    unitree_go2_with_markers = autoconnect(
        unitree_go2_basic,
        MarkerModule.blueprint(
            marker_length_m=0.18,
            camera_info=GO2Connection.camera_info_static,
        ),
        MarkerTfModule.blueprint(),
    )
"""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MarkerTfModuleConfig(ModuleConfig):
    """TF naming configuration for :class:`MarkerTfModule`."""

    world_frame: str = "world"
    markers_frame: str = "markers"
    marker_namespace_prefix: str | None = None


class MarkerTfModule(Module):
    """Subscribe to marker detections and publish marker poses on ``self.tf``."""

    config: MarkerTfModuleConfig

    detections_3d: In[Detection3DArray]

    def _markers_parent_frame(self) -> str:
        p = self.config.marker_namespace_prefix
        base = self.config.markers_frame
        return f"{p}/{base}" if p else base

    def _marker_child_frame(self, marker_id: str) -> str:
        p = self.config.marker_namespace_prefix
        name = f"marker_{marker_id}"
        return f"{p}/{name}" if p else name

    def _process_detections(self, detections: Detection3DArray) -> None:
        if detections.detections_length == 0:
            return

        marker_detections = detections.detections[: detections.detections_length]
        if not marker_detections:
            return

        markers_parent = self._markers_parent_frame()
        ts = detections.ts
        out: list[Transform] = [
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id=self.config.world_frame,
                child_frame_id=markers_parent,
                ts=ts,
            )
        ]

        for detection in marker_detections:
            marker_id = self._marker_id_from_detection(detection)
            if marker_id is None:
                logger.debug("MarkerTfModule: skipping detection without marker id")
                continue

            pose = detection.bbox.center
            out.append(
                Transform(
                    translation=pose.position,
                    rotation=pose.orientation,
                    frame_id=markers_parent,
                    child_frame_id=self._marker_child_frame(marker_id),
                    ts=ts,
                )
            )

        # A non-empty array can still contain no usable marker identities.
        # In that case, skip TF entirely rather than publishing only the
        # namespace anchor without any marker child frames.
        if len(out) > 1:
            self.tf.publish(*out)

    @staticmethod
    def _marker_id_from_detection(detection: Detection3D) -> str | None:
        marker_id = str(getattr(detection, "id", "")).strip()
        if marker_id:
            return marker_id

        for result in detection.results[: detection.results_length]:
            class_id = str(result.hypothesis.class_id).strip()
            if ":" not in class_id:
                continue
            parsed = class_id.rsplit(":", 1)[1].strip()
            if parsed:
                return parsed
        return None

    @rpc
    def start(self) -> None:
        super().start()
        unsub = self.detections_3d.subscribe(self._process_detections)
        self.register_disposable(Disposable(unsub) if callable(unsub) else unsub)

    @rpc
    def stop(self) -> None:
        super().stop()
