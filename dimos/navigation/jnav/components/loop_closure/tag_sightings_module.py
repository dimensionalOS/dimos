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

"""TagSightings: AprilTag detections in, clustered landmark sightings out (GSC's `landmark_factor`).

Clustering lives here, not in GSC: `SightingClusterer` (apriltags.py, also behind the offline sightings CSV) groups a
tag's detections within the cooldown into one sighting, compares them in the odom frame, and publishes the sighting's
medoid once enough detections agree with it (Huber outlier test). Each publish carries a constraint_instance_id per
sighting; when the medoid moves the sighting is republished under the same id and GSC replaces its factor.
"""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.navigation.jnav.msgs.LocationConstraint import LocationConstraint
from dimos.navigation.jnav.utils.tag_sighting_clusterer import (
    STABLE_POSE_KEY,
    Detection,
    SightingClusterer,
    SightingClustering,
    SightingUpdate,
)
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def pose_matrix(pose: list[float]) -> np.ndarray:
    """4x4 from xyz + xyzw quaternion."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(pose[3:7]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def matrix_pose(matrix: np.ndarray) -> list[float]:
    """xyz + xyzw quaternion from a 4x4."""
    return [*matrix[:3, 3].tolist(), *Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()]


class TagSightingsConfig(ModuleConfig):
    # the frame the detections' tag poses are in (the marker detector's world_frame), which only drifts slowly
    odom_frame: str = "odom"
    # each sighting is published relative to this frame (GSC's body frame, so GSC needs no mount lookup)
    body_frame: str = "base_link"
    # camera optical frame for the distance / view-angle gates ("" turns them off)
    camera_frame: str = "camera_optical"
    # how far from a detection's time a tf lookup may land (the camera and tf streams are not in lockstep)
    tf_tolerance_s: float = 0.5
    # landmark name = prefix + marker id; markers not fixed in the world are left out
    landmark_prefix: str = "apriltag/"
    exclude_markers: list[int] = Field(default_factory=list)
    # per-axis sd of a published sighting; a non-positive rotation sd leaves the rotation unknown (a point sighting)
    position_sd_m: float = 0.05
    rotation_sd_rad: float = -1.0
    clustering: SightingClustering = Field(default_factory=SightingClustering)


class TagSightings(Module):
    """Cluster AprilTag detections into landmark sightings and publish (and supersede) them for GSC."""

    config: TagSightingsConfig

    detections: In[Detection3DArray]
    tf: In[TFMessage]
    landmark_factor: Out[LocationConstraint]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._clusterer = SightingClusterer(self.config.clustering)

    def detection(self, marker_id: int, ts: float, odom_marker: list[float]) -> Detection | None:
        """A clusterer detection (odom-frame pose, body- and camera-frame poses at `ts`), None without tf."""
        config = self.config

        def lookup(frame: str) -> Any:
            return self.tfbuffer.get(
                frame, config.odom_frame, time_point=ts, time_tolerance=config.tf_tolerance_s
            )

        body_from_odom = lookup(config.body_frame)
        if body_from_odom is None:
            return None
        detection: Detection = {
            "ts": ts,
            "marker_id": marker_id,
            STABLE_POSE_KEY: odom_marker,
            "t_body_marker": matrix_pose(body_from_odom.to_matrix() @ pose_matrix(odom_marker)),
        }
        if config.camera_frame and (camera_from_odom := lookup(config.camera_frame)) is not None:
            detection["t_cam_marker"] = matrix_pose(
                camera_from_odom.to_matrix() @ pose_matrix(odom_marker)
            )
        return detection

    def constraint(self, update: SightingUpdate) -> LocationConstraint:
        """The LocationConstraint for a sighting update; its instance id names the sighting, so a republish replaces it."""
        config = self.config
        medoid = update.medoid
        to_id = f"{config.landmark_prefix}{medoid['marker_id']}"
        x, y, z, qx, qy, qz, qw = medoid["t_body_marker"]
        rotation_variance = config.rotation_sd_rad**2 if config.rotation_sd_rad > 0 else -1.0
        diagonal = [rotation_variance] * 3 + [config.position_sd_m**2] * 3
        covariance = [diagonal[i] if i == j else 0.0 for i in range(6) for j in range(6)]
        return LocationConstraint(
            to_id=to_id,
            frame_id=config.body_frame,
            pose=Pose(position=Vector3(x, y, z), orientation=Quaternion(qx, qy, qz, qw)),
            covariance=covariance,
            constraint_instance_id=f"{to_id}#{update.sighting_id}",
            kind="apriltag",
            ts=float(medoid["ts"]),
        )

    def _on_detections(self, detections: Detection3DArray) -> None:
        for marker in detections.detections[: detections.detections_length]:
            marker_id = MarkerTfModule._marker_id_from_detection(marker)
            if marker_id is None or not marker_id.lstrip("-").isdigit():
                continue
            if int(marker_id) in self.config.exclude_markers:
                continue
            center = marker.bbox.center
            odom_marker = [
                center.position.x,
                center.position.y,
                center.position.z,
                center.orientation.x,
                center.orientation.y,
                center.orientation.z,
                center.orientation.w,
            ]
            detection = self.detection(int(marker_id), float(detections.ts), odom_marker)
            if detection is None:
                logger.debug(
                    "TagSightings: no tf %s <- %s", self.config.body_frame, self.config.odom_frame
                )
                continue
            update = self._clusterer.add(detection)
            if update is not None:
                self.landmark_factor.publish(self.constraint(update))

    @rpc
    def start(self) -> None:
        super().start()
        logger.info("TagSightings: %s", asdict(self.config.clustering))
        unsubscribe = self.detections.subscribe(self._on_detections)
        self.register_disposable(Disposable(unsubscribe) if callable(unsubscribe) else unsubscribe)

    @rpc
    def stop(self) -> None:
        super().stop()
