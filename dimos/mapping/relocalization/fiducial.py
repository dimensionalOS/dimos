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

from __future__ import annotations

import threading
from typing import Literal

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.mapping.relocalization.prior import PriorConfigBase
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.marker_aggregation import matrix_from_pose7
from dimos.perception.fiducial.marker_map import MARKER_MAP_SUFFIX, load_marker_map
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FiducialPriorConfig(PriorConfigBase):
    """Marker sightings Huber-aggregated into one world->map candidate per tag (``FiducialPrior``). Owns the whole fiducial parameter surface."""

    type: Literal["fiducial"] = "fiducial"
    # Surveyed marker map (map_T_marker per id), a .json path resolved via resolve_named_path; required -- start() no-ops the prior without it.
    marker_map_file: str | None = None


class FiducialPrior:
    """Aggregated fiducial tag poses -> ONE map_T_world candidate per tag, ``map_T_marker @ inv(world_T_marker_aggregated)`` (local_map->global_map, as RANSAC)."""

    name = "fiducial"

    def __init__(self, marker_map: dict[int, np.ndarray]) -> None:
        # marker_id -> map_T_marker (4x4); the surveyed marker map.
        self._map_T_marker = marker_map
        # marker_id -> map_T_world (4x4) awaiting its ONE trip past the judge.
        self._pending: dict[int, np.ndarray] = {}
        self._pending_lock = threading.Lock()

    @classmethod
    def from_config(cls, config: FiducialPriorConfig) -> FiducialPrior | None:
        """Load the surveyed marker map this prior composes against; ``None`` when the config names none."""
        # A constructor cannot decline to exist, and without a map every candidate would be dropped at observe(); None keeps the prior out of the live list and off the detections stream.
        if not config.marker_map_file:
            logger.warning(
                "relocalize: fiducial prior enabled but no marker_map_file; fiducial prior disabled"
            )
            return None
        marker_map_path = resolve_named_path(config.marker_map_file, MARKER_MAP_SUFFIX)
        marker_map = {
            marker_id: transform.to_matrix()
            for marker_id, transform in load_marker_map(marker_map_path).items()
        }
        logger.info(
            "fiducial prior enabled",
            marker_map_file=config.marker_map_file,
            n_markers=len(marker_map),
        )
        return cls(marker_map)

    def observe_detections(self, msg: Detection3DArray) -> None:
        """Compose every aggregated tag pose in this burst into that tag's world->map fix."""
        for detection in msg.detections[: msg.detections_length]:
            # the same parse MarkerTfModule publishes its marker TF from
            raw_id = MarkerTfModule._marker_id_from_detection(detection)
            if raw_id is None or not raw_id.isdigit():
                continue
            center = detection.bbox.center  # world_T_marker_aggregated (frame_id == world)
            self.observe(
                int(raw_id),
                matrix_from_pose7(
                    (
                        center.position.x,
                        center.position.y,
                        center.position.z,
                        center.orientation.x,
                        center.orientation.y,
                        center.orientation.z,
                        center.orientation.w,
                    )
                ),
            )

    def observe(self, marker_id: int, world_T_marker_aggregated: np.ndarray) -> None:
        """Compose this tag's map_T_world fix from one aggregated pose."""
        map_T_marker = self._map_T_marker.get(marker_id)
        if map_T_marker is None:
            return
        with self._pending_lock:
            self._pending[marker_id] = map_T_marker @ np.linalg.inv(world_T_marker_aggregated)

    @property
    def has_pending(self) -> bool:
        """A composed fix is waiting for the judge -- the module's fire signal."""
        return bool(self._pending)

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[np.ndarray]:
        # consume on use (a re-offered fix scores worse, world has drifted); locked against observe()'s read-modify-write on another thread
        with self._pending_lock:
            pending, self._pending = self._pending, {}
        return list(pending.values())
