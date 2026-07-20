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

import time
from typing import Any

import numpy as np
import reactivex as rx
from reactivex import Subject, combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.priors import (
    FiducialPrior,
    LastPosePrior,
    RansacPrior,
    relocalize_with_priors as _relocalize_with_priors,
)
from dimos.mapping.relocalization.relocalize import relocalize as _relocalize
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.apriltag_aggregation import matrix_from_pose7
from dimos.perception.fiducial.fiducial_relocalization import load_marker_map
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

PUBLISH_INTERVAL = 2.0  # for loaded_map + TF
RELOC_INTERVAL = 2.0
MIN_LOCAL_POINTS = 50_000
MAP_SUFFIX = ".pc2.lcm"


class Config(ModuleConfig):
    map_file: str | None = (
        None  # e.g. `-o relocalizationmodule.map_file=go2_hongkong_office_twopass_map`
    )
    publish_loaded_map: bool = False
    fitness_threshold: float = 0.45
    use_carving: bool = True
    # False (default) = today's behavior exactly, via the unchanged relocalize().
    # True = RANSAC + a carried-forward last-pose seed both propose candidates,
    # judged by the same fine-ICP tail (dimos/mapping/relocalization/priors.py).
    use_last_pose_seed: bool = False
    # True = marker sightings on the `detections` In stream (from
    # MarkerDetectionStreamModule) are Huber-fused per tag into one world->map
    # candidate that proposes into the same judge, age-gated. Never bypasses it.
    use_fiducial_prior: bool = False
    # Surveyed marker map (map_T_marker per id), resolve_named_path convention;
    # required for the fiducial prior (start() no-ops the prior without it).
    marker_map_file: str | None = None
    marker_length_m: float = 0.10
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    # IPPE mirror-ambiguity gate: best/runner-up reproj ratio a glimpse must
    # beat. 2.0 provisional; 5.0 was single-recording village3 n=112 -- pending
    # bigger-n. Only bites when corners_px reach the prior (offline harness /
    # an enriched detections wire); today's Detection3DArray drops the pixels,
    # so live the medoid/Huber fusion carries the mirror-flip rejection.
    ambiguity_ratio_min: float = 2.0
    # Static intrinsics for the ambiguity gate's solvePnP; same info the detector
    # holds. The seam a per-deployment camera_info override plugs into later.
    camera_info: CameraInfo | None = None


class RelocalizationModule(Module):
    config: Config
    global_map: In[PointCloud2]
    detections: In[Detection3DArray]  # MarkerDetectionStreamModule's marker sightings
    loaded_map: Out[PointCloud2]
    merged_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._premap: PointCloud2 | None = None
        self._last_skip_log = 0.0
        self._world_to_map: Subject[Transform | None] = Subject()
        self._last_pose_prior = LastPosePrior()
        # Built in start() once the marker map is loaded (needs map_T_marker).
        self._fiducial_prior: FiducialPrior | None = None

    @rpc
    def start(self) -> None:
        super().start()

        if not self.config.map_file:
            logger.info("Relocalization module disabled (no map_file configured)")
            return

        path = resolve_named_path(self.config.map_file, MAP_SUFFIX)
        self._premap = PointCloud2.lcm_decode(path.read_bytes())
        self._premap.frame_id = FRAME_MAP

        self.register_disposable(
            backpressure(
                self.global_map.observable().pipe(  # type: ignore[no-untyped-call]
                    ops.throttle_first(RELOC_INTERVAL),
                    ops.do_action(self._maybe_log_skip),
                    ops.filter(self._has_enough_points),
                )
            )
            .pipe(ops.map(self._try_relocalize))
            .subscribe(self._publish_tf)
        )

        self.register_disposable(
            backpressure(
                combine_latest(
                    self.global_map.observable(),  # type: ignore[no-untyped-call]
                    self._world_to_map.pipe(ops.start_with(None)),
                )
            ).subscribe(self._on_merge_input)
        )

        self.register_disposable(
            rx.interval(PUBLISH_INTERVAL)
            .pipe(ops.with_latest_from(self._world_to_map))
            .subscribe(self._publish_periodic)
        )

        if self.config.use_fiducial_prior:
            self._start_fiducial_prior()

        logger.info(
            f"Relocalization module started: map_file={self.config.map_file!r}  "
            f"loaded_map.frame_id={self._premap.frame_id!r}"
        )

    def _start_fiducial_prior(self) -> None:
        """Load the surveyed marker map, build the FiducialPrior, and route the
        detector's marker sightings into it. No-ops (prior stays off) without a
        marker_map_file, mirroring start()'s no-map_file guard."""
        if not self.config.marker_map_file:
            logger.warning(
                "relocalize: use_fiducial_prior set but no marker_map_file; "
                "fiducial prior disabled"
            )
            return
        marker_map = {
            marker_id: transform.to_matrix()
            for marker_id, transform in load_marker_map(
                resolve_named_path(self.config.marker_map_file, ".yaml")
            ).items()
        }
        self._fiducial_prior = FiducialPrior(
            marker_map,
            camera_info=self.config.camera_info,
            marker_length_m=self.config.marker_length_m,
            ambiguity_ratio_min=self.config.ambiguity_ratio_min,
        )
        self.register_disposable(
            self.detections.observable().subscribe(self._on_detections)  # type: ignore[no-untyped-call]
        )
        logger.info(
            "relocalize: fiducial prior enabled "
            f"marker_map_file={self.config.marker_map_file!r} n_markers={len(marker_map)}"
        )

    def _maybe_log_skip(self, msg: PointCloud2) -> None:
        if self._has_enough_points(msg):
            return
        now = time.monotonic()
        if now - self._last_skip_log > 5.0:
            logger.warning(
                f"relocalize skipped: n_pts={len(msg)} < MIN_LOCAL_POINTS={MIN_LOCAL_POINTS}"
            )
            self._last_skip_log = now

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= MIN_LOCAL_POINTS

    def _publish_tf(self, tf: Transform | None) -> None:
        if tf is None:
            return
        self._world_to_map.on_next(tf)

    def _on_detections(self, msg: Detection3DArray) -> None:
        """Feed each marker sighting's world_T_marker to the fiducial prior. The
        wire Detection3DArray carries marker_id + the world-frame marker pose
        (bbox.center) but NOT corners_px or the camera transform, so the prior
        fuses on min-obs + time window; the pixel-gated ambiguity/reproj/view
        checks activate wherever the corners are available (offline harness)."""
        if self._fiducial_prior is None:
            return
        ts = msg.ts
        for detection in msg.detections[: msg.detections_length]:
            marker_id = self._marker_id_from_detection(detection)
            if marker_id is None:
                continue
            center = detection.bbox.center  # world_T_marker (frame_id == world)
            world_T_marker = matrix_from_pose7(
                (
                    center.position.x,
                    center.position.y,
                    center.position.z,
                    center.orientation.x,
                    center.orientation.y,
                    center.orientation.z,
                    center.orientation.w,
                )
            )
            self._fiducial_prior.observe(marker_id, world_T_marker, ts)

    @staticmethod
    def _marker_id_from_detection(detection: Detection3D) -> int | None:
        """Marker id from a wire Detection3D: the ``id`` field, else the numeric
        tail of a ``DICT:id`` class label (the detector's marker_label encoding)."""
        raw = str(getattr(detection, "id", "")).strip()
        if raw.isdigit():
            return int(raw)
        for result in detection.results[: detection.results_length]:
            class_id = str(result.hypothesis.class_id).strip()
            if ":" in class_id:
                tail = class_id.rsplit(":", 1)[1].strip()
                if tail.isdigit():
                    return int(tail)
        return None

    def _try_relocalize(self, msg: PointCloud2) -> Transform | None:
        assert self._premap is not None
        t0 = time.monotonic()
        winning_source: str | None = None
        extra_priors: list[LastPosePrior | FiducialPrior] = []
        if self.config.use_last_pose_seed:
            extra_priors.append(self._last_pose_prior)
        if self.config.use_fiducial_prior and self._fiducial_prior is not None:
            extra_priors.append(self._fiducial_prior)
        try:
            if extra_priors:
                T, fitness, winning_source = _relocalize_with_priors(
                    self._premap.pointcloud,
                    msg.pointcloud,
                    [RansacPrior(), *extra_priors],
                )
            else:
                T, fitness = _relocalize(self._premap.pointcloud, msg.pointcloud)
        except Exception:
            logger.exception("relocalize() failed")
            return None
        dt = time.monotonic() - t0
        n_pts = len(msg)

        if fitness < self.config.fitness_threshold:
            logger.warning(
                f"relocalize rejected: fitness={fitness:.3f} < threshold={self.config.fitness_threshold} "
                f"time_cost={dt:.1f}s n_pts={n_pts}"
            )
            return None

        if self.config.use_last_pose_seed:
            # Seed the next call's LastPosePrior candidate with THIS T, in
            # relocalize()'s own convention (map_T_world) -- pre-inversion,
            # never the published world->map TF below. See LastPosePrior's
            # docstring in priors.py for the full frame-direction note.
            self._last_pose_prior.update(T)

        # relocalize(scan, map) returns T such that scan_in_map_frame = T(scan_raw).
        # We are publishing a TF for map_in_scan_frame, notice that the base frame is `world`
        # so inverse the transform T here to get map_in_scan_frame
        T_inv = np.linalg.inv(T)
        new_tf = Transform(
            translation=Vector3(*T_inv[:3, 3]),
            rotation=Quaternion.from_rotation_matrix(T_inv[:3, :3]),
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        )
        source_suffix = f" source={winning_source}" if winning_source is not None else ""
        logger.info(
            f"relocalize: fitness={fitness:.3f} time_cost={dt:.1f}s n_pts={n_pts} "
            f"reloc_t={T[:3, 3].round(3).tolist()} "
            f"TF {FRAME_WORLD!r} -> {FRAME_MAP!r} "
            f"published_t={T_inv[:3, 3].round(3).tolist()}"
            f"{source_suffix}"
        )
        return new_tf

    def _publish_periodic(self, pair: tuple[int, Transform]) -> None:
        _, tf = pair
        if self._premap is None:
            return
        if self.config.publish_loaded_map:
            self.loaded_map.publish(self._premap)
        self.tf.publish(tf.now())

    def _on_merge_input(self, pair: tuple[PointCloud2, Transform | None]) -> None:
        local, tf = pair
        if self._premap is None:
            return
        if tf is None:
            # self.merged_map.publish(local)
            # costmap fallbacks to local map, skip publishing
            return
        premap_in_world = self._premap.transform(tf)
        if self.config.use_carving:
            grid = VoxelGrid(carve_columns=True, frame_id=local.frame_id, show_startup_log=False)
            try:
                grid.add_frame(premap_in_world)
                grid.add_frame(local)
                self.merged_map.publish(grid.get_global_pointcloud2())
            finally:
                grid.dispose()
        else:
            self.merged_map.publish(local + premap_in_world)
