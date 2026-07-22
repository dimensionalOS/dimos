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
from pydantic import Field
import reactivex as rx
from reactivex import Subject, combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.priors import (
    FiducialPrior,
    FiducialPriorConfig,
    LastPosePrior,
    LastPosePriorConfig,
    PriorConfig,
    RansacPrior,
    RansacPriorConfig,
    RelocPrior,
    relocalize_with_priors as _relocalize_with_priors,
)
from dimos.mapping.relocalization.relocalize import relocalize as _relocalize
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
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

PUBLISH_INTERVAL_S = 2.0  # s; for loaded_map + TF
MAP_SUFFIX = ".pc2.lcm"
SKIP_LOG_INTERVAL_S = 5.0  # s; throttle relocalize-skip warnings so a starved feed can't spam


class Config(ModuleConfig):
    map_file: str | None = (
        None  # e.g. `-o relocalizationmodule.map_file=go2_hongkong_office_twopass_map`
    )
    # Operator -o override for the fiducial marker map, e.g.
    # `-o relocalizationmodule.marker_map_file=/abs/site_markers.json`. A dotted -o
    # can't index the priors list, and a partial override would drop the fiducial
    # entry's discriminator; this sibling field wins over the blueprint entry in
    # _start_fiducial_prior. None -> use the entry's own.
    marker_map_file: str | None = None
    publish_loaded_map: bool = False
    # --- shared judge params: apply to whichever priors are enabled below ---
    fitness_threshold: float = Field(default=0.45, ge=0.0, le=1.0)  # min wall fitness to accept
    # Min local-map points (post VoxelGridMapper) before a solve; below this the
    # wall-only rerank has too little geometry, so the frame is skipped (throttled log).
    min_local_points: int = Field(default=50_000, ge=0)
    # Seconds between attempts (throttle_first on global_map). One solve costs
    # ~1-2 s, so this bounds CPU without starving fresh fixes.
    reloc_interval_s: float = Field(default=2.0, gt=0.0)
    # Max z-axis tilt (deg) a candidate may keep at the judge's gravity gate;
    # matches relocalize.py's GRAVITY_TILT_MAX_DEG, so unchanged unless overridden.
    gravity_tilt_max_deg: float = Field(default=10.0, ge=0.0)
    use_carving: bool = True
    # The prior pool: each entry an EQUAL, toggleable candidate proposer. REQUIRED,
    # no default -- a blueprint (or -o) must state it, so there is no silent
    # module-level reloc behavior. RANSAC is a first-class toggleable entry, not
    # always-on; the fiducial-only preset omits it.
    priors: list[PriorConfig]


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
                    ops.throttle_first(self.config.reloc_interval_s),
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
            rx.interval(PUBLISH_INTERVAL_S)
            .pipe(ops.with_latest_from(self._world_to_map))
            .subscribe(self._publish_periodic)
        )

        self._start_fiducial_prior()

        logger.info(
            "relocalization module started",
            map_file=self.config.map_file,
            loaded_map_frame_id=self._premap.frame_id,
        )

    def _fiducial_config(self) -> FiducialPriorConfig | None:
        """The first enabled fiducial entry, or None -- one detector feeds one prior."""
        for prior_config in self.config.priors:
            if isinstance(prior_config, FiducialPriorConfig) and prior_config.enabled:
                return prior_config
        return None

    def _start_fiducial_prior(self) -> None:
        """Load the marker map, build the FiducialPrior, route sightings into it.
        No-ops with no enabled fiducial prior, or (enabled) without a
        marker_map_file -- mirroring start()'s no-map_file guard."""
        fiducial = self._fiducial_config()
        if fiducial is None:
            return
        # -o override wins over the blueprint entry; falls back to the entry's own.
        marker_map_file = self.config.marker_map_file or fiducial.marker_map_file
        if not marker_map_file:
            logger.warning(
                "relocalize: fiducial prior enabled but no marker_map_file; fiducial prior disabled"
            )
            return
        marker_map = {
            marker_id: transform.to_matrix()
            for marker_id, transform in load_marker_map(
                resolve_named_path(marker_map_file, ".json")
            ).items()
        }
        self._fiducial_prior = FiducialPrior(
            marker_map,
            camera_info=fiducial.camera_info,
            marker_length_m=fiducial.marker_length_m,
            ambiguity_ratio_min=fiducial.ambiguity_ratio_min,
            config=fiducial.aggregation,
            age_max_s=fiducial.age_max_s,
        )
        self.register_disposable(
            self.detections.observable().subscribe(self._on_detections)  # type: ignore[no-untyped-call]
        )
        logger.info(
            "fiducial prior enabled",
            marker_map_file=marker_map_file,
            n_markers=len(marker_map),
        )

    def _maybe_log_skip(self, msg: PointCloud2) -> None:
        if self._has_enough_points(msg):
            return
        now = time.monotonic()
        if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
            logger.warning(
                "relocalize skipped",
                n_pts=len(msg),
                min_local_points=self.config.min_local_points,
            )
            self._last_skip_log = now

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= self.config.min_local_points

    def _publish_tf(self, tf: Transform | None) -> None:
        if tf is None:
            return
        self._world_to_map.on_next(tf)

    def _on_detections(self, msg: Detection3DArray) -> None:
        """Feed each sighting's world_T_marker to the fiducial prior. The wire
        Detection3DArray carries marker_id + world-frame pose (bbox.center) but NOT
        corners_px or the camera transform, so the prior fuses on min-obs + time
        window; the pixel-gated checks activate only where corners exist (offline)."""
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

    def _enabled_prior_objects(self) -> list[RelocPrior]:
        """The candidate PROPOSERS for enabled entries, in list order -- candidate
        order (and the judge's tie-break) follows the config. An enabled fiducial
        entry start() couldn't build (no marker map) contributes nothing."""
        objects: list[RelocPrior] = []
        for prior_config in self.config.priors:
            if not prior_config.enabled:
                continue
            if isinstance(prior_config, RansacPriorConfig):
                objects.append(RansacPrior())
            elif isinstance(prior_config, LastPosePriorConfig):
                objects.append(self._last_pose_prior)
            elif isinstance(prior_config, FiducialPriorConfig) and self._fiducial_prior is not None:
                objects.append(self._fiducial_prior)
        return objects

    def _try_relocalize(self, msg: PointCloud2) -> Transform | None:
        if self._premap is None:
            raise RuntimeError(
                "_try_relocalize before start() loaded premap; call start() with map_file set"
            )
        priors = self._enabled_prior_objects()
        if not priors:
            return None  # every prior disabled (or fiducial-only with no marker map yet)
        t0 = time.monotonic()
        winning_source: str | None = None
        last_pose_active = any(prior is self._last_pose_prior for prior in priors)
        try:
            # RANSAC alone == today's default: single-source relocalize(), accept
            # log carries no source= tag. Any richer pool goes through the
            # multi-source judge, which reports the winning source.
            if len(priors) == 1 and isinstance(priors[0], RansacPrior):
                T, fitness = _relocalize(
                    self._premap.pointcloud,
                    msg.pointcloud,
                    gravity_tilt_max_deg=self.config.gravity_tilt_max_deg,
                )
            else:
                T, fitness, winning_source = _relocalize_with_priors(
                    self._premap.pointcloud,
                    msg.pointcloud,
                    priors,
                    gravity_tilt_max_deg=self.config.gravity_tilt_max_deg,
                )
        except Exception:
            logger.exception("relocalize() failed")
            return None
        dt = time.monotonic() - t0
        n_pts = len(msg)

        if fitness < self.config.fitness_threshold:
            logger.warning(
                "relocalize rejected",
                fitness=round(fitness, 3),
                threshold=self.config.fitness_threshold,
                time_cost_s=round(dt, 1),
                n_pts=n_pts,
            )
            return None

        if last_pose_active:
            # Seed next call's LastPosePrior with THIS T in relocalize()'s own
            # convention (map_T_world) -- pre-inversion, never the published TF
            # below. See LastPosePrior's docstring in priors.py.
            self._last_pose_prior.update(T)

        # relocalize() returns map_T_world (scan_in_map = T @ scan_raw). We publish
        # the world->map TF, so invert to world_T_map here.
        T_inv = np.linalg.inv(T)
        new_tf = Transform(
            translation=Vector3(*T_inv[:3, 3]),
            rotation=Quaternion.from_rotation_matrix(T_inv[:3, :3]),
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        )
        # source= present only when a prior won the judge; absent = single-source path.
        source_kw: dict[str, str] = {} if winning_source is None else {"source": winning_source}
        logger.info(
            "relocalize accepted",
            **source_kw,
            fitness=round(fitness, 3),
            time_cost_s=round(dt, 1),
            n_pts=n_pts,
            reloc_t_m=T[:3, 3].round(3).tolist(),
            tf_from=FRAME_WORLD,
            tf_to=FRAME_MAP,
            published_t_m=T_inv[:3, 3].round(3).tolist(),
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
