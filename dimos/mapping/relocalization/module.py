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

from collections.abc import Callable
from pathlib import Path
import time
from typing import Any

import numpy as np
from pydantic import Field
import reactivex as rx
from reactivex import Subject, combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.eval import SourceTally, format_eval_summary
from dimos.mapping.relocalization.priors import (
    EmptyProposalError,
    FiducialPrior,
    FiducialPriorConfig,
    PriorConfig,
    RansacPrior,
    RansacPriorConfig,
    RelocPrior,
    load_marker_map,
    relocalize_with_priors,
)
from dimos.mapping.relocalization.relocalize import (
    InsufficientWallEvidenceError,
    NoUprightCandidateError,
    relocalize as _relocalize,
)
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.apriltag_aggregation import matrix_from_pose7
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

PUBLISH_INTERVAL = 2.0  # for loaded_map + TF
MAP_SUFFIX = ".pc2.lcm"
# Suffixes load_marker_map parses; a bare name gets the .json default (see _start_fiducial_prior).
MARKER_MAP_SUFFIXES = (".json", ".yaml", ".yml")
SKIP_LOG_INTERVAL_S = 5.0  # s; throttle relocalize-skip warnings so a starved feed can't spam
# Jump guard, TRACKING only: largest per-second step from the previous accepted fix. T is world->map, so between accepts it moves only by the drift the fix corrects; this refuses the mirror-flipped fix that lands a room over. Floored at one second's worth below so back-to-back priors keep a real budget.
MAX_JUMP_M_PER_S = 5.0  # m/s; 10 m at the 2 s RANSAC interval
MAX_JUMP_YAW_DEG_PER_S = 45.0  # deg/s; 90 deg at the 2 s RANSAC interval


class Config(ModuleConfig):
    map_file: str | None = (
        None  # e.g. `-o relocalizationmodule.map_file=go2_hongkong_office_twopass_map`
    )
    # Operator -o override for the fiducial marker map (a dotted -o can't index the priors list); wins over the blueprint entry in _start_fiducial_prior. None -> use the entry's own.
    marker_map_file: str | None = None
    publish_loaded_map: bool = False
    # Max z-axis tilt (deg) a candidate may keep at the judge's gravity gate; matches relocalize.py's GRAVITY_TILT_MAX_DEG, so unchanged unless overridden.
    gravity_tilt_max_deg: float = Field(default=10.0, ge=0.0)
    use_carving: bool = True
    # False (default): one line per accepted fix plus throttled warnings. True (`--eval`): each accept also logs its published pose, timing and point count.
    verbose_eval_logging: bool = False
    # The prior pool: each entry a toggleable candidate proposer (RANSAC polled, fiducial event-driven). REQUIRED, no default -- a blueprint must state it, so there is no silent reloc behavior.
    priors: list[PriorConfig]


class RelocalizationModule(Module):
    config: Config
    global_map: In[PointCloud2]
    aggregated_detections: In[Detection3DArray]  # the detector's per-burst aggregated tag poses
    loaded_map: Out[PointCloud2]
    merged_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._premap: PointCloud2 | None = None
        self._last_skip_log = 0.0
        # Per-source accept/reject tally, filled only under verbose_eval_logging (--eval) and rendered once at stop(). The module already owns the accept/reject data.
        self._eval_tally: dict[str, SourceTally] = {}
        self._world_to_map: Subject[Transform | None] = Subject()
        # Prior objects are built ONCE, not per frame: the fiducial holds pending-fix state across bursts that a fresh instance would reset. The RANSAC prior is a pure source; the module owns its poll timer (below).
        ransac_entry = next(
            (p for p in self.config.priors if isinstance(p, RansacPriorConfig)),
            RansacPriorConfig(),
        )
        self._ransac_prior = RansacPrior()
        # Per-source accept gate {Candidate.source: min wall fitness}. Each fire pools one source, so gating the winner by its own source's bar IS per-source. Key = the entry's `type` discriminator, which equals the source its prior proposes.
        self._accept_threshold: dict[str, float] = {
            p.type: p.fitness_threshold for p in self.config.priors
        }
        # RANSAC poll timer (the prior is a pure source): its interval and last fire. None == never fired, so the first dense-enough frame relocalizes immediately.
        self._ransac_interval_s = ransac_entry.interval_s
        self._last_ransac_fired_s: float | None = None
        # RANSAC-only point floor (a fiducial burst fires regardless); read from the ransac entry, or the default when no ransac entry is enabled (then unused).
        self._ransac_min_local_points = ransac_entry.min_local_points
        # Built in start() once the marker map is loaded (needs map_T_marker).
        self._fiducial_prior: FiducialPrior | None = None
        # Previous ACCEPTED fix (map_T_world) and when, for the gross-jump guard. None until the first accept, which is what leaves acquisition unguarded.
        self._last_fix_map_T_world: np.ndarray | None = None
        self._last_fix_ts_s = 0.0
        # Latest global_map, so a completed tag burst is judged the instant it lands. Sound because the stream ACCUMULATES in the world frame: the previous cloud scores wall fitness as well as the newest, and the tag candidate needs no lidar at all.
        self._last_local_map: PointCloud2 | None = None
        # One monotonic timebase for every prior trigger, injectable so a test can drive the triggers without sleeping (FiducialPrior takes the same seam).
        self._now_fn: Callable[[], float] = time.monotonic

    @rpc
    def start(self) -> None:
        super().start()

        if not self.config.map_file:
            logger.info("Relocalization module disabled (no map_file configured)")
            return

        path = resolve_named_path(self.config.map_file, MAP_SUFFIX)
        self._premap = PointCloud2.lcm_decode(path.read_bytes())
        self._premap.frame_id = FRAME_MAP

        # No throttle here: the priors own the cadence. EVERY frame reaches _on_local_map, which asks each prior whether it wants a fire. Trigger accounting runs on the backpressure worker thread only -- a frame backpressure coalesces away never consumes a trigger, so a burst edge cannot be dropped by a slow solve.
        self.register_disposable(
            backpressure(
                self.global_map.observable()  # type: ignore[no-untyped-call]
            ).subscribe(self._on_local_map)
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

        self._start_fiducial_prior()

        logger.info(
            f"Relocalization module started: map_file={self.config.map_file!r}  "
            f"loaded_map.frame_id={self._premap.frame_id!r}"
        )

    @rpc
    def stop(self) -> None:
        # Emit the per-source accept/reject table once, at shutdown, from the tally the fire path filled -- the module owns the data, so no log parsing.
        if self.config.verbose_eval_logging and self._eval_tally:
            logger.info("relocalize eval summary", table=format_eval_summary(self._eval_tally))
        super().stop()

    def _fiducial_config(self) -> FiducialPriorConfig | None:
        """The first enabled fiducial entry, or None -- one detector feeds one prior."""
        for prior_config in self.config.priors:
            if isinstance(prior_config, FiducialPriorConfig) and prior_config.enabled:
                return prior_config
        return None

    def _start_fiducial_prior(self) -> None:
        """Load the marker map, build the FiducialPrior, route sightings into it."""
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
        # A bare name gets the .json default; a name that already states its format keeps it (resolve_named_path would otherwise look up "<survey>.yaml.json", which nothing writes).
        suffix = "" if Path(marker_map_file).suffix in MARKER_MAP_SUFFIXES else ".json"
        marker_map = {
            marker_id: transform.to_matrix()
            for marker_id, transform in load_marker_map(
                resolve_named_path(marker_map_file, suffix)
            ).items()
        }
        self._fiducial_prior = FiducialPrior(marker_map)
        self.register_disposable(
            self.aggregated_detections.observable().subscribe(self._on_aggregated_detections)  # type: ignore[no-untyped-call]
        )
        logger.info(
            "fiducial prior enabled",
            marker_map_file=marker_map_file,
            n_markers=len(marker_map),
        )

    def _maybe_log_skip(self, msg: PointCloud2) -> None:
        """Throttled warning that a sparse submap starved the RANSAC search (RANSAC-only)."""
        if self._has_enough_points(msg):
            return
        now = time.monotonic()
        if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
            logger.warning(
                f"relocalize skipped: n_pts={len(msg)} < "
                f"min_local_points={self._ransac_min_local_points}"
            )
            self._last_skip_log = now

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= self._ransac_min_local_points

    def _maybe_log_wall_skip(self, n_pts: int, reason: str) -> None:
        """Throttled warning that the judge refused a fire for too little wall evidence; shares the RANSAC-skip throttle timer -- both are 'cloud too sparse this fire'."""
        now = time.monotonic()
        if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
            logger.warning(
                "relocalize skipped: insufficient wall evidence", n_pts=n_pts, reason=reason
            )
            self._last_skip_log = now

    def _publish_tf(self, tf: Transform | None) -> None:
        if tf is None:
            return
        self._world_to_map.on_next(tf)

    def _on_aggregated_detections(self, msg: Detection3DArray) -> None:
        """Compose each aggregated tag pose into this tag's world->map fix, then fire."""
        if self._fiducial_prior is None:
            return
        for detection in msg.detections[: msg.detections_length]:
            marker_id = self._marker_id_from_detection(detection)
            if marker_id is None:
                continue
            center = detection.bbox.center  # world_T_marker_aggregated (frame_id == world)
            world_T_marker_aggregated = matrix_from_pose7(
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
            self._fiducial_prior.observe(marker_id, world_T_marker_aggregated)
        # Fire HERE, not at the next cloud: the tag candidate is composed from the marker alone, so waiting on lidar is dead time on acquisition. No cloud yet -> pending stays and _on_local_map fires it on the first frame.
        cloud = self._last_local_map
        if cloud is not None and self._fiducial_prior.has_pending:
            self._fire(self._fiducial_prior, cloud)

    def _fire(self, prior: RelocPrior, local_map: PointCloud2) -> None:
        """Run this prior's ONE relocalization and publish the resulting TF."""
        self._publish_tf(self._try_relocalize(local_map, [prior]))

    @staticmethod
    def _marker_id_from_detection(detection: Detection3D) -> int | None:
        """Marker id from a wire Detection3D: the ``id`` field, else the numeric tail of a ``DICT:id`` class label (the detector's marker_label encoding)."""
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
        """The candidate PROPOSERS for enabled entries, in list order (the judge's tie-break)."""
        objects: list[RelocPrior] = []
        for prior_config in self.config.priors:
            if not prior_config.enabled:
                continue
            if isinstance(prior_config, RansacPriorConfig):
                objects.append(self._ransac_prior)
            elif isinstance(prior_config, FiducialPriorConfig) and self._fiducial_prior is not None:
                objects.append(self._fiducial_prior)
        return objects

    def _on_local_map(self, msg: PointCloud2) -> None:
        """Poll the RANSAC prior on the module's timer; on cold start fire a pending fiducial fix."""
        self._last_local_map = msg  # what a burst-triggered fire judges against
        now = self._now_fn()
        # _ransac_prior is always constructed, so gate it on the enabled set (the toggle); _fiducial_prior is None unless its enabled entry loaded a marker map.
        enabled = self._enabled_prior_objects()

        if self._ransac_prior in enabled:
            time_due = (
                self._last_ransac_fired_s is None
                or now - self._last_ransac_fired_s >= self._ransac_interval_s
            )
            if time_due and self._has_enough_points(msg):
                self._last_ransac_fired_s = now
                self._fire(self._ransac_prior, msg)
            elif time_due:
                # Starved: log, leave the timer so the trigger stands and fires next dense frame.
                self._maybe_log_skip(msg)

        if self._fiducial_prior is not None and self._fiducial_prior.has_pending:
            # Cold start: a burst arrived before any cloud was cached; fire it on the first cloud.
            self._fire(self._fiducial_prior, msg)

    def _try_relocalize(self, msg: PointCloud2, priors: list[RelocPrior]) -> Transform | None:
        assert self._premap is not None
        if not priors:
            return None  # every prior disabled (or fiducial-only with no marker map yet)
        t0 = time.monotonic()
        winning_source: str | None = None
        # Plain relocalize() is reserved for the lidar-only MODULE, where the SOLVE is the pre-prior path bit-for-bit and the accept line carries no source=. Once a tag prior is configured every fire goes through the judge instead, so both sources are counted.
        solo_prior_module = len(self._enabled_prior_objects()) == 1
        try:
            if solo_prior_module and len(priors) == 1 and isinstance(priors[0], RansacPrior):
                T, fitness = _relocalize(
                    self._premap.pointcloud,
                    msg.pointcloud,
                    gravity_tilt_max_deg=self.config.gravity_tilt_max_deg,
                )
            else:
                T, fitness, winning_source = relocalize_with_priors(
                    self._premap.pointcloud,
                    msg.pointcloud,
                    priors,
                    gravity_tilt_max_deg=self.config.gravity_tilt_max_deg,
                )
        except EmptyProposalError:
            # Double-fire race: the other thread drained the pending tag fix first, so this cycle judges an empty pool. The winner already published it -- benign no-op.
            logger.debug("relocalize: no candidates this cycle")
            return None
        except InsufficientWallEvidenceError as e:
            # Too few wall points to judge the pool (a sparse acquisition cloud a tag burst is allowed to fire on): drop the unjudged fix -- a pose scored against <100 walls isn't trustworthy, and the tag is re-seen as the robot moves into structure.
            self._maybe_log_wall_skip(len(msg), str(e))
            return None
        except NoUprightCandidateError:
            # Every candidate tilted past the gravity gate: a real rejection, so the fix is consumed (re-judging a tilted pose only re-rejects it). Throttled warn, shares the skip window.
            now = time.monotonic()
            if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
                logger.warning("relocalize rejected: all candidates tilted past gravity gate")
                self._last_skip_log = now
            return None
        except Exception:
            logger.exception("relocalize() failed")
            return None
        dt = time.monotonic() - t0
        n_pts = len(msg)
        # source= emission: ACCEPT names the prior that WON (omitted on the single-source path, which the parsers read as ransac); REJECT names the prior whose BAR refused the fix, so --eval can bucket rejects by source instead of "unknown".
        source_kw: dict[str, str] = {} if winning_source is None else {"source": winning_source}

        # The winner is gated on ITS OWN source's bar. winning_source is None only on the plain (ransac-only) path -> resolve to the ransac entry.
        gate_source = winning_source if winning_source is not None else RansacPrior.name
        threshold = self._accept_threshold[gate_source]
        if fitness < threshold:
            if self.config.verbose_eval_logging:
                self._eval_tally.setdefault(gate_source, SourceTally()).rejects += 1
            # threshold= IS the reason: this is the only reject path, and the operator's next move is to compare the two numbers.
            extra: dict[str, Any] = (
                {"time_cost_s": round(dt, 1), "n_pts": n_pts}
                if self.config.verbose_eval_logging
                else {}
            )
            logger.warning(
                "relocalize rejected",
                source=gate_source,
                fitness=round(fitness, 3),
                threshold=threshold,
                **extra,
            )
            return None

        # Jump guard, TRACKING only: a fix stepping further from the previous one than the budgets above allow is a mis-localization, not a drift correction. No previous fix means acquisition, which is never blocked. Both terms are on map_T_world.
        now_s = self._now_fn()
        if self._last_fix_map_T_world is not None:
            dt_s = now_s - self._last_fix_ts_s
            # Floored at one second's worth: two priors fire back to back may disagree by a normal correction, and a budget shrinking with the gap would refuse exactly the cross-source fix the fiducial preset exists to publish.
            budget_s = max(dt_s, 1.0)
            jump_m = float(np.linalg.norm(T[:3, 3] - self._last_fix_map_T_world[:3, 3]))
            rot_rel = self._last_fix_map_T_world[:3, :3].T @ T[:3, :3]
            # Yaw is the whole story: the judge's gravity gate already holds tilt under GRAVITY_TILT_MAX_DEG, so a flip between two accepted fixes is about z.
            yaw_deg = abs(float(np.degrees(np.arctan2(rot_rel[1, 0], rot_rel[0, 0]))))
            if jump_m > MAX_JUMP_M_PER_S * budget_s or yaw_deg > MAX_JUMP_YAW_DEG_PER_S * budget_s:
                logger.warning(
                    "relocalize jump rejected",
                    source=gate_source,
                    jump_m=round(jump_m, 2),
                    yaw_deg=round(yaw_deg, 1),
                    dt_s=round(dt_s, 2),
                )
                return None
        self._last_fix_map_T_world = T
        self._last_fix_ts_s = now_s

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
        if self.config.verbose_eval_logging:
            entry = self._eval_tally.setdefault(gate_source, SourceTally())
            entry.accepts += 1
            entry.fitnesses.append(round(fitness, 3))
        # The published pose rides on --eval only: the quiet line is the fix plus its health.
        pose_kw: dict[str, Any] = (
            {
                "n_pts": n_pts,
                "reloc_t_m": T[:3, 3].round(3).tolist(),
                "tf_from": FRAME_WORLD,
                "tf_to": FRAME_MAP,
                "published_t_m": T_inv[:3, 3].round(3).tolist(),
            }
            if self.config.verbose_eval_logging
            else {}
        )
        logger.info(
            "relocalize accepted",
            **source_kw,
            fitness=round(fitness, 3),
            time_cost_s=round(dt, 1),
            **pose_kw,
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
