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

from pathlib import Path
import time
from typing import Any

import numpy as np
from pydantic import Field, model_validator
import reactivex as rx
from reactivex import Subject, combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.eval import SourceTally, format_eval_summary
from dimos.mapping.relocalization.priors import (
    FiducialPrior,
    FiducialPriorConfig,
    PriorConfig,
    RansacPrior,
    RansacPriorConfig,
    RelocPrior,
    load_marker_map,
    relocalize_with_prior,
)
from dimos.mapping.relocalization.relocalize import (
    GRAVITY_TILT_MAX_DEG,
    InsufficientWallEvidenceError,
    NoUprightCandidateError,
)
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.apriltag_aggregation import matrix_from_pose7
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

PUBLISH_INTERVAL = 2.0  # for loaded_map + TF
MAP_SUFFIX = ".pc2.lcm"
MARKER_MAP_SUFFIX = ".json"  # what write_marker_map emits and load_marker_map parses
SKIP_LOG_INTERVAL_S = 5.0  # s; throttle relocalize-skip warnings so a starved feed can't spam


class Config(ModuleConfig):
    map_file: str | None = (
        None  # e.g. `-o relocalizationmodule.map_file=go2_hongkong_office_twopass_map`
    )
    publish_loaded_map: bool = False
    # Max z-axis tilt (deg) a candidate may keep at the judge's gravity gate.
    gravity_tilt_max_deg: float = Field(default=GRAVITY_TILT_MAX_DEG, ge=0.0)
    use_carving: bool = True
    # False (default): one line per accepted fix plus throttled warnings. True (`--eval`): each accept also logs its published pose, timing and point count.
    verbose_eval_logging: bool = False
    # The prior pool keyed by `type`, so every knob is `-o relocalizationmodule.priors.<key>.<field>`: each entry a toggleable candidate proposer (RANSAC polled, fiducial event-driven). Both on by default, so a blueprint declares nothing; `--disable marker-detection-stream-module` leaves RANSAC alone.
    priors: dict[str, PriorConfig] = {
        "ransac": RansacPriorConfig(),
        "fiducial": FiducialPriorConfig(),
    }

    @model_validator(mode="before")
    @classmethod
    def _tag_priors_by_key(cls, data: Any) -> Any:
        """Overlay the given prior entries onto the default pool, each tagged with its key."""
        overlay = data.get("priors") if isinstance(data, dict) else None
        if not isinstance(overlay, dict):
            return data
        # A dotted `-o` arrives as one bare entry: the key supplies the `type` discriminator it cannot carry, and the default pool the priors it did not name.
        pool: dict[str, Any] = {
            key: entry.model_copy(deep=True)
            for key, entry in cls.model_fields["priors"].default.items()
        }
        pool.update({k: {**v, "type": k} if isinstance(v, dict) else v for k, v in overlay.items()})
        return {**data, "priors": pool}


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
            (p for p in self.config.priors.values() if isinstance(p, RansacPriorConfig)),
            RansacPriorConfig(),
        )
        self._ransac_prior = RansacPrior()
        # Per-source accept gate {prior name: min wall fitness}. Key = the entry's `type` discriminator, which equals the ``name`` of the prior it builds.
        self._accept_threshold: dict[str, float] = {
            p.type: p.fitness_threshold for p in self.config.priors.values()
        }
        # RANSAC poll timer (the prior is a pure source): its interval and last fire. None == never fired, so the first dense-enough frame relocalizes immediately.
        self._ransac_interval_s = ransac_entry.interval_s
        self._last_ransac_fired_s: float | None = None
        # RANSAC-only point floor (a fiducial burst fires regardless); read from the ransac entry, or the default when no ransac entry is enabled (then unused).
        self._ransac_min_local_points = ransac_entry.min_local_points
        # Built in start() once the marker map is loaded (needs map_T_marker).
        self._fiducial_prior: FiducialPrior | None = None
        # Latest global_map, so a completed tag burst is judged the instant it lands. Sound because the stream ACCUMULATES in the world frame: the previous cloud scores wall fitness as well as the newest, and the tag candidate needs no lidar at all.
        self._last_local_map: PointCloud2 | None = None

    @rpc
    def start(self) -> None:
        super().start()

        if not self.config.map_file:
            logger.info("Relocalization module disabled (no map_file configured)")
            return

        path = resolve_named_path(self.config.map_file, MAP_SUFFIX)
        self._premap = PointCloud2.lcm_decode(path.read_bytes())
        self._premap.frame_id = FRAME_MAP

        # Trigger accounting runs on the backpressure worker thread only, so a frame coalesced away cannot drop a burst edge.
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
        # Both priors default on and the blueprint names none, so this is where an operator sees what actually came up: an enabled fiducial entry with no marker map is inert.
        live = [prior.name for prior in self._enabled_prior_objects()]
        logger.info(
            "relocalize priors",
            live=live,
            inert=[p.type for p in self.config.priors.values() if p.type not in live],
        )

    @rpc
    def stop(self) -> None:
        # Emit the per-source accept/reject table once, at shutdown, from the tally the fire path filled -- the module owns the data, so no log parsing.
        if self.config.verbose_eval_logging and self._eval_tally:
            logger.info("relocalize eval summary", table=format_eval_summary(self._eval_tally))
        super().stop()

    def _fiducial_config(self) -> FiducialPriorConfig | None:
        """The first enabled fiducial entry, or None -- one detector feeds one prior."""
        for prior_config in self.config.priors.values():
            if isinstance(prior_config, FiducialPriorConfig) and prior_config.enabled:
                return prior_config
        return None

    def _start_fiducial_prior(self) -> None:
        """Load the marker map, build the FiducialPrior, route sightings into it."""
        fiducial = self._fiducial_config()
        if fiducial is None:
            return
        marker_map_file = fiducial.marker_map_file
        if not marker_map_file:
            logger.warning(
                "relocalize: fiducial prior enabled but no marker_map_file; fiducial prior disabled"
            )
            return
        # A bare name gets the default suffix; a name that already carries it keeps it (resolve_named_path would otherwise look up "<survey>.json.json").
        suffix = "" if Path(marker_map_file).suffix == MARKER_MAP_SUFFIX else MARKER_MAP_SUFFIX
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

    def _throttled_warn(self, event: str, **kw: Any) -> None:
        """Warn that this fire was refused, at most once per SKIP_LOG_INTERVAL_S across every refusal."""
        now = time.monotonic()
        if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
            logger.warning(event, **kw)
            self._last_skip_log = now

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= self._ransac_min_local_points

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
        self._publish_tf(self._try_relocalize(local_map, prior))

    @staticmethod
    def _marker_id_from_detection(detection: Detection3D) -> int | None:
        """Numeric marker id off the wire, via the same parse MarkerTfModule publishes TF from."""
        raw = MarkerTfModule._marker_id_from_detection(detection)
        return int(raw) if raw is not None and raw.isdigit() else None

    def _enabled_prior_objects(self) -> list[RelocPrior]:
        """The candidate PROPOSERS for enabled entries."""
        objects: list[RelocPrior] = []
        for prior_config in self.config.priors.values():
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
        now = time.monotonic()
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
                self._throttled_warn(
                    "relocalize skipped: sparse submap",
                    n_pts=len(msg),
                    min_local_points=self._ransac_min_local_points,
                )

        if self._fiducial_prior is not None and self._fiducial_prior.has_pending:
            # Cold start: a burst arrived before any cloud was cached; fire it on the first cloud.
            self._fire(self._fiducial_prior, msg)

    def _try_relocalize(self, msg: PointCloud2, prior: RelocPrior) -> Transform | None:
        """Solve, gate on fitness, publish."""
        t0 = time.monotonic()
        solved = self._solve(msg, prior)
        if solved is None:
            return None
        map_T_world, fitness = solved
        solve_s, n_pts = time.monotonic() - t0, len(msg)
        if not self._fitness_ok(prior, fitness, solve_s, n_pts):
            return None
        return self._publish_fix(map_T_world, prior, fitness, solve_s, n_pts)

    def _solve(self, msg: PointCloud2, prior: RelocPrior) -> tuple[np.ndarray, float] | None:
        """Judge this prior's candidates, turning each refusal into a None sentinel."""
        assert self._premap is not None, "start() loads the premap before any fire"
        try:
            # None here is the double-fire race: the other thread's propose() drained the pending tag fix first and already published it -- benign no-op.
            return relocalize_with_prior(
                self._premap.pointcloud,
                msg.pointcloud,
                prior,
                gravity_tilt_max_deg=self.config.gravity_tilt_max_deg,
            )
        except InsufficientWallEvidenceError as e:
            # Too few wall points to judge the pool (a sparse acquisition cloud a tag burst is allowed to fire on): drop the unjudged fix -- a pose scored against <100 walls isn't trustworthy, and the tag is re-seen as the robot moves into structure.
            self._throttled_warn(
                "relocalize skipped: insufficient wall evidence", n_pts=len(msg), reason=str(e)
            )
            return None
        except NoUprightCandidateError:
            # Every candidate tilted past the gravity gate: a real rejection, so the fix is consumed (re-judging a tilted pose only re-rejects it).
            self._throttled_warn(
                "relocalize rejected: every candidate tilted past the gravity gate"
            )
            return None
        except Exception:
            logger.exception("relocalize() failed")
            return None

    def _fitness_ok(self, prior: RelocPrior, fitness: float, solve_s: float, n_pts: int) -> bool:
        """Gate the fix on its own prior's bar."""
        threshold = self._accept_threshold[prior.name]
        if fitness >= threshold:
            return True
        if self.config.verbose_eval_logging:
            self._eval_tally.setdefault(prior.name, SourceTally()).rejects += 1
        # threshold= IS the reason: this is the only fitness reject path, and the operator's next move is to compare the two numbers.
        extra: dict[str, Any] = (
            {"time_cost_s": round(solve_s, 1), "n_pts": n_pts}
            if self.config.verbose_eval_logging
            else {}
        )
        logger.warning(
            "relocalize rejected",
            source=prior.name,
            fitness=round(fitness, 3),
            threshold=threshold,
            **extra,
        )
        return False

    def _publish_fix(
        self, map_T_world: np.ndarray, prior: RelocPrior, fitness: float, solve_s: float, n_pts: int
    ) -> Transform:
        """Return the accepted fix as the world->map TF."""
        # relocalize(scan, map) returns T such that scan_in_map_frame = T(scan_raw).
        # We are publishing a TF for map_in_scan_frame, notice that the base frame is `world`
        # so inverse the transform T here to get map_in_scan_frame
        world_T_map = np.linalg.inv(map_T_world)
        self._log_accept(prior, fitness, solve_s, n_pts, map_T_world, world_T_map)
        return Transform(
            translation=Vector3(*world_T_map[:3, 3]),
            rotation=Quaternion.from_rotation_matrix(world_T_map[:3, :3]),
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        )

    def _log_accept(
        self,
        prior: RelocPrior,
        fitness: float,
        solve_s: float,
        n_pts: int,
        map_T_world: np.ndarray,
        world_T_map: np.ndarray,
    ) -> None:
        """One accept line: the fix plus its health, with pose and timing under --eval."""
        if self.config.verbose_eval_logging:
            entry = self._eval_tally.setdefault(prior.name, SourceTally())
            entry.accepts += 1
            entry.fitnesses.append(round(fitness, 3))
        logger.info(
            "relocalize accepted",
            source=prior.name,
            fitness=round(fitness, 3),
            time_cost_s=round(solve_s, 1),
            **self._pose_kw(n_pts, map_T_world, world_T_map),
        )

    def _pose_kw(
        self, n_pts: int, map_T_world: np.ndarray, world_T_map: np.ndarray
    ) -> dict[str, Any]:
        """The published pose rides on --eval only: the quiet line is the fix plus its health."""
        if not self.config.verbose_eval_logging:
            return {}
        return {
            "n_pts": n_pts,
            "reloc_t_m": map_T_world[:3, 3].round(3).tolist(),
            "tf_from": FRAME_WORLD,
            "tf_to": FRAME_MAP,
            "published_t_m": world_T_map[:3, 3].round(3).tolist(),
        }

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
