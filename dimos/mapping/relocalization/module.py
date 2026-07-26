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

from collections import deque
import threading
import time
from typing import Annotated, Any, TypeVar

import numpy as np
from pydantic import Field, model_validator
import reactivex as rx
from reactivex import Subject, combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.eval import SourceTally, format_eval_summary
from dimos.mapping.relocalization.fiducial import FiducialPrior, FiducialPriorConfig
from dimos.mapping.relocalization.prior import PriorConfigBase, RansacPrior, RansacPriorConfig
from dimos.mapping.relocalization.relocalize import (
    InsufficientWallEvidenceError,
    NoUprightCandidateError,
    refine_candidates,
)
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.msgs.visualization_msgs.EntityMarkers import EntityMarkers, Marker
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

PUBLISH_INTERVAL = 2.0  # for loaded_map + TF
MAP_SUFFIX = ".pc2.lcm"
SKIP_LOG_INTERVAL_S = 5.0  # s; throttle relocalize-skip warnings so a starved feed can't spam
# EntityMarkers.TYPE_COLORS key per prior, so the two read apart in rerun: ransac blue, fiducial green.
ACCEPTED_FIX_TYPE = {"ransac": "location", "fiducial": "object"}
MAX_ACCEPTED_FIXES = 500  # a 730 s survey polls RANSAC ~365 times, so a whole run fits

_PriorConfigT = TypeVar("_PriorConfigT", bound=PriorConfigBase)

# A prior proposes candidates; it must not self-select a winner -- that is refine_candidates' job.
RelocPrior = RansacPrior | FiducialPrior

# Discriminated on ``type`` (kinematics/config.py:54 is the exemplar); assembled here, its only consumer, so prior.py stays unaware of the leaves.
PriorConfig = Annotated[
    RansacPriorConfig | FiducialPriorConfig,
    Field(discriminator="type"),
]


class Config(ModuleConfig):
    map_file: str | None = (
        None  # e.g. `-o relocalizationmodule.map_file=go2_hongkong_office_twopass_map`
    )
    publish_loaded_map: bool = False
    use_carving: bool = True
    # True (`--eval`): raise the throttled refusal lines to warning and tally accepts/rejects per prior for the summary at stop().
    eval: bool = False
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
        pool.update({k: {**v, "type": k} for k, v in overlay.items()})
        return {**data, "priors": pool}


class RelocalizationModule(Module):
    config: Config
    global_map: In[PointCloud2]
    aggregated_detections: In[Detection3DArray]  # the detector's per-burst aggregated tag poses
    loaded_map: Out[PointCloud2]
    merged_map: Out[PointCloud2]
    accepted_fixes: Out[EntityMarkers]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._premap: PointCloud2 | None = None
        self._last_skip_log = 0.0
        # Per-source accept/reject tally, filled only under eval and rendered once at stop(). The module already owns the accept/reject data.
        self._eval_tally: dict[str, SourceTally] = {}
        # Accepted-fix trail for rerun, filled only under eval and republished whole so the view accumulates.
        self._accepted_fixes: deque[Marker] = deque(maxlen=MAX_ACCEPTED_FIXES)
        self._world_to_map: Subject[Transform | None] = Subject()
        # Prior objects are built ONCE, not per frame: the fiducial holds pending-fix state across bursts that a fresh instance would reset. The RANSAC prior is a pure source; the module owns its poll timer (below).
        self._ransac = self._enabled_entry(RansacPriorConfig)
        self._ransac_prior = RansacPrior() if self._ransac else None
        self._last_ransac_fired_s: float | None = None  # None: fire on the first dense frame
        # Built in start(), where the prior loads its own marker map.
        self._fiducial_prior: FiducialPrior | None = None
        # Latest global_map, so a completed tag burst is judged the instant it lands. Sound because the stream ACCUMULATES in the world frame: the previous cloud scores wall fitness as well as the newest, and the tag candidate needs no lidar at all.
        self._last_local_map: PointCloud2 | None = None
        # Guards the publish decision only: the solve stays outside, so a 7 s RANSAC sweep never blocks a 0.1 s tag fix.
        self._publish_lock = threading.Lock()
        self._last_solve_start_s = 0.0

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
        live = [p.name for p in (self._ransac_prior, self._fiducial_prior) if p is not None]
        logger.info("relocalize priors", live=live)

    @rpc
    def stop(self) -> None:
        # Emit the per-source accept/reject table once, at shutdown, from the tally the fire path filled -- the module owns the data, so no log parsing.
        if self.config.eval and self._eval_tally:
            logger.info("relocalize eval summary", table=format_eval_summary(self._eval_tally))
        super().stop()

    def _enabled_entry(self, entry_type: type[_PriorConfigT]) -> _PriorConfigT | None:
        """This prior's pool entry when it is present and enabled -- the pool key IS the entry's ``type``."""
        entry = self.config.priors.get(entry_type.model_fields["type"].default)
        return entry if isinstance(entry, entry_type) and entry.enabled else None

    def _start_fiducial_prior(self) -> None:
        """Build the fiducial prior from its config and route tag sightings into it."""
        fiducial = self._enabled_entry(FiducialPriorConfig)
        self._fiducial_prior = FiducialPrior.from_config(fiducial) if fiducial else None
        if self._fiducial_prior is None:
            return
        self.register_disposable(
            self.aggregated_detections.observable().subscribe(self._on_aggregated_detections)  # type: ignore[no-untyped-call]
        )

    def _throttled_warn(self, event: str, **kw: Any) -> None:
        """Report that this fire was refused, at most once per SKIP_LOG_INTERVAL_S across every refusal."""
        now = time.monotonic()
        if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
            # Refusals are the steady state while the submap fills, so they surface only under --eval; the console otherwise carries accepted fixes alone.
            log = logger.warning if self.config.eval else logger.debug
            log(event, **kw)
            self._last_skip_log = now

    def _on_aggregated_detections(self, msg: Detection3DArray) -> None:
        """Hand the burst to the fiducial prior, then fire it."""
        if self._fiducial_prior is None:
            return
        self._fiducial_prior.observe_detections(msg)
        # Fire HERE, not at the next cloud: the tag candidate is composed from the marker alone, so waiting on lidar is dead time on acquisition. No cloud yet -> pending stays and _on_local_map fires it on the first frame.
        cloud = self._last_local_map
        if cloud is not None and self._fiducial_prior.has_pending:
            self._fire(self._fiducial_prior, cloud)

    def _fire(self, prior: RelocPrior, local_map: PointCloud2) -> None:
        """Run this prior's ONE relocalization and publish the resulting TF."""
        tf = self._try_relocalize(local_map, prior)
        if tf is not None:
            self._world_to_map.on_next(tf)

    def _on_local_map(self, msg: PointCloud2) -> None:
        """Poll the RANSAC prior on the module's timer; on cold start fire a pending fiducial fix."""
        self._last_local_map = msg  # what a burst-triggered fire judges against
        now = time.monotonic()
        if self._ransac is not None and self._ransac_prior is not None:
            time_due = (
                self._last_ransac_fired_s is None
                or now - self._last_ransac_fired_s >= self._ransac.interval_s
            )
            if time_due and len(msg) >= self._ransac.min_local_points:
                self._last_ransac_fired_s = now
                self._fire(self._ransac_prior, msg)
            elif time_due:
                # Starved: log, leave the timer so the trigger stands and fires next dense frame.
                self._throttled_warn(
                    "relocalize skipped: sparse submap",
                    n_pts=len(msg),
                    min_local_points=self._ransac.min_local_points,
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
        dt = time.monotonic() - t0
        n_pts = len(msg)
        if not self._fitness_ok(prior, fitness, dt, n_pts):
            return None
        # The two _fire paths run on separate backpressure threads, so a slow RANSAC solve can outlive a fiducial fix that started later; publishing it would rewind the anchor.
        with self._publish_lock:
            if t0 < self._last_solve_start_s:
                return None
            self._last_solve_start_s = t0
            return self._publish_fix(map_T_world, prior, fitness, dt, n_pts)

    def _solve(self, msg: PointCloud2, prior: RelocPrior) -> tuple[np.ndarray, float] | None:
        """Judge this prior's candidates, turning each refusal into a None sentinel."""
        assert self._premap is not None, "start() loads the premap before any fire"
        global_map, local_map = self._premap.pointcloud, msg.pointcloud
        try:
            # empty is the double-fire race: the other thread's propose() drained the pending tag fix and published it
            candidates = prior.propose(global_map, local_map)
            if not candidates:
                return None
            return refine_candidates(global_map, local_map, candidates)
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

    def _fitness_ok(self, prior: RelocPrior, fitness: float, dt: float, n_pts: int) -> bool:
        """Gate the fix on its own prior's bar."""
        threshold = self.config.priors[prior.name].fitness_threshold
        if fitness >= threshold:
            return True
        if self.config.eval:
            self._eval_tally.setdefault(prior.name, SourceTally()).rejects += 1
        # threshold= IS the reason: the operator's next move is to compare the two numbers.
        logger.warning(
            "relocalize rejected",
            source=prior.name,
            fitness=round(fitness, 3),
            threshold=threshold,
            time_cost_s=round(dt, 1),
            n_pts=n_pts,
        )
        return False

    def _publish_fix(
        self, map_T_world: np.ndarray, prior: RelocPrior, fitness: float, dt: float, n_pts: int
    ) -> Transform:
        """Return the accepted fix as the world->map TF."""
        world_T_map = np.linalg.inv(map_T_world)
        self._log_accept(prior, fitness, dt, n_pts, map_T_world, world_T_map)
        self._publish_accepted_fixes(prior, fitness, world_T_map)
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
        dt: float,
        n_pts: int,
        map_T_world: np.ndarray,
        world_T_map: np.ndarray,
    ) -> None:
        """One accept line: the published fix, its health, and which prior won."""
        if self.config.eval:
            entry = self._eval_tally.setdefault(prior.name, SourceTally())
            entry.accepts += 1
            entry.fitnesses.append(round(fitness, 3))
        logger.info(
            "relocalize accepted",
            source=prior.name,
            fitness=round(fitness, 3),
            time_cost_s=round(dt, 1),
            n_pts=n_pts,
            reloc_t_m=map_T_world[:3, 3].round(3).tolist(),
            published_t_m=world_T_map[:3, 3].round(3).tolist(),
        )

    def _publish_accepted_fixes(
        self, prior: RelocPrior, fitness: float, world_T_map: np.ndarray
    ) -> None:
        """Append this accepted fix to the rerun marker trail and republish the trail."""
        if not self.config.eval:
            return
        # EntityMarkers carries no frame_id, so the bridge leaves it unparented at the rerun root -- the world frame the live clouds are drawn in.
        x, y, z = world_T_map[:3, 3]
        self._accepted_fixes.append(
            Marker(
                entity_id=prior.name,
                label=f"{fitness:.2f}",
                entity_type=ACCEPTED_FIX_TYPE[prior.name],
                x=float(x),
                y=float(y),
                z=float(z),
            )
        )
        self.accepted_fixes.publish(EntityMarkers(markers=list(self._accepted_fixes)))

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
