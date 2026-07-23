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
from pydantic import Field, model_validator
import reactivex as rx
from reactivex import Subject, combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.priors import (
    EmptyProposalError,
    FiducialPrior,
    FiducialPriorConfig,
    PriorConfig,
    RansacPrior,
    RansacPriorConfig,
    RelocPrior,
    relocalize_with_priors as _relocalize_with_priors,
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
from dimos.perception.fiducial.fiducial_relocalization import load_marker_map
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

PUBLISH_INTERVAL_S = 2.0  # s; for loaded_map + TF
MAP_SUFFIX = ".pc2.lcm"
# Survey formats load_marker_map parses. A name already carrying one keeps it;
# only a bare name gets the .json default (see _start_fiducial_prior).
MARKER_MAP_SUFFIXES = (".json", ".yaml", ".yml")
SKIP_LOG_INTERVAL_S = 5.0  # s; throttle relocalize-skip warnings so a starved feed can't spam
# Jump guard, TRACKING only: the largest step from the previous accepted fix a new
# one may take, per second since that fix. T is the world->map ALIGNMENT, not the
# robot's pose, so between two accepts it moves only by the drift the fix corrects
# (sub-metre over a 730 s survey). What this refuses is the mirror-flipped fix
# landing 15-22 m away with 100-176 deg of rotation, which republishes the whole
# premap one room over. DELIBERATELY LOOSE (~5-10x any correction measured); per
# second, and floored at one second's worth below, so two priors firing back to
# back keep a real budget and a robot that really was carried re-acquires.
MAX_JUMP_M_PER_S = 5.0  # m/s; 10 m at the 2 s RANSAC interval
MAX_JUMP_YAW_DEG_PER_S = 45.0  # deg/s; 90 deg at the 2 s RANSAC interval

# Keys that moved onto the prior entries. BaseConfig is extra="forbid", so a stale
# key already raises -- but "extra inputs are not permitted" never says where the
# knob went, and a config silently losing its cadence is a robot relocalizing at a
# rate nobody chose.
_MOVED_TO_PRIORS = {
    "reloc_interval_s": (
        "reloc_interval_s moved onto the ransac prior entry; set "
        "priors=[RansacPriorConfig(interval_s=...)] instead -- each prior "
        "now owns its own trigger"
    ),
    "fitness_threshold": (
        "fitness_threshold moved onto each prior entry; set e.g. "
        "priors=[RansacPriorConfig(fitness_threshold=...)] -- the accept "
        "bar is now per prior"
    ),
    "min_local_points": (
        "min_local_points moved onto the ransac prior entry; set "
        "priors=[RansacPriorConfig(min_local_points=...)] -- only the "
        "RANSAC search is gated on submap density"
    ),
}


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
    # --- shared judge param: applies to whichever priors are enabled below ---
    # The accept gate (fitness_threshold) and the RANSAC point floor (min_local_points)
    # are NOT here: they moved onto the prior entries (PriorConfig.fitness_threshold,
    # RansacPriorConfig.min_local_points), since the bar is per prior and the floor is
    # RANSAC's alone. See _MOVED_TO_PRIORS for the redirect.
    # Max z-axis tilt (deg) a candidate may keep at the judge's gravity gate;
    # matches relocalize.py's GRAVITY_TILT_MAX_DEG, so unchanged unless overridden.
    gravity_tilt_max_deg: float = Field(default=10.0, ge=0.0)
    use_carving: bool = True
    # Log volume for the reloc path. False (operating default): ONE line per
    # accepted fix, plus the throttled skip/reject warnings. True (`--eval`): the
    # full per-fire trace -- proposal census plus the fat accept line (every kwarg) --
    # which the eval parsers read. At the RANSAC prior's 2 s interval that is two
    # lines every 2 s for the life of the robot, so it is opt-in.
    verbose_eval_logging: bool = False
    # The prior pool: each entry an EQUAL, toggleable candidate proposer that owns
    # its own trigger (interval, burst edge). REQUIRED, no default -- a blueprint
    # (or -o) must state it, so there is no silent module-level reloc behavior.
    # RANSAC is a first-class toggleable entry, not always-on; the fiducial-only
    # preset omits it and then runs with no periodic timer at all.
    priors: list[PriorConfig]

    @model_validator(mode="before")
    @classmethod
    def _reject_moved_fields(cls, data: Any) -> Any:
        if isinstance(data, dict):
            for key, message in _MOVED_TO_PRIORS.items():
                if key in data:
                    raise ValueError(message)
        return data


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
        self._world_to_map: Subject[Transform | None] = Subject()
        # Prior objects are built ONCE, not per frame: each holds its own trigger
        # state (RANSAC's interval timer, the fiducial's burst edge), which a fresh
        # instance would reset every frame into firing forever. No ransac entry ->
        # the prior is never enabled, so the default config's interval is moot.
        ransac_entry = next(
            (p for p in self.config.priors if isinstance(p, RansacPriorConfig)),
            RansacPriorConfig(),
        )
        self._ransac_prior = RansacPrior(interval_s=ransac_entry.interval_s)
        # Per-source accept gate {Candidate.source: min wall fitness}. Each fire pools
        # ONE source (per-prior triggers), so gating the winner by its own source's bar
        # IS per-source gating -- the judge needs no change. The key is the entry's
        # `type` discriminator, which equals the source its prior proposes
        # (RansacPrior.name == "ransac", ...).
        self._accept_threshold: dict[str, float] = {
            p.type: p.fitness_threshold for p in self.config.priors
        }
        # RANSAC-only point floor (a fiducial burst fires regardless); read from the
        # ransac entry, or the default when no ransac entry is enabled (then unused).
        self._ransac_min_local_points = ransac_entry.min_local_points
        # Built in start() once the marker map is loaded (needs map_T_marker).
        self._fiducial_prior: FiducialPrior | None = None
        # Previous ACCEPTED fix (map_T_world) and when, for the gross-jump guard.
        # None until the first accept, which is what leaves acquisition unguarded.
        self._last_fix_map_T_world: np.ndarray | None = None
        self._last_fix_ts_s = 0.0
        # Latest global_map, so a completed tag burst is judged the instant it lands
        # instead of idling until the next cloud (_on_aggregated_detections). Sound because
        # the stream ACCUMULATES in the world frame: the previous cloud scores wall
        # fitness as well as the newest, and the tag candidate needs no lidar at all.
        self._last_local_map: PointCloud2 | None = None
        # One monotonic timebase for every prior trigger, injectable so a test can
        # drive the triggers without sleeping (FiducialPrior takes the same seam).
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

        # No throttle here any more: the priors own the cadence. EVERY frame reaches
        # _on_local_map (no point-count filter here), which asks each prior whether
        # it wants a fire; the min_local_points floor is applied per prior there so a
        # fiducial burst can fire on a sparse submap while RANSAC waits for geometry.
        # Trigger accounting therefore runs on the backpressure worker thread only --
        # a frame that backpressure coalesces away never consumes a trigger, so a
        # burst edge cannot be dropped by a slow solve.
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
        # A bare name gets the .json default; a name that already states its format
        # keeps it -- resolve_named_path appends the suffix when the file isn't local,
        # so a .json default would look up "<survey>.yaml.json", which nothing writes
        # (the marker-map derivation pipeline writes yaml).
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
        """Throttled warning that a sparse submap starved the RANSAC search this
        frame. RANSAC-only: a fiducial burst fires regardless of point count, so
        this is not a general 'relocalize skipped'."""
        now = time.monotonic()
        if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
            logger.warning(
                "ransac reloc skipped",
                n_pts=len(msg),
                min_local_points=self._ransac_min_local_points,
            )
            self._last_skip_log = now

    def _maybe_log_wall_skip(self, n_pts: int) -> None:
        """Throttled warning that the judge refused a fire for too little wall evidence
        (a sparse submap at acquisition). Shares the RANSAC-skip throttle timer: both
        are 'no fix this fire because the cloud was too sparse', and one 5 s window
        covering both keeps a starved feed from spamming."""
        now = time.monotonic()
        if now - self._last_skip_log > SKIP_LOG_INTERVAL_S:
            logger.warning("relocalize skipped: insufficient wall evidence", n_pts=n_pts)
            self._last_skip_log = now

    def _publish_tf(self, tf: Transform | None) -> None:
        if tf is None:
            return
        self._world_to_map.on_next(tf)

    def _on_aggregated_detections(self, msg: Detection3DArray) -> None:
        """Compose each aggregated tag pose into this tag's world->map fix, then fire.

        Every entry here is one already-gated, already-aggregated world_T_marker (the
        detector's AggregateTagBursts publishes one per marker per visit), so the work
        left is id-parse + compose -- no gating, no aggregation, no timestamp."""
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
        # Fire HERE, not at the next cloud: the tag candidate is composed from the
        # marker alone, so waiting on lidar is dead time on acquisition -- the latency
        # that matters, since the planner is blind to the premap until the first fix.
        # No cloud yet -> leave the trigger unacked and let _on_local_map fire it on
        # the first frame, the same decline path a starved RANSAC cycle takes.
        local_map = self._last_local_map
        if local_map is None:
            return
        now_s = self._now_fn()
        if self._fiducial_prior.is_due(now_s):
            self._fire(self._fiducial_prior, local_map, now_s)

    def _fire(self, prior: RelocPrior, local_map: PointCloud2, now_s: float) -> None:
        """Ack the trigger that asked and run its ONE relocalization. Two paths reach
        here, on two threads, neither waiting on the other::

            global_map -> _on_local_map -> cache the cloud
                                        +-> ransac.is_due   --> _fire(this cloud)
                                        +-> fiducial.is_due --> _fire(this cloud)    (a)
            aggregated_detections -> _on_aggregated_detections -> observe()
                                        +-> fiducial.is_due --> _fire(cached cloud)  (b)

        (b) is the tag path: a fix publishes at the tag's latency instead of idling
        until the next cloud. (a) catches only the burst that completed before any
        cloud arrived, which (b) leaves unacked having nothing to judge against.
        """
        prior.on_fired(now_s)
        self._publish_tf(self._try_relocalize(local_map, [prior]))

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
                objects.append(self._ransac_prior)
            elif isinstance(prior_config, FiducialPriorConfig) and self._fiducial_prior is not None:
                objects.append(self._fiducial_prior)
        return objects

    def _on_local_map(self, msg: PointCloud2) -> None:
        """One INDEPENDENT relocalization per prior asking for one.

        Triggers are per prior and so are the pools: a completed tag burst is judged
        on the tag's own candidates and never waits for -- or pays for -- the RANSAC
        search, which costs seconds per solve. A pool of one is the expected case;
        refine_candidates is the validator (wall fitness, gravity tilt, wall
        evidence), and each prior's own fitness_threshold is the accept gate.

        RANSAC lives on THIS path because it GENERATES its candidates from the cloud
        in hand. The fiducial prior normally fires from _on_aggregated_detections instead
        and reaches here only when its burst beat the first cloud.

        ``min_local_points`` (the ransac entry's) gates the RANSAC prior ONLY: a
        sparse submap starves the FPFH search, but a fiducial fix comes from the tag,
        so a tag burst fires even below the floor. A starved RANSAC cycle is NOT acked
        -- it re-fires on the next dense frame, exactly as when the floor was a
        stream-level filter.
        """
        self._last_local_map = msg  # what a burst-triggered fire judges against
        now_s = self._now_fn()
        enabled = self._enabled_prior_objects()
        has_enough_points = len(msg) >= self._ransac_min_local_points
        # Every prior is asked -- no short-circuit -- so each runs its own timer/edge
        # bookkeeping on this tick even when an earlier one already said yes.
        due = [prior for prior in enabled if prior.is_due(now_s)]
        for prior in due:
            if isinstance(prior, RansacPrior) and not has_enough_points:
                self._maybe_log_skip(msg)  # RANSAC starved: log, do not ack, retry next dense frame
                continue
            self._fire(prior, msg, now_s)

    def _try_relocalize(self, msg: PointCloud2, priors: list[RelocPrior]) -> Transform | None:
        if self._premap is None:
            raise RuntimeError(
                "_try_relocalize before start() loaded premap; call start() with map_file set"
            )
        if not priors:
            return None  # every prior disabled (or fiducial-only with no marker map yet)
        t0 = time.monotonic()
        winning_source: str | None = None
        # Plain relocalize() is reserved for the lidar-only MODULE -- RANSAC the whole
        # enabled pool -- where the SOLVE is the pre-prior path bit-for-bit and the
        # accept line carries no source= because no other source won it. Judged by
        # the module's pool, not this fire's: once a tag prior is configured, every
        # fire goes through the judge, so a ransac fire and a tag fire are both
        # attributed and both counted in the eval's per-fire census.
        solo_prior_module = len(self._enabled_prior_objects()) == 1
        try:
            if solo_prior_module and len(priors) == 1 and isinstance(priors[0], RansacPrior):
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
                    verbose_eval_logging=self.config.verbose_eval_logging,
                )
        except EmptyProposalError:
            # Double-fire race: the other thread drained the pending tag fix first, so
            # this cycle judges an empty pool. The winner already published it -- nothing
            # is lost, so this is a benign no-op, not a crash to log.exception.
            logger.debug("relocalize: no candidates this cycle")
            return None
        except InsufficientWallEvidenceError:
            # The judge could not evaluate the pool (too few wall points -- a sparse
            # acquisition cloud, which a tag burst is deliberately allowed to fire on).
            # The unjudged fix is dropped on purpose: a pose scored against <100 walls
            # isn't trustworthy, and the tag is re-seen as the robot moves into
            # structure. This is EXPECTED, so warn (throttled), not the ERROR traceback
            # logger.exception prints.
            self._maybe_log_wall_skip(len(msg))
            return None
        except NoUprightCandidateError:
            # Every candidate tilted past the gravity gate: a real rejection, so the fix
            # is consumed (re-judging a tilted pose only re-rejects it). Expected on a
            # bad pose -> warn rather than log.exception.
            logger.warning("relocalize rejected: all candidates tilted past gravity gate")
            return None
        except Exception:
            logger.exception("relocalize() failed")
            return None
        dt = time.monotonic() - t0
        n_pts = len(msg)
        # source= emission, the one rule both logs follow:
        #  - ACCEPT names the prior that WON the judge, and omits source= on the
        #    single-source path where nothing was won. The parsers read that absence
        #    as ransac (eval_module.parse_health_lines), so the line stays readable.
        #  - REJECT names the prior whose BAR refused the fix, on EVERY branch (quiet,
        #    verbose, solo). It prints that prior's own threshold=, and a per-prior
        #    number with no owner cannot be read; it is also what --eval buckets
        #    rejects by (eval_module.parse_reject_lines), which would otherwise
        #    attribute a whole verbose run to "unknown".
        source_kw: dict[str, str] = {} if winning_source is None else {"source": winning_source}

        # The winner is gated on ITS OWN source's bar. Each fire pools one source, so
        # this single-winner gate IS per-source. winning_source is None only on the
        # plain (ransac-only) path -> resolve to the ransac entry, the only prior that
        # path can have proposed.
        gate_source = winning_source if winning_source is not None else RansacPrior.name
        threshold = self._accept_threshold[gate_source]
        if fitness < threshold:
            # threshold= IS the reason: this is the only reject path, and the
            # operator's next move is to compare the two numbers.
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

        # Jump guard, TRACKING only: a fix stepping further from the previous one than
        # the budgets above allow is a mis-localization, not a drift correction. No
        # previous fix means acquisition, which is never blocked. Both terms are on
        # map_T_world: the relative rotation's yaw is what the flip IS, while jump_m is
        # the step at the world origin -- the robot's world position is not an input
        # here, so this is the cheap signal available.
        now_s = self._now_fn()
        if self._last_fix_map_T_world is not None:
            dt_s = now_s - self._last_fix_ts_s
            # Floored at one second's worth: two priors fire back to back (a tag burst
            # landing just after a RANSAC sweep), and two independent sources may
            # disagree by a normal correction however close together they land -- a
            # budget shrinking with the gap would refuse exactly the cross-source fix
            # the fiducial preset exists to publish.
            budget_s = max(dt_s, 1.0)
            jump_m = float(np.linalg.norm(T[:3, 3] - self._last_fix_map_T_world[:3, 3]))
            rot_rel = self._last_fix_map_T_world[:3, :3].T @ T[:3, :3]
            # Yaw is the whole story: the judge's gravity gate already holds tilt under
            # GRAVITY_TILT_MAX_DEG, so a flip between two accepted fixes is about z.
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

        # relocalize() returns map_T_world (scan_in_map = T @ scan_raw). We publish
        # the world->map TF, so invert to world_T_map here.
        T_inv = np.linalg.inv(T)
        new_tf = Transform(
            translation=Vector3(*T_inv[:3, 3]),
            rotation=Quaternion.from_rotation_matrix(T_inv[:3, :3]),
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        )
        if self.config.verbose_eval_logging:
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
        else:
            logger.info(
                "relocalize accepted",
                **source_kw,
                fitness=round(fitness, 3),
                time_cost_s=round(dt, 1),
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
