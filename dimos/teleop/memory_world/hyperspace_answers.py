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

"""The memory world's Hyperspace side: heat-map answers, clusters, routes.

A mixin for :class:`MemoryWorldModule` (``module.py`` is at the repository's
file-size limit). It owns:

* the :class:`HyperspaceSearch` over the recording's own keyframes, warmed on
  the prepare thread when the db exists, or built on request by the viewer's
  "Prepare search" button (:mod:`hyperspace_ingest` in a subprocess);
* the answer path: text -> clusters -> ``query_result`` (+ ``MSG_HEATMAP``
  voxels, ``query_pyramids``, one ``MSG_QUERY_IMAGE`` per evidence frame);
* the routes the viewer calls: typed questions, search status, tf frames
  for the orbit picker, and a planned route to a cluster.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from fastapi import HTTPException
import numpy as np
from pydantic import BaseModel, Field

from dimos.agents.skill_result import SkillResult
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.teleop.memory_world.embed import EmbeddingJob
from dimos.teleop.memory_world.hyperspace_ingest import ingest_command
from dimos.teleop.memory_world.hyperspace_search import (
    EVIDENCE_PER_CLUSTER,
    MAX_CLUSTERS,
    HeatmapAnswer,
    HyperspaceSearch,
    memory_db_for,
    memory_db_index_stamp,
    memory_db_ready,
)
from dimos.teleop.memory_world.messages import (
    MSG_HEATMAP,
    MSG_QUERY_IMAGE,
    encode_binary,
    encode_text,
)
from dimos.teleop.memory_world.query import (
    ClusterSummary,
    HighlightPath,
    HighlightPoint,
    MemoryQueryResult,
)
from dimos.teleop.memory_world.replay import frame_positions
from dimos.teleop.memory_world.route import (
    MLS_MAX_VOXELS,
    MlsRoutePlanner,
    RoutePlanner,
    mls_available,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A storey. Used to decide which floor the viewer is standing on: they cannot be standing
# on one above their own head, and the one they are on is within a storey below it.
STOREY_M = 3.0

# How many times a reload of the SAME index is retried before it is left alone, and the
# unit of the widening gap between those tries.
RELOAD_ATTEMPTS = 3
RELOAD_BACKOFF_S = 10.0

EVIDENCE_CLUSTERS = 16  # every place an answer names, in practice
# The ceiling is what the answer can NAME, not a round number chosen beside it. At 64 the
# budget was smaller than MAX_CLUSTERS x EVIDENCE_PER_CLUSTER and was spent first-come, so
# the best places took it all and the last places an answer named got no picture at all --
# while still reporting, in `n_evidence`, how many they had. Measured on
# sf_office1_2/main.db: "a monitor" reports [8,8,8,6,6,6,6,5,4,4,4,4] and published
# [8,8,8,6,6,6,6,5,4,4,3,0]. Still bounded, which was the point of having a ceiling.
EVIDENCE_IMAGES_MAX = MAX_CLUSTERS * EVIDENCE_PER_CLUSTER


class AskRequest(BaseModel):
    text: str = Field(min_length=1, max_length=400)


class NavigateRequest(BaseModel):
    query_id: str | None = None  # the answer the cluster belongs to, when the viewer knows it
    cluster: int = Field(default=0, ge=0)
    # Start from here (world xyz) instead of under the viewer.
    start: tuple[float, float, float] | None = None
    # Walk to the PHOTO the viewer has stepped to, not to the middle of the blob. The
    # index is the one the query-image header carries, so the viewer names a picture it
    # was actually sent rather than posting a position of its own.
    view: int | None = None


class HyperspaceAnswers:
    """Heat-map answers for the memory world. Expects the host module's
    ``config``, ``_ensure_store``, ``_broadcast``, ``_publish_query_result``,
    ``_clients_lock``, ``_active_query_images``, ``_camera_hfov``,
    ``_encode_jpeg``, ``_tf_tree``, ``_frame_pose_at``, ``_replay_index_json``
    and ``_cached_cloud``."""

    # The host's attributes and methods this mixin relies on.
    config: Any
    _clients_lock: threading.Lock
    _active_query_images: list[tuple[dict[str, Any], bytes]]
    _cached_cloud: tuple[dict[str, Any], bytes] | None
    _world_clients: Any
    _map_xyz: np.ndarray | None  # the whole map, before the viewer's stride
    _viewer_position: tuple[float, float, float] | None
    _store_lock: Any  # an RLock

    if TYPE_CHECKING:

        def _broadcast(self, message: bytes | str) -> None: ...
        def _publish_query_result(self, result: MemoryQueryResult) -> str: ...
        def _ensure_store(self) -> Any: ...
        def _camera_hfov(self) -> float: ...
        def _encode_jpeg(self, img: Any, max_size: int, quality: int) -> bytes: ...
        def _tf_tree(self) -> Any: ...
        def _effective_orbit_frame(self) -> str: ...
        def _frame_pose_at(self, frame: str, ts: float) -> Any: ...
        def _replay_index_json(self) -> dict[str, Any]: ...
        def find_in_memory(self, query: str) -> SkillResult: ...

    def _init_hyperspace(self) -> None:
        self._hyperspace: HyperspaceSearch | None = None
        self._hyperspace_lock = threading.Lock()
        # Serialises BUILDS. Kept separate from _hyperspace_lock, which guards the
        # published search and is held for the length of a question: warming under
        # that lock makes every question block for the whole warm.
        self._hyperspace_build_lock = threading.Lock()
        self._hyperspace_error: str | None = None
        self._adopting = threading.Lock()  # held while a background load is in flight
        self._adopted_stamp: tuple[int, float] = (0, 0.0)  # the index the search was built from
        # A reload that failed, and when. Neither extreme is right: retrying with nothing
        # recorded starts a fresh warm on every status poll for ever, and recording it as
        # simply "tried" makes ONE transient failure -- an allocation that lost a race
        # with another heavy process -- permanent, leaving the module answering from a
        # stale index with `ready: true` and no error. So: a few attempts, spaced out.
        self._failed_stamp: tuple[int, float] | None = None
        self._failed_count = 0
        self._failed_at = 0.0
        self._prepare_job = EmbeddingJob(
            on_finished=lambda _job: self._broadcast_search_status(),
            name="hyperspace ingest",
            done="search ready",
        )
        self._active_heatmap: tuple[dict[str, Any], bytes] | None = None
        self._active_pyramids: str | None = None
        # The Hyperspace answer on screen and the query id it was published under.
        self._last_answer: tuple[HeatmapAnswer | None, str | None] = (None, None)
        self._route_planner: RoutePlanner | MlsRoutePlanner | None = None
        # First in the lock order: the planner reads the orbit, which reads the replay.
        self._planner_lock = threading.RLock()
        self._orbit_cache: dict[str, dict[str, Any]] = {}

    # ---- loading -----------------------------------------------------------

    def _load_hyperspace(self, reload: bool = False) -> bool:
        """Warm the search over the recording's memory db when it has one. Slow (seconds); off the request path.
        Not while the ingest is still writing that db (it would load a partial index); *reload*
        replaces a search already loaded, for when the ingest has just finished."""
        # Read BEFORE the search is built. An ingest that writes more while we warm must
        # still read as a change at the next poll, which it would not if this were taken
        # afterwards -- the stamp would describe a db newer than the search built from it.
        stamp = memory_db_index_stamp(self.config.store_path)
        if stamp[0] <= 0:
            return False
        if self._prepare_job.status()["embedding"] == "running" and not reload:
            return False
        with self._hyperspace_lock:
            previous = self._hyperspace
            if previous is not None and not reload:
                return True
        # Built and warmed OUTSIDE `_hyperspace_lock`. Two bugs live here and the first
        # fix only moved the second one. Closing the old search first left `_hyperspace`
        # None for the seconds a warm takes and every question answered "Hyperspace is
        # reloading"; keeping it but warming under the lock meant every question BLOCKED
        # for the warm instead, because `_find_with_hyperspace` holds the same lock for
        # the length of an answer. A hung request is not an improvement on a failed one.
        # The build lock serialises builds; the query lock is taken only to read and to
        # swap.
        with self._hyperspace_build_lock:
            with self._hyperspace_lock:
                previous = self._hyperspace  # re-read: another build may have published
                if previous is not None and not reload:
                    return True
            try:
                search = HyperspaceSearch(
                    memory_db_for(self.config.store_path),
                    model_name=self.config.hyperspace_model_name,
                    world_frame=self.config.world_frame,
                    voxel_size=self.config.hyperspace_voxel_size,
                    device=self.config.hyperspace_device,
                    use_segments=self.config.hyperspace_segments,
                    refine=self.config.hyperspace_refine,
                )
                search.warm()
            except (Exception, SystemExit) as error:
                # SystemExit is how this package reports an expected failure, and it is
                # not an Exception. Letting it out leaves the search neither loaded nor
                # failed, so the viewer hides its button and the status poll starts a
                # fresh load thread every second, for ever, silently.
                logger.exception("hyperspace failed to load")
                if previous is not None:
                    # A reload that fails must not cost the working search -- it used to,
                    # because the close happened first and the error latched with nothing
                    # behind it. But returning without recording anything traded that for
                    # the opposite: the adopt compares `stamp` against `_adopted_stamp`,
                    # so an unchanged stamp and no error meant EVERY status poll started
                    # another full build and warm, for ever, with no backoff and nothing
                    # said. Measured at 7 attempts over 10 polls and climbing.
                    #
                    # Recording the stamp says "this index has been tried". The search we
                    # have keeps answering, no error is latched (the module is not
                    # broken), and an index that changes AGAIN still gets a fresh attempt,
                    # which is the one case worth retrying.
                    if self._failed_stamp == stamp:
                        self._failed_count += 1
                    else:
                        self._failed_stamp, self._failed_count = stamp, 1
                    self._failed_at = time.monotonic()
                    logger.warning(
                        "keeping the index that is already loaded; attempt %d of %d on this one",
                        self._failed_count,
                        RELOAD_ATTEMPTS,
                    )
                    return False
                self._hyperspace_error = str(error)[-200:] or type(error).__name__
                return False
            # A module stopped while this was warming must not be handed a live search:
            # stop() neither joins this thread nor holds `_hyperspace_lock`, so it sees
            # nothing to close and finishes, and publishing here would leave the model
            # and its store open in a stopped module until the process exits.
            stopping = getattr(self, "_stopping", None)  # a sibling mixin's, not ours
            if (stopping is not None and stopping.is_set()) or getattr(
                self, "_module_closed", False
            ):
                logger.info("stopped while the index was warming; closing it again")
                with contextlib.suppress(Exception):
                    search.close()
                return False
            with self._hyperspace_lock:
                replaced = self._hyperspace
                self._hyperspace = search
                self._adopted_stamp = stamp
                self._hyperspace_error = None
                self._failed_stamp, self._failed_count = None, 0
        # Closed after the swap and OUTSIDE both locks: close() takes the search's own
        # lock, so it waits on any query still inside it, and by now every new query
        # goes to the replacement.
        if replaced is not None and replaced is not search:
            with contextlib.suppress(Exception):
                replaced.close()
        self._broadcast_search_status()
        return True

    def _map_points(self) -> np.ndarray | None:
        """The ray-traced map's voxel centres, when the world cache is built: the whole
        map, not the stride-sampled payload the viewer gets."""
        if self._map_xyz is not None:
            return self._map_xyz
        cached = self._cached_cloud  # read once: a reopen clears it
        if cached is None:
            return None
        header, payload = cached
        n = int(header.get("n", 0))
        return np.frombuffer(payload, dtype=np.float32, count=n * 3).reshape(n, 3)

    def _hyperspace_ready(self) -> bool:
        return self._hyperspace is not None

    def _adopt_an_index_that_appeared(self) -> None:
        """Load a memory db that finished being written after this module started.

        The keyframes are in the recording now, so an ingest run from a terminal finishes
        into the very file this process already has open, with nothing to tell it. The
        viewer hides its Prepare button the moment the index is there, so without this the
        page waits on "loading Hyperspace" with no way to ask again.
        """
        if self._hyperspace_error is not None:
            return
        if self._prepare_job.status()["embedding"] == "running":
            return  # our own ingest, which adopts on its own when it finishes
        stamp = memory_db_index_stamp(self.config.store_path)
        if stamp[0] <= 0:
            return
        search = self._hyperspace
        # A DIFFERENT index, not merely a bigger one. Without a completion marker the
        # first keyframe flush makes the db read as ready, so a terminal ingest gets
        # adopted a second or two in -- and this used to latch on `self._hyperspace is
        # not None`, serving that one frame as the whole recording for the life of the
        # process. Comparing only "did it grow" fixed that case and left two others: a
        # drop_index and re-ingest landing on the same count, or on fewer keyframes,
        # would both keep serving an index that no longer exists. The stamp moves
        # whenever the keyframes are rewritten, in either direction.
        if search is not None and stamp == self._adopted_stamp:
            return
        if stamp == self._failed_stamp:
            # Tried and failed before. Give it a few more goes, spaced further apart each
            # time, so a transient failure recovers and a permanent one stops costing a
            # full text tower and every keyframe on every poll.
            if self._failed_count >= RELOAD_ATTEMPTS:
                return
            if time.monotonic() - self._failed_at < RELOAD_BACKOFF_S * self._failed_count:
                return
        if not self._adopting.acquire(blocking=False):
            return  # already loading; warming takes seconds and must not block the poll
        replacing = search is not None
        if replacing:
            logger.info(
                "the index changed (%d keyframes -> %d); loading it again",
                self._adopted_stamp[0],
                stamp[0],
            )

        def load() -> None:
            try:
                self._load_hyperspace(reload=replacing)
            finally:
                self._adopting.release()

        threading.Thread(target=load, name="hyperspace adopt", daemon=True).start()

    def _search_status(self) -> dict[str, Any]:
        """What the viewer shows: which engine answers, whether it is ready, and how preparing is going."""
        self._adopt_an_index_that_appeared()
        search = self._hyperspace
        status: dict[str, Any] = {
            "engine": "hyperspace" if search is not None else None,
            "ready": search is not None,
            "memory_db": str(memory_db_for(self.config.store_path)),
            "memory_db_present": memory_db_ready(self.config.store_path),
            "keyframes": search.keyframe_count if search else 0,
            "segments": search.segment_count if search else 0,
            "error": self._hyperspace_error,
        }
        job = self._prepare_job.status()
        status["prepare"] = {"state": job["embedding"], "progress": job["progress"]}
        return status

    def _broadcast_search_status(self) -> None:
        try:
            self._broadcast(encode_text("search_status", **self._search_status()))
        except Exception:
            logger.exception("search status broadcast failed")

    def _start_prepare(self) -> bool:
        """Embed the recording's keyframes into its memory db, in the background."""
        self._ensure_store()  # names the streams: the ingest must get the viewer's camera
        return self._prepare_job.start(
            ingest_command(
                self.config.store_path,
                model_name=self.config.hyperspace_model_name,
                device=self.config.hyperspace_ingest_device,
                hz=self.config.hyperspace_ingest_hz,
                streams={  # the camera the viewer shows, not whichever detects first
                    "image": self.config.image_stream_name,
                    "depth": self.config.depth_stream_name,
                    "camera_info": self.config.camera_info_stream_name,
                    "tf": self.config.tf_stream_name,
                },
            ),
            None,
            adopt=self._adopt_prepared,
        )

    def _adopt_prepared(self) -> None:
        self._load_hyperspace(reload=True)

    # ---- answering ---------------------------------------------------------

    def _find_with_hyperspace(self, phrase: str, started: float) -> SkillResult:
        # One question at a time, publication included: two answers interleaving
        # would leave the heat map of one under the clusters of the other.
        with self._hyperspace_lock:
            return self._answer(phrase, started)

    def _answer(self, phrase: str, started: float) -> SkillResult:
        search = self._hyperspace
        if search is None:  # the ingest just finished and is reloading it
            return SkillResult.fail("INDEX_NOT_READY", "Hyperspace is reloading; ask again")
        try:
            answer = search.query(phrase)
        except Exception as error:
            logger.exception("hyperspace query failed")
            return SkillResult.fail(
                "QUERY_FAILED", f"Hyperspace could not answer {phrase!r}: {error}"
            )
        if not answer.clusters:
            self._publish_empty(phrase, answer)
            return SkillResult.fail("NOT_FOUND", f"Nothing in the recording matches {phrase!r}")

        best = answer.clusters[0]
        short = phrase[:80]  # labels are capped at 120 characters; questions at 400
        result = MemoryQueryResult(
            answer=f"{phrase}: {len(answer.clusters)} place{'s' if len(answer.clusters) != 1 else ''}, "
            f"best {best.peak:.2f} from {best.views} view{'s' if best.views != 1 else ''}",
            focus_point=best.centre,
            points=[
                HighlightPoint(
                    position=cluster.centre,
                    label=f"#{cluster.index + 1} {short} ({cluster.peak:.2f}, {cluster.views} views)",
                    color="#ff5c3a" if cluster.index == 0 else "#ffb347",
                )
                for cluster in answer.clusters
            ],
            clusters=[
                ClusterSummary(
                    index=cluster.index,
                    centre=cluster.centre,
                    radius=cluster.radius,
                    score=cluster.score,
                    peak=cluster.peak,
                    n_voxels=cluster.n_voxels,
                    n_views=cluster.views,
                    n_evidence=len(cluster.evidence),
                    label=f"{short} #{cluster.index + 1}",
                )
                for cluster in answer.clusters
            ],
            engine="hyperspace",
            query_text=phrase,
        )
        query_id = self._publish_query_result(result)
        with self._clients_lock:
            if self._query_is_current(query_id):  # else a newer answer replaced it
                self._last_answer = (answer, query_id)
        self._publish_heatmap(query_id, answer)
        self._publish_pyramids(query_id, answer)
        # The pictures come from the recording (seconds on an mcap); the answer does not wait for them.
        threading.Thread(
            target=self._publish_cluster_images,
            args=(query_id, phrase, answer),
            daemon=True,
            name="MemoryWorldEvidence",
        ).start()
        return SkillResult(
            success=True,
            message=result.answer,
            duration_ms=(time.monotonic() - started) * 1000,
            metadata={
                "query_id": query_id,
                "query": phrase,
                "engine": "hyperspace",
                "clusters": [cluster.summary() for cluster in answer.clusters],
                "stats": answer.stats,
                "seconds": round(answer.seconds, 3),
            },
        )

    def _publish_empty(self, phrase: str, answer: HeatmapAnswer) -> None:
        result = MemoryQueryResult(
            answer=f"Nothing matches {phrase!r}", engine="hyperspace", query_text=phrase
        )
        query_id = self._publish_query_result(result)
        with self._clients_lock:
            if self._query_is_current(query_id):  # else a newer answer replaced it
                self._last_answer = (answer, query_id)
        self._publish_heatmap(query_id, answer)
        self._publish_pyramids(query_id, answer)  # clears the previous answer's frusta too

    def _publish_heatmap(self, query_id: str, answer: HeatmapAnswer) -> None:
        payload = (
            np.ascontiguousarray(answer.centres, dtype="<f4").tobytes()
            + np.clip(np.round(answer.scores * 255), 0, 255).astype(np.uint8).tobytes()
            + np.ascontiguousarray(answer.cluster_of, dtype="<i2").tobytes()
        )
        header = {
            "query_id": query_id,
            "n": int(answer.n_voxels),
            "voxel_size": float(answer.voxel_size),
            "frame": answer.frame,
            "clusters": len(answer.clusters),
            "stats": {k: v for k, v in answer.stats.items() if not isinstance(v, dict | list)},
            "seconds": round(answer.seconds, 3),
        }
        with self._clients_lock:  # state and send together, under the answer's lock
            if not self._query_is_current(query_id):
                return  # a newer answer replaced this one
            self._active_heatmap = (header, payload)
            message = encode_binary(MSG_HEATMAP, header, payload)
            for client in tuple(self._world_clients):
                client.send_threadsafe(message)

    def empty_overlay_messages(self, query_id: str) -> tuple[bytes, str]:
        """The pair that takes a previous answer's heat map and frusta off the screen.

        Here rather than at the caller because this is where the header's shape is
        decided: the viewer's `heatmap.set` clears and returns on ``n == 0``, but the
        tour card reads ``header.seconds`` without guarding it, so a short header
        throws in the browser instead of clearing it.
        """
        header = {
            "query_id": query_id,
            "n": 0,
            "voxel_size": 0.0,
            "frame": self.config.world_frame,
            "clusters": 0,
            "stats": {},
            "seconds": 0.0,
        }
        return (
            encode_binary(MSG_HEATMAP, header, b""),
            encode_text("query_pyramids", query_id=query_id, pyramids=[]),
        )

    def _publish_pyramids(self, query_id: str, answer: HeatmapAnswer) -> None:
        message = encode_text(
            "query_pyramids",
            query_id=query_id,
            pyramids=[pyramid.summary() for pyramid in answer.pyramids],
        )
        with self._clients_lock:
            if not self._query_is_current(query_id):
                return  # a newer answer replaced this one
            self._active_pyramids = message
            for client in tuple(self._world_clients):
                client.send_threadsafe(message)

    def _publish_cluster_images(self, query_id: str, phrase: str, answer: HeatmapAnswer) -> None:
        """Every cluster's evidence frames, posed where their cameras stood."""
        hfov_deg = self._camera_hfov()
        sent: list[tuple[dict[str, Any], bytes]] = []
        # Decoding frames out of an mcap costs CPU the next question needs, so only the
        # best places get pictures; the rest still have their heat and markers.
        for cluster in answer.clusters[:EVIDENCE_CLUSTERS]:
            for evidence in cluster.evidence:
                if len(sent) >= EVIDENCE_IMAGES_MAX:
                    break
                try:
                    with self._store_lock:  # resolved and read together: a reopen swaps the store
                        images = self._ensure_store().streams[self.config.image_stream_name]
                        frame = images.at(evidence.ts, tolerance=0.02).first()
                        image = frame.data
                    jpeg = self._encode_jpeg(
                        image, self.config.query_image_max_size, self.config.thumbnail_jpeg_quality
                    )
                except Exception as error:
                    logger.warning(
                        "no frame at %.3f for cluster %d: %s", evidence.ts, cluster.index, error
                    )
                    continue
                height, width = frame.data.shape[:2]
                header = {
                    "query_id": query_id,
                    "index": len(sent),
                    "cluster": cluster.index,
                    "label": f"{phrase[:80]} #{cluster.index + 1} ({evidence.score:+.2f})",
                    "position": list(evidence.position),
                    "forward": list(evidence.forward),
                    "up": list(evidence.up),
                    "hfov_deg": hfov_deg,
                    "aspect": float(width) / float(height),
                    "distance_m": float(self.config.query_image_distance_m),
                    "uv": [evidence.image_uv[0] / width, evidence.image_uv[1] / height],
                    "point": list(evidence.point),
                    "score": round(evidence.score, 3),
                    "channel": evidence.channel,
                    "ts": evidence.ts,
                }
                with self._clients_lock:
                    if not self._query_is_current(query_id):
                        return  # a newer question replaced this one
                    if self._active_query_images is not sent:
                        self._active_query_images = sent  # the list itself; appended under the lock
                    sent.append((header, jpeg))
                self._broadcast(encode_binary(MSG_QUERY_IMAGE, header, jpeg))

    def _query_is_current(self, query_id: str) -> bool:
        current = getattr(self, "_active_query_result", None)
        return isinstance(current, dict) and current.get("query_id") == query_id

    def _resend_hyperspace(self, conn: Any) -> None:
        """A viewer that (re)connects gets the current heat map and pyramids after the result."""
        with self._clients_lock:  # queued under the lock: they belong to the result just sent
            if self._active_heatmap is not None:
                conn.send_threadsafe(encode_binary(MSG_HEATMAP, *self._active_heatmap))
            if self._active_pyramids is not None:
                conn.send_threadsafe(self._active_pyramids)
        conn.send_threadsafe(encode_text("search_status", **self._search_status()))

    # ---- navigation --------------------------------------------------------

    def _planner(self) -> RoutePlanner | MlsRoutePlanner:
        """The MLS 3D planner over the map, built once; the 2D costmap when the
        binding is missing or the map is a city."""
        with self._planner_lock:
            if self._route_planner is not None:
                return self._route_planner
            points = self._map_points()
            if points is None:
                raise HTTPException(status_code=503, detail="the map is still building")
            voxels = points.astype(np.float64)
            voxel_size = float(self.config.voxel_size)
            if mls_available() and len(voxels) <= MLS_MAX_VOXELS:
                started = time.monotonic()
                mls = MlsRoutePlanner(voxels, voxel_size=voxel_size)
                logger.info(
                    "MLS planner: %d surface cells from %d voxels in %.1f s",
                    mls.surface_cells,
                    len(voxels),
                    time.monotonic() - started,
                )
                self._route_planner = mls
            else:
                logger.info(
                    "costmap planner (mls binding %s, %d voxels)",
                    "present" if mls_available() else "missing",
                    len(voxels),
                )
                known = self._orbit_positions_for(self._effective_orbit_frame()).get("positions")
                if not known:
                    # The costmap's free corridor IS the robot's path. With no path there
                    # is nothing to plan over, and a 500 out of RoutePlanner says less than
                    # this does. An explicit start reaches here too, hence the check.
                    raise HTTPException(status_code=503, detail="the robot's path is not known yet")
                path = np.asarray(known, dtype=np.float64).reshape(-1, 3)
                self._route_planner = RoutePlanner.from_voxels(voxels, path, voxel_size=voxel_size)
            return self._route_planner

    def _robot_end_pose(self) -> tuple[float, float, float]:
        """Where the robot's orbit frame was at the end of the recording."""
        positions = self._orbit_positions_for(self._effective_orbit_frame()).get("positions") or []
        if positions:
            return tuple(float(v) for v in positions[-1])  # type: ignore[return-value]
        raise HTTPException(status_code=503, detail="the robot's path is not known yet")

    def _ground_under_viewer(self) -> tuple[float, float, float] | None:
        """Where the viewer is standing, at a height the planner can actually use.

        A route has to start where the person asking for it is standing. It used to start
        where the ROBOT stopped at the end of the recording, so walking anywhere and
        pressing Navigate drew a green tube that began somewhere else entirely.

        The viewer's x and y are kept exactly. Their z is not: it is a CAMERA height, not
        a floor, and the client only started sending a real one recently -- for most of
        this package's history `getViewerRobotPosition()` returned `[x, y, 0]`, a literal
        zero. So z is used only to decide WHICH FLOOR, never as the answer, and a zero
        from an old client has to keep working.

        The height comes from the nearest sample of the robot's own path, and that is the
        point of this function. Two earlier versions took it from the map voxels in a
        column around the viewer and both were wrong in a way only the live demo showed:
        the highest voxel at or below `viewer[2]` is a test against zero, which on a floor
        at 0.7 matched nothing; and the LOWEST voxel in the column is a stray return under
        the floor, measured at z=-1.24 and -1.56 on the office recording. A start at an
        impossible height is worse than a wrong one -- the planner snapped it to some far
        corner of its graph and returned the SAME 47-point route for two viewers 2 m
        apart, beginning 4 m from either of them, while the payload's `start` field went
        on claiming the viewer's position.

        The robot drove its path, so every height along it is one the planner can stand
        at. Restrict to the samples in the storey-deep band below the camera, then take
        the nearest of those in x and y and use its z.

        That band is not the same thing as the viewer's floor, and on a map with two
        levels less than a storey apart it can hold both -- a viewer on a platform at 1.4
        with their camera at 3.0 gets the 0.3 floor underneath it if that is nearer in x
        and y. Separating those two needs a prior on how tall the person is, which is
        exactly the guess this function exists to avoid; a storey-deep band is the widest
        rule that needs no such guess. Single-level maps, which is every map this demo
        runs on, are unaffected.
        """
        with self._clients_lock:
            viewer = self._viewer_position
        if viewer is None:
            return None
        known = self._orbit_positions_for(self._effective_orbit_frame()).get("positions")
        if not known:
            return None
        path = np.asarray(known, dtype=np.float64).reshape(-1, 3)
        if not len(path):
            return None
        # Which floor, then which point on it. Nearest in the plane alone puts a viewer
        # standing on a mezzanine three metres below their own feet; nearest in 3-D is
        # worse, and worse on the maps we actually have -- the gap between a camera and
        # the sensor that drove the path is metres, so on a SINGLE-storey map a sample
        # upstairs can be nearer the viewer's eyes than the floor they are standing on
        # (measured: samples at z=0.3 and z=2.8, a ground-floor viewer at 1.7, and 2.8
        # wins).
        #
        # The signal that does not need a guess about human height: you cannot be standing
        # on a floor above your own head, and the one you are on is within a storey below
        # it. Restrict to those, then take the nearest in the plane. Nothing in the band
        # (an old client sending z=0, a viewer flying) falls back to the plane over every
        # sample, which is the behaviour before any of this.
        flat = (path[:, 0] - viewer[0]) ** 2 + (path[:, 1] - viewer[1]) ** 2
        on_this_floor = (path[:, 2] <= viewer[2]) & (path[:, 2] >= viewer[2] - STOREY_M)
        usable = np.flatnonzero(on_this_floor)
        nearest = int(usable[np.argmin(flat[usable])]) if len(usable) else int(np.argmin(flat))
        return (float(viewer[0]), float(viewer[1]), float(path[nearest, 2]))

    def _navigate_to(self, request: NavigateRequest) -> dict[str, Any]:
        with self._clients_lock:
            answer, query_id = self._last_answer
            active = getattr(self, "_active_query_result", None)
        if not isinstance(active, dict) or active.get("query_id") != query_id:
            raise HTTPException(status_code=409, detail="the last answer is not a Hyperspace one")
        if request.query_id not in (None, query_id):
            raise HTTPException(status_code=409, detail="that answer has been replaced")
        if answer is None or request.cluster >= len(answer.clusters):
            raise HTTPException(status_code=404, detail="no such cluster in the last answer")
        cluster = answer.clusters[request.cluster]
        # Where the viewer is, then where the robot ended. An explicit start still wins:
        # the request carries one when the caller knows better than either.
        start = request.start or self._ground_under_viewer() or self._robot_end_pose()
        with self._clients_lock:
            images = list(self._active_query_images)

        def pose_of(index: int) -> tuple[float, float, float] | None:
            header = images[index][0]
            if header.get("query_id") != query_id or header.get("cluster") != cluster.index:
                return None
            try:
                where = tuple(float(v) for v in (header.get("position") or []))
            except (TypeError, ValueError):
                return None
            if len(where) != 3 or not all(math.isfinite(v) for v in where):
                return None
            return where  # type: ignore[return-value]

        asked: tuple[float, float, float] | None = None
        if request.view is not None:
            if not 0 <= request.view < len(images):
                raise HTTPException(status_code=404, detail="no such view in the last answer")
            asked = pose_of(request.view)
            if asked is None:
                raise HTTPException(status_code=409, detail="that view is not in this place")

        # The candidates, in order of what the person asked for. The centre of a cluster
        # is a weighted mean of its voxels, so for a thing on a wall it is INSIDE the
        # wall: neither where the viewer is looking nor anywhere a body can stand. A
        # photo's camera pose is somewhere the robot already stood.
        #
        # Every one of them is tried, and that is the point. Measured on the office
        # recording from one standing position: the centre was unreachable, only 2 of
        # cluster 0's 12 views could be routed to, and the FIRST photo of all 12 places
        # was unreachable -- so Navigate answered "no route" for every place on screen
        # while a 3.48 m route to the second photo of the first place existed the whole
        # time. The closest photo the robot can still reach is a better answer than
        # refusing, and saying WHICH one it picked keeps it honest.
        # Nearest first. `images` is in publication (score) order, and taking the first
        # routable one in THAT order hands back a photo 10 m away when a reachable one
        # 1 m away was in the same list -- which is not what the paragraph above promises
        # and not what someone pressing Navigate wants to walk.
        others = sorted(
            (i for i in range(len(images)) if i != request.view and pose_of(i) is not None),
            key=lambda i: math.dist(start, pose_of(i)),  # type: ignore[arg-type]
        )
        candidates: list[tuple[int | None, tuple[float, float, float]]] = []
        if request.view is not None and asked is not None:
            candidates.append((request.view, asked))
        candidates.append((None, tuple(cluster.centre)))
        candidates.extend((i, pose) for i in others if (pose := pose_of(i)) is not None)

        planner = self._planner()
        # Everything the payload needs is captured HERE, not read off the loop variable
        # afterwards. `route` does survive the loop correctly today, because the only way
        # out with a goal is the break -- but a payload that reads a name the loop last
        # happened to leave behind is one edit away from reporting a rejected candidate's
        # length beside the accepted candidate's points.
        goal, goal_view, points, taken = None, None, [], None
        # Every candidate, with no cap. A cap of 8 was arbitrary and did the very thing
        # this loop exists to stop: with twelve photos of a place and only the ninth
        # reachable, it refused while a 9 m route existed. The list is already bounded --
        # it is one place's photographs, not the whole answer's.
        for view_index, candidate in candidates:
            route = (
                planner.plan(tuple(start), tuple(candidate))
                if isinstance(planner, MlsRoutePlanner)
                else planner.plan(start[:2], candidate[:2])
            )
            if route is None:
                continue
            found = [(float(x), float(y), float(z)) for x, y, z in route.points]
            if len(found) < 2:
                continue
            if math.dist(found[0], found[-1]) <= 1e-9:
                continue  # went nowhere; see below
            goal, goal_view, points, taken = candidate, view_index, found, route
            break
        if goal is None or taken is None:
            raise HTTPException(status_code=422, detail="no route through the known free space")
        # The "went nowhere" test in the loop above: a route has to GO somewhere. The
        # planner can return a handful of identical points when it cannot connect the
        # start to the goal, and a length check alone passes them -- measured live at
        # three copies of (2.95, 2.15, 0.7), length 0.0, returned as HTTP 200 with the
        # goal 2.45 m away, so the viewer drew a tube of no length and the person was
        # told a route existed.
        #
        # It tests the route's own EXTENT, not its progress toward the goal, because
        # extent is what actually failed. Comparing distances to the goal imports that
        # goal's height into the verdict: a real 0.5 m route down a 15 cm step, to a
        # camera pose 1.8 m up, closed 0.38 m of horizontal gap and was still refused,
        # since 3-D it read 1.856 -> 1.951. The plane fixes that particular case and
        # still asks a question the planner was never posed -- in the costmap branch z is
        # not even an input. Zero extent is unambiguous and needs nothing but the points.
        payload = {
            "query_id": query_id,
            "cluster": cluster.index,
            "start": [float(v) for v in start],
            "goal": [float(v) for v in goal],
            "view": goal_view,  # which picture, when the route is to one
            "length_m": round(taken.length_m, 2),
            "points": [[round(v, 3) for v in p] for p in points],
            "cells": taken.cells,
            "planner": taken.planner,
        }
        with self._clients_lock:  # still the answer on screen? then reconnects get the route too
            if self._active_query_result is active:
                active["route"] = HighlightPath(
                    points=points, label=f"Route to #{cluster.index + 1}", color="#64ff8f"
                ).model_dump(mode="json")
            else:
                raise HTTPException(status_code=409, detail="the answer changed while planning")
        self._broadcast(encode_text("route", **payload))
        return payload

    # ---- the recording's own costmap (Go2 recordings carry one) -------------

    def _add_route_to_result(self, result: MemoryQueryResult) -> None:
        # Routes are server-owned: only the planner may label one collision-aware.
        result.route = None
        with self._clients_lock:
            viewer_position = self._viewer_position
        if result.focus_point is None or viewer_position is None:
            return
        try:
            with self._store_lock:
                store = self._ensure_store()
                if "global_costmap" not in store.list_streams():
                    return
                costmap = store.streams.global_costmap.last().data
            route = min_cost_astar(
                costmap,
                goal=result.focus_point[:2],
                start=viewer_position[:2],
            )
            if route is None:
                return
            # Half a cell, for the same reason RoutePlanner.plan adds it: min_cost_astar
            # returns OccupancyGrid.grid_to_world, which is `origin + cell * resolution`
            # -- the cell's CORNER. Without it every waypoint of a route over the
            # recording's own costmap sits down and left of the cell it was planned
            # through. The round-41 fix only reached the other caller.
            half = costmap.resolution / 2
            points = [(pose.x + half, pose.y + half, pose.z + 0.08) for pose in route.poses]
            if len(points) >= 2:
                result.route = HighlightPath(
                    points=points,
                    label="Route to answer",
                    color="#64ff8f",
                )
        except Exception:
            logger.exception("failed to build route to memory query result")

    # ---- tf frames for the orbit picker -----------------------------------

    def _tf_frames(self) -> list[str]:
        tree = self._tf_tree()
        return sorted(tree.frames) if tree is not None else []

    def _orbit_positions_for(self, frame: str) -> dict[str, Any]:
        """Where *frame* was at each replay scan (cached per frame)."""
        with self._planner_lock:  # a reopen clears the cache under it
            if frame in self._orbit_cache:
                return self._orbit_cache[frame]
            try:
                index = self._replay_index_json()
            except Exception as error:  # building, or failed: 503, like the replay routes
                raise HTTPException(
                    status_code=503, detail=f"replay {self._replay_progress}"
                ) from error
            if frame == index.get("orbit", {}).get("frame"):
                self._orbit_cache[frame] = index["orbit"]
                return index["orbit"]
            tree = self._tf_tree()
            if tree is None or frame not in tree.frames:
                raise HTTPException(status_code=404, detail=f"no tf frame {frame!r}")
            stamps = np.asarray(
                index.get("scans") or [], dtype=np.float64
            )  # one stamp per replay scan
            positions = frame_positions(stamps, lambda ts: self._frame_pose_at(frame, ts))
            result = {"frame": frame, "positions": positions}
            self._orbit_cache[frame] = result
            return result

    # ---- routes ------------------------------------------------------------

    def _setup_hyperspace_routes(self, app: Any) -> None:
        base = self.config.client_route

        @app.get(f"{base}/search/status")  # type: ignore[misc]
        async def memory_world_search_status() -> dict[str, Any]:
            return await asyncio.to_thread(self._search_status)

        @app.post(f"{base}/search/prepare")  # type: ignore[misc]
        async def memory_world_search_prepare() -> dict[str, Any]:
            """Embed the recording for Hyperspace, in the background; poll the status."""
            if not await asyncio.to_thread(self._start_prepare):
                raise HTTPException(status_code=409, detail="already preparing")
            return await asyncio.to_thread(self._search_status)

        @app.post(f"{base}/ask")  # type: ignore[misc]
        async def memory_world_ask(request: AskRequest) -> dict[str, Any]:
            """A typed question: same path as a spoken one."""
            self._broadcast(encode_text("voice_transcript", text=request.text))
            outcome = await asyncio.to_thread(self.find_in_memory, request.text)
            return {
                "success": outcome.success,
                "answer": outcome.message,
                "metadata": outcome.metadata,
            }

        @app.get(f"{base}/frames")  # type: ignore[misc]
        async def memory_world_frames() -> dict[str, Any]:
            frames = await asyncio.to_thread(self._tf_frames)
            # The configured frame only if tf has it: /orbit falls back to the camera,
            # and offering a default nothing matches lets the browser pick its own.
            chosen = self.config.orbit_frame
            if frames and chosen not in frames:
                # to_thread, like every other route here: _camera_frame takes the store
                # lock and reads the store on a cold cache, and the cache's only warmer
                # runs INSIDE that lock. On the event loop it freezes every websocket and
                # every route for every client, not just this request. Called once, too.
                camera = await asyncio.to_thread(self._camera_frame)
                chosen = camera if camera in frames else frames[0]
            return {"frames": frames, "default": chosen}

        @app.get(f"{base}/orbit")  # type: ignore[misc]
        async def memory_world_orbit(frame: str) -> dict[str, Any]:
            return await asyncio.to_thread(self._orbit_positions_for, frame)

        @app.post(f"{base}/navigate")  # type: ignore[misc]
        async def memory_world_navigate(request: NavigateRequest) -> dict[str, Any]:
            return await asyncio.to_thread(self._navigate_to, request)

        @app.get(f"{base}/answer")  # type: ignore[misc]
        async def memory_world_answer() -> dict[str, Any]:
            """The last answer's clusters and stats, for scripts and the tour."""
            answer, _ = self._last_answer
            if answer is None:
                return {"text": None, "clusters": []}
            return {
                "text": answer.text,
                "frame": answer.frame,
                "voxels": answer.n_voxels,
                "clusters": [c.summary() for c in answer.clusters],
                "stats": {k: v for k, v in answer.stats.items() if not isinstance(v, dict | list)},
                "seconds": round(answer.seconds, 3),
            }
