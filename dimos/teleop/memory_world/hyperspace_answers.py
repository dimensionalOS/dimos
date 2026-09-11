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

* the :class:`HyperspaceSearch` over ``<recording>.hyperspace.db``, warmed on
  the prepare thread when the db exists, or built on request by the viewer's
  "Prepare search" button (:mod:`hyperspace_ingest` in a subprocess);
* the answer path: text -> clusters -> ``query_result`` (+ ``MSG_HEATMAP``
  voxels, ``query_pyramids``, one ``MSG_QUERY_IMAGE`` per evidence frame);
* the routes the viewer calls: typed questions, search status, tf frames
  for the orbit picker, and a planned route to a cluster.
"""

from __future__ import annotations

import asyncio
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
    HeatmapAnswer,
    HyperspaceSearch,
    memory_db_for,
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

EVIDENCE_CLUSTERS = 8


class AskRequest(BaseModel):
    text: str = Field(min_length=1, max_length=400)


class NavigateRequest(BaseModel):
    query_id: str | None = None  # the answer the cluster belongs to, when the viewer knows it
    cluster: int = Field(default=0, ge=0)
    # Start from here (world xyz) instead of where the robot ended the recording.
    start: tuple[float, float, float] | None = None


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
        def _frame_pose_at(self, frame: str, ts: float) -> Any: ...
        def _replay_index_json(self) -> dict[str, Any]: ...
        def find_in_memory(self, query: str) -> SkillResult: ...

    def _init_hyperspace(self) -> None:
        self._hyperspace: HyperspaceSearch | None = None
        self._hyperspace_lock = threading.Lock()
        self._hyperspace_error: str | None = None
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
        self._planner_lock = threading.Lock()
        self._orbit_cache: dict[str, dict[str, Any]] = {}

    # ---- loading -----------------------------------------------------------

    def _load_hyperspace(self, reload: bool = False) -> bool:
        """Warm the search over the recording's memory db when it has one. Slow (seconds); off the request path.
        Not while the ingest is still writing that db (it would load a partial index); *reload*
        replaces a search already loaded, for when the ingest has just finished."""
        if not memory_db_ready(self.config.store_path):
            return False
        if self._prepare_job.status()["embedding"] == "running" and not reload:
            return False
        with self._hyperspace_lock:
            if self._hyperspace is not None and not reload:
                return True
            if self._hyperspace is not None:
                self._hyperspace.close()
                self._hyperspace = None
            try:
                search = HyperspaceSearch(
                    memory_db_for(self.config.store_path),
                    model_name=self.config.hyperspace_model_name,
                    world_frame=self.config.world_frame,
                    voxel_size=self.config.hyperspace_voxel_size,
                    device=self.config.hyperspace_device,
                    use_segments=self.config.hyperspace_segments,
                    refine=self.config.hyperspace_refine,
                    scene=self._map_points(),
                )
                search.warm()
            except Exception as error:
                logger.exception("hyperspace failed to load")
                self._hyperspace_error = str(error)[-200:]
                return False
            self._hyperspace = search
            self._hyperspace_error = None
        self._broadcast_search_status()
        return True

    def _map_points(self) -> np.ndarray | None:
        """The ray-traced map's voxel centres, when the world cache is built: the whole
        map, not the stride-sampled payload the viewer gets."""
        if self._map_xyz is not None:
            return self._map_xyz
        if self._cached_cloud is None:
            return None
        header, payload = self._cached_cloud
        n = int(header.get("n", 0))
        return np.frombuffer(payload, dtype=np.float32, count=n * 3).reshape(n, 3)

    def _hyperspace_ready(self) -> bool:
        return self._hyperspace is not None

    def _search_status(self) -> dict[str, Any]:
        """What the viewer shows: which engine answers, whether it is ready, and how preparing is going."""
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
        return self._prepare_job.start(
            ingest_command(
                self.config.store_path,
                model_name=self.config.hyperspace_model_name,
                device=self.config.hyperspace_ingest_device,
                hz=self.config.hyperspace_ingest_hz,
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
        assert search is not None
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
            f"best {best.peak:.2f} from {len(best.evidence)} view{'s' if len(best.evidence) != 1 else ''}",
            focus_point=best.centre,
            points=[
                HighlightPoint(
                    position=cluster.centre,
                    label=f"#{cluster.index + 1} {short} ({cluster.peak:.2f}, {len(cluster.evidence)} views)",
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
        with self._clients_lock:
            self._active_heatmap = (header, payload)
        self._broadcast(encode_binary(MSG_HEATMAP, header, payload))

    def _publish_pyramids(self, query_id: str, answer: HeatmapAnswer) -> None:
        message = encode_text(
            "query_pyramids",
            query_id=query_id,
            pyramids=[pyramid.summary() for pyramid in answer.pyramids],
        )
        with self._clients_lock:
            self._active_pyramids = message
        self._broadcast(message)

    def _publish_cluster_images(self, query_id: str, phrase: str, answer: HeatmapAnswer) -> None:
        """Every cluster's evidence frames, posed where their cameras stood."""
        store = self._ensure_store()
        images = store.streams[self.config.image_stream_name]
        hfov_deg = self._camera_hfov()
        sent: list[tuple[dict[str, Any], bytes]] = []
        # Decoding frames out of an mcap costs CPU the next question needs, so only the
        # best places get pictures; the rest still have their heat and markers.
        for cluster in answer.clusters[:EVIDENCE_CLUSTERS]:
            for evidence in cluster.evidence:
                try:
                    with self._store_lock:  # the scrubber reads the same connection
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
        with self._clients_lock:
            heatmap, pyramids = self._active_heatmap, self._active_pyramids
        if heatmap is not None:
            conn.send_threadsafe(encode_binary(MSG_HEATMAP, *heatmap))
        if pyramids is not None:
            conn.send_threadsafe(pyramids)
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
                path = np.asarray(
                    self._orbit_positions_for(self.config.orbit_frame).get("positions") or [],
                    dtype=np.float64,
                ).reshape(-1, 3)
                self._route_planner = RoutePlanner.from_voxels(voxels, path, voxel_size=voxel_size)
            return self._route_planner

    def _robot_end_pose(self) -> tuple[float, float, float]:
        """Where the robot's orbit frame was at the end of the recording."""
        positions = self._orbit_positions_for(self.config.orbit_frame).get("positions") or []
        if positions:
            return tuple(float(v) for v in positions[-1])  # type: ignore[return-value]
        raise HTTPException(status_code=503, detail="the robot's path is not known yet")

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
        start = request.start or self._robot_end_pose()
        planner = self._planner()
        route = (
            planner.plan(tuple(start), tuple(cluster.centre))
            if isinstance(planner, MlsRoutePlanner)
            else planner.plan(start[:2], cluster.centre[:2])
        )
        if route is None:
            raise HTTPException(status_code=422, detail="no route through the known free space")
        points = [(float(x), float(y), float(z)) for x, y, z in route.points]
        if len(points) < 2:
            raise HTTPException(status_code=422, detail="already there")
        payload = {
            "query_id": query_id,
            "cluster": cluster.index,
            "start": [float(v) for v in start],
            "goal": [float(v) for v in cluster.centre],
            "length_m": round(route.length_m, 2),
            "points": [[round(v, 3) for v in p] for p in points],
            "cells": route.cells,
            "planner": route.planner,
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
            points = [(pose.x, pose.y, pose.z + 0.08) for pose in route.poses]
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
        if frame in self._orbit_cache:
            return self._orbit_cache[frame]
        index = self._replay_index_json()
        if frame == index.get("orbit", {}).get("frame"):
            self._orbit_cache[frame] = index["orbit"]
            return index["orbit"]
        tree = self._tf_tree()
        if tree is None or frame not in tree.frames:
            raise HTTPException(status_code=404, detail=f"no tf frame {frame!r}")
        stamps = np.asarray(index.get("scans") or [], dtype=np.float64)  # one stamp per replay scan
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
            return {
                "frames": await asyncio.to_thread(self._tf_frames),
                "default": self.config.orbit_frame,
            }

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
