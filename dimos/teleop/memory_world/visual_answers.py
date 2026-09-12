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

"""The SigLIP fallback of the memory world: the per-frame visual index, the
"add embeddings" job, and answers placed by raycasting hot patches through depth.

Mixed into MemoryWorldModule; the Hyperspace path (hyperspace_answers.py) is the
one the demo uses, this one answers when a recording has no Hyperspace memory."""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.agents.skill_result import SkillResult
from dimos.teleop.memory_world.embed import EmbeddingJob, siglipify_command, siglipify_config
from dimos.teleop.memory_world.messages import MSG_QUERY_IMAGE, encode_binary
from dimos.teleop.memory_world.query import ClusterSummary, HighlightPoint, MemoryQueryResult
from dimos.teleop.memory_world.recording import depth_info_stream_for
from dimos.teleop.memory_world.tf_tree import pose_matrix
from dimos.teleop.memory_world.visual_search import (
    PatchHit,
    Place,
    VisualMemoryIndex,
    cluster_hits,
    cluster_places,
    hot_patches,
    patch_world_position,
    sensor_intrinsics,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _best_phrase(best: Place, located: bool) -> str:
    """How to describe the place an answer flies to.

    The two producers rank differently, and the sentence has to say which one it got.
    `cluster_hits` (the depth path) ranks by VIEWING DIRECTIONS first, so its `places[0]`
    is the most-seen place and a higher-scoring one is drawn in the same world at the same
    moment with its own number printed on it -- "best match 0.150" beside a marker reading
    "0.170" is a contradiction the reader has no way to resolve. Name both numbers there,
    the way the Hyperspace sentence already does. `cluster_places` (no depth) really does
    rank by similarity, so "best match" is true on that branch and stays.
    """
    if not located:
        return f"best match {best.similarity:+.3f}"
    return f"best {best.similarity:+.3f} from {best.views} view{'s' if best.views != 1 else ''}"


def _place_metadata(place: Place, located: bool) -> dict[str, Any]:
    """One place, as the skill result reports it.

    `views` is measured only on the depth path; on the other one it is the dataclass
    default, and a reader cannot tell a measured 1 from an unmeasured one. So it is
    omitted there rather than guessed. This is a function and not an inline dict because
    the marker label makes the same decision, and making it twice is how the first version
    of this fix dropped the count from the label and kept sending it in the payload.
    """
    reported: dict[str, Any] = {"position": place.position, "similarity": place.similarity}
    if located:
        reported["views"] = place.views
    return reported


class VisualAnswers:
    """Needs, from the module: ``config``, the store and index locks,
    ``_ensure_store``, ``_ensure_visual_index``, ``_reopen_recording``,
    ``_hyperspace_ready``, ``_query_is_current``, ``_camera_hfov``, ``_encode_jpeg``, ``_broadcast``,
    ``whisper`` and the client bookkeeping."""

    config: Any
    _store_lock: Any
    _index_lock: Any
    _clients_lock: Any
    _visual_index: VisualMemoryIndex | None
    _index_progress: str
    _embed_job: EmbeddingJob
    _active_query_images: list[tuple[dict[str, Any], bytes]]
    _cached_image_poses: tuple[dict[str, Any], bytes] | None

    if TYPE_CHECKING:

        def _ensure_store(self) -> Any: ...
        def _ensure_visual_index(self) -> VisualMemoryIndex: ...
        def _reopen_recording(self) -> None: ...
        def _hyperspace_ready(self) -> bool: ...
        def _query_is_current(self, query_id: str) -> bool: ...
        def _camera_hfov(self) -> float: ...
        def _broadcast(self, payload: bytes) -> None: ...
        @property
        def whisper(self) -> Any: ...
        @staticmethod
        def _encode_jpeg(img: Any, max_size: int, quality: int) -> bytes: ...

    def _build_visual_index(self) -> None:
        """Load the recording's index, building it first when configured to.

        Slow and one-shot; runs off the request path. A recording with no
        vectors and no build configured is left for the viewer's "Add
        embeddings" button. An index the recording holds for another model,
        camera, pose convention or world frame is reported, not used.
        """
        with self._store_lock:
            missing = self.config.image_stream_name not in self._ensure_store().list_streams()
        if missing:
            # Nothing to index; saves loading a 3.7 GB model to find that out.
            self._index_progress = f"no {self.config.image_stream_name!r} stream"
            logger.warning("visual index skipped: %s", self._index_progress)
            return
        stale = ""
        with self._store_lock, self._index_lock:  # the build reads every image
            index = self._ensure_visual_index()
            try:
                existing = index.count()
            except ValueError as mismatch:  # another model, camera, pose convention or frame
                logger.warning("visual index will be rebuilt: %s", mismatch)
                existing = 0  # build() drops the stale rows once it has vectors to replace them
                stale = str(mismatch)  # said below, since 'no embeddings' is not why
            except Exception as error:
                self._index_progress = f"failed: {error}"
                logger.exception("visual index unusable")
                return
            if existing == 0 and not self.config.build_image_index_on_start:
                self._index_progress = stale or "no embeddings; add them from the viewer"
                logger.info("visual index: %s", self._index_progress)
                return
            self._index_progress = f"building (had {existing} frames)"
            try:
                added = index.build(stride=self.config.image_index_stride)
                logger.info("visual index built: %d frames (+%d new)", index.count(), added)
                # Warm the model loads and the index here, off the request path: cold
                # they add ~18s (and, for precomputed vectors, the pooling-head pass)
                # to whichever query comes first, which is the one being demoed. Still
                # under both locks: an embedding adoption would stop this index meanwhile.
                if index.count() > 0:
                    self._index_progress = f"loading ({index.count()} frames)"
                    index.load()
                index.model.embed_text("warmup")  # this index's own model
                self._index_progress = f"ready ({index.count()} frames)"
            except Exception as error:  # building, loading or warming: one status line
                self._index_progress = f"failed: {error}"
                logger.exception("visual index unusable")
                return
        # Its own try: faster-whisper is in the optional `agents` group and downloads a
        # model on first touch, so this raises on a machine without it or with a cold hub.
        # Outside a try it propagated and reported a fully successful index build as
        # FAILED -- voice is an extra, and it must not be able to condemn the search.
        try:
            _ = self.whisper  # needs neither the store nor the index
            logger.info("voice query path warm")
        except Exception:
            logger.exception("voice query path unavailable; search is unaffected")

    def _index_status(self) -> dict[str, Any]:
        """What the viewer shows for search: a query button, or an offer to embed first.

        Never waits: a build holds the store lock for minutes, and a viewer that
        connects meanwhile needs the progress line, not a stalled request thread.
        """
        present = False
        if self._store_lock.acquire(blocking=False):
            try:
                index = self._ensure_visual_index()
                present = index.precomputed_stream_name is not None or index.count() > 0
            except Exception as error:
                logger.warning("index status unavailable: %s", error)
            finally:
                self._store_lock.release()
        present = present or self._hyperspace_ready()
        return {"present": present, "index": self._index_progress, **self._embed_job.status()}

    def _start_embedding(self) -> bool:
        """Run siglipify over the recording in the background unless it already is."""
        self._ensure_store()  # names the streams: siglipify must get the viewer's camera
        return self._embed_job.start(
            siglipify_command(self.config.siglipify_flake, self.config.store_path),
            siglipify_config(
                self.config.siglip_model_name,
                self.config.image_stream_name,
                self.config.image_index_stride,
            ),
            adopt=self._adopt_embeddings,
        )

    def _adopt_embeddings(self) -> None:
        """Pick up the stream siglipify just wrote and load the index from it."""
        if self.config.store_path.endswith(".mcap"):
            self._reopen_recording()  # drops the index with the store it read
        else:
            with self._store_lock, self._index_lock:  # store first, like every index user
                self._drop_visual_index()
        self._build_visual_index()

    def _drop_visual_index(self) -> None:
        """Under the store and index locks: a search holding the old index would read
        a stream of a store that is about to close."""
        if self._visual_index is not None:
            self._visual_index.stop()
            self._visual_index = None

    def _publish_query_images(self, query_id: str, phrase: str, places: list[Place]) -> None:
        """Send the frame behind each place, posed where its camera stood.

        The header carries the camera position, its forward and up directions
        and its field of view, so the viewer can hang the picture on the
        camera's image plane.
        """
        hfov_deg = self._camera_hfov()
        sent: list[tuple[dict[str, Any], bytes]] = []
        for index, place in enumerate(places):
            try:
                with self._store_lock:  # resolved and read together: a reopen swaps the store
                    images = self._ensure_store().streams[self.config.image_stream_name]
                    frame = images.at(place.ts, tolerance=0.005).first()
                    image = frame.data
                jpeg = self._encode_jpeg(
                    image, self.config.query_image_max_size, self.config.thumbnail_jpeg_quality
                )
            except Exception:
                logger.exception("could not fetch the frame behind place %d", index)
                continue
            camera = pose_matrix(place.camera_position or place.position, place.orientation)
            forward, up = camera[:3, 2], -camera[:3, 1]  # optical: z forward, y down
            height, width = frame.data.shape[:2]
            header = {
                "query_id": query_id,
                "index": index,
                "label": f"{phrase} ({place.similarity:+.3f})",
                "position": [float(v) for v in camera[:3, 3]],
                "forward": [float(v) for v in forward],
                "up": [float(v) for v in up],
                "hfov_deg": hfov_deg,
                "aspect": float(width) / float(height),
                "distance_m": float(self.config.query_image_distance_m),
            }
            sent.append((header, jpeg))
        with self._clients_lock:
            if not self._query_is_current(query_id):
                return  # a newer question replaced this one while its frames decoded
            self._active_query_images = sent
        for header, jpeg in sent:
            self._broadcast(encode_binary(MSG_QUERY_IMAGE, header, jpeg))

    def _locate_objects(self, phrase: str) -> list[Place]:
        """Raycast the hot patches of the best frames through depth and group the hits.

        Empty when the recording has no depth stream or intrinsics, or when
        no hot patch lands on valid depth.
        """
        if self.config.depth_stream_name is None or self.config.camera_info_stream_name is None:
            return []
        with self._store_lock:
            store = self._ensure_store()
            info = depth_info_stream_for(
                store,
                self.config.depth_stream_name,
                self.config.camera_info_stream_name,
            )
            try:
                dinfo = store.streams[info].first().data  # the depth camera's own
                k = dinfo.K
            except LookupError:  # declared, never published
                return []
            if not (k[0] and k[4]):  # uncalibrated: nothing to raycast through
                return []
            # The patch coordinates are the COLOUR camera's, so its intrinsics are what
            # turns them into a ray; without them the depth is sampled at the wrong pixel.
            colour = self.config.camera_info_stream_name
            try:
                cinfo = (
                    store.streams[colour].first().data if colour in store.list_streams() else None
                )
            except LookupError:
                cinfo = None
        # Adjusted for whatever roi and binning the message declares, and returned with
        # the raster those numbers then address -- so a remaining mismatch against the
        # depth image is a resize and scales, while a crop does not. Without any of this a
        # 1280x720 calibration indexed at an 848x480 depth pixel lifts the patch through
        # cx=640 and lands it 43 cm off-axis.
        intrinsics, intrinsics_size = sensor_intrinsics(dinfo)
        if not intrinsics_size[0] or not intrinsics_size[1]:
            intrinsics_size = None
        # The colour side through the same adjustment. `color_size` is the raster its
        # intrinsics address, which after a roi or binning is NOT cinfo.width: the uv is
        # normalised in the published image, so meeting K means multiplying by the
        # published size, not by the calibrated one.
        colour_intrinsics: tuple[float, float, float, float] | None = None
        colour_size: tuple[int, int] | None = None
        if cinfo is not None and cinfo.K[0] and cinfo.K[4]:
            colour_intrinsics, colour_size = sensor_intrinsics(cinfo)
            if not colour_size[0] or not colour_size[1]:
                colour_intrinsics, colour_size = None, None  # half a calibration
        if info == colour:
            # No depth calibration: `depth_info_stream_for` fell back to the COLOUR info,
            # so `k` above is the colour camera's. Correcting colour-to-colour round-trips
            # uv back through the colour WIDTH and indexes that into the depth raster --
            # with a 1280x720 info and an 848x480 depth image, uv 0.875 lands at column
            # 1120 and is dropped, and everything below it samples ~1.5x too far right.
            # One calibration means the streams are taken as aligned: the uncorrected path.
            colour_intrinsics, colour_size = None, None

        hits: list[PatchHit] = []
        with self._store_lock:
            frames = list(
                self._ensure_visual_index().frame_patches(phrase, k=self.config.locate_frames)
            )
        for frame in frames:
            try:
                with self._store_lock:  # resolved and read together: a reopen swaps the store
                    depth_stream = self._ensure_store().streams[self.config.depth_stream_name]
                    depth = depth_stream.at(
                        frame.ts, tolerance=self.config.depth_tolerance_s
                    ).first()
                    depth_mm = np.asarray(depth.data.data)
            except LookupError:
                continue
            camera_to_world = pose_matrix(frame.position, frame.orientation)
            for image_uv, score in hot_patches(frame.similarity, frame.rows, frame.cols):
                position = patch_world_position(
                    image_uv,
                    depth_mm,
                    intrinsics,
                    camera_to_world,
                    color_intrinsics=colour_intrinsics,
                    color_size=colour_size,
                    intrinsics_size=intrinsics_size,
                )
                if position is not None:
                    hits.append(
                        PatchHit(
                            position=position,
                            similarity=score,
                            source_id=frame.source_id,
                            ts=frame.ts,
                            camera_position=frame.position,
                            camera_orientation=frame.orientation,
                        )
                    )
        return cluster_hits(
            hits, radius=self.config.object_radius_m, max_places=self.config.max_places
        )

    def _markers_near(self, positions: list[tuple[float, float, float]]) -> list[int]:
        """Ids of the capture-pose markers closest to each place.

        The viewer only holds thumbnails for the ``n_image_markers`` poses it was
        sent, and a matching frame is usually not one of them. Highlighting the
        nearest marker instead puts a visible photo at each answer location.
        """
        cached = self._cached_image_poses  # read once: a reopen clears it
        if cached is None:
            return []
        header, payload = cached
        n = int(header.get("n", 0))
        ids = header.get("ids") or []
        if n == 0 or len(ids) < n:
            return []
        marker_xyz = np.frombuffer(payload, dtype=np.float32, count=n * 3).reshape(n, 3)
        nearest = {
            int(ids[int(np.argmin(np.linalg.norm(marker_xyz - np.asarray(p, np.float32), axis=1)))])
            for p in positions
        }
        return sorted(nearest)

    def _find_with_siglip(self, phrase: str, started: float) -> SkillResult:
        """The fallback answer: SigLIP frame search, placed by depth when it can be."""
        with self._store_lock:  # resolved and counted together: a reopen swaps the store
            indexed = self._ensure_visual_index().count()
        if indexed == 0:
            return SkillResult.fail(
                "INDEX_NOT_READY",
                f"The SigLIP index for {self.config.store_path} holds no frames "
                f"({self._index_progress}). Build it with "
                f"`python -m dimos.teleop.memory_world.visual_search {self.config.store_path}`.",
            )

        places = self._locate_objects(phrase)
        located = bool(places)
        if not located:
            # No depth or extrinsics: answer with the poses the frames were taken from.
            with self._store_lock:  # the index reads its stream; a reopen swaps the store
                hits = self._ensure_visual_index().search(phrase, k=self.config.search_top_k)
            places = cluster_places(
                hits, radius=self.config.place_radius_m, max_places=self.config.max_places
            )
        if not places:
            return SkillResult.fail("NOT_FOUND", f"Nothing in the recording matches {phrase!r}")

        # The same shape Hyperspace publishes, from the embeddings' own places. The client
        # builds its results bar, its place stepping and its Navigate button from the
        # answer's `clusters`; an answer that carries only `points` leaves all three inert
        # -- the bar reads "0 places" and `results.navigate()` returns null before it
        # reaches the route at all. It has to ride the RESULT, not the SkillResult's
        # metadata: `_publish_query_result` broadcasts `result.model_dump()`, and nothing
        # of the skill's metadata ever reaches the websocket.
        radius = float(self.config.object_radius_m if located else self.config.place_radius_m)
        clusters = [
            ClusterSummary(
                index=index,
                centre=place.position,
                radius=radius,
                score=float(place.similarity),
                peak=float(place.similarity),
                n_views=int(place.views),
                n_evidence=int(place.views),
                label=f"{phrase[:80]} #{index + 1}",
            )
            for index, place in enumerate(places)
        ]

        result = MemoryQueryResult(
            engine="siglip",
            query_text=phrase,
            clusters=clusters,
            answer=f"Found {phrase} in {len(places)} place(s), {_best_phrase(places[0], located)}",
            focus_point=places[0].position,
            points=[
                HighlightPoint(
                    position=place.position,
                    label=f"{phrase[:80]} ({place.similarity:+.3f}"
                    # Only the depth path measures viewing directions. On the other one
                    # `views` is the dataclass default, so printing "1 view" beside a
                    # score would report a constant in the place of a measurement -- and
                    # `cluster_places` has just thrown away the near-identical frames
                    # that would have made it interesting.
                    + (
                        f", {place.views} view{'s' if place.views != 1 else ''})"
                        if located
                        else ")"
                    ),
                    radius=self.config.object_radius_m if located else None,
                )
                for place in places
            ],
            observation_ids=self._markers_near([place.position for place in places]),
        )
        self._add_route_to_result(result)
        query_id = self._publish_query_result(result)
        # Navigate reads the answer's PLACES off `_last_answer`, which only the Hyperspace
        # path used to set -- so /navigate 409'd ("the last answer is not a Hyperspace one")
        # against every embedding answer, which is now every answer there is. It wants two
        # fields per place, an index and a centre, so the places give it those directly
        # rather than the route growing a second way to be asked.
        with self._clients_lock:
            self._last_answer = (
                SimpleNamespace(
                    clusters=[
                        SimpleNamespace(index=i, centre=tuple(place.position), radius=radius)
                        for i, place in enumerate(places)
                    ]
                ),
                query_id,
            )
        self._publish_query_images(query_id, phrase, places)

        return SkillResult(
            success=True,
            message=result.answer,
            duration_ms=(time.monotonic() - started) * 1000,
            metadata={
                "query_id": query_id,
                "query": phrase,
                "engine": "siglip",
                "clusters": [cluster.model_dump(mode="json") for cluster in clusters],
                "places": [_place_metadata(place, located) for place in places],
                "located": located,
            },
        )
