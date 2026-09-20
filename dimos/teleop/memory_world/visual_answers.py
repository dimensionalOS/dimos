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

"""How the memory world answers a question: the per-frame SigLIP index, the
"add embeddings" job, and answers placed by raycasting hot patches through depth.

Mixed into MemoryWorldModule. This is the only engine -- a question is answered from
the recording's own CLIP/SigLIP frame embeddings and nothing else."""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.agents.skill_result import SkillResult
from dimos.teleop.memory_world.embed import (
    EmbeddingJob,
    in_process_command,
    siglipify_command,
    siglipify_config,
)
from dimos.teleop.memory_world.messages import MSG_QUERY_IMAGE, encode_binary
from dimos.teleop.memory_world.query import ClusterSummary, HighlightPoint, MemoryQueryResult
from dimos.teleop.memory_world.recording import recorded_payload
from dimos.teleop.memory_world.tf_tree import pose_matrix
from dimos.teleop.memory_world.visual_search import (
    Place,
    VisualMemoryIndex,
    cluster_places,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _place_metadata(place: Place, recording_start: float) -> dict[str, Any]:
    """One place, as the skill result reports it.

    ``seconds_into_recording`` is what makes "the FIRST basket" answerable: places come
    back ranked by similarity, so without a time on each one the only order a reader has
    is how well each matched, which is not what "first" means.
    """
    return {
        "position": place.position,
        "similarity": place.similarity,
        "seconds_into_recording": round(place.ts - recording_start, 2),
    }


class VisualAnswers:
    """Needs, from the module: ``config``, the store and index locks,
    ``_ensure_store``, ``_ensure_visual_index``, ``_reopen_recording``,
    ``_query_is_current``, ``_camera_hfov``, ``_encode_jpeg``, ``_broadcast``,
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
        def _ensure_world_cache(self) -> Any: ...
        def _reopen_recording(self) -> None: ...
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
            except (Exception, SystemExit) as error:
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
            # SystemExit too, which is how this package reports an expected failure and
            # is not an Exception. `index.build()` reaches `pose_of` -> `_tf_tree` ->
            # `refuse_if_a_rebuild_is_half_done`, so a recording left with both `tf` and
            # `tf__rebuilt` -- a killed calibration -- raised straight through. And
            # `threading.excepthook` IGNORES a SystemExit out of a thread silently, so
            # the prepare thread died with no log at all, `_index_progress` stayed on
            # "building (had 0 frames)", and the viewer matched that against
            # /^(not started|building|loading)/ and polled every three seconds for the
            # rest of the session with no reason shown anywhere.
            #
            # The same fix `_ensure_world_cache` and `_build_replay` already carry.
            except (Exception, SystemExit) as error:  # building, loading or warming
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
        # `present` is whether the siglip index holds vectors; `ask` is whether a QUESTION
        # can be answered, which on the hyperspace blueprint needs no siglip index at all.
        # The viewer gated the ask box and the mic on `present` alone, so that build sat
        # on "Search not ready" while every curl'd question was answered and drawn.
        via_hyperspace = bool(
            self.config.ask_via_agent and getattr(self, "_hyperspace_live", False)
        )
        index = self._index_progress
        if not present and via_hyperspace:
            index = "not needed: hyperspace answers questions"
        return {
            "present": present,
            "ask": present or via_hyperspace,
            "index": index,
            **self._embed_job.status(),
        }

    def _start_embedding(self) -> bool:
        """Add embeddings in the background unless that is already happening.

        siglipify ONLY where the file positively says the colour is a plain `Image`; this
        package's own indexer otherwise. siglipify reads the recording itself and knows
        only `Image` streams, and `lite_record` writes its colour as a `CompressedImage`.
        On such a recording it said "no image stream 'color_image' in the recording; it
        has ['depth_image']" and stopped -- naming the one stream it could see, which
        reads as the colour being missing. It is not missing; it is webp, and this module
        decodes it for the viewer on every frame.

        The test is on a POSITIVE "Image", not on "not Image", because
        `recorded_payload` answers None for a file that cannot say -- **which every mcap
        is**. Read the other way round, `grocery.mcap` (the recording DEMO.md tells you to
        run) sent its compressed colour to siglipify and the viewer's only offer to make
        the recording searchable failed with that same misleading sentence. The in-process
        indexer goes through `open_recording`, so it handles both kinds; it is the safe
        default, and siglipify is the optimisation taken only when it is known to apply.
        """
        self._ensure_store()  # names the streams: the indexer must get the viewer's camera
        recorded = recorded_payload(self.config.store_path, self.config.image_stream_name)
        if recorded != "Image":
            return self._embed_job.start(
                in_process_command(
                    self.config.store_path,
                    self.config.image_stream_name,
                    self.config.siglip_model_name,
                    self.config.image_index_stride,
                    self.config.tf_stream_name,
                    self.config.world_frame,
                    self.config.camera_optical_frame,
                    self.config.image_index_stream_name,
                    self.config.tf_tolerance_s,
                ),
                None,
                adopt=self._adopt_embeddings,
            )
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
        """Pick up the stream the indexer just wrote and load the index from it."""
        reopened = self.config.store_path.endswith(".mcap")
        if reopened:
            self._reopen_recording()  # drops the index with the store it read
        else:
            with self._store_lock, self._index_lock:  # store first, like every index user
                self._drop_visual_index()
        self._build_visual_index()
        if reopened:
            # A reopen clears every world cache, `_map_xyz` among them -- and `_map_xyz`
            # is the map the ROUTE PLANNER walks over. Nothing rebuilt it until the next
            # viewer connected, so on the demo's own path (add embeddings, ask, press
            # Navigate) the answer came back and Navigate then answered 503 "the map is
            # still building" for ever, over a map that was fully built and drawn on
            # screen in front of you. Measured live on grocery.mcap; a reload fixed it,
            # which is exactly what makes it look like a client problem.
            #
            # Rebuilt HERE, on the job's own thread, because this is where the cache was
            # invalidated and this thread is already off the request path. The cloud
            # comes back from the replay's last keyframe in about a second; the capture
            # markers are the slow part, and an answer needs those too.
            try:
                self._ensure_world_cache()
            except (Exception, SystemExit):  # a refusal is a SystemExit, not an Exception
                logger.exception("could not rebuild the world cache after the reopen")

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
                    # The NEAREST frame in the window, not the first one in it. `.first()`
                    # takes the earliest within the tolerance, so on a camera running
                    # faster than the window is wide the evidence was a different
                    # photograph from the one the answer matched -- measured, frames at
                    # 1.000 and 1.004 with the answer on 1.004 sent the 1.000 picture,
                    # carrying the matched frame's camera pose with it. `_replay_frame`
                    # has picked the nearest since two frames a tenth of a millisecond
                    # apart were served each other's JPEG.
                    near = list(images.at(place.ts, tolerance=0.005))
                    if not near:
                        raise LookupError(f"no frame within 5 ms of {place.ts}")
                    frame = min(near, key=lambda obs: abs(float(obs.ts) - float(place.ts)))
                    image = frame.data
                jpeg = self._encode_jpeg(
                    image, self.config.query_image_max_size, self.config.thumbnail_jpeg_quality
                )
            except Exception:
                logger.exception("could not fetch the frame behind place %d", index)
                continue
            camera = pose_matrix(place.position, place.orientation)
            forward, up = camera[:3, 2], -camera[:3, 1]  # optical: z forward, y down
            height, width = frame.data.shape[:2]
            header = {
                "query_id": query_id,
                "index": index,
                # Which place this photograph belongs to. One per place here, so it is
                # the same number -- but it has to be SAID. `/navigate`'s `pose_of`
                # rejects any image whose `cluster` is not the one being routed to, so
                # without this key every photograph was refused and the only candidate
                # left was the cluster CENTRE, which for a thing on a wall is inside the
                # wall: "no route through the known free space", every time, for every
                # place, while the pose the robot actually stood at to take the picture
                # was in the list. The viewer's place filter and `jumpTo` read it too.
                "cluster": index,
                "label": f"{phrase} ({place.similarity:+.3f})",
                "position": [float(v) for v in camera[:3, 3]],
                "forward": [float(v) for v in forward],
                "up": [float(v) for v in up],
                "hfov_deg": hfov_deg,
                "aspect": float(width) / float(height),
                "distance_m": float(self.config.query_image_distance_m),
                # No `uv`: one vector per image scores the WHOLE picture, so there is no
                # in-frame hotspot to ring. The viewer draws the ring and its leader line
                # only when both keys are present, which is the honest rendering here --
                # the evidence is the photograph, not a pixel inside it.
                "point": [float(v) for v in place.position],
            }
            sent.append((header, jpeg))
        with self._clients_lock:
            if not self._query_is_current(query_id):
                return  # a newer question replaced this one while its frames decoded
            self._active_query_images = sent
        for header, jpeg in sent:
            self._broadcast(encode_binary(MSG_QUERY_IMAGE, header, jpeg))

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

    def _find_with_siglip(
        self,
        phrase: str,
        started: float,
        span: tuple[float, float] | None = None,
    ) -> SkillResult:
        """The answer: SigLIP image search, placed at the poses the frames were taken from.

        *span* is the fraction of the recording to look in, ``(from, to)`` with 0 the
        start and 1 the end; None means the whole thing.
        """
        with self._store_lock:  # resolved and counted together: a reopen swaps the store
            indexed = self._ensure_visual_index().count()
        if indexed == 0:
            return SkillResult.fail(
                "INDEX_NOT_READY",
                f"The SigLIP index for {self.config.store_path} holds no frames "
                f"({self._index_progress}). Build it with "
                f"`python -m dimos.teleop.memory_world.visual_search {self.config.store_path}`.",
            )

        # Where the camera STOOD when it saw the thing. One vector per image is all the
        # index holds, so that is all an answer claims -- nothing is back-projected into
        # the map to guess the object's own coordinates.
        with self._store_lock:  # the index reads its stream; a reopen swaps the store
            index = self._ensure_visual_index()
            first, last = index.time_span()
            window = None
            if span is not None:
                low, high = span
                window = (first + (last - first) * low, first + (last - first) * high)
            hits = index.search(phrase, k=self.config.search_top_k, window=window)
        where = "" if span is None else " in that part of the recording"
        if not hits:
            return SkillResult.fail("NOT_FOUND", f"No frames{where} to compare against {phrase!r}")
        # A cosine ranking always returns SOMETHING: the top of the list is the most
        # like the question, not necessarily like it at all. Without a floor the answer
        # to "were there any people" is yes on every recording ever made, so a best match
        # this weak is reported as nothing found -- and the score is named either way, so
        # the reader can see how near the line it fell.
        best = max(hit.similarity for hit in hits)
        if best < self.config.min_similarity:
            return SkillResult.fail(
                "NOT_FOUND",
                f"Nothing{where} matches {phrase!r} "
                f"(closest frame {best:+.3f}, below {self.config.min_similarity:+.3f})",
            )
        places = cluster_places(
            [hit for hit in hits if hit.similarity >= self.config.min_similarity],
            radius=self.config.place_radius_m,
            max_places=self.config.max_places,
        )
        if not places:
            return SkillResult.fail("NOT_FOUND", f"Nothing{where} matches {phrase!r}")

        # The client builds its results bar, its place stepping and its Navigate button
        # from the answer's `clusters`; an answer that carries only `points` leaves all three inert
        # -- the bar reads "0 places" and `results.navigate()` returns null before it
        # reaches the route at all. It has to ride the RESULT, not the SkillResult's
        # metadata: `_publish_query_result` broadcasts `result.model_dump()`, and nothing
        # of the skill's metadata ever reaches the websocket.
        radius = float(self.config.place_radius_m)
        clusters = [
            ClusterSummary(
                index=index,
                centre=place.position,
                radius=radius,
                score=float(place.similarity),
                peak=float(place.similarity),
                # Zero means "never measured", which is the truth here: `cluster_places`
                # keeps the best frame per location and throws the near-identical ones
                # away, so there is no count of distinct sightings to report. The client
                # prints nothing for a zero rather than a made-up "1 view".
                n_views=0,
                n_evidence=0,
                label=f"{phrase[:80]} #{index + 1}",
            )
            for index, place in enumerate(places)
        ]

        result = MemoryQueryResult(
            engine="siglip",
            query_text=phrase,
            clusters=clusters,
            answer=(
                f"Found {phrase} in {len(places)} place(s) it was seen from, "
                f"best match {places[0].similarity:+.3f}"
            ),
            focus_point=places[0].position,
            points=[
                HighlightPoint(
                    position=place.position,
                    label=f"{phrase[:80]} ({place.similarity:+.3f})",
                    radius=None,
                )
                for place in places
            ],
            observation_ids=self._markers_near([place.position for place in places]),
        )
        self._add_route_to_result(result)
        query_id = self._publish_query_result(result)
        # Navigate reads the answer's PLACES off `_last_answer`. It wants two fields per
        # place, an index and a centre, so the places give it those directly rather than
        # the route growing a second way to be asked.
        with self._clients_lock:
            # The SAME ClusterSummary objects the answer published, not a look-alike:
            # `/navigate` reads index, centre and radius off these, and `/answer` asks
            # each for a summary. The fields `/answer` reads off the ANSWER are here too.
            # Only if this is still the answer on screen. A search that takes ten seconds
            # can finish after a newer question has already replaced it, and writing
            # `_last_answer` unconditionally put the OLD places behind the NEW answer:
            # `/answer` described a question nobody asked and `/navigate` routed to it.
            # `_publish_query_images` four lines down has always checked; this did not.
            if self._query_is_current(query_id):
                self._last_answer = (
                    SimpleNamespace(
                        clusters=clusters,
                        text=result.answer,
                        frame=self.config.world_frame,
                        stats={"places": len(places)},
                        seconds=time.monotonic() - started,
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
                "places": [_place_metadata(place, first) for place in places],
            },
        )
