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

"""
Temporal Memory module for creating entity-based temporal understanding of video streams.

This module implements a sophisticated temporal memory system inspired by VideoRAG,
using VLM (Vision-Language Model) API calls to maintain entity rosters, rolling summaries,
and temporal relationships across video frames.
"""

from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from reactivex import Subject, interval
from reactivex.disposable import Disposable

from dimos import spec
from dimos.agents import skill
from dimos.core import DimosCluster, In, rpc
from dimos.core.module import ModuleConfig
from dimos.core.skill_module import SkillModule
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import sharpness_barrier
from dimos.perception.clip_filter import (
    CLIP_AVAILABLE,
    CLIPFrameFilter,
    select_diverse_frames_simple,
)
from dimos.perception.entity_graph_db import EntityGraphDB
from dimos.perception.videorag_utils import (
    apply_summary_update,
    build_batch_distance_estimation_prompt,
    build_query_prompt,
    build_summary_prompt,
    build_window_prompt,
    default_state,
    extract_time_window,
    format_timestamp,
    get_structured_output_format,
    parse_batch_distance_response,
    parse_window_response,
    update_state_from_window,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Constants
STALE_SCENE_THRESHOLD = 5.0  # Skip window if scene hasn't changed (perceptual hash distance)
MAX_DISTANCE_PAIRS = 5  # Max entity pairs to estimate distances for per window
MAX_RELATIONS_PER_ENTITY = 10  # Max relations to include in graph context
NEARBY_DISTANCE_METERS = 5.0  # Distance threshold for "nearby" entities
MAX_RECENT_WINDOWS = 50  # Max recent windows to keep in memory


# Pure functions
def is_scene_stale(frames: list["Frame"]) -> bool:
    """Check if scene hasn't changed meaningfully between first and last frame.

    Args:
        frames: List of frames to check

    Returns:
        True if scene is stale (hasn't changed enough), False otherwise
    """
    if len(frames) < 2:
        return False
    first_img = frames[0].image
    last_img = frames[-1].image
    if first_img is None or last_img is None:
        return False
    diff = np.abs(first_img.data.astype(float) - last_img.data.astype(float))
    return diff.mean() < STALE_SCENE_THRESHOLD


def build_graph_context(
    graph_db: "EntityGraphDB", entity_ids: list[str], time_window_s: float | None = None
) -> dict[str, Any]:
    """Build enriched context from graph database for given entities.

    Args:
        graph_db: Entity graph database instance
        entity_ids: List of entity IDs to get context for
        time_window_s: Optional time window in seconds (e.g., 3600 for last hour)

    Returns:
        Dictionary with graph context including relationships, distances, and semantics
    """
    if not graph_db or not entity_ids:
        return {}

    try:
        graph_context: dict[str, Any] = {
            "relationships": [],
            "spatial_info": [],
            "semantic_knowledge": [],
        }

        # Convert time_window_s to a (start_ts, end_ts) tuple if provided
        time_window_tuple = None
        if time_window_s is not None:
            current_time = time.time()
            time_window_tuple = (current_time - time_window_s, current_time)

        # Get recent relationships for each entity
        for entity_id in entity_ids:
            # Get relationships (Graph 1: interactions)
            relations = graph_db.get_relations_for_entity(
                entity_id=entity_id,
                relation_type=None,  # all types
                time_window=time_window_tuple,
            )
            for rel in relations[-MAX_RELATIONS_PER_ENTITY:]:
                graph_context["relationships"].append(
                    {
                        "subject": rel["subject_id"],
                        "relation": rel["relation_type"],
                        "object": rel["object_id"],
                        "confidence": rel["confidence"],
                        "when": rel["timestamp_s"],
                    }
                )

            # Get spatial relationships (Graph 2: distances)
            # Find entities nearby this one
            nearby = graph_db.get_nearby_entities(
                entity_id=entity_id, max_distance=NEARBY_DISTANCE_METERS, latest_only=True
            )
            for dist in nearby:
                graph_context["spatial_info"].append(
                    {
                        "entity_a": dist["entity_a"],
                        "entity_b": dist["entity_b"],
                        "distance": dist.get("distance_m"),
                        "category": dist.get("distance_category"),
                        "confidence": dist["confidence"],
                    }
                )

            # Get semantic knowledge (Graph 3: conceptual relations)
            semantic_rels = graph_db.get_semantic_relations(
                entity_id=entity_id,
                relation_type=None,  # all types
            )
            for sem in semantic_rels:
                graph_context["semantic_knowledge"].append(
                    {
                        "entity_a": sem["entity_a"],
                        "relation": sem["relation_type"],
                        "entity_b": sem["entity_b"],
                        "confidence": sem["confidence"],
                        "observations": sem["observation_count"],
                    }
                )

        # Get graph statistics for context
        if entity_ids:
            stats = graph_db.get_stats()
            graph_context["total_entities"] = stats.get("total_entities", 0)
            graph_context["total_relations"] = stats.get("total_relations", 0)

        return graph_context

    except Exception as e:
        logger.warning(f"failed to build graph context: {e}")
        return {}


@dataclass
class Frame:
    frame_index: int
    timestamp_s: float
    image: Image


@dataclass
class TemporalMemoryConfig(ModuleConfig):
    fps: float = 1.0
    window_s: float = 2.0
    stride_s: float = 2.0
    summary_interval_s: float = 10.0
    max_frames_per_window: int = 3
    frame_buffer_size: int = 50
    output_dir: str | Path | None = None
    max_tokens: int = 900
    temperature: float = 0.2
    use_clip_filtering: bool = True
    clip_model: str = "ViT-B/32"


class TemporalMemory(SkillModule):
    """
    builds temporal understanding of video streams using vlms.

    processes frames reactively, maintains entity rosters, tracks temporal
    relationships, builds rolling summaries. responds to queries about current
    state and recent events.
    """

    color_image: In[Image]

    def __init__(
        self, vlm: VlModel | None = None, config: TemporalMemoryConfig | None = None
    ) -> None:
        super().__init__()

        self._vlm = vlm  # Can be None for blueprint usage
        self.config = config or TemporalMemoryConfig()

        # single lock protects all state
        self._state_lock = threading.Lock()
        self._stopped = False

        # protected state
        self._state = default_state()
        self._state["next_summary_at_s"] = float(self.config.summary_interval_s)
        self._frame_buffer: deque[Frame] = deque(maxlen=self.config.frame_buffer_size)
        self._recent_windows: deque[dict[str, Any]] = deque(maxlen=MAX_RECENT_WINDOWS)
        self._frame_count = 0
        self._last_analysis_time = 0.0
        self._video_start_wall_time: float | None = None

        # clip filter
        self._clip_filter: CLIPFrameFilter | None = None
        if self.config.use_clip_filtering and CLIP_AVAILABLE:
            try:
                self._clip_filter = CLIPFrameFilter(model_name=self.config.clip_model)
                logger.info("clip filtering enabled")
            except Exception as e:
                logger.warning(f"clip init failed: {e}")
                self.config.use_clip_filtering = False
        elif self.config.use_clip_filtering:
            logger.warning("clip not available")
            self.config.use_clip_filtering = False

        # output directory
        if self.config.output_dir:
            self._output_path = Path(self.config.output_dir)
            self._output_path.mkdir(parents=True, exist_ok=True)
            self._evidence_file = self._output_path / "evidence.jsonl"
            self._state_file = self._output_path / "state.json"
            self._entities_file = self._output_path / "entities.json"
            self._frames_index_file = self._output_path / "frames_index.jsonl"

            # Initialize entity graph database
            self._graph_db = EntityGraphDB(db_path=self._output_path / "entity_graph.db")

            logger.info(f"artifacts save to: {self._output_path}")
        else:
            self._graph_db = None

        logger.info(
            f"temporalmemory init: fps={self.config.fps}, "
            f"window={self.config.window_s}s, stride={self.config.stride_s}s"
        )

    @property
    def vlm(self) -> VlModel:
        """Get or create VLM instance lazily."""
        if self._vlm is None:
            from dimos.models.vl.openai import OpenAIVlModel

            # Load API key from environment
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError(
                    "OPENAI_API_KEY environment variable not set. "
                    "Either set it or pass a vlm instance to TemporalMemory constructor."
                )
            self._vlm = OpenAIVlModel(api_key=api_key)
            logger.info("Created OpenAIVlModel from OPENAI_API_KEY environment variable")
        return self._vlm

    @rpc
    def start(self) -> None:
        super().start()

        with self._state_lock:
            self._stopped = False
            if self._video_start_wall_time is None:
                self._video_start_wall_time = time.time()

        def on_frame(image: Image) -> None:
            with self._state_lock:
                video_start = self._video_start_wall_time
                if image.ts is not None:
                    timestamp_s = image.ts - video_start
                else:
                    timestamp_s = time.time() - video_start

                frame = Frame(
                    frame_index=self._frame_count,
                    timestamp_s=timestamp_s,
                    image=image,
                )
                self._frame_buffer.append(frame)

                self._frame_count += 1

        # pipe through sharpness filter before buffering
        frame_subject = Subject()
        self._disposables.add(
            frame_subject.pipe(sharpness_barrier(self.config.fps)).subscribe(on_frame)
        )

        unsub_image = self.color_image.subscribe(frame_subject.on_next)
        self._disposables.add(Disposable(unsub_image))

        self._disposables.add(
            interval(self.config.stride_s).subscribe(lambda _: self._analyze_window())
        )

        logger.info("temporalmemory started")

    @rpc
    def stop(self) -> None:
        # Save state before clearing (bypass _stopped check by saving directly)
        if self.config.output_dir:
            try:
                with self._state_lock:
                    state_copy = self._state.copy()
                    entity_roster = list(self._state.get("entity_roster", []))
                with open(self._state_file, "w") as f:
                    json.dump(state_copy, f, indent=2, ensure_ascii=False)
                logger.info(f"saved state to {self._state_file}")
                with open(self._entities_file, "w") as f:
                    json.dump(entity_roster, f, indent=2, ensure_ascii=False)
                logger.info(f"saved {len(entity_roster)} entities")
            except Exception as e:
                logger.error(f"save failed during stop: {e}", exc_info=True)

        self.save_frames_index()

        # Set stopped flag and clear state
        with self._state_lock:
            self._stopped = True

        # Close graph database
        if self._graph_db:
            self._graph_db.close()
            self._graph_db = None

        if self._clip_filter:
            self._clip_filter.close()
            self._clip_filter = None

        # Clear buffers to release image references
        with self._state_lock:
            self._frame_buffer.clear()
            self._recent_windows.clear()
            self._state = default_state()

        super().stop()

        # Stop all stream transports to clean up LCM/shared memory threads
        for stream in list(self.inputs.values()) + list(self.outputs.values()):
            if stream.transport is not None and hasattr(stream.transport, "stop"):
                stream.transport.stop()
                stream._transport = None

        logger.info("temporalmemory stopped")

    def _analyze_window(self) -> None:
        try:
            # get snapshot
            with self._state_lock:
                if not self._frame_buffer:
                    return
                current_time = self._frame_buffer[-1].timestamp_s
                if current_time - self._last_analysis_time < self.config.stride_s:
                    return

                frames_needed = max(1, int(self.config.fps * self.config.window_s))
                if len(self._frame_buffer) < frames_needed:
                    return

                window_frames = list(self._frame_buffer)[-frames_needed:]
                state_snapshot = self._state.copy()

            w_start = window_frames[0].timestamp_s
            w_end = window_frames[-1].timestamp_s

            # add this check early, before any filtering or VLM calls
            if is_scene_stale(window_frames):
                logger.debug(f"skipping stale window [{w_start:.1f}-{w_end:.1f}s]")
                with self._state_lock:
                    self._last_analysis_time = w_end
                return

            # filter frames
            # NOTE: no longer using clip filter for now (alternative: sharpness barrier and stale scene check)
            # if len(window_frames) > self.config.max_frames_per_window:
            #     if self._clip_filter:
            #         window_frames = self._clip_filter.select_diverse_frames(
            #             window_frames, max_frames=self.config.max_frames_per_window
            #         )
            #     else:
            window_frames = select_diverse_frames_simple(
                window_frames, max_frames=self.config.max_frames_per_window
            )

            logger.info(f"analyzing [{w_start:.1f}-{w_end:.1f}s] with {len(window_frames)} frames")

            # build prompt
            query = build_window_prompt(
                w_start=w_start,
                w_end=w_end,
                frame_count=len(window_frames),
                state=state_snapshot,
            )

            # query vlm (slow, outside lock)
            # use query_batch for multiple frames to send all filtered frames in one API call
            try:
                response_format = get_structured_output_format()
                if len(window_frames) > 1:
                    # Use query_batch to send all filtered frames in one API call
                    # This gives the model more temporal context
                    frame_images = [frame.image for frame in window_frames]
                    responses = self.vlm.query_batch(
                        frame_images, query, response_format=response_format
                    )
                    # query_batch returns list[str] with one response for all images
                    response_text = responses[0] if responses else ""
                else:
                    # Single frame - use regular query
                    response_text = self.vlm.query(
                        window_frames[0].image, query, response_format=response_format
                    )
            except Exception as e:
                logger.error(f"vlm query failed [{w_start:.1f}-{w_end:.1f}s]: {e}")
                with self._state_lock:
                    self._last_analysis_time = w_end
                return

            # parse response
            parsed = parse_window_response(response_text, w_start, w_end, len(window_frames))

            if "_error" in parsed:
                logger.error(f"parse error: {parsed['_error']}")
            else:
                logger.info(f"parsed. caption: {parsed.get('caption', '')[:100]}")

            # estimate distances between entities
            # note: batched into single VLM call for efficiency
            if self._graph_db and len(window_frames) > 0:
                # use the middle frame from the window for distance estimation
                mid_frame = window_frames[len(window_frames) // 2]
                if mid_frame.image:
                    self._estimate_distances(parsed, mid_frame.image, w_end)

            # update state
            with self._state_lock:
                needs_summary = update_state_from_window(
                    self._state, parsed, w_end, self.config.summary_interval_s
                )
                self._recent_windows.append(parsed)
                self._last_analysis_time = w_end

            # save to graph database
            if self._graph_db:
                self._save_to_graph_db(parsed, w_end)

            # save evidence
            if self.config.output_dir:
                self._append_evidence(parsed)

            # update summary if needed
            if needs_summary:
                logger.info(f"updating summary at t≈{w_end:.1f}s")
                self._update_rolling_summary(w_end)

            # periodic save
            with self._state_lock:
                window_count = len(self._recent_windows)

            if window_count % 10 == 0:
                self.save_state()
                self.save_entities()

        except Exception as e:
            logger.error(f"error analyzing window: {e}", exc_info=True)

    def _update_rolling_summary(self, w_end: float) -> None:
        try:
            # get state
            with self._state_lock:
                if self._stopped:
                    return
                rolling_summary = str(self._state.get("rolling_summary", ""))
                chunk_buffer = list(self._state.get("chunk_buffer", []))
                if self._frame_buffer:
                    latest_frame = self._frame_buffer[-1].image
                else:
                    latest_frame = None

            if not chunk_buffer or not latest_frame:
                return

            # build prompt
            prompt = build_summary_prompt(
                rolling_summary=rolling_summary,
                chunk_windows=chunk_buffer,
            )

            # query vlm (slow, outside lock)
            try:
                summary_text = self.vlm.query(latest_frame, prompt)
                if summary_text and summary_text.strip():
                    with self._state_lock:
                        if self._stopped:
                            return
                        apply_summary_update(
                            self._state, summary_text, w_end, self.config.summary_interval_s
                        )
                    logger.info(f"updated summary: {summary_text[:100]}...")
                    # Save state after summary update to persist entities
                    if self.config.output_dir and not self._stopped:
                        self.save_state()
                        self.save_entities()
            except Exception as e:
                logger.error(f"summary update failed: {e}", exc_info=True)

        except Exception as e:
            logger.error(f"error updating summary: {e}", exc_info=True)

    def _estimate_distances(self, parsed: dict[str, Any], frame_image, timestamp_s: float) -> None:
        """Estimate distances between entities using VLM and save to graph database.

        Batches all entity pairs into a single VLM call for efficiency.

        Args:
            parsed: Parsed window data containing entities
            frame_image: Frame image to analyze
            timestamp_s: Timestamp for the distance observations
        """
        if not self._graph_db or not frame_image:
            return

        # Collect all entities present in this window
        all_entities = []
        for entity in parsed.get("new_entities", []):
            all_entities.append(entity)
        for entity in parsed.get("entities_present", []):
            if isinstance(entity, dict) and "id" in entity:
                all_entities.append(entity)

        # Need at least 2 entities to estimate distances
        if len(all_entities) < 2:
            return

        # Generate entity pairs (avoid duplicates by only doing i < j)
        entity_pairs = []
        for i in range(len(all_entities)):
            for j in range(i + 1, len(all_entities)):
                entity_pairs.append((all_entities[i], all_entities[j]))

        # Limit to avoid excessive prompt length
        entity_pairs = entity_pairs[:MAX_DISTANCE_PAIRS]

        if not entity_pairs:
            return

        try:
            # Build batched prompt for all pairs
            prompt = build_batch_distance_estimation_prompt(entity_pairs)

            # Single VLM call for all pairs
            response = self.vlm.query(frame_image, prompt)

            # Parse all distance estimates
            results = parse_batch_distance_response(response, entity_pairs)

            # Save all valid distances to database
            for result in results:
                if result["category"] in ["near", "medium", "far"]:
                    self._graph_db.add_distance(
                        entity_a=result["entity_a_id"],
                        entity_b=result["entity_b_id"],
                        distance_m=result.get("distance_m"),
                        distance_category=result["category"],
                        confidence=result.get("confidence", 0.5),
                        timestamp_s=timestamp_s,
                        method="vlm_estimation_batch",
                    )
                    logger.debug(
                        f"estimated distance {result['entity_a_id']}-{result['entity_b_id']}: "
                        f"{result['category']} ({result.get('distance_m')}m)"
                    )

        except Exception as e:
            logger.warning(f"failed to estimate distances: {e}")

    @skill()
    def query(self, question: str) -> str:
        """Answer a question about the video stream using temporal memory and graph knowledge.

        This skill analyzes the current video stream and temporal memory state
        to answer questions about what is happening, what entities are present,
        recent events, spatial relationships, and conceptual knowledge.

        The system automatically accesses three knowledge graphs:
        - Interactions: relationships between entities (holds, looks_at, talks_to)
        - Spatial: distance and proximity information
        - Semantic: conceptual relationships (goes_with, used_for, etc.)

        Example:
            query("What entities are currently visible?")
            query("What did I do last week?")
            query("Where did I leave my keys?")
            query("What objects are near the person?")

        Args:
            question (str): The question to ask about the video stream.
                Examples: "What entities are visible?", "What happened recently?",
                "Is there a person in the scene?", "What am I holding?"

        Returns:
            str: Answer based on temporal memory, graph knowledge, and current frame.
        """
        # read state
        with self._state_lock:
            entity_roster = list(self._state.get("entity_roster", []))
            rolling_summary = str(self._state.get("rolling_summary", ""))
            last_present = list(self._state.get("last_present", []))
            recent_windows = list(self._recent_windows)
            latest_frame = self._frame_buffer[-1].image if self._frame_buffer else None

        if not latest_frame:
            return "no frames available"

        # build context from temporal state
        currently_present = {e["id"] for e in last_present if isinstance(e, dict) and "id" in e}
        for window in recent_windows[-3:]:
            for entity in window.get("entities_present", []):
                if isinstance(entity, dict) and isinstance(entity.get("id"), str):
                    currently_present.add(entity["id"])

        context = {
            "entity_roster": entity_roster,
            "rolling_summary": rolling_summary,
            "currently_present_entities": sorted(currently_present),
            "recent_windows_count": len(recent_windows),
            "timestamp": time.time(),
        }

        # enhance context with graph database knowledge
        if self._graph_db and currently_present:
            # Extract time window from question using VLM
            time_window_s = extract_time_window(question, self.vlm, latest_frame)

            graph_context = build_graph_context(
                graph_db=self._graph_db,
                entity_ids=list(currently_present),
                time_window_s=time_window_s,
            )
            context["graph_knowledge"] = graph_context

        # build query prompt using videorag utils
        prompt = build_query_prompt(question=question, context=context)

        # query vlm (slow, outside lock)
        try:
            answer_text = self.vlm.query(latest_frame, prompt)
            return answer_text.strip()
        except Exception as e:
            logger.error(f"query failed: {e}", exc_info=True)
            return f"error: {e}"

    @rpc
    def clear_history(self) -> None:
        """Clear temporal memory state."""
        with self._state_lock:
            self._state = default_state()
            self._state["next_summary_at_s"] = float(self.config.summary_interval_s)
            self._recent_windows.clear()
        logger.info("cleared history")

    @rpc
    def get_state(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "entity_count": len(self._state.get("entity_roster", [])),
                "entities": list(self._state.get("entity_roster", [])),
                "rolling_summary": str(self._state.get("rolling_summary", "")),
                "frame_count": self._frame_count,
                "buffer_size": len(self._frame_buffer),
                "recent_windows": len(self._recent_windows),
                "currently_present": list(self._state.get("last_present", [])),
            }

    @rpc
    def get_entity_roster(self) -> list[dict[str, Any]]:
        with self._state_lock:
            return list(self._state.get("entity_roster", []))

    @rpc
    def get_rolling_summary(self) -> str:
        with self._state_lock:
            return str(self._state.get("rolling_summary", ""))

    @rpc
    def get_graph_db_stats(self) -> dict[str, Any]:
        """Get statistics and sample data from the graph database."""
        if not self._graph_db:
            logger.warning("graph database not initialized")
            return {"stats": {}, "entities": [], "recent_relations": []}

        try:
            stats = self._graph_db.get_stats()
            all_entities = self._graph_db.get_all_entities()
            recent_relations = self._graph_db.get_recent_relations(limit=10)

            return {
                "stats": stats,
                "entities": all_entities,
                "recent_relations": recent_relations[:5],  # just show top 5
            }
        except Exception as e:
            logger.error(f"failed to get graph db stats: {e}", exc_info=True)
            return {"stats": {}, "entities": [], "recent_relations": []}

    @rpc
    def save_state(self) -> bool:
        if not self.config.output_dir:
            return False
        try:
            with self._state_lock:
                # Don't save if stopped (state has been cleared)
                if self._stopped:
                    return False
                state_copy = self._state.copy()
            with open(self._state_file, "w") as f:
                json.dump(state_copy, f, indent=2, ensure_ascii=False)
            logger.info(f"saved state to {self._state_file}")
            return True
        except Exception as e:
            logger.error(f"save state failed: {e}", exc_info=True)
            return False

    def _append_evidence(self, evidence: dict[str, Any]) -> None:
        try:
            with open(self._evidence_file, "a") as f:
                f.write(json.dumps(evidence, ensure_ascii=False) + "\n")
        except Exception as e:
            logger.error(f"append evidence failed: {e}")

    def _save_to_graph_db(self, parsed: dict[str, Any], timestamp_s: float) -> None:
        """Save parsed window data to the entity graph database."""
        if not self._graph_db:
            return

        try:
            # Save new entities
            for entity in parsed.get("new_entities", []):
                self._graph_db.upsert_entity(
                    entity_id=entity["id"],
                    entity_type=entity["type"],
                    descriptor=entity["descriptor"],
                    timestamp_s=timestamp_s,
                )

            # Save existing entities (update last_seen)
            for entity in parsed.get("entities_present", []):
                if isinstance(entity, dict) and "id" in entity:
                    # Only update with descriptor if we have one, otherwise pass empty to preserve existing
                    descriptor = entity.get("descriptor")
                    if descriptor:
                        self._graph_db.upsert_entity(
                            entity_id=entity["id"],
                            entity_type=entity.get("type", "unknown"),
                            descriptor=descriptor,
                            timestamp_s=timestamp_s,
                        )
                    else:
                        # Just update last_seen without changing descriptor
                        # Get existing entity to preserve its descriptor
                        existing = self._graph_db.get_entity(entity["id"])
                        if existing:
                            self._graph_db.upsert_entity(
                                entity_id=entity["id"],
                                entity_type=existing["entity_type"],
                                descriptor=existing["descriptor"],
                                timestamp_s=timestamp_s,
                            )

            # Save relations
            for relation in parsed.get("relations", []):
                # Parse subject/object (format: "E1|person" or just "E1")
                subject_id = (
                    relation["subject"].split("|")[0]
                    if "|" in relation["subject"]
                    else relation["subject"]
                )
                object_id = (
                    relation["object"].split("|")[0]
                    if "|" in relation["object"]
                    else relation["object"]
                )

                self._graph_db.add_relation(
                    relation_type=relation["type"],
                    subject_id=subject_id,
                    object_id=object_id,
                    confidence=relation.get("confidence", 1.0),
                    timestamp_s=timestamp_s,
                    evidence=relation.get("evidence"),
                    notes=relation.get("notes"),
                )

            logger.debug(
                f"Saved window data to graph DB: {len(parsed.get('new_entities', []))} new entities, "
                f"{len(parsed.get('relations', []))} relations"
            )

        except Exception as e:
            logger.error(f"Failed to save to graph DB: {e}", exc_info=True)

    def save_entities(self) -> bool:
        if not self.config.output_dir:
            return False
        try:
            with self._state_lock:
                # Don't save if stopped (state has been cleared)
                if self._stopped:
                    return False
                entity_roster = list(self._state.get("entity_roster", []))
            with open(self._entities_file, "w") as f:
                json.dump(entity_roster, f, indent=2, ensure_ascii=False)
            logger.info(f"saved {len(entity_roster)} entities")
            return True
        except Exception as e:
            logger.error(f"save entities failed: {e}", exc_info=True)
            return False

    def save_frames_index(self) -> bool:
        if not self.config.output_dir:
            return False
        try:
            with self._state_lock:
                frames = list(self._frame_buffer)

            frames_index = [
                {
                    "frame_index": f.frame_index,
                    "timestamp_s": f.timestamp_s,
                    "timestamp": format_timestamp(f.timestamp_s),
                }
                for f in frames
            ]

            if frames_index:
                with open(self._frames_index_file, "w", encoding="utf-8") as f:
                    for rec in frames_index:
                        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                logger.info(f"saved {len(frames_index)} frames")
            return True
        except Exception as e:
            logger.error(f"save frames failed: {e}", exc_info=True)
            return False


# Backwards compatibility: import deploy from separate module
# from dimos.perception.temporal_memory_deploy import deploy
def deploy(
    dimos: DimosCluster,
    camera: spec.Camera,
    vlm: VlModel | None = None,
    config: TemporalMemoryConfig | None = None,
) -> TemporalMemory:
    """Deploy TemporalMemory with a camera.

    Args:
        dimos: Dimos cluster instance
        camera: Camera module to connect to
        vlm: Optional VLM instance (creates OpenAI VLM if None)
        config: Optional temporal memory configuration
    """
    if vlm is None:
        from dimos.models.vl.openai import OpenAIVlModel

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")
        vlm = OpenAIVlModel(api_key=api_key)

    temporal_memory = dimos.deploy(TemporalMemory, vlm=vlm, config=config)  # type: ignore[attr-defined]

    if camera.color_image.transport is None:
        from dimos.core.transport import JpegShmTransport

        transport = JpegShmTransport("/temporal_memory/color_image")
        camera.color_image.transport = transport

    temporal_memory.color_image.connect(camera.color_image)
    temporal_memory.start()
    return temporal_memory


temporal_memory = TemporalMemory.blueprint

__all__ = ["Frame", "TemporalMemory", "TemporalMemoryConfig", "deploy", "temporal_memory"]
