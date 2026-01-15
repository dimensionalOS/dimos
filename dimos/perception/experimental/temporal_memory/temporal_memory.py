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

from reactivex import Subject, interval
from reactivex.disposable import Disposable

from dimos.agents import skill
from dimos.core import In, rpc
from dimos.core.module import ModuleConfig
from dimos.core.skill_module import SkillModule
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import sharpness_barrier

from . import temporal_utils as tu
from .clip_filter import (
    CLIP_AVAILABLE,
    adaptive_keyframes,
    select_diverse_frames_simple,
)

try:
    from .clip_filter import CLIPFrameFilter
except ImportError:
    CLIPFrameFilter = type(None)  # type: ignore[misc,assignment]
from dimos.utils.logging_config import setup_logger

from .entity_graph_db import EntityGraphDB

logger = setup_logger()

# Constants
MAX_RECENT_WINDOWS = 50  # Max recent windows to keep in memory


@dataclass
class Frame:
    frame_index: int
    timestamp_s: float
    image: Image


@dataclass
class TemporalMemoryConfig(ModuleConfig):
    # Frame processing
    fps: float = 1.0
    window_s: float = 2.0
    stride_s: float = 2.0
    summary_interval_s: float = 10.0
    max_frames_per_window: int = 3
    frame_buffer_size: int = 50

    # Output
    output_dir: str | Path | None = "assets/temporal_memory"

    # VLM parameters
    max_tokens: int = 900
    temperature: float = 0.2

    # Frame filtering
    use_clip_filtering: bool = True
    clip_model: str = "ViT-B/32"
    stale_scene_threshold: float = 5.0

    # Graph database
    persistent_memory: bool = True  # Keep graph across sessions
    clear_memory_on_start: bool = False  # Wipe DB on startup
    enable_distance_estimation: bool = True  # Estimate entity distances
    max_distance_pairs: int = 5  # Max entity pairs per window

    # Graph context
    max_relations_per_entity: int = 10  # Max relations in query context
    nearby_distance_meters: float = 5.0  # "Nearby" threshold


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
        self.config: TemporalMemoryConfig = config or TemporalMemoryConfig()

        # single lock protects all state
        self._state_lock = threading.Lock()
        self._stopped = False

        # protected state
        self._state = tu.default_state()
        self._state["next_summary_at_s"] = float(self.config.summary_interval_s)
        self._frame_buffer: deque[Frame] = deque(maxlen=self.config.frame_buffer_size)
        self._recent_windows: deque[dict[str, Any]] = deque(maxlen=MAX_RECENT_WINDOWS)
        self._frame_count = 0
        # Start at -inf so first analysis passes stride_s check regardless of elapsed time
        self._last_analysis_time = -float("inf")
        self._video_start_wall_time: float | None = None

        # Track background distance estimation threads
        self._distance_threads: list[threading.Thread] = []

        # clip filter - use instance state to avoid mutating shared config
        self._clip_filter: CLIPFrameFilter | None = None
        self._use_clip_filtering = self.config.use_clip_filtering
        if self._use_clip_filtering and CLIP_AVAILABLE:
            try:
                self._clip_filter = CLIPFrameFilter(model_name=self.config.clip_model)
                logger.info("clip filtering enabled")
            except Exception as e:
                logger.warning(f"clip init failed: {e}")
                self._use_clip_filtering = False
        elif self._use_clip_filtering:
            logger.warning("clip not available")
            self._use_clip_filtering = False

        # output directory
        self._graph_db: EntityGraphDB | None
        if self.config.output_dir:
            self._output_path = Path(self.config.output_dir)
            self._output_path.mkdir(parents=True, exist_ok=True)
            self._evidence_file = self._output_path / "evidence.jsonl"
            self._state_file = self._output_path / "state.json"
            self._entities_file = self._output_path / "entities.json"
            self._frames_index_file = self._output_path / "frames_index.jsonl"

            db_path = self._output_path / "entity_graph.db"
            if not self.config.persistent_memory or self.config.clear_memory_on_start:
                if db_path.exists():
                    db_path.unlink()
                    reason = (
                        "non-persistent mode"
                        if not self.config.persistent_memory
                        else "clear_memory_on_start=True"
                    )
                    logger.info(f"Deleted existing database: {reason}")

            self._graph_db = EntityGraphDB(db_path=db_path)

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
                if video_start is None:
                    return  # Not started yet
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

        frame_subject: Subject[Image] = Subject()
        self._disposables.add(
            frame_subject.pipe(sharpness_barrier(self.config.fps)).subscribe(on_frame)
        )

        unsub_image = self.color_image.subscribe(frame_subject.on_next)
        self._disposables.add(Disposable(unsub_image))

        # Schedule window analysis every stride_s seconds
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
        with self._state_lock:
            self._stopped = True

        # Wait for background distance estimation threads to complete before closing DB
        if self._distance_threads:
            logger.info(f"Waiting for {len(self._distance_threads)} distance estimation threads...")
            for thread in self._distance_threads:
                thread.join(timeout=10.0)  # Wait max 10s per thread
            self._distance_threads.clear()

        if self._graph_db:
            db_path = self._graph_db.db_path
            self._graph_db.commit()  # save all pending transactions
            self._graph_db.close()
            self._graph_db = None

            if not self.config.persistent_memory and db_path.exists():
                db_path.unlink()
                logger.info("Deleted non-persistent database")

        if self._clip_filter:
            self._clip_filter.close()
            self._clip_filter = None

        with self._state_lock:
            self._frame_buffer.clear()
            self._recent_windows.clear()
            self._state = tu.default_state()

        super().stop()

        # Stop all stream transports to clean up LCM/shared memory threads
        # Note: We use public stream.transport API and rely on transport.stop() to clean up
        for stream in list(self.inputs.values()) + list(self.outputs.values()):
            if stream.transport is not None and hasattr(stream.transport, "stop"):
                try:
                    stream.transport.stop()
                except Exception as e:
                    logger.warning(f"Failed to stop stream transport: {e}")

        logger.info("temporalmemory stopped")

    def _get_window_frames(self) -> tuple[list[Frame], dict[str, Any]] | None:
        """Extract window frames from buffer with guards."""
        with self._state_lock:
            if not self._frame_buffer:
                return None
            current_time = self._frame_buffer[-1].timestamp_s
            if current_time - self._last_analysis_time < self.config.stride_s:
                return None
            frames_needed = max(1, int(self.config.fps * self.config.window_s))
            if len(self._frame_buffer) < frames_needed:
                return None
            window_frames = list(self._frame_buffer)[-frames_needed:]
            state_snapshot = self._state.copy()
        return window_frames, state_snapshot

    def _query_vlm_for_window(
        self,
        window_frames: list[Frame],
        state_snapshot: dict[str, Any],
        w_start: float,
        w_end: float,
    ) -> str | None:
        """Query VLM for window analysis."""
        query = tu.build_window_prompt(
            w_start=w_start, w_end=w_end, frame_count=len(window_frames), state=state_snapshot
        )
        try:
            fmt = tu.get_structured_output_format()
            if len(window_frames) > 1:
                responses = self.vlm.query_batch(
                    [f.image for f in window_frames], query, response_format=fmt
                )
                return responses[0] if responses else ""
            else:
                return self.vlm.query(window_frames[0].image, query, response_format=fmt)
        except Exception as e:
            logger.error(f"vlm query failed [{w_start:.1f}-{w_end:.1f}s]: {e}", exc_info=True)
            return None

    def _save_window_artifacts(self, parsed: dict[str, Any], w_end: float) -> None:
        """Save window data to graph DB and evidence file."""
        if self._graph_db:
            self._graph_db.save_window_data(parsed, w_end)
        if self.config.output_dir:
            self._append_evidence(parsed)

    def _analyze_window(self) -> None:
        """Analyze a temporal window of frames using VLM."""
        # Extract window frames with guards
        result = self._get_window_frames()
        if result is None:
            return
        window_frames, state_snapshot = result
        w_start, w_end = window_frames[0].timestamp_s, window_frames[-1].timestamp_s

        # Skip if scene hasn't changed
        if tu.is_scene_stale(window_frames, self.config.stale_scene_threshold):
            with self._state_lock:
                self._last_analysis_time = w_end
            return

        # Select diverse frames for analysis
        window_frames = (
            adaptive_keyframes(  # TODO: unclear if clip vs. diverse vs. this solution is best
                window_frames, max_frames=self.config.max_frames_per_window
            )
        )
        logger.info(f"analyzing [{w_start:.1f}-{w_end:.1f}s] with {len(window_frames)} frames")

        # Query VLM and parse response
        response_text = self._query_vlm_for_window(window_frames, state_snapshot, w_start, w_end)
        if response_text is None:
            with self._state_lock:
                self._last_analysis_time = w_end
            return

        parsed = tu.parse_window_response(response_text, w_start, w_end, len(window_frames))
        if "_error" in parsed:
            logger.error(f"parse error: {parsed['_error']}")
        # else:
        #     logger.info(f"parsed. caption: {parsed.get('caption', '')[:100]}")

        # Start distance estimation in background
        if self._graph_db and window_frames and self.config.enable_distance_estimation:
            mid_frame = window_frames[len(window_frames) // 2]
            if mid_frame.image:
                thread = threading.Thread(
                    target=self._graph_db.estimate_and_save_distances,
                    args=(parsed, mid_frame.image, self.vlm, w_end, self.config.max_distance_pairs),
                    daemon=True,
                )
                thread.start()
                self._distance_threads = [t for t in self._distance_threads if t.is_alive()]
                self._distance_threads.append(thread)

        # Update temporal state
        with self._state_lock:
            needs_summary = tu.update_state_from_window(
                self._state, parsed, w_end, self.config.summary_interval_s
            )
            self._recent_windows.append(parsed)
            self._last_analysis_time = w_end

        # Save artifacts
        self._save_window_artifacts(parsed, w_end)

        # Trigger summary update if needed
        if needs_summary:
            logger.info(f"updating summary at t≈{w_end:.1f}s")
            self._update_rolling_summary(w_end)

        # Periodic state saves
        with self._state_lock:
            window_count = len(self._recent_windows)
        if window_count % 10 == 0:
            self.save_state()
            self.save_entities()

    def _update_rolling_summary(self, w_end: float) -> None:
        with self._state_lock:
            if self._stopped:
                return
            rolling_summary = str(self._state.get("rolling_summary", ""))
            chunk_buffer = list(self._state.get("chunk_buffer", []))
            latest_frame = self._frame_buffer[-1].image if self._frame_buffer else None

        if not chunk_buffer or not latest_frame:
            return

        prompt = tu.build_summary_prompt(
            rolling_summary=rolling_summary, chunk_windows=chunk_buffer
        )

        try:
            summary_text = self.vlm.query(latest_frame, prompt)
            if summary_text and summary_text.strip():
                with self._state_lock:
                    if self._stopped:
                        return
                    tu.apply_summary_update(
                        self._state, summary_text, w_end, self.config.summary_interval_s
                    )
                logger.info(f"updated summary: {summary_text[:100]}...")
                if self.config.output_dir and not self._stopped:
                    self.save_state()
                    self.save_entities()
        except Exception as e:
            logger.error(f"summary update failed: {e}", exc_info=True)

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
            if self._frame_buffer:
                latest_frame = self._frame_buffer[-1].image
                current_video_time_s = self._frame_buffer[-1].timestamp_s
            else:
                latest_frame = None
                current_video_time_s = 0.0

        if not latest_frame:
            return "no frames available"

        # build context from temporal state
        # Include entities from last_present and recent windows (both entities_present and new_entities)
        currently_present = {e["id"] for e in last_present if isinstance(e, dict) and "id" in e}
        for window in recent_windows[-3:]:
            # Add entities that were present
            for entity in window.get("entities_present", []):
                if isinstance(entity, dict) and isinstance(entity.get("id"), str):
                    currently_present.add(entity["id"])
            # Also include newly detected entities (they're present now)
            for entity in window.get("new_entities", []):
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
        if self._graph_db:
            # Extract time window from question using VLM
            time_window_s = tu.extract_time_window(question, self.vlm, latest_frame)

            # Query graph for ALL entities in roster (not just currently present)
            # This allows queries about entities that disappeared or were seen in the past
            all_entity_ids = [e["id"] for e in entity_roster if isinstance(e, dict) and "id" in e]

            if all_entity_ids:
                graph_context = tu.build_graph_context(
                    graph_db=self._graph_db,
                    entity_ids=all_entity_ids,
                    time_window_s=time_window_s,
                    max_relations_per_entity=self.config.max_relations_per_entity,
                    nearby_distance_meters=self.config.nearby_distance_meters,
                    current_video_time_s=current_video_time_s,
                )
                context["graph_knowledge"] = graph_context

        # build query prompt using temporal utils
        prompt = tu.build_query_prompt(question=question, context=context)

        # query vlm (slow, outside lock)
        try:
            answer_text = self.vlm.query(latest_frame, prompt)
            return answer_text.strip()
        except Exception as e:
            logger.error(f"query failed: {e}", exc_info=True)
            return f"error: {e}"

    @rpc
    def clear_history(self) -> bool:
        """Clear temporal memory state."""
        try:
            with self._state_lock:
                self._state = tu.default_state()
                self._state["next_summary_at_s"] = float(self.config.summary_interval_s)
                self._recent_windows.clear()
            logger.info("cleared history")
            return True
        except Exception as e:
            logger.error(f"clear_history failed: {e}", exc_info=True)
            return False

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
        """Get statistics and sample data from the graph database.

        Returns empty structures when no database is available (no-error pattern).
        """
        if not self._graph_db:
            return {"stats": {}, "entities": [], "recent_relations": []}
        return self._graph_db.get_summary()

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
            logger.error(f"append evidence failed: {e}", exc_info=True)

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
                    "timestamp": tu.format_timestamp(f.timestamp_s),
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


temporal_memory = TemporalMemory.blueprint

__all__ = ["Frame", "TemporalMemory", "TemporalMemoryConfig", "temporal_memory"]
