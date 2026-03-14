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
Temporal Memory — thin orchestrator module.

Streams frames through ``FrameWindowAccumulator``, delegates VLM calls to
``WindowAnalyzer``, and persists results in ``EntityGraphDB`` + per-run JSONL.
"""

from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

from reactivex import Subject, interval
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.vl.base import VlModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import sharpness_barrier
from dimos.msgs.visualization_msgs.EntityMarkers import EntityMarkers, Marker
from dimos.utils.logging_config import get_run_log_dir, setup_logger

from . import temporal_utils as tu
from .clip_filter import CLIP_AVAILABLE, adaptive_keyframes
from .entity_graph_db import EntityGraphDB
from .frame_window_accumulator import Frame, FrameWindowAccumulator
from .temporal_state import TemporalState
from .window_analyzer import WindowAnalyzer

try:
    from .clip_filter import CLIPFrameFilter
except ImportError:
    CLIPFrameFilter = type(None)  # type: ignore[misc,assignment]

logger = setup_logger()

MAX_RECENT_WINDOWS = 50


class TemporalMemoryConfig(ModuleConfig):
    """Configuration for the temporal memory module.

    All VLM frequency knobs are exposed at the top level so users can
    tune cost / latency / accuracy without touching code.
    """

    vlm: VlModel[Any] | None = None

    # Frame processing
    fps: float = 1.0
    window_s: float = 5.0
    stride_s: float = 5.0
    max_frames_per_window: int = 3
    max_buffer_frames: int = 100

    # VLM call frequencies
    summary_interval_s: float = 30.0
    enable_distance_estimation: bool = True
    max_distance_pairs: int = 5
    stale_scene_threshold: float = 0.0  # 0 = disabled (CLIP filter handles duplicates)

    # VLM parameters
    max_tokens: int = 900
    temperature: float = 0.2

    # Storage
    db_dir: str | Path | None = (
        None  # Persistent memory dir (default: ~/.local/state/dimos/temporal_memory/)
    )
    new_memory: bool = False  # Clear persistent DB on start

    # Visualization
    visualize: bool = True

    # CLIP filtering
    use_clip_filtering: bool = True
    clip_model: str = "ViT-B/32"

    # Graph context (query-time)
    max_relations_per_entity: int = 10
    nearby_distance_meters: float = 5.0


class TemporalMemory(Module[TemporalMemoryConfig]):
    """Thin orchestrator that wires frames → window accumulator → VLM → state + DB.

    Uses RxPY reactive streams for the frame pipeline and ``interval`` for
    periodic window analysis.
    """

    default_config = TemporalMemoryConfig

    color_image: In[Image]
    odom: In[PoseStamped]
    entity_markers: Out[EntityMarkers]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._vlm_raw = self.config.vlm

        # new_memory is set via TemporalMemoryConfig by the blueprint factory
        # (which runs in the main process where GlobalConfig is available).

        # Components
        self._accumulator = FrameWindowAccumulator(
            max_buffer_frames=self.config.max_buffer_frames,
            window_s=self.config.window_s,
            stride_s=self.config.stride_s,
            fps=self.config.fps,
        )
        self._state = TemporalState(next_summary_at_s=self.config.summary_interval_s)
        self._recent_windows: deque[dict[str, Any]] = deque(maxlen=MAX_RECENT_WINDOWS)

        self._stopped = False
        self._distance_threads: list[threading.Thread] = []

        # Robot pose for entity world positioning
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_z: float = 0.0

        # CLIP filter
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

        # Persistent DB — stored in XDG state dir (same root as per-run logs)
        if self.config.db_dir:
            db_dir = Path(self.config.db_dir)
        else:
            # Default: ~/.local/state/dimos/temporal_memory/
            # XDG state dir — predictable, works for pip install and git clone.
            xdg = os.environ.get("XDG_STATE_HOME")
            state_root = Path(xdg) if xdg else Path.home() / ".local" / "state"
            db_dir = state_root / "dimos" / "temporal_memory"
        db_dir.mkdir(parents=True, exist_ok=True)
        db_path = db_dir / "entity_graph.db"
        if self.config.new_memory and db_path.exists():
            db_path.unlink()
            logger.info("Deleted existing DB (new_memory=True)")
        self._graph_db = EntityGraphDB(db_path=db_path)
        logger.info(f"persistent DB: {db_path}")

        # Persistent JSONL — accumulates across runs (raw VLM output + parsed)
        self._persistent_jsonl_path: Path = db_dir / "temporal_memory.jsonl"
        if self.config.new_memory and self._persistent_jsonl_path.exists():
            self._persistent_jsonl_path.unlink()
            logger.info("Deleted existing persistent JSONL (new_memory=True)")
        logger.info(f"persistent JSONL: {self._persistent_jsonl_path}")

        # Per-run JSONL log
        # get_run_log_dir() checks the in-process global; fall back to the
        # env var which is inherited by forkserver worker processes.
        self._jsonl_path: Path | None = None
        run_log_dir = get_run_log_dir()
        if run_log_dir is None:
            env_dir = os.environ.get("DIMOS_RUN_LOG_DIR")
            if env_dir:
                run_log_dir = Path(env_dir)
        if run_log_dir:
            tm_log_dir = run_log_dir / "temporal_memory"
            tm_log_dir.mkdir(parents=True, exist_ok=True)
            self._jsonl_path = tm_log_dir / "temporal_memory.jsonl"
            logger.info(f"per-run JSONL: {self._jsonl_path}")
        else:
            logger.warning("no run log dir found — JSONL logging disabled")

        logger.info(
            f"TemporalMemory init: fps={self.config.fps}, "
            f"window={self.config.window_s}s, stride={self.config.stride_s}s"
        )

    # ------------------------------------------------------------------
    # VLM access (lazy)
    # ------------------------------------------------------------------

    @property
    def vlm(self) -> VlModel[Any]:
        if self._vlm_raw is None:
            from dimos.models.vl.openai import OpenAIVlModel

            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY not set and no vlm instance provided")
            self._vlm_raw = OpenAIVlModel(api_key=api_key)
            logger.info("Created OpenAIVlModel from OPENAI_API_KEY")
        return self._vlm_raw

    @property
    def _analyzer(self) -> WindowAnalyzer:
        """Lazy WindowAnalyzer — avoids instantiating VLM at __init__ time."""
        if not hasattr(self, "__analyzer"):
            self.__analyzer = WindowAnalyzer(
                self.vlm,
                max_tokens=self.config.max_tokens,
                temperature=self.config.temperature,
            )
        return self.__analyzer

    # ------------------------------------------------------------------
    # JSONL logging
    # ------------------------------------------------------------------

    def _log_jsonl(self, record: dict[str, Any]) -> None:
        line = json.dumps(record, ensure_ascii=False) + "\n"
        # Write to per-run JSONL
        if self._jsonl_path is not None:
            try:
                with open(self._jsonl_path, "a") as f:
                    f.write(line)
            except Exception as e:
                logger.warning(f"per-run jsonl log failed: {e}")
        # Write to persistent JSONL (accumulates across runs)
        try:
            with open(self._persistent_jsonl_path, "a") as f:
                f.write(line)
        except Exception as e:
            logger.warning(f"persistent jsonl log failed: {e}")

    # ------------------------------------------------------------------
    # Rerun visualization
    # ------------------------------------------------------------------

    def _publish_entity_markers(self) -> None:
        """Publish entity positions as 3D markers for Rerun overlay on the map."""
        if not self.config.visualize:
            return
        try:
            all_entities = self._graph_db.get_all_entities()
            if not all_entities:
                return

            markers: list[Marker] = []
            for e in all_entities:
                meta = e.get("metadata") or {}
                x = meta.get("world_x")
                y = meta.get("world_y")
                z = meta.get("world_z")
                if x is None or y is None:
                    continue
                markers.append(
                    Marker(
                        entity_id=e["entity_id"],
                        label=(e.get("descriptor") or "")[:40],
                        entity_type=e.get("entity_type", "object"),
                        x=x,
                        y=y,
                        z=(z or 0.0) + 0.3,  # Offset up so labels float above ground
                    )
                )

            if markers:
                self.entity_markers.publish(EntityMarkers(markers=markers))
                logger.info(f"[temporal-memory] published {len(markers)} entity markers to Rerun")
        except Exception as e:
            logger.debug(f"entity marker publish error: {e}")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        self._stopped = False
        self._accumulator.set_start_time(time.time())

        # Frame ingest via reactive pipeline
        self._frame_count = 0
        self._odom_count = 0
        frame_subject: Subject[Image] = Subject()

        def _on_frame(img: Image) -> None:
            self._accumulator.add_frame(img, time.time())
            self._frame_count += 1
            if self._frame_count == 1 or self._frame_count % 20 == 0:
                logger.info(
                    f"[temporal-memory] frames={self._frame_count}, "
                    f"odom={self._odom_count}, "
                    f"buffered={len(self._accumulator._buffer)}"
                )

        self._disposables.add(
            frame_subject.pipe(sharpness_barrier(self.config.fps)).subscribe(_on_frame)
        )
        unsub_image = self.color_image.subscribe(frame_subject.on_next)
        self._disposables.add(Disposable(unsub_image))

        # Odometry tracking for entity world positioning (optional —
        # module works without it, entities just won't have world positions)
        def _on_odom(msg: PoseStamped) -> None:
            self._robot_x = msg.position.x
            self._robot_y = msg.position.y
            self._robot_z = msg.position.z
            self._odom_count += 1

        if self.odom.transport is not None:
            unsub_odom = self.odom.subscribe(_on_odom)
            self._disposables.add(Disposable(unsub_odom))
        else:
            logger.warning(
                "[temporal-memory] odom stream not connected — entity positions will be (0,0,0)"
            )

        # Periodic window analysis
        self._disposables.add(
            interval(self.config.stride_s).subscribe(lambda _: self._analyze_window())
        )
        logger.info("TemporalMemory started")

    @rpc
    def stop(self) -> None:
        self._stopped = True

        # Wait for distance threads
        for t in self._distance_threads:
            t.join(timeout=10.0)
        self._distance_threads.clear()

        if self._graph_db:
            self._graph_db.commit()
            self._graph_db.close()
            self._graph_db = None  # type: ignore[assignment]

        if self._clip_filter:
            self._clip_filter.close()
            self._clip_filter = None

        self._accumulator.clear()
        self._recent_windows.clear()
        self._state.clear(self.config.summary_interval_s)

        super().stop()

        for stream in list(self.inputs.values()) + list(self.outputs.values()):
            if stream.transport is not None and hasattr(stream.transport, "stop"):
                try:
                    stream.transport.stop()
                except Exception as e:
                    logger.warning(f"Failed to stop stream transport: {e}")

        logger.info("TemporalMemory stopped")

    # ------------------------------------------------------------------
    # Core loop
    # ------------------------------------------------------------------

    def _analyze_window(self) -> None:
        if self._stopped:
            return

        window_frames = self._accumulator.try_extract_window()
        if window_frames is None:
            if not hasattr(self, "_no_window_count"):
                self._no_window_count = 0
            self._no_window_count += 1
            if self._no_window_count <= 3 or self._no_window_count % 10 == 0:
                logger.info(
                    f"[temporal-memory] waiting for frames "
                    f"(buffered={len(self._accumulator._buffer)}, poll #{self._no_window_count})"
                )
            return
        w_start, w_end = window_frames[0].timestamp_s, window_frames[-1].timestamp_s

        # Skip stale scenes (frames too close together / camera not moving)
        if tu.is_scene_stale(window_frames, self.config.stale_scene_threshold):
            logger.info(f"[temporal-memory] skipping stale window [{w_start:.1f}-{w_end:.1f}s]")
            return

        # Select diverse keyframes
        window_frames = adaptive_keyframes(
            window_frames, max_frames=self.config.max_frames_per_window
        )
        logger.info(f"analyzing [{w_start:.1f}-{w_end:.1f}s] with {len(window_frames)} frames")

        # VLM Call #1: window analysis
        state_dict = self._state.to_dict()
        result = self._analyzer.analyze_window(window_frames, state_dict, w_start, w_end)
        if result is None:
            return

        parsed = result.parsed
        if "_error" in parsed:
            logger.error(f"parse error: {parsed['_error']}")

        # Log insights to terminal for user visibility
        caption = parsed.get("window", {}).get("caption") or parsed.get("caption", "")
        new_entities = parsed.get("new_entities", [])
        entities_present = parsed.get("entities_present", [])
        relations = parsed.get("relations", [])
        if caption:
            logger.info(f"[temporal-memory] caption: {caption[:200]}")
        if new_entities:
            names = ", ".join(
                f"{e.get('id')}({e.get('type', '?')}): {e.get('descriptor', '?')[:40]}"
                for e in new_entities
            )
            logger.info(f"[temporal-memory] NEW entities: {names}")
        if entities_present:
            ids = ", ".join(
                e.get("id", "?") if isinstance(e, dict) else str(e) for e in entities_present
            )
            logger.info(f"[temporal-memory] entities present: {ids}")
        if relations:
            rels = ", ".join(
                f"{r.get('subject', '?')}-[{r.get('type', '?')}]->{r.get('object', '?')}"
                for r in relations
            )
            logger.info(f"[temporal-memory] relations: {rels}")

        # Log raw VLM response
        self._log_jsonl(
            {
                "ts": time.time(),
                "type": "window_analysis",
                "window": [w_start, w_end],
                "raw_vlm_response": result.raw_vlm_response,
                "parsed": parsed,
            }
        )

        # VLM Call #2: distance estimation (background thread)
        if self._graph_db and self.config.enable_distance_estimation and window_frames:
            mid_frame = window_frames[len(window_frames) // 2]
            if mid_frame.image:
                thread = threading.Thread(
                    target=self._graph_db.estimate_and_save_distances,
                    args=(
                        parsed,
                        mid_frame.image,
                        self.vlm,
                        w_end,
                        self.config.max_distance_pairs,
                    ),
                    daemon=True,
                )
                thread.start()
                self._distance_threads = [t for t in self._distance_threads if t.is_alive()]
                self._distance_threads.append(thread)

        # Update state
        needs_summary = self._state.update_from_window(
            parsed, w_end, self.config.summary_interval_s
        )
        self._recent_windows.append(parsed)

        # Save to graph DB with robot world position
        if self._graph_db:
            self._graph_db.save_window_data(
                parsed,
                w_end,
                metadata={
                    "world_x": self._robot_x,
                    "world_y": self._robot_y,
                    "world_z": self._robot_z,
                },
            )

        # Publish entity markers for Rerun 3D overlay
        self._publish_entity_markers()

        # VLM Call #3: rolling summary
        if needs_summary:
            logger.info(f"updating summary at t≈{w_end:.1f}s")
            self._update_rolling_summary(w_end)

    def _update_rolling_summary(self, w_end: float) -> None:
        if self._stopped:
            return
        snap = self._state.snapshot()
        latest = self._accumulator.latest_frame()
        if not snap.chunk_buffer or not latest:
            return

        sr = self._analyzer.update_summary(latest.image, snap.rolling_summary, snap.chunk_buffer)
        if sr is not None:
            self._state.apply_summary(sr.summary_text, w_end, self.config.summary_interval_s)
            self._log_jsonl(
                {
                    "ts": time.time(),
                    "type": "rolling_summary",
                    "raw_vlm_response": sr.raw_vlm_response,
                    "summary": sr.summary_text,
                }
            )
            logger.info(f"[temporal-memory] SUMMARY: {sr.summary_text[:300]}")

    # ------------------------------------------------------------------
    # Query (agent skill)
    # ------------------------------------------------------------------

    @skill
    def query(self, question: str) -> str:
        """Answer a question about the video stream using temporal memory and graph knowledge.

        This skill analyzes the current video stream and temporal memory state
        to answer questions about what is happening, what entities are present,
        recent events, spatial relationships, and conceptual knowledge.

        The system automatically accesses knowledge graphs:
        - Interactions: relationships between entities (holds, looks_at, talks_to)
        - Spatial: distance and proximity information

        Example:
            query("What entities are currently visible?")
            query("What did I do last week?")
            query("Where did I leave my keys?")
            query("What objects are near the person?")

        Args:
            question (str): The question to ask about the video stream.

        Returns:
            str: Answer based on temporal memory, graph knowledge, and current frame.
        """
        snap = self._state.snapshot()
        latest = self._accumulator.latest_frame()
        if not latest:
            return "no frames available"

        current_video_time_s = latest.timestamp_s

        # Build currently-present set
        currently_present: set[str] = set()
        for e in snap.last_present:
            if isinstance(e, dict) and "id" in e:
                currently_present.add(e["id"])
        recent = list(self._recent_windows)
        for window in recent[-3:]:
            for entity in window.get("entities_present", []):
                if isinstance(entity, dict) and isinstance(entity.get("id"), str):
                    currently_present.add(entity["id"])
            for entity in window.get("new_entities", []):
                if isinstance(entity, dict) and isinstance(entity.get("id"), str):
                    currently_present.add(entity["id"])

        context: dict[str, Any] = {
            "entity_roster": snap.entity_roster,
            "rolling_summary": snap.rolling_summary,
            "currently_present_entities": sorted(currently_present),
            "recent_windows_count": len(recent),
            "timestamp": time.time(),
        }

        # Graph context
        if self._graph_db:
            time_window_s = tu.extract_time_window(question)
            all_entity_ids = [
                e["id"] for e in snap.entity_roster if isinstance(e, dict) and "id" in e
            ]
            if all_entity_ids:
                logger.info(f"query: building graph context for {len(all_entity_ids)} entities")
                graph_context = tu.build_graph_context(
                    graph_db=self._graph_db,
                    entity_ids=all_entity_ids,
                    time_window_s=time_window_s,
                    max_relations_per_entity=self.config.max_relations_per_entity,
                    nearby_distance_meters=self.config.nearby_distance_meters,
                    current_video_time_s=current_video_time_s,
                )
                context["graph_knowledge"] = graph_context

        logger.info(
            f"query: calling VLM with {len(context.get('currently_present_entities', []))} present entities"
        )
        qr = self._analyzer.answer_query(question, context, latest.image)
        if qr is None:
            return "error: VLM query failed"

        self._log_jsonl(
            {
                "ts": time.time(),
                "type": "query",
                "question": question,
                "raw_vlm_response": qr.raw_vlm_response,
                "answer": qr.answer,
            }
        )
        return qr.answer

    # ------------------------------------------------------------------
    # RPC accessors (backward compat)
    # ------------------------------------------------------------------

    @rpc
    def clear_history(self) -> bool:
        try:
            self._state.clear(self.config.summary_interval_s)
            self._recent_windows.clear()
            logger.info("cleared history")
            return True
        except Exception as e:
            logger.error(f"clear_history failed: {e}", exc_info=True)
            return False

    @rpc
    def get_state(self) -> dict[str, Any]:
        snap = self._state.snapshot()
        return {
            "entity_count": len(snap.entity_roster),
            "entities": snap.entity_roster,
            "rolling_summary": snap.rolling_summary,
            "frame_count": self._accumulator.frame_count,
            "buffer_size": self._accumulator.buffer_size,
            "recent_windows": len(self._recent_windows),
            "currently_present": snap.last_present,
        }

    @rpc
    def get_entity_roster(self) -> list[dict[str, Any]]:
        return self._state.snapshot().entity_roster

    @rpc
    def get_rolling_summary(self) -> str:
        return self._state.snapshot().rolling_summary

    @rpc
    def get_graph_db_stats(self) -> dict[str, Any]:
        if not self._graph_db:
            return {"stats": {}, "entities": [], "recent_relations": []}
        return self._graph_db.get_summary()


temporal_memory = TemporalMemory.blueprint

__all__ = ["Frame", "TemporalMemory", "TemporalMemoryConfig", "temporal_memory"]
