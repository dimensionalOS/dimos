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

from reactivex import interval
from reactivex.disposable import Disposable

from dimos import spec
from dimos.agents import skill
from dimos.core import DimosCluster, In, rpc
from dimos.core.module import ModuleConfig
from dimos.core.skill_module import SkillModule
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.clip_filter import (
    CLIP_AVAILABLE,
    CLIPFrameFilter,
    select_diverse_frames_simple,
)
from dimos.perception.videorag_utils import (
    apply_summary_update,
    build_query_prompt,
    build_summary_prompt,
    build_window_prompt,
    get_structured_output_format,
    parse_window_response,
    update_state_from_window,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
    frame_buffer_size: int = 300
    output_dir: str | Path | None = None
    max_tokens: int = 900
    temperature: float = 0.2
    use_clip_filtering: bool = True
    clip_model: str = "ViT-B/32"


def default_state() -> dict[str, Any]:
    return {
        "entity_roster": [],
        "rolling_summary": "",
        "chunk_buffer": [],
        "next_summary_at_s": 0.0,
        "last_present": [],
    }


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

        # protected state
        self._state = default_state()
        self._state["next_summary_at_s"] = float(self.config.summary_interval_s)
        self._frame_buffer: deque[Frame] = deque(maxlen=self.config.frame_buffer_size)
        self._recent_windows: deque[dict[str, Any]] = deque(maxlen=50)
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
            logger.info(f"artifacts save to: {self._output_path}")

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

        unsub_image = self.color_image.subscribe(on_frame)
        self._disposables.add(Disposable(unsub_image))

        self._disposables.add(
            interval(self.config.stride_s).subscribe(lambda _: self._analyze_window())
        )

        logger.info("temporalmemory started")

    @rpc
    def stop(self) -> None:
        self.save_state()
        self.save_entities()
        self.save_frames_index()

        if self._clip_filter:
            self._clip_filter.close()
            self._clip_filter = None

        # Stop all stream transports to clean up LCM/shared memory threads
        for stream in list(self.inputs.values()) + list(self.outputs.values()):
            if stream.transport is not None and hasattr(stream.transport, "stop"):
                stream.transport.stop()
                stream._transport = None

        super().stop()
        logger.info("temporalmemory stopped")

    def _format_timestamp(self, seconds: float) -> str:
        m = int(seconds // 60)
        s = seconds - 60 * m
        return f"{m:02d}:{s:06.3f}"

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

            # filter frames
            if len(window_frames) > self.config.max_frames_per_window:
                if self._clip_filter:
                    window_frames = self._clip_filter.select_diverse_frames(
                        window_frames, max_frames=self.config.max_frames_per_window
                    )
                else:
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
            # use middle frame for window analysis
            try:
                middle_frame = window_frames[len(window_frames) // 2]
                response_format = get_structured_output_format()
                response_text = self.vlm.query(
                    middle_frame.image, query, response_format=response_format
                )
            except Exception as e:
                logger.error(f"vlm agent query failed [{w_start:.1f}-{w_end:.1f}s]: {e}")
                with self._state_lock:
                    self._last_analysis_time = w_end
                return

            # parse response
            parsed = parse_window_response(response_text, w_start, w_end, len(window_frames))

            if "_error" in parsed:
                logger.error(f"parse error: {parsed['_error']}")
            else:
                logger.info(f"parsed. caption: {parsed.get('caption', '')[:100]}")

            # update state
            with self._state_lock:
                needs_summary = update_state_from_window(
                    self._state, parsed, w_end, self.config.summary_interval_s
                )
                self._recent_windows.append(parsed)
                self._last_analysis_time = w_end

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
                        apply_summary_update(
                            self._state, summary_text, w_end, self.config.summary_interval_s
                        )
                    logger.info(f"updated summary: {summary_text[:100]}...")
            except Exception as e:
                logger.error(f"summary update failed: {e}", exc_info=True)

        except Exception as e:
            logger.error(f"error updating summary: {e}", exc_info=True)

    @skill()
    def query(self, question: str) -> str:
        """
        Answer a question about the video stream using temporal memory.

        Args:
            question: Question to ask about the video stream

        Returns:
            Answer based on temporal memory state and video context
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

        # build context
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
    def save_state(self) -> bool:
        if not self.config.output_dir:
            return False
        try:
            with self._state_lock:
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

    def save_entities(self) -> bool:
        if not self.config.output_dir:
            return False
        try:
            with self._state_lock:
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
                    "timestamp": self._format_timestamp(f.timestamp_s),
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


def deploy(
    dimos: DimosCluster,
    camera: spec.Camera,
    vlm: VlModel | None = None,
    config: TemporalMemoryConfig | None = None,
) -> TemporalMemory:
    """
    Deploy TemporalMemory with a camera.

    Args:
        dimos: DimosCluster instance
        camera: Camera module
        vlm: Optional VlModel instance (will create OpenAIVlModel if None)
        config: Optional TemporalMemoryConfig

    Returns:
        Deployed TemporalMemory module
    """
    if vlm is None:
        from dimos.models.vl.openai import OpenAIVlModel

        # Load API key from environment
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
