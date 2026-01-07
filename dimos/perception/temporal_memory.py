"""
Temporal Memory module for creating entity-based temporal understanding of video streams.

This module implements a sophisticated temporal memory system inspired by VideoRAG,
using VLM (Vision-Language Model) API calls to maintain entity rosters, rolling summaries,
and temporal relationships across video frames.
"""

from collections import deque
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

from reactivex import interval

from dimos import spec
from dimos.agents import skill  # type: ignore[attr-defined]
from dimos.core import DimosCluster, In, Module, rpc
from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image
from dimos.perception.clip_filter import (
    CLIP_AVAILABLE,
    CLIPFrameFilter,
    select_diverse_frames_simple,
)
from dimos.perception.videorag_utils import (
    apply_summary_update,
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
    """Represents a single video frame with temporal metadata."""

    frame_index: int
    timestamp_s: float
    timestamp: str  # Formatted as MM:SS.mmm
    image: Image


@dataclass
class TemporalMemoryConfig:
    """Configuration for TemporalMemory module."""

    fps: float = 1.0  # Processing rate in frames per second
    window_s: float = 2.0  # Window size in seconds
    stride_s: float = 2.0  # Stride between windows in seconds
    summary_interval_s: float = 10.0  # How often to update rolling summary
    max_frames_per_window: int = 3  # Max frames to analyze per window
    frame_buffer_size: int = 300  # Max frames to keep in buffer (30s at 10fps)
    output_dir: str | Path | None = None  # Optional directory for artifacts
    max_tokens: int = 900  # Max tokens for VLM responses
    temperature: float = 0.2  # Temperature for VLM queries
    use_clip_filtering: bool = True  # Use CLIP to select diverse frames
    clip_model: str = "ViT-B/32"  # CLIP model for frame filtering
    use_structured_output: bool = True  # Use structured output for VLM (GPT-4o mini supports json_schema)
    use_multi_image: bool = True  # Send multiple frames to VLM in single request (GPT-4o mini supports multi-image)


def default_state() -> dict[str, Any]:
    """Create default state dictionary for temporal memory."""
    return {
        "entity_roster": [],  # List of known entities: [{"id": "E1", "descriptor": "..."}, ...]
        "rolling_summary": "",  # Rolling summary of recent events
        "chunk_buffer": [],  # Buffer of recent windows for summarization
        "next_summary_at_s": 0.0,  # When to update rolling summary next
        "last_present": [],  # Entities present in last window
    }


def safe_float(value: Any, default: float) -> float:
    """Safely convert value to float with fallback."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class TemporalMemory(Module):
    """
    A Dask module for building temporal understanding of video streams using VLMs.

    This module processes video frames, maintains entity rosters, tracks temporal
    relationships, and builds rolling summaries of what has happened over time.
    Unlike SpatialMemory which focuses on spatial locations, TemporalMemory focuses
    on understanding what entities are present, how they relate, and what events
    have occurred over time.
    """

    # LCM inputs
    color_image: In[Image]

    def __init__(
        self,
        vlm: VlModel,
        config: TemporalMemoryConfig | None = None,
        output_dir: str | Path | None = None,
    ) -> None:
        """
        Initialize the temporal memory system.

        Args:
            vlm: Vision-Language Model instance for image analysis
            config: Configuration parameters for temporal memory
            output_dir: Optional directory for storing artifacts
        """
        super().__init__()

        self.vlm = vlm
        self.config = config or TemporalMemoryConfig()

        # Verify API key if using OpenAIVlModel
        if hasattr(vlm, "__class__") and "OpenAI" in vlm.__class__.__name__:
            api_key = getattr(vlm.config, "api_key", None) if hasattr(vlm, "config") else None
            if not api_key and not os.getenv("OPENAI_API_KEY"):
                logger.warning(
                    "OpenAIVlModel detected but OPENAI_API_KEY not found. "
                    "Set OPENAI_API_KEY environment variable or pass api_key in config."
                )

        # Override output_dir if provided
        if output_dir is not None:
            self.config.output_dir = output_dir

        # Thread safety for state access
        self.state_lock = threading.Lock()

        # Frame buffer (holds recent frames for temporal analysis)
        self.frame_buffer: deque[Frame] = deque(maxlen=self.config.frame_buffer_size)

        # Temporal state
        self.state = default_state()
        self.state["next_summary_at_s"] = float(self.config.summary_interval_s)

        # Recent windows for query context
        self.recent_windows: deque[dict[str, Any]] = deque(maxlen=50)

        # Tracking
        self.frame_count = 0
        self.video_start_time = 0.0
        self.video_start_wall_time: float | None = None

        # Latest frame for processing
        self._latest_image: Image | None = None
        self._last_analysis_time = 0.0

        # CLIP filter for frame selection (if enabled and available)
        self.clip_filter: CLIPFrameFilter | None = None
        if self.config.use_clip_filtering and CLIP_AVAILABLE:
            try:
                self.clip_filter = CLIPFrameFilter(model_name=self.config.clip_model)
                logger.info("CLIP frame filtering enabled")
            except Exception as e:
                logger.warning(f"Failed to initialize CLIP filter: {e}. Using simple sampling.")
                self.config.use_clip_filtering = False
        elif self.config.use_clip_filtering and not CLIP_AVAILABLE:
            logger.warning(
                "CLIP filtering requested but CLIP not available. "
                "Install with: pip install torch torchvision openai-clip"
            )
            self.config.use_clip_filtering = False

        # Output directory setup
        if self.config.output_dir:
            self.output_path = Path(self.config.output_dir)
            self.output_path.mkdir(parents=True, exist_ok=True)
            self.evidence_file = self.output_path / "evidence.jsonl"
            self.state_file = self.output_path / "state.json"
            self.entities_file = self.output_path / "entities.json"
            self.frames_index_file = self.output_path / "frames_index.jsonl"
            logger.info(f"Temporal memory artifacts will be saved to: {self.output_path}")

        logger.info(
            f"TemporalMemory initialized with fps={self.config.fps}, "
            f"window={self.config.window_s}s, stride={self.config.stride_s}s"
        )

    def __getstate__(self) -> dict[str, Any]:
        """Exclude unpicklable runtime attributes when serializing."""
        state = self.__dict__.copy()
        # Remove unpicklable attributes
        state.pop("state_lock", None)
        # Ensure VLM's API key is preserved if it's an OpenAIVlModel
        if hasattr(self.vlm, "config") and hasattr(self.vlm.config, "api_key") and self.vlm.config.api_key:
            # The API key should already be in the VLM instance, but ensure it's preserved
            pass
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        """Restore object from pickled state."""
        self.__dict__.update(state)
        # Reinitialize runtime attributes
        self.state_lock = threading.Lock()
        # Reload environment variables on worker if needed for OpenAI VLM
        if hasattr(self.vlm, "__class__") and "OpenAI" in self.vlm.__class__.__name__:
            if hasattr(self.vlm, "config") and not getattr(self.vlm.config, "api_key", None):
                # Try to load from environment on worker
                api_key = _load_openai_api_key()
                if api_key and hasattr(self.vlm, "config"):
                    self.vlm.config.api_key = api_key

    @rpc
    def start(self) -> None:
        """Start the temporal memory system."""
        super().start()

        # Initialize video start time
        if self.video_start_wall_time is None:
            self.video_start_wall_time = time.time()

        # Subscribe to image stream
        def set_image(image_msg: Image) -> None:
            self._latest_image = image_msg

            # Calculate video timestamp (relative to video start)
            video_timestamp_s = time.time() - self.video_start_wall_time

            # Create Frame object
            frame_obj = Frame(
                frame_index=self.frame_count,
                timestamp_s=video_timestamp_s,
                timestamp=self._format_timestamp(video_timestamp_s),
                image=image_msg,
            )

            # Add to buffer
            self.frame_buffer.append(frame_obj)
            self.frame_count += 1

        # Subscribe to image stream - subscribe() returns a Disposable directly
        self._disposables.add(self.color_image.subscribe(set_image))

        # Start periodic analysis using interval
        analysis_interval_s = int(self.config.stride_s)
        self._disposables.add(interval(analysis_interval_s).subscribe(lambda _: self._analyze_window()))

        logger.info("TemporalMemory started")

    @rpc
    def stop(self) -> None:
        """Stop the temporal memory system and save artifacts."""
        # Save all artifacts before stopping
        self.save_state()
        self.save_entities()
        self.save_frames_index()

        # Clean up CLIP filter
        if self.clip_filter is not None:
            self.clip_filter.close()
            self.clip_filter = None

        super().stop()
        # Clean up shared memory transport if it exists
        if hasattr(self.color_image, 'transport') and self.color_image.transport is not None:
            # JpegShmTransport should clean up shared memory on close/dispose
            if hasattr(self.color_image.transport, 'close'):
                try:
                    self.color_image.transport.close()
                except Exception as e:
                    logger.warning(f"Error closing transport: {e}")
            elif hasattr(self.color_image.transport, 'dispose'):
                try:
                    self.color_image.transport.dispose()
                except Exception as e:
                    logger.warning(f"Error disposing transport: {e}")
            logger.info("TemporalMemory stopped")

    def _format_timestamp(self, seconds: float) -> str:
        """Format timestamp as MM:SS.mmm"""
        m = int(seconds // 60)
        s = seconds - 60 * m
        return f"{m:02d}:{s:06.3f}"

    def _analyze_window(self) -> None:
        """Analyze a window of frames using VLM."""
        try:
            # Get current video time from buffer
            if len(self.frame_buffer) == 0:
                return

            current_video_time = self.frame_buffer[-1].timestamp_s

            # Check if enough time has passed
            if current_video_time - self._last_analysis_time < self.config.stride_s:
                return

            # Check if we have enough frames for a window
            frames_needed = max(1, int(self.config.fps * self.config.window_s))
            if len(self.frame_buffer) < frames_needed:
                logger.debug(f"Not enough frames: {len(self.frame_buffer)} < {frames_needed}")
                return

            # Get window frames (most recent N frames)
            window_frames = list(self.frame_buffer)[-frames_needed:]

            w_start = window_frames[0].timestamp_s
            w_end = window_frames[-1].timestamp_s

            # Apply frame filtering to select diverse/key frames
            if len(window_frames) > self.config.max_frames_per_window:
                if self.clip_filter is not None:
                    # Use CLIP to select diverse frames
                    window_frames = self.clip_filter.select_diverse_frames(
                        window_frames, max_frames=self.config.max_frames_per_window
                    )
                    logger.debug(
                        f"CLIP filtered {frames_needed} frames to {len(window_frames)} diverse frames"
                    )
                else:
                    # Fallback to simple uniform sampling
                    window_frames = select_diverse_frames_simple(
                        window_frames, max_frames=self.config.max_frames_per_window
                    )
                    logger.debug(
                        f"Simple sampling: {frames_needed} frames to {len(window_frames)} frames"
                    )

            logger.info(
                f"Analyzing window [{w_start:.1f}-{w_end:.1f}s] with {len(window_frames)} frames"
            )

            # Build VLM query using VideoRAG prompt
            with self.state_lock:
                state_snapshot = self.state.copy()

            query = build_window_prompt(
                w_start=w_start,
                w_end=w_end,
                frame_count=len(window_frames),
                state=state_snapshot,
            )

            # Query VLM with multi-image support or single image
            try:
                if self.config.use_multi_image and hasattr(self.vlm, "query_multi_images"):
                    # Multi-image query (all frames in one request)
                    images = [f.image for f in window_frames]

                    # Check if VLM supports structured output
                    if self.config.use_structured_output and hasattr(self.vlm, "query_multi_images"):
                        # Try with structured output
                        try:
                            response_format = get_structured_output_format()
                            response_text = self.vlm.query_multi_images(
                                images, query, response_format=response_format
                            )
                        except (TypeError, Exception) as e:
                            # Fallback if structured output not supported
                            logger.debug(f"Structured output not supported, using standard: {e}")
                            response_text = self.vlm.query_multi_images(images, query)
                    else:
                        response_text = self.vlm.query_multi_images(images, query)

                    logger.debug(f"Used multi-image query with {len(images)} frames")
                else:
                    # Fallback: Single image query with middle frame
                    middle_frame = window_frames[len(window_frames) // 2]
                    response_text = self.vlm.query(middle_frame.image, query)
                    logger.debug("Used single-image query (multi-image not available)")

            except Exception as e:
                logger.error(f"VLM query failed for window [{w_start:.1f}-{w_end:.1f}s]: {e}")
                self._last_analysis_time = w_end
                return

            # Log raw VLM response for debugging
            logger.info(f"VLM raw response (first 1000 chars): {response_text[:1000]}")

            # Parse response using VideoRAG parser
            parsed = parse_window_response(response_text, w_start, w_end, len(window_frames))

            # Log parsing results
            if "_error" in parsed:
                logger.error(f"Parse error: {parsed['_error']}")
                logger.error(f"Raw text was: {response_text}")
            else:
                logger.info(f"Parsed successfully. Caption: {parsed.get('caption', '')[:100]}")
                logger.info(f"New entities: {parsed.get('new_entities', [])}")
                logger.info(f"Entities present: {parsed.get('entities_present', [])}")

            # Log parsed response for debugging
            new_entities_count = len(parsed.get("new_entities", []))
            present_entities_count = len(parsed.get("entities_present", []))
            if new_entities_count > 0 or present_entities_count > 0:
                logger.debug(
                    f"Parsed window [{w_start:.1f}-{w_end:.1f}s]: "
                    f"{new_entities_count} new entities, {present_entities_count} present entities"
                )
            elif parsed.get("caption"):
                logger.debug(
                    f"Window [{w_start:.1f}-{w_end:.1f}s] parsed but no entities extracted. "
                    f"Caption: {parsed.get('caption', '')[:100]}"
                )

            # Update state using VideoRAG logic
            with self.state_lock:
                needs_summary = update_state_from_window(
                    self.state, parsed, w_end, self.config.summary_interval_s
                )

            # Store in recent windows
            self.recent_windows.append(parsed)

            # Save evidence incrementally
            if self.config.output_dir:
                self._append_evidence(parsed)

            # Update rolling summary if needed
            if needs_summary:
                logger.info(f"Updating rolling summary at t≈{w_end:.1f}s")
                self._update_rolling_summary(w_end)

            # Save state periodically (every 10 windows)
            if len(self.recent_windows) % 10 == 0:
                self.save_state()
                self.save_entities()

            self._last_analysis_time = w_end
            logger.info(
                f"Analyzed window [{w_start:.1f}-{w_end:.1f}s] - "
                f"{len(parsed.get('new_entities', []))} new entities"
            )

        except Exception as e:
            logger.error(f"Error analyzing window: {e}", exc_info=True)

    def _update_rolling_summary(self, w_end: float) -> None:
        """Update rolling summary from recent chunks using VLM with VideoRAG prompt."""
        try:
            with self.state_lock:
                rolling_summary = str(self.state.get("rolling_summary", ""))
                chunk_buffer = list(self.state.get("chunk_buffer", []))

            if not chunk_buffer:
                return

            # Build summary prompt using VideoRAG format
            prompt = build_summary_prompt(
                rolling_summary=rolling_summary,
                chunk_windows=chunk_buffer,
            )

            # Query VLM with latest frame as context
            if len(self.frame_buffer) > 0:
                latest_frame = self.frame_buffer[-1]
                try:
                    summary_text = self.vlm.query(latest_frame.image, prompt)
                    if summary_text and summary_text.strip():
                        with self.state_lock:
                            apply_summary_update(
                                self.state, summary_text, w_end, self.config.summary_interval_s
                            )
                        logger.info(f"Updated rolling summary: {summary_text[:100]}...")
                except Exception as e:
                    logger.error(f"Failed to update rolling summary: {e}", exc_info=True)

        except Exception as e:
            logger.error(f"Error updating rolling summary: {e}", exc_info=True)

    def _query_impl(self, question: str) -> str:
        """Internal implementation for querying temporal memory."""
        with self.state_lock:
            entity_roster = list(self.state.get("entity_roster", []))
            rolling_summary = str(self.state.get("rolling_summary", ""))
            last_present = list(self.state.get("last_present", []))

        # Get currently present entities
        currently_present = {e["id"] for e in last_present if isinstance(e, dict) and "id" in e}

        # Also check last 3 windows
        for window in list(self.recent_windows)[-3:]:
            for entity in window.get("entities_present", []):
                if isinstance(entity, dict) and isinstance(entity.get("id"), str):
                    currently_present.add(entity["id"])

        context = {
            "entity_roster": entity_roster,
            "rolling_summary": rolling_summary,
            "currently_present_entities": sorted(currently_present),
            "recent_windows_count": len(self.recent_windows),
            "timestamp": time.time(),
        }

        query_prompt = f"""Answer the following question about the video stream using the provided context.

**Question:** {question}

**Context:**
{json.dumps(context, indent=2, ensure_ascii=False)}

**Instructions:**
- Entities have stable IDs like E1, E2, etc.
- The 'currently_present_entities' list shows which entities are visible now
- If an entity is NOT in 'currently_present_entities', it is no longer visible
- Answer based ONLY on the provided context
- If information isn't available, say so clearly

Provide a concise answer.
"""

        # Query VLM with latest frame as context
        if len(self.frame_buffer) > 0:
            latest_frame = self.frame_buffer[-1]
            try:
                answer = self.vlm.query(latest_frame.image, query_prompt)
                return answer.strip()
            except Exception as e:
                logger.error(f"Query failed: {e}", exc_info=True)
                return f"Error answering question: {e}"

        return "No frames available to answer question"

    @skill()
    def query(self, question: str) -> str:
        """
        Answer a question about the video stream using temporal memory.

        This skill allows the agent to query what entities are present,
        what has happened recently, and ask questions about the video stream.

        Args:
            question: The question to ask about the video stream (e.g.,
                     "What entities are currently visible?",
                     "What has happened in the last few seconds?")

        Returns:
            A text answer based on the temporal memory state and current video context
        """
        return self._query_impl(question)

    @rpc
    def get_state(self) -> dict[str, Any]:
        """
        Get current temporal memory state.

        Returns:
            Dictionary containing entity roster, rolling summary, and statistics
        """
        with self.state_lock:
            return {
                "entity_count": len(self.state.get("entity_roster", [])),
                "entities": list(self.state.get("entity_roster", [])),
                "rolling_summary": str(self.state.get("rolling_summary", "")),
                "frame_count": self.frame_count,
                "buffer_size": len(self.frame_buffer),
                "recent_windows": len(self.recent_windows),
                "currently_present": list(self.state.get("last_present", [])),
            }

    @skill()
    def get_entity_roster(self) -> list[dict[str, Any]]:
        """
        Get the current list of entities tracked in the video stream.

        This skill allows the agent to see what entities (people, objects, etc.)
        have been identified and are being tracked over time.

        Returns:
            List of entities with their IDs, types, and descriptors
        """
        with self.state_lock:
            return list(self.state.get("entity_roster", []))

    @skill()
    def get_rolling_summary(self) -> str:
        """
        Get the current rolling summary of recent events in the video stream.

        This skill allows the agent to get a concise summary of what has
        happened recently, including entity interactions and key events.

        Returns:
            Rolling summary text describing recent events
        """
        with self.state_lock:
            return str(self.state.get("rolling_summary", ""))

    @rpc
    def save_state(self) -> bool:
        """
        Save current state to disk.

        Returns:
            True if successful, False otherwise
        """
        if not self.config.output_dir:
            return False

        try:
            with self.state_lock:
                state_copy = self.state.copy()
            with open(self.state_file, "w") as f:
                json.dump(state_copy, f, indent=2, ensure_ascii=False)
            logger.info(f"Saved state to {self.state_file}")
            return True
        except Exception as e:
            logger.error(f"Failed to save state: {e}", exc_info=True)
            return False

    def _append_evidence(self, evidence: dict[str, Any]) -> None:
        """Append evidence to JSONL file."""
        try:
            with open(self.evidence_file, "a") as f:
                f.write(json.dumps(evidence, ensure_ascii=False) + "\n")
        except Exception as e:
            logger.error(f"Failed to append evidence: {e}")

    def save_entities(self) -> bool:
        """
        Save entity roster to a separate JSON file.

        Returns:
            True if successful, False otherwise
        """
        if not self.config.output_dir:
            return False

        try:
            with self.state_lock:
                entity_roster = list(self.state.get("entity_roster", []))
            with open(self.entities_file, "w") as f:
                json.dump(entity_roster, f, indent=2, ensure_ascii=False)
            logger.info(f"Saved {len(entity_roster)} entities to {self.entities_file}")
            return True
        except Exception as e:
            logger.error(f"Failed to save entities: {e}", exc_info=True)
            return False

    def save_frames_index(self) -> bool:
        """
        Save frames index to JSONL file (similar to VideoRAG).

        Returns:
            True if successful, False otherwise
        """
        if not self.config.output_dir:
            return False

        try:
            frames_index = []
            for frame in self.frame_buffer:
                # Note: Frame.image is an Image object, not a path
                # We'll store metadata about the frame instead
                frames_index.append(
                    {
                        "frame_index": frame.frame_index,
                        "timestamp_s": frame.timestamp_s,
                        "timestamp": frame.timestamp,
                        # Note: In VideoRAG, frames have paths, but here we have Image objects
                        # We can't save the path, but we can save the metadata
                    }
                )

            if frames_index:
                with open(self.frames_index_file, "w", encoding="utf-8") as f:
                    for rec in frames_index:
                        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                logger.info(f"Saved {len(frames_index)} frames to {self.frames_index_file}")
            return True
        except Exception as e:
            logger.error(f"Failed to save frames index: {e}", exc_info=True)
            return False


def _load_openai_api_key() -> str | None:
    """Load OPENAI_API_KEY from .env or default.env files.

    Returns:
        API key string if found, None otherwise
    """
    from pathlib import Path

    from dotenv import load_dotenv

    script_dir = Path(__file__).resolve().parent
    root_dir = script_dir.parent.parent
    env_file = root_dir / ".env"
    default_env_file = root_dir / "default.env"

    # Load .env first if it exists, then check if OPENAI_API_KEY is set
    # If not set (or empty), load default.env to get the API key
    if env_file.exists():
        load_dotenv(env_file, override=True)
        api_key_after_env = os.getenv("OPENAI_API_KEY")
        if not api_key_after_env and default_env_file.exists():
            load_dotenv(default_env_file, override=True)
    elif default_env_file.exists():
        load_dotenv(default_env_file, override=True)
    else:
        load_dotenv(override=True)

    return os.getenv("OPENAI_API_KEY")


def deploy(  # type: ignore[no-untyped-def]
    dimos: DimosCluster,
    camera: spec.Camera,
    vlm: VlModel | None = None,
    config: TemporalMemoryConfig | None = None,
    output_dir: str | Path | None = None,
) -> TemporalMemory:
    if vlm is None:
        from dimos.models.vl.openai import OpenAIVlModel

        # Load API key from environment (ensures it's available when pickled to Dask workers)
        api_key = _load_openai_api_key()

        if not api_key:
            logger.error(
                "OPENAI_API_KEY not found in environment. "
                "Make sure default.env is loaded or set OPENAI_API_KEY environment variable."
            )
        else:
            logger.info(f"OpenAI API key loaded (length: {len(api_key)})")
        vlm = OpenAIVlModel(api_key=api_key)

    temporal_memory = dimos.deploy(TemporalMemory, vlm=vlm, config=config, output_dir=output_dir)  # type: ignore[attr-defined]

    # Set up shared memory transport for the connection if camera doesn't have one
    if camera.color_image.transport is None:
        from dimos.core.transport import JpegShmTransport

        transport = JpegShmTransport("/temporal_memory/color_image")
        camera.color_image.transport = transport

    temporal_memory.color_image.connect(camera.color_image)
    temporal_memory.start()
    return temporal_memory  # type: ignore[no-any-return]


# Blueprint for deployment
temporal_memory = TemporalMemory.blueprint

__all__ = [
    "Frame",
    "TemporalMemory",
    "TemporalMemoryConfig",
    "_load_openai_api_key",
    "deploy",
    "temporal_memory",
]
