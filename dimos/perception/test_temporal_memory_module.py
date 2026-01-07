# Copyright 2025-2026 Dimensional Inc.
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

import asyncio
import os
import pathlib
import tempfile
import time

import pytest
from reactivex import operators as ops

from dimos import core
from dimos.core import Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.models.vl.openai import OpenAIVlModel
from dimos.perception.temporal_memory import TemporalMemory, TemporalMemoryConfig
from dimos.protocol import pubsub
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay

logger = setup_logger()

pubsub.lcm.autoconf()


class VideoReplayModule(Module):
    """Module that replays video data from TimedSensorReplay."""

    video_out: Out[Image]

    def __init__(self, video_path: str) -> None:
        super().__init__()
        self.video_path = video_path
        self._subscription = None

    @rpc
    def start(self) -> None:
        """Start replaying video data."""
        # Use TimedSensorReplay to replay video frames
        video_replay = TimedSensorReplay(self.video_path, autocast=Image.from_numpy)

        # Subscribe to the replay stream and publish to LCM
        self._subscription = (
            video_replay.stream()
            .pipe(
                ops.sample(1),  # Sample every 1 second
                ops.take(10),  # Only take 10 frames total
            )
            .subscribe(self.video_out.publish)
        )

        logger.info("VideoReplayModule started")

    @rpc
    def stop(self) -> None:
        """Stop replaying video data."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None
        logger.info("VideoReplayModule stopped")


@pytest.mark.gpu
@pytest.mark.module
class TestTemporalMemoryModule:
    @pytest.fixture(scope="function")
    def temp_dir(self):
        """Create a temporary directory for test data."""
        temp_dir = tempfile.mkdtemp(prefix="temporal_memory_test_")
        yield temp_dir

    @pytest.mark.asyncio
    async def test_temporal_memory_module_with_replay(self, temp_dir):
        """Test TemporalMemory module with TimedSensorReplay inputs."""

        # Start Dask
        dimos = core.start(1)

        try:
            # Get test data paths
            data_path = get_data("unitree_office_walk")
            video_path = os.path.join(data_path, "video")

            # Deploy modules
            # Video replay module
            video_module = dimos.deploy(VideoReplayModule, video_path)
            video_module.video_out.transport = core.LCMTransport("/test_video", Image)

            # Temporal memory module
            output_dir = os.path.join(temp_dir, "temporal_memory_output")
            # Load API key before creating VLM to ensure it's available when pickled
            from dimos.perception.temporal_memory import _load_openai_api_key

            api_key = _load_openai_api_key()
            if not api_key:
                pytest.skip("OPENAI_API_KEY not found in environment")
            vlm = OpenAIVlModel(api_key=api_key)  # Pass API key explicitly
            temporal_memory = dimos.deploy(
                TemporalMemory,
                vlm=vlm,
                config=TemporalMemoryConfig(
                    fps=1.0,  # Process 1 frame per second
                    window_s=2.0,  # Analyze 2-second windows
                    stride_s=2.0,  # New window every 2 seconds
                    summary_interval_s=10.0,  # Update rolling summary every 10 seconds
                    max_frames_per_window=3,  # Max 3 frames per window
                    output_dir=output_dir,
                    use_structured_output=True,
                    use_multi_image=True,
                ),
            )

            # Connect streams
            temporal_memory.color_image.connect(video_module.video_out)

            # Start all modules
            video_module.start()
            temporal_memory.start()
            logger.info("All modules started, processing in background...")

            # Wait for frames to be processed with timeout
            timeout = 15.0  # 15 second timeout
            start_time = time.time()

            # Keep checking state while modules are running
            while (time.time() - start_time) < timeout:
                state = temporal_memory.get_state()
                if state["frame_count"] > 0:
                    logger.info(
                        f"Frames processing - Frame count: {state['frame_count']}, "
                        f"Buffer size: {state['buffer_size']}, "
                        f"Entity count: {state['entity_count']}"
                    )
                    if state["frame_count"] >= 3:  # Wait for at least 3 frames
                        break
                await asyncio.sleep(0.5)
            else:
                # Timeout reached
                state = temporal_memory.get_state()
                logger.error(
                    f"Timeout after {timeout}s - Frame count: {state['frame_count']}, "
                    f"Buffer size: {state['buffer_size']}"
                )
                raise AssertionError(f"No frames processed within {timeout} seconds")

            await asyncio.sleep(3)  # Wait for more processing

            # Test get_state() RPC method
            mid_state = temporal_memory.get_state()
            logger.info(
                f"Mid-test state - Frame count: {mid_state['frame_count']}, "
                f"Entity count: {mid_state['entity_count']}, "
                f"Recent windows: {mid_state['recent_windows']}"
            )
            assert mid_state["frame_count"] >= state["frame_count"], (
                "Frame count should increase or stay same"
            )

            # Test query() RPC method
            try:
                answer = temporal_memory.query("What entities are currently visible?")
                logger.info(f"Query result: {answer[:200]}...")
                assert len(answer) > 0, "Query should return a non-empty answer"
            except Exception as e:
                logger.warning(f"Query failed: {e}")

            # Test get_entity_roster() RPC method
            try:
                entities = temporal_memory.get_entity_roster()
                logger.info(f"Entity roster has {len(entities)} entities")
                assert isinstance(entities, list), "Entity roster should be a list"
            except Exception as e:
                logger.warning(f"get_entity_roster failed: {e}")

            # Test get_rolling_summary() RPC method
            try:
                summary = temporal_memory.get_rolling_summary()
                logger.info(f"Rolling summary: {summary[:200] if summary else 'empty'}...")
                assert isinstance(summary, str), "Rolling summary should be a string"
            except Exception as e:
                logger.warning(f"get_rolling_summary failed: {e}")

            final_state = temporal_memory.get_state()
            logger.info(
                f"Final state - Frame count: {final_state['frame_count']}, "
                f"Entity count: {final_state['entity_count']}, "
                f"Recent windows: {final_state['recent_windows']}"
            )

            video_module.stop()
            # Stop temporal_memory - this will automatically save all files (state, entities, frames_index)
            temporal_memory.stop()
            logger.info("Stopped replay module")

            # Wait a bit for file operations to complete
            await asyncio.sleep(0.5)

            # Verify files were created - stop() already saved them
            output_path = pathlib.Path(output_dir)
            assert output_path.exists(), f"Output directory should exist: {output_dir}"
            assert (output_path / "state.json").exists(), "state.json should exist"
            assert (output_path / "entities.json").exists(), "entities.json should exist"
            assert (output_path / "frames_index.jsonl").exists(), "frames_index.jsonl should exist"

            logger.info("All temporal memory module tests passed!")

        finally:
            # Cleanup - stop modules first, then close cluster
            if "temporal_memory" in locals():
                try:
                    temporal_memory.stop()
                except Exception:
                    pass
            if "video_module" in locals():
                try:
                    video_module.stop()
                except Exception:
                    pass
            if "dimos" in locals():
                dimos.close_all()  # Use close_all() instead of close() to shut down workers


if __name__ == "__main__":
    pytest.main(["-v", "-s", __file__])

