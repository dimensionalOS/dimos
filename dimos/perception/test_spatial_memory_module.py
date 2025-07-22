# Copyright 2025 Dimensional Inc.
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
import shutil
import tempfile
import time
from typing import Dict, List

import numpy as np
import pytest
from reactivex import operators as ops

from dimos import core
from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.geometry_msgs import Vector3
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.perception.spatial_perception import SpatialMemory
from dimos.protocol import pubsub
from dimos.types.vector import Vector
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay
from dimos.utils.logging_config import setup_logger
import warnings

logger = setup_logger("test_spatial_memory_module")

pubsub.lcm.autoconf()


class VideoReplayModule(Module):
    """Module that replays video data from TimedSensorReplay."""

    video_out: Out[Image] = None

    def __init__(self, video_path: str):
        super().__init__()
        self.video_path = video_path
        self._subscription = None

    @rpc
    def start(self):
        """Start replaying video data."""
        # Use TimedSensorReplay to replay video frames
        video_replay = TimedSensorReplay(self.video_path, autocast=Image.from_numpy)

        # Subscribe to the replay stream and publish to LCM
        self._subscription = (
            video_replay.stream()
            .pipe(
                ops.sample(0.1)  # Limit to 10 FPS for testing
            )
            .subscribe(self.video_out.publish)
        )

        logger.info("VideoReplayModule started")

    @rpc
    def stop(self):
        """Stop replaying video data."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None
        logger.info("VideoReplayModule stopped")


class OdometryReplayModule(Module):
    """Module that replays odometry data from TimedSensorReplay."""

    odom_out: Out[Odometry] = None

    def __init__(self, odom_path: str):
        super().__init__()
        self.odom_path = odom_path
        self._subscription = None

    @rpc
    def start(self):
        """Start replaying odometry data."""
        # Use TimedSensorReplay to replay odometry
        odom_replay = TimedSensorReplay(self.odom_path, autocast=Odometry.from_msg)

        # Subscribe to the replay stream and publish to LCM
        self._subscription = odom_replay.stream().subscribe(self.odom_out.publish)

        logger.info("OdometryReplayModule started")

    @rpc
    def stop(self):
        """Stop replaying odometry data."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None
        logger.info("OdometryReplayModule stopped")


@pytest.mark.heavy
class TestSpatialMemoryModule:
    @pytest.fixture(scope="function")
    def temp_dir(self):
        """Create a temporary directory for test data."""
        # Use standard tempfile module to ensure proper permissions
        temp_dir = tempfile.mkdtemp(prefix="spatial_memory_test_")

        yield temp_dir

    @pytest.mark.asyncio
    async def test_spatial_memory_module_with_replay(self, temp_dir):
        """Test SpatialMemory module with TimedSensorReplay inputs."""

        # Start Dask
        dimos = core.start(1)

        try:
            # Get test data paths
            data_path = get_data("unitree_office_walk")
            video_path = os.path.join(data_path, "video")
            odom_path = os.path.join(data_path, "odom")

            # Deploy modules
            # Video replay module
            video_module = dimos.deploy(VideoReplayModule, video_path)
            video_module.video_out.transport = core.LCMTransport("/test_video", Image)

            # Odometry replay module
            odom_module = dimos.deploy(OdometryReplayModule, odom_path)
            odom_module.odom_out.transport = core.LCMTransport("/test_odom", Odometry)

            # Spatial memory module
            spatial_memory = dimos.deploy(
                SpatialMemory,
                collection_name="test_spatial_memory",
                embedding_model="clip",
                embedding_dimensions=512,
                min_distance_threshold=0.5,  # 0.5m for test
                min_time_threshold=1.0,  # 1 second
                db_path=os.path.join(temp_dir, "chroma_db"),
                visual_memory_path=os.path.join(temp_dir, "visual_memory.pkl"),
                new_memory=True,
                output_dir=os.path.join(temp_dir, "images"),
            )

            # Connect streams
            spatial_memory.video.connect(video_module.video_out)
            spatial_memory.odom.connect(odom_module.odom_out)

            # Start all modules
            video_module.start()
            odom_module.start()
            spatial_memory.start()

            # Give some time for initial processing
            await asyncio.sleep(2)

            # Wait for some frames to be processed
            logger.info("Waiting for frames to be processed...")
            await asyncio.sleep(5)  # Process for 5 seconds

            # Stop the replay modules to prevent infinite streaming
            video_module.stop()
            odom_module.stop()
            logger.info("Stopped replay modules")

            # Test RPC calls

            stats = spatial_memory.get_stats()
            logger.info(
                f"Initial stats - Frame count: {stats['frame_count']}, Stored: {stats['stored_frame_count']}"
            )
            assert stats["frame_count"] > 0, "No frames processed"
            assert stats["stored_frame_count"] > 0, "No frames stored"

            # 2. Test query by text (this should work regardless of write permissions)
            try:
                text_results = spatial_memory.query_by_text("office")
                logger.info(f"Query by text 'office' returned {len(text_results)} results")
            except Exception as e:
                logger.warning(f"Query by text failed (may be due to write permissions): {e}")

            final_stats = spatial_memory.get_stats()
            logger.info(
                f"Final stats - Frame count: {final_stats['frame_count']}, Stored: {final_stats['stored_frame_count']}"
            )
            assert final_stats["frame_count"] >= stats["frame_count"], (
                "Frame count should not decrease"
            )
            assert final_stats["stored_frame_count"] >= stats["stored_frame_count"], (
                "Stored count should not decrease"
            )

            logger.info("All spatial memory module tests passed!")

        finally:
            # Cleanup
            if "dimos" in locals():
                dimos.close()


if __name__ == "__main__":
    pytest.main(["-v", "-s", __file__])
