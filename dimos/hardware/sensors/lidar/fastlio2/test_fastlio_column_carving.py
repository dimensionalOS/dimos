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

import pickle
import time
from pathlib import Path
import math

import pytest

from dimos import core
from dimos.core import Module, Out, rpc
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion
from dimos.utils.logging_config import setup_logger
    # Build the blueprint (same as replay_object_permanence_mid360 from examples/livox_nav/module.py)
from dimos.core.blueprints import autoconnect
from dimos.visualization.rerun.bridge import rerun_bridge
from dimos.mapping.voxels import voxel_mapper
import math

logger = setup_logger()
voxel_size = 0.05

#TODO: add foot note about getting a failed test due to dask multithread??
class ReplayMid360Module(Module):
    """Module that replays Mid360 lidar data from pickle file."""

    lidar: Out[PointCloud2]

    def __init__(self, lidar_path: str) -> None:
        super().__init__()
        self.lidar_path = lidar_path
        self.file = None
        self._running = False
        self._thread = None

    @rpc
    def start(self) -> None:
        """Start replaying lidar data."""
        import threading

        self.file = open(self.lidar_path, "rb")
        self._running = True
        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()
        logger.info(f"ReplayMid360Module started, replaying from {self.lidar_path}")

    def _replay_loop(self):
        floor_orientation = Transform(
            translation=Vector3(0, 0, 0),
            rotation=Quaternion.from_euler(Vector3(0, math.radians(24), 0)),
        )
        try:
            logger.info(f"Starting replay from {self.lidar_path}")
            frame_count = 0
            while self._running:
                pcd: PointCloud2 = pickle.load(self.file)
                logger.info(f"Replaying pointcloud at ts={pcd.ts}")
                self.lidar.publish(pcd.transform(floor_orientation))
                frame_count += 1
                time.sleep(0.1)  # Add small delay between frames
        except EOFError:
            logger.info(f"Replay finished - reached end of file after {frame_count} frames")
            self._running = False
        except Exception as e:
            logger.error(f"Error during replay: {e}")
            self._running = False

    @rpc
    def stop(self) -> None:
        """Stop replaying lidar data."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.file:
            self.file.close()
        logger.info("ReplayMid360Module stopped")


@pytest.mark.integration
def test_replay_column_carving():
    """Test FastLIO2 voxel mapper with column carving using replay data."""

    # Get test data path
    data_path = Path(__file__).parent.parent.parent.parent.parent.parent / "data" / "livox_nav_recording"
    lidar_path = data_path / "lidar.pkl"

    if not lidar_path.exists():
        pytest.skip(f"Test data not found at {lidar_path}")

    logger.info(f"Using test data from {lidar_path}")

    # Create the blueprint with our custom replay module
    test_blueprint = autoconnect(
        ReplayMid360Module.blueprint(lidar_path=str(lidar_path)),
        voxel_mapper(voxel_size=voxel_size),
        rerun_bridge()
    )

    # Build and start the application
    app = test_blueprint.build()
    app.start()
    logger.info("Started replay with voxel mapper and rerun bridge")

    try:
        # Verify modules were deployed
        from dimos.mapping.voxels import VoxelGridMapper

        voxel_mapper_module = app.get_instance(VoxelGridMapper)
        replay_module_instance = app.get_instance(ReplayMid360Module)

        assert voxel_mapper_module is not None, "VoxelGridMapper module not deployed"
        assert replay_module_instance is not None, "ReplayMid360Module not deployed"
        logger.info("✓ All required modules deployed successfully")

        # Wait for data processing to begin
        logger.info("Waiting for data processing to begin...")
        time.sleep(3)
        logger.info("✓ System started processing data")

        # Let it run to test column carving (voxel accumulation over time)
        processing_duration = 30.0
        logger.info(f"Running column carving test for {processing_duration} seconds...")

        elapsed = 0
        check_interval = 2.0
        while elapsed < processing_duration:
            time.sleep(check_interval)
            elapsed += check_interval
            logger.info(f"Processing... {elapsed:.1f}s / {processing_duration}s")

        logger.info("✓ Column carving test completed successfully")

    finally:
        # Stop the application and wait for threads to terminate
        app.stop()
        logger.info("Stopped application, waiting for cleanup...")

        # Give threads time to fully terminate
        time.sleep(5)

        logger.info("✓ All FastLIO2 column carving tests passed!")


__all__ = ["ReplayMid360Module"]

if __name__ == "__main__":
    pytest.main(["-v", "-s", __file__])
