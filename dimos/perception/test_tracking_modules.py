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

"""Tests for object and person tracking modules with LCM integration."""

import asyncio
import os
import pytest
import numpy as np
from typing import Dict
from reactivex import operators as ops

from dimos import core
from dimos.core import Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.protocol import pubsub
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay
from dimos.utils.logging_config import setup_logger
import tempfile
from dimos.core import stop

logger = setup_logger("test_tracking_modules")

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

        self._subscription = (
            video_replay.stream().pipe(ops.sample(0.1)).subscribe(self.video_out.publish)
        )

        logger.info("VideoReplayModule started")

    @rpc
    def stop(self):
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None
        logger.info("VideoReplayModule stopped")


@pytest.mark.skip(reason="Tracking tests hanging due to ONNX/CUDA cleanup issues")
@pytest.mark.heavy
class TestTrackingModules:
    @pytest.fixture(scope="function")
    def temp_dir(self):
        temp_dir = tempfile.mkdtemp(prefix="tracking_test_")
        yield temp_dir

    @pytest.mark.asyncio
    async def test_person_tracking_module_with_replay(self, temp_dir):
        """Test PersonTrackingStream module with TimedSensorReplay inputs."""

        # Start Dask
        dimos = core.start(1)

        try:
            data_path = get_data("unitree_office_walk")
            video_path = os.path.join(data_path, "video")

            video_module = dimos.deploy(VideoReplayModule, video_path)
            video_module.video_out.transport = core.LCMTransport("/test_video", Image)

            person_tracker = dimos.deploy(
                PersonTrackingStream,
                camera_intrinsics=[619.061157, 619.061157, 317.883459, 238.543800],
                camera_pitch=-0.174533,
                camera_height=0.3,
            )

            person_tracker.tracking_data.transport = core.pLCMTransport("/person_tracking")
            person_tracker.video.connect(video_module.video_out)

            video_module.start()
            person_tracker.start()
            person_tracker.enable_tracking()
            await asyncio.sleep(2)

            results = []

            from dimos.protocol.pubsub.lcmpubsub import PickleLCM

            lcm_instance = PickleLCM()
            lcm_instance.start()

            def on_message(msg, topic):
                results.append(msg)

            lcm_instance.subscribe("/person_tracking", on_message)

            await asyncio.sleep(3)

            video_module.stop()

            assert len(results) > 0

            for msg in results:
                assert "targets" in msg
                assert isinstance(msg["targets"], list)

            tracking_data = person_tracker.get_tracking_data()
            assert isinstance(tracking_data, dict)
            assert "targets" in tracking_data

            logger.info(f"Person tracking test passed with {len(results)} messages")

        finally:
            lcm_instance.stop()
            # stop(dimos)
            dimos.close()
            dimos.shutdown()

    @pytest.mark.asyncio
    async def test_object_tracking_module_with_replay(self, temp_dir):
        """Test ObjectTrackingStream module with TimedSensorReplay inputs."""

        # Start Dask
        dimos = core.start(1)

        try:
            data_path = get_data("unitree_office_walk")
            video_path = os.path.join(data_path, "video")

            video_module = dimos.deploy(VideoReplayModule, video_path)
            video_module.video_out.transport = core.LCMTransport("/test_video", Image)

            object_tracker = dimos.deploy(
                ObjectTrackingStream,
                camera_intrinsics=[619.061157, 619.061157, 317.883459, 238.543800],
                camera_pitch=-0.174533,
                camera_height=0.3,
            )

            object_tracker.tracking_data.transport = core.pLCMTransport("/object_tracking")
            object_tracker.video.connect(video_module.video_out)

            video_module.start()
            object_tracker.start()
            # object_tracker.track([100, 100, 200, 200])
            results = []

            from dimos.protocol.pubsub.lcmpubsub import PickleLCM

            lcm_instance = PickleLCM()
            lcm_instance.start()

            def on_message(msg, topic):
                results.append(msg)

            lcm_instance.subscribe("/object_tracking", on_message)

            await asyncio.sleep(5)

            video_module.stop()

            assert len(results) > 0

            for msg in results:
                assert "targets" in msg
                assert isinstance(msg["targets"], list)

            logger.info(f"Object tracking test passed with {len(results)} messages")

        finally:
            lcm_instance.stop()
            # stop(dimos)
            dimos.close()
            dimos.shutdown()

    @pytest.mark.asyncio
    async def test_tracking_rpc_methods(self, temp_dir):
        """Test RPC methods on tracking modules while they're running with video."""

        # Start Dask
        dimos = core.start(1)

        try:
            data_path = get_data("unitree_office_walk")
            video_path = os.path.join(data_path, "video")

            video_module = dimos.deploy(VideoReplayModule, video_path)
            video_module.video_out.transport = core.LCMTransport("/test_video", Image)

            person_tracker = dimos.deploy(
                PersonTrackingStream,
                camera_intrinsics=[619.061157, 619.061157, 317.883459, 238.543800],
                camera_pitch=-0.174533,
                camera_height=0.3,
            )

            object_tracker = dimos.deploy(
                ObjectTrackingStream,
                camera_intrinsics=[619.061157, 619.061157, 317.883459, 238.543800],
                camera_pitch=-0.174533,
                camera_height=0.3,
            )

            person_tracker.tracking_data.transport = core.pLCMTransport("/person_tracking")
            object_tracker.tracking_data.transport = core.pLCMTransport("/object_tracking")

            person_tracker.video.connect(video_module.video_out)
            object_tracker.video.connect(video_module.video_out)

            video_module.start()
            person_tracker.start()
            object_tracker.start()

            # person_tracker.enable_tracking()
            # object_tracker.track([100, 100, 200, 200])
            await asyncio.sleep(2)

            person_data = person_tracker.get_tracking_data()
            assert isinstance(person_data, dict)
            assert "frame" in person_data
            assert "viz_frame" in person_data
            assert "targets" in person_data
            assert isinstance(person_data["targets"], list)

            object_data = object_tracker.get_tracking_data()
            assert isinstance(object_data, dict)
            assert "frame" in object_data
            assert "viz_frame" in object_data
            assert "targets" in object_data
            assert isinstance(object_data["targets"], list)

            assert person_data["frame"] is not None
            assert object_data["frame"] is not None

            video_module.stop()

            logger.info("RPC methods test passed")

        finally:
            # stop(dimos)
            dimos.close()
            dimos.shutdown()

    @pytest.mark.asyncio
    async def test_visualization_streams(self, temp_dir):
        """Test that visualization frames are properly generated."""

        # Start Dask
        dimos = core.start(1)

        try:
            data_path = get_data("unitree_office_walk")
            video_path = os.path.join(data_path, "video")

            video_module = dimos.deploy(VideoReplayModule, video_path)
            video_module.video_out.transport = core.LCMTransport("/test_video", Image)

            person_tracker = dimos.deploy(
                PersonTrackingStream,
                camera_intrinsics=[619.061157, 619.061157, 317.883459, 238.543800],
                camera_pitch=-0.174533,
                camera_height=0.3,
            )

            object_tracker = dimos.deploy(
                ObjectTrackingStream,
                camera_intrinsics=[619.061157, 619.061157, 317.883459, 238.543800],
                camera_pitch=-0.174533,
                camera_height=0.3,
            )

            person_tracker.tracking_data.transport = core.pLCMTransport("/person_tracking")
            object_tracker.tracking_data.transport = core.pLCMTransport("/object_tracking")

            person_tracker.video.connect(video_module.video_out)
            object_tracker.video.connect(video_module.video_out)

            video_module.start()
            person_tracker.start()
            object_tracker.start()

            # person_tracker.enable_tracking()
            # object_tracker.track([100, 100, 200, 200])

            person_data = person_tracker.get_tracking_data()
            object_data = object_tracker.get_tracking_data()

            video_module.stop()

            if person_data["viz_frame"] is not None:
                viz_frame = person_data["viz_frame"]
                assert isinstance(viz_frame, np.ndarray)
                assert len(viz_frame.shape) == 3
                assert viz_frame.shape[2] == 3
                logger.info("Person tracking visualization frame verified")

            if object_data["viz_frame"] is not None:
                viz_frame = object_data["viz_frame"]
                assert isinstance(viz_frame, np.ndarray)
                assert len(viz_frame.shape) == 3
                assert viz_frame.shape[2] == 3
                logger.info("Object tracking visualization frame verified")

        finally:
            # stop(dimos)
            dimos.close()
            dimos.shutdown()


if __name__ == "__main__":
    pytest.main(["-v", "-s", __file__])
