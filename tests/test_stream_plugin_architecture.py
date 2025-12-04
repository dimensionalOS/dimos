#!/usr/bin/env python3
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

# Copyright 2025 Dimensional Inc.

"""Test the stream plugin architecture with UnitreeGo2."""

import argparse
import logging
from dimos.robot.unitree import UnitreeGo2
from dimos.robot.unitree.stream_configs_example import (
    get_cpu_minimal_config,
    get_gpu_full_config,
    get_hybrid_config,
)
from dimos.stream import StreamConfig


def test_default_configuration():
    """Test robot with default stream configuration."""
    print("\n=== Testing Default Configuration ===")

    robot = UnitreeGo2(
        use_ros=True,
        mock_connection=True,
        disable_video_stream=True,  # For testing without actual video
    )

    # Check available streams
    streams = robot.get_perception_streams()
    print(f"Available streams: {list(streams.keys())}")

    # Check stream registry
    from dimos.stream import stream_registry

    print(f"Initialized streams: {stream_registry.list_initialized_streams()}")

    robot.cleanup()


def test_cpu_configuration():
    """Test robot with CPU-only configuration."""
    print("\n=== Testing CPU Configuration ===")

    camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
    configs = get_cpu_minimal_config(camera_intrinsics)

    robot = UnitreeGo2(
        use_ros=True,
        mock_connection=True,
        disable_video_stream=True,
        stream_configs=configs,
        force_cpu=True,
    )

    streams = robot.get_perception_streams()
    print(f"Available streams: {list(streams.keys())}")

    # Verify CPU usage
    from dimos.stream import stream_registry

    for name in stream_registry.list_initialized_streams():
        stream = stream_registry.get_stream(name)
        print(f"Stream '{name}' device: {stream.config.device}")

    robot.cleanup()


def test_custom_configuration():
    """Test robot with custom stream configuration."""
    print("\n=== Testing Custom Configuration ===")

    # Create custom configuration with only object tracking
    configs = [
        StreamConfig(
            name="object_tracking",
            enabled=True,
            device="cpu",
            parameters={
                "camera_intrinsics": [819.553492, 820.646595, 625.284099, 336.808987],
                "camera_pitch": 0.0,
                "camera_height": 0.44,
                "use_depth_model": False,
                "reid_threshold": 10,
            },
            priority=10,
        ),
    ]

    robot = UnitreeGo2(
        use_ros=True,
        mock_connection=True,
        disable_video_stream=True,
        stream_configs=configs,
    )

    streams = robot.get_perception_streams()
    print(f"Available streams: {list(streams.keys())}")

    # Verify only object tracking is available
    assert "object_tracking" in streams
    assert "person_tracking" not in streams
    print("✓ Custom configuration successful")

    robot.cleanup()


def test_stream_info():
    """Test stream information retrieval."""
    print("\n=== Testing Stream Information ===")

    from dimos.stream import stream_registry, StreamConfig
    from dimos.stream.plugins import PersonTrackingPlugin

    # Register and configure a stream
    stream_registry.register_stream_class("test_person", PersonTrackingPlugin)

    config = StreamConfig(
        name="test_person",
        enabled=True,
        device="cpu",
        parameters={
            "camera_intrinsics": [819.553492, 820.646595, 625.284099, 336.808987],
        },
    )

    stream_registry.configure_stream(config)
    stream_registry.initialize_streams(force_cpu=True)

    # Get stream info
    stream = stream_registry.get_stream("test_person")
    if stream:
        info = stream.get_info()
        print(f"Stream info: {info}")
        print(f"  - Name: {info['name']}")
        print(f"  - Device: {info['device']}")
        print(f"  - Initialized: {info['initialized']}")
        print(f"  - Requires GPU: {info['requires_gpu']}")

    stream_registry.cleanup_all()


def main():
    """Run stream plugin architecture tests."""
    parser = argparse.ArgumentParser(description="Test stream plugin architecture")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level",
    )
    args = parser.parse_args()

    # Configure logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    print("Stream Plugin Architecture Test")
    print("=" * 50)

    try:
        test_default_configuration()
        test_cpu_configuration()
        test_custom_configuration()
        test_stream_info()

        print("\n✅ All tests passed!")

    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        raise


if __name__ == "__main__":
    main()
