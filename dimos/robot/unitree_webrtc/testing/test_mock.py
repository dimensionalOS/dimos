#!/usr/bin/env python3
import time
import pytest
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.testing.mock import Mock

@pytest.mark.needsdata
def test_mock_load_cast():
    mock = Mock("test")

    # Load a frame with type casting
    frame = mock.load("a")

    # Verify it's a LidarMessage object
    assert frame.__class__.__name__ == "LidarMessage"
    assert hasattr(frame, "timestamp")
    assert hasattr(frame, "origin")
    assert hasattr(frame, "resolution")
    assert hasattr(frame, "pointcloud")

    # Verify pointcloud has points
    assert frame.pointcloud.has_points()
    assert len(frame.pointcloud.points) > 0


@pytest.mark.needsdata
def test_mock_iterate():
    """Test the iterate method of the Mock class."""
    mock = Mock("office")

    # Test iterate method
    frames = list(mock.iterate())
    assert len(frames) > 0
    for frame in frames:
        assert isinstance(frame, LidarMessage)
        assert frame.pointcloud.has_points()


@pytest.mark.needsdata
def test_mock_stream():
    frames = []
    sub1 = Mock("office").stream(rate_hz=30.0).subscribe(on_next=frames.append)
    time.sleep(0.1)
    sub1.dispose()

    assert len(frames) >= 2
    assert isinstance(frames[0], LidarMessage)
