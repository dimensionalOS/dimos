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

import pytest
import time
import open3d as o3d

from dimos.types.vector import Vector
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage

from dimos.robot.unitree_webrtc.testing.mock import Mock
from dimos.robot.unitree_webrtc.testing.helpers import show3d, multivis, benchmark


@pytest.mark.needsdata
def test_load():
    mock = Mock("test")
    frame = mock.load("a")

    # Validate the result
    assert isinstance(frame, LidarMessage)
    assert isinstance(frame.timestamp, float)
    assert isinstance(frame.origin, Vector)
    assert isinstance(frame.resolution, float)
    assert isinstance(frame.pointcloud, o3d.geometry.PointCloud)
    assert len(frame.pointcloud.points) > 0


@pytest.mark.needsdata
def test_add():
    mock = Mock("test")
    [frame_a, frame_b] = mock.load("a", "b")

    # Get original point counts
    points_a = len(frame_a.pointcloud.points)
    points_b = len(frame_b.pointcloud.points)

    # Add the frames
    combined = frame_a + frame_b

    assert isinstance(combined, LidarMessage)
    assert len(combined.pointcloud.points) == points_a + points_b

    # Check metadata is from the most recent message
    if frame_a.timestamp >= frame_b.timestamp:
        assert combined.timestamp == frame_a.timestamp
        assert combined.origin == frame_a.origin
        assert combined.resolution == frame_a.resolution
    else:
        assert combined.timestamp == frame_b.timestamp
        assert combined.origin == frame_b.origin
        assert combined.resolution == frame_b.resolution


@pytest.mark.vis
@pytest.mark.needsdata
def test_icp_vis():
    mock = Mock("test")
    [framea, frameb] = mock.load("a", "b")

    # framea.pointcloud = framea.pointcloud.voxel_down_sample(voxel_size=0.1)
    # frameb.pointcloud = frameb.pointcloud.voxel_down_sample(voxel_size=0.1)

    framea.color(0)
    frameb.color(1)

    # Normally this is a mutating operation (for efficiency)
    # but here we need an original frame A for the visualizer
    framea_icp = framea.copy().icptransform(frameb)

    multivis(
        show3d(framea, title="frame a"),
        show3d(frameb, title="frame b"),
        show3d((framea + frameb), title="union"),
        show3d((framea_icp + frameb), title="ICP"),
    )


@pytest.mark.benchmark
@pytest.mark.needsdata
def test_benchmark_icp():
    frames = Mock("dynamic_house").iterate()

    prev_frame = None

    def icptest():
        nonlocal prev_frame
        start = time.time()

        current_frame = frames.__next__()
        if not prev_frame:
            prev_frame = frames.__next__()
        end = time.time()

        current_frame.icptransform(prev_frame)
        # for subtracting the time of the function exec
        return (end - start) * -1

    ms = benchmark(100, icptest)
    assert ms < 20, "ICP took too long"

    print(f"ICP takes {ms:.2f} ms")


@pytest.mark.vis
@pytest.mark.needsdata
def test_downsample():
    mock = Mock("test")
    [framea, frameb] = mock.load("a", "b")

    # framea.pointcloud = framea.pointcloud.voxel_down_sample(voxel_size=0.1)
    # frameb.pointcloud = frameb.pointcloud.voxel_down_sample(voxel_size=0.1)

    # framea.color(0)
    # frameb.color(1)

    # Normally this is a mutating operation (for efficiency)
    # but here we need an original frame A for the visualizer
    # framea_icp = framea.copy().icptransform(frameb)
    pcd = framea.copy().pointcloud
    newpcd, _, _ = pcd.voxel_down_sample_and_trace(
        voxel_size=0.25,
        min_bound=pcd.get_min_bound(),
        max_bound=pcd.get_max_bound(),
        approximate_class=False,
    )

    multivis(
        show3d(framea, title="frame a"),
        show3d(newpcd, title="frame a downsample"),
    )
