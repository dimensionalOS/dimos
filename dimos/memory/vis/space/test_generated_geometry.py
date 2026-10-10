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

"""Generated geometry reaches both scene renderers without message convenience methods."""

import math
import xml.etree.ElementTree as ET

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point as GeoPoint,
    Pose as GeoPose,
    PoseStamped,
    Quaternion,
)
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
import rerun as rr

from dimos.memory.vis.space.elements import Arrow, Camera, Point, Polyline, Pose
from dimos.memory.vis.space.rerun import render
from dimos.memory.vis.space.space import Space
from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz


def test_rerun_logs_generated_pose_point_and_path(mocker):
    stamped = PoseStamped(
        pose=GeoPose(
            position=GeoPoint(x=1, y=2, z=3), orientation=quaternion_from_euler(0, 0, math.pi / 2)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    path = Path(
        poses=[
            stamped,
            PoseStamped(
                pose=GeoPose(
                    position=GeoPoint(x=4, y=5, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            ),
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    space = Space()
    space.add(Point(stamped)).add(Pose(stamped)).add(Arrow(stamped))
    space.add(Polyline(path)).add(Camera(stamped))
    mocker.patch("dimos.visualization.rerun.init.rerun_init")
    log = mocker.spy(rr, "log")

    with rr.RecordingStream("generated-geometry-test") as recording:
        memory = recording.memory_recording()
        try:
            render(space, spawn=False)
            assert memory.num_msgs() > 0
        finally:
            recording.disconnect()

    logged = {call.args[0]: call.args[1] for call in log.call_args_list}
    assert logged["scene/points"].positions.as_arrow_array().to_pylist() == [[1, 2, 3]]
    assert logged["scene/poses/headings"].vectors.as_arrow_array().to_pylist()[0] == pytest.approx(
        [0, 0.3, 0], abs=1e-7
    )
    assert logged["scene/polylines/0"].strips.as_arrow_array().to_pylist() == [
        [[1, 2, 0], [4, 5, 0]]
    ]
    assert logged["scene/cameras/0"].translation.as_arrow_array().to_pylist() == [[1, 2, 3]]


def test_both_renderers_apply_rotated_occupancy_origin(mocker):
    grid = OccupancyGrid(
        info=MapMetaData(
            width=2,
            height=1,
            resolution=1.0,
            origin=GeoPose(
                position=GeoPoint(x=2, y=3, z=0.0),
                orientation=quaternion_from_euler(0, 0, math.pi / 2),
            ),
            map_load_time=Time(sec=0, nanosec=0),
        ),
        data=np.array([0, 100], dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    space = Space().base_map(cdr_decode(cdr_encode(grid), OccupancyGrid))
    svg = ET.fromstring(space.to_svg())
    image = svg.find(".//{http://www.w3.org/2000/svg}image")
    assert image is not None
    transform = image.attrib["transform"].removeprefix("matrix(").removesuffix(")")
    assert [float(value) for value in transform.split()] == pytest.approx([0, -1, 1, 0, 1, -3])
    mocker.patch("dimos.visualization.rerun.init.rerun_init")
    log = mocker.spy(rr, "log")
    with rr.RecordingStream("generated-grid-test") as recording:
        memory = recording.memory_recording()
        try:
            render(space, spawn=False)
            assert memory.num_msgs() > 0
        finally:
            recording.disconnect()
    meshes = [call.args[1] for call in log.call_args_list if call.args[0] == "scene/map/0"]
    assert len(meshes) == 1
    np.testing.assert_allclose(
        meshes[0].vertex_positions.as_arrow_array().to_pylist(),
        [[2, 3, 0], [2, 5, 0], [1, 5, 0], [1, 3, 0]],
        atol=1e-7,
    )


def test_generated_cloud_renders_in_svg_and_rerun(mocker):
    cloud = pointcloud_from_xyz(
        np.array([[0.0, 0.0, 0.0], [0.2, 0.3, 1.0], [1.0, 1.0, 0.0]]),
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
    )
    space = Space().add(cdr_decode(cdr_encode(cloud), PointCloud2))
    svg = ET.fromstring(space.to_svg())
    assert svg.find(".//{http://www.w3.org/2000/svg}image") is not None
    mocker.patch("dimos.visualization.rerun.init.rerun_init")
    log = mocker.spy(rr, "log")
    with rr.RecordingStream("generated-cloud-test") as recording:
        memory = recording.memory_recording()
        try:
            render(space, spawn=False)
            assert memory.num_msgs() > 0
        finally:
            recording.disconnect()
    logged = {call.args[0]: call.args[1] for call in log.call_args_list}
    np.testing.assert_allclose(
        logged["scene/pointcloud/0"].positions.as_arrow_array().to_pylist(),
        [[0, 0, 0], [0.2, 0.3, 1], [1, 1, 0]],
    )


def test_rerun_camera_uses_generated_calibration_and_image_helpers(mocker):
    calibration = camera_info_from_intrinsics(
        20, 30, 2, 2, 4, 4, header=Header(frame_id="optical", stamp=Time(sec=0, nanosec=0))
    )
    image = image_from_array(
        np.zeros((4, 4, 3), dtype=np.uint8), encoding="rgb8", header=calibration.header
    )
    pose = PoseStamped(
        pose=GeoPose(
            position=GeoPoint(x=2, y=0.0, z=0.0), orientation=quaternion_from_euler(0, 0, 0)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    space = Space().add(Camera(pose=pose, image=image, camera_info=calibration))
    original = (cdr_encode(calibration), cdr_encode(image))
    mocker.patch("dimos.visualization.rerun.init.rerun_init")
    log = mocker.spy(rr, "log")
    with rr.RecordingStream("generated-camera-test") as recording:
        recording.memory_recording()
        try:
            render(space, spawn=False)
        finally:
            recording.disconnect()
    values = [call.args[1] for call in log.call_args_list if call.args[0] == "scene/cameras/0"]
    assert any(isinstance(value, rr.Pinhole) for value in values)
    pinhole = next(value for value in values if isinstance(value, rr.Pinhole))
    assert pinhole.parent_frame.as_arrow_array().to_pylist() == ["tf#/optical"]
    assert any(
        call.args[0] == "scene/cameras/0/image" and isinstance(call.args[1], rr.EncodedImage)
        for call in log.call_args_list
    )
    assert (cdr_encode(calibration), cdr_encode(image)) == original
