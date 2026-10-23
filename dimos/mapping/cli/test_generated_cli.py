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


import cv2
from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest
import rerun as rr
from typer.testing import CliRunner

from dimos.cli.commands.map import map_app
from dimos.mapping.cli.map import _accumulate, _camera_info_static, _denoise
from dimos.memory.cli import dataset
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.message_utils.image import image_from_array
from dimos.message_utils.pointcloud import (
    pointcloud_from_xyz,
    pointcloud_from_xyz_rgb,
    pointcloud_stamps,
    pointcloud_xyz,
)
from dimos.message_utils.time import time_from_nanoseconds, time_from_seconds
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import Image, PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.navigation.go2.loop_closure.pgo import Keyframe, PoseGraph
from dimos.visualization.rerun.cdr import cloud_archetype


@pytest.fixture
def recorded_dataset(tmp_path):
    path = tmp_path / "canonical.db"
    pixels = np.full((720, 1280, 3), 255, dtype=np.uint8)
    tile = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 200
    )
    pixels[260:460, 540:740] = tile[:, :, None]
    with SqliteStore(path=str(path)) as store:
        lidar = store.stream("lidar", PointCloud2)
        images = store.stream("color_image", Image)
        odom = store.stream("odom", PoseStamped)
        for index in range(3):
            stamp = time_from_nanoseconds(1700000000123456789 + index * 1000000000)
            header = Header(stamp=stamp, frame_id="world")
            cloud = pointcloud_from_xyz(
                np.array([[index + 1.025, 0.025, 0.025]], dtype=np.float32), header=header
            )
            pose = (float(index + 1), 0, 0, 0, 0, 0.05, 0.998749217771909)
            lidar.append(cloud, ts=10.0 + index, pose=pose)
            odom.append(
                PoseStamped(
                    header=header,
                    pose=Pose(position=Point(*pose[:3]), orientation=Quaternion(*pose[3:])),
                ),
                ts=10.0 + index,
                pose=pose,
            )
            header.frame_id = "camera_optical"
            images.append(
                image_from_array(pixels, encoding="rgb8", header=header), ts=10.0 + index, pose=pose
            )
    return path


@pytest.fixture
def recording_calls(mocker):
    log = mocker.spy(rr, "log")
    opened = mocker.spy(dataset, "open_store")
    try:
        yield log
    finally:
        recording = rr.get_global_data_recording()
        if recording is not None:
            recording.flush()
        rr.disconnect()
        for store in opened.spy_return_list:
            store.stop()


def invoke(*arguments):
    result = CliRunner().invoke(map_app, list(arguments))
    assert result.exit_code == 0, (result.output, result.exception)
    return result


def test_global_cli_exports_cdr_and_view_reads_it(
    recorded_dataset, recording_calls, monkeypatch, tmp_path
):
    monkeypatch.chdir(tmp_path)
    invoke(
        "global",
        str(recorded_dataset),
        "--device",
        "CPU:0",
        "--export",
        "--full-pgo",
        "--no-gui",
        "--out",
        str(tmp_path / "map.rrd"),
    )
    exported = tmp_path / "canonical.pc2.cdr"
    cloud = decode(exported.read_bytes(), PointCloud2)
    assert cloud.width == 3 and cloud.height == 1
    assert cloud.header.frame_id == "world_corrected"
    assert cloud.header.stamp == time_from_nanoseconds(1700000002123456789)
    np.testing.assert_allclose(
        np.sort(pointcloud_xyz(cloud)[:, 0]), [1.025, 2.025, 3.025], atol=1e-6
    )
    invoke("view", str(exported), "--out", str(tmp_path / "view.rrd"))
    visible = [
        call.args[1]
        for call in recording_calls.call_args_list
        if call.args[0] == "world/canonical/pointcloud"
    ]
    assert len(visible) == 1
    np.testing.assert_allclose(
        np.sort(np.asarray(visible[0].positions.as_arrow_array().to_pylist())[:, 0]),
        [1.025, 2.025, 3.025],
        atol=1e-6,
    )


@pytest.mark.parametrize("mode", ["points", "boxes", "spheres"])
def test_replay_cli_logs_cdr_clouds_images_and_maps(
    recorded_dataset, recording_calls, tmp_path, mode
):
    output = tmp_path / f"replay-{mode}.rrd"
    invoke(
        "replay",
        str(recorded_dataset),
        "--no-gui",
        "--out",
        str(output),
        "--map",
        "--map-final",
        "--map-device",
        "CPU:0",
        "--map-emit-every",
        "1",
        "--point-mode",
        mode,
    )
    paths = [call.args[0] for call in recording_calls.call_args_list]
    assert paths.count("world/lidar") == 3
    assert paths.count("world/lidar_voxels") == 3
    assert paths.count("world/camera/image") == 3
    assert "world/lidar_map" in paths
    recording = rr.get_global_data_recording()
    assert recording is not None
    recording.flush()
    assert output.stat().st_size > 1000


def test_replay_marker_cli_detects_actual_apriltag(recorded_dataset, recording_calls, tmp_path):
    invoke(
        "replay-marker",
        str(recorded_dataset),
        "--no-gui",
        "--out",
        str(tmp_path / "markers.rrd"),
        "--marker-max-speed",
        "0",
        "--quality-window",
        "0",
        "--smoothing-window",
        "0",
    )
    detections = [
        call.args[1]
        for call in recording_calls.call_args_list
        if call.args[0].startswith("world/markers/det_")
    ]
    assert len(detections) == 3
    assert all(item.labels.as_arrow_array().to_pylist() == ["id=7"] for item in detections)


def test_registration_then_drift_correction_preserves_header_and_input():
    header = Header(frame_id="lidar", stamp=time_from_nanoseconds(1700000000123456789))
    cloud = pointcloud_from_xyz(np.array([[0, 0, 0]], dtype=np.float32), header=header)
    before = encode(cloud)

    def transform(x, frame, child):
        return TransformStamped(
            header=Header(frame_id=frame, stamp=time_from_seconds(10)),
            child_frame_id=child,
            transform=Transform(translation=Vector3(x, 0, 0), rotation=Quaternion(0, 0, 0, 1)),
        )

    graph = PoseGraph(
        keyframes=(
            Keyframe(
                ts=10,
                local=transform(0, "world_raw", "body"),
                optimized=transform(2, "world_corrected", "body"),
            ),
        )
    )
    obs = Observation(ts=10, _data=cloud, pose=(1, 0, 0, 0, 0, 0, 1))
    output = _accumulate(
        [obs],
        voxel=0.05,
        block_count=128,
        device="CPU:0",
        graph=graph,
        register=lambda _: transform(1, "world", "lidar"),
    )
    assert output is not None
    np.testing.assert_allclose(pointcloud_xyz(output), [[3.025, 0.025, 0.025]], atol=1e-6)
    assert output.header.frame_id == "world_corrected" and output.header.stamp == header.stamp
    assert encode(cloud) == before


def test_denoise_retains_point_capture_times_and_exact_header():
    rng = np.random.default_rng(7)
    points = np.concatenate([rng.normal(0, 0.03, (100, 3)), [[100, 100, 100]]])
    header = Header(frame_id="world", stamp=time_from_nanoseconds(1700000000123456789))
    cloud = pointcloud_from_xyz(points, header=header, stamps=np.arange(101, dtype=np.float64))
    before = encode(cloud)
    output = _denoise(cloud)
    assert output is not None and output.width < cloud.width
    assert np.linalg.norm(pointcloud_xyz(output), axis=1).max() < 1
    assert output.header == header and output.fields == cloud.fields
    assert np.max(pointcloud_stamps(output)) < 100
    assert encode(cloud) == before


@pytest.mark.parametrize("mode", ["points", "boxes", "spheres"])
def test_cloud_rendering_filters_geometry_and_own_colors_together(mode):
    cloud = pointcloud_from_xyz_rgb(
        np.array([[1, 2, -1], [3, 4, 2]], dtype=np.float32),
        np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8),
        header=Header(frame_id="world", stamp=time_from_seconds(0)),
    )
    before = encode(cloud)
    value = cloud_archetype(cloud, voxel_size=0.2, mode=mode, bottom_cutoff=0)
    assert value.colors.as_arrow_array().to_pylist() == [0x00FF00FF]
    positions = value.centers if mode == "boxes" else value.positions
    np.testing.assert_array_equal(positions.as_arrow_array().to_pylist(), [[3, 4, 2]])
    assert encode(cloud) == before


def test_packaged_camera_calibration_is_a_generated_value():
    camera = _camera_info_static()
    assert camera.__msgtype__ == "sensor_msgs/msg/CameraInfo"
    assert camera.width == 1280 and camera.height == 720
    assert camera.header.frame_id == "camera_optical"


def test_global_markers_compose_external_pose_stream_and_pgo(
    recorded_dataset, recording_calls, tmp_path
):
    invoke(
        "global",
        str(recorded_dataset),
        "--device",
        "CPU:0",
        "--pgo",
        "--no-gui",
        "--markers",
        "--marker-max-speed",
        "0",
        "--marker-quality-window",
        "0",
        "--marker-smoothing",
        "0",
        "--image-pose",
        "odom",
        "--out",
        str(tmp_path / "posed-markers.rrd"),
    )
    paths = [call.args[0] for call in recording_calls.call_args_list]
    assert "world/raw_map/markers/fill" in paths and "world/pgo_map/markers/fill" in paths
    fill = next(
        call.args[1]
        for call in recording_calls.call_args_list
        if call.args[0] == "world/pgo_map/markers/fill"
    )
    assert len(fill.centers.as_arrow_array()) == 3


def test_global_cli_registers_cdr_sensor_cloud_using_recorded_tf(recording_calls, tmp_path):
    path = tmp_path / "sensor.db"
    header = Header(frame_id="lidar", stamp=time_from_nanoseconds(1700000000123456789))
    cloud = pointcloud_from_xyz(np.array([[1.025, 0.025, 0.025]], dtype=np.float32), header=header)
    transform = TransformStamped(
        header=Header(frame_id="map", stamp=time_from_seconds(10)),
        child_frame_id="lidar",
        transform=Transform(translation=Vector3(2, 0, 0), rotation=Quaternion(0, 0, 0, 1)),
    )
    with SqliteStore(path=str(path)) as store:
        store.stream("lidar", PointCloud2).append(cloud, ts=10)
        store.stream("tf", TFMessage).append(TFMessage(transforms=[transform]), ts=10)
    invoke(
        "global",
        str(path),
        "--device",
        "CPU:0",
        "--no-gui",
        "--out",
        str(tmp_path / "registered.rrd"),
    )
    points = next(
        call.args[1]
        for call in recording_calls.call_args_list
        if call.args[0] == "world/raw_map/pointcloud"
    )
    np.testing.assert_allclose(
        points.positions.as_arrow_array().to_pylist(), [[3.025, 0.025, 0.025]], atol=1e-6
    )
