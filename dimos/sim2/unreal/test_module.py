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

"""Camera calibration, world mounts and reset isolation without a GPU process."""

from pathlib import Path

import mujoco
import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.unreal.client import RenderedFrame
from dimos.sim2.unreal.module import UnrealCameraModule, camera_view, frame_messages
from dimos.sim2.world.runtime import STATE


@pytest.fixture
def reader(tmp_path):
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
          <body name="robotg1_base" pos="1 -2 3" axisangle="0 0 1 90">
            <freejoint/><geom type="box" size=".1 .1 .1"/>
            <site name="robotg1_mount" pos=".2 .3 .4" axisangle="0 1 0 90"/>
            <camera name="robotg1_front_camera" pos=".4 .5 .6"
              sensorsize=".036 .024" resolution="600 400"
              focalpixel="500 520" principalpixel="12 8"/>
          </body>
        </worldbody></mujoco>
    """)
    path = tmp_path / "world.mjb"
    mujoco.mj_saveModel(model, str(path))
    value = WorldReader(
        {
            "model": str(path),
            "model_prefixes": {"g1": "robotg1_"},
            "physics_dt": 0.005,
            "snapshot_stride": 1,
            "snapshot_slots": 8,
        }
    )
    mujoco.mj_forward(value.model, value.data)
    return value


def _snapshot(reader, *, sequence, episode, tick, timestamp):
    reader.data.time = tick * 0.005
    state = np.empty(mujoco.mj_stateSize(reader.model, STATE))
    mujoco.mj_getState(reader.model, reader.data, state, STATE)
    return StateFrame(
        FrameMetadata(sequence, episode, tick, reader.data.time),
        {"state": state, "wall_time": np.array([timestamp])},
    )


def _rendered(timestamp):
    return RenderedFrame(
        {"timestamp": timestamp, "capture_ms": 12.5},
        np.array(
            [[[1, 2, 3, 255], [4, 5, 6, 255]], [[7, 8, 9, 255], [10, 11, 12, 255]]], dtype=np.uint8
        ),
        np.array([[1.25, 2.5], [3.75, np.inf]], dtype=np.float32),
        0.02,
    )


def test_named_camera_retains_model_calibration_and_publishes_world_optical_pose(reader):
    calibration, position, optical = camera_view(
        reader, Camera("front", "front_camera", width=1200, height=800), "g1"
    )

    assert_allclose(
        [calibration[key] for key in ("fx", "fy", "cx", "cy")],
        [1000, 1040, 575.5, 383.5],
        rtol=1e-6,
    )
    assert_allclose(position, [0.5, -1.6, 3.6])
    assert_allclose(calibration["position"], [50, 160, 360])
    assert_allclose(
        Rotation.from_quat(optical).as_matrix(), [[0, 1, 0], [1, 0, 0], [0, 0, -1]], atol=1e-14
    )
    # UE forward/right/up and ROS optical right/down/forward describe one view.
    assert_allclose(
        Rotation.from_quat(calibration["quaternion"]).as_matrix(),
        [[0, 0, -1], [0, -1, 0], [-1, 0, 0]],
        atol=1e-14,
    )


@pytest.mark.parametrize(
    ("robot_id", "link", "expected_position", "expected_optical"),
    [
        (None, "world", [0.4, 0.5, 0.6], [[1, 0, 0], [0, 0, 1], [0, -1, 0]]),
        ("g1", "base", [0.5, -1.6, 3.6], [[0, 0, -1], [1, 0, 0], [0, -1, 0]]),
        ("g1", "mount", [0.2, -1.2, 3.0], [[0, 0, -1], [0, -1, 0], [-1, 0, 0]]),
    ],
)
def test_added_camera_mount_composes_parent_pose_and_local_offset(
    reader, robot_id, link, expected_position, expected_optical
):
    sensor = Camera(
        "observer",
        Mount(link, xyz=(0.4, 0.5, 0.6), rpy=(np.pi / 2, 0, 0)),
        width=320,
        height=240,
        fovy=90,
    )
    calibration, position, optical = camera_view(reader, sensor, robot_id)

    assert_allclose(position, expected_position)
    assert_allclose(Rotation.from_quat(optical).as_matrix(), expected_optical, atol=1e-14)
    assert_allclose(calibration["position"], np.array(expected_position) * [100, -100, 100])
    assert_allclose(
        [calibration[key] for key in ("fx", "fy", "cx", "cy")], [120, 120, 159.5, 119.5]
    )


def test_frame_messages_preserve_pixels_axial_metres_calibration_and_acquisition_time():
    rendered = _rendered(123.125)
    calibration = {"width": 2, "height": 2, "fx": 80, "fy": 95, "cx": 0.2, "cy": 0.6}
    position = np.array([1.2, -3.4, 5.6])
    optical = Rotation.from_euler("xyz", [0.2, -0.4, 1.2]).as_quat()

    color, depth, info, transforms = frame_messages(
        rendered, calibration, position, optical, "g1/front_optical"
    )

    assert_array_equal(color.data, rendered.rgba[:, :, :3])
    assert color.data.flags.c_contiguous
    assert color.format == ImageFormat.RGB
    assert_array_equal(depth.data, rendered.depth)
    assert depth.format == ImageFormat.DEPTH
    assert info.K == [80, 0, 0.2, 0, 95, 0.6, 0, 0, 1]
    assert (info.width, info.height) == (2, 2)
    assert color.ts == depth.ts == info.ts == transforms[0].ts == 123.125
    assert color.frame_id == depth.frame_id == info.frame_id == "g1/front_optical"
    assert len(transforms) == 1
    assert transforms[0].frame_id == "world"
    assert transforms[0].child_frame_id == "g1/front_optical"
    assert_allclose(transforms[0].translation.to_tuple(), position)
    assert_allclose(transforms[0].rotation.to_tuple(), optical)


@pytest.fixture
def camera_module(reader, mocker):
    # Keep the actual Module lifecycle/event loop; only RPC and output I/O are absent.
    mocker.patch.object(ZenohRPC, "start", autospec=True)
    mocker.patch.object(ZenohRPC, "serve_module_rpc", autospec=True)
    module = UnrealCameraModule(
        executable=Path("/unused/unreal"),
        rpc_transport=ZenohRPC,
        robot_id="g1",
        sensor=Camera("front", "front_camera", width=2, height=2),
    )
    module.reader = reader
    publishers = {
        name: mocker.patch.object(getattr(module, name), "publish")
        for name in (
            "color_image",
            "depth_image",
            "camera_info",
            "depth_camera_info",
            "tf",
            "frame_metadata",
        )
    }
    try:
        yield module, publishers
    finally:
        module.stop()


def test_reset_received_during_capture_discards_old_episode_before_any_output(camera_module):
    module, publishers = camera_module
    first = _snapshot(module.reader, sequence=1, episode=0, tick=40, timestamp=100.2)
    module._on_snapshot(first)
    assert module.reader.update()
    view = camera_view(module.reader, module.config.sensor, "g1")
    old_metadata = module.reader.metadata

    reset = _snapshot(module.reader, sequence=2, episode=1, tick=0, timestamp=200)
    module._on_snapshot(reset)
    # A reset notification invalidates the in-flight capture before update() restores it.
    assert module.reader.metadata is old_metadata
    assert module.reader.latest_episode == 1
    module._publish_frame(old_metadata, _rendered(100.2), *view)

    for publish in publishers.values():
        publish.assert_not_called()
    assert module.renderer_status()["discarded"] == 1
    assert module.reader.update()
    new_view = camera_view(module.reader, module.config.sensor, "g1")
    module._publish_frame(module.reader.metadata, _rendered(module.reader.timestamp), *new_view)

    for publish in publishers.values():
        publish.assert_called_once()
    publishers["frame_metadata"].assert_called_once_with(reset.metadata)
    assert publishers["color_image"].call_args.args[0].ts == 200
    assert publishers["depth_image"].call_args.args[0].ts == 200
    assert (
        publishers["camera_info"].call_args.args[0].K
        == publishers["depth_camera_info"].call_args.args[0].K
    )
    assert module.renderer_status()["frames"] == 1


def test_stopped_module_never_publishes_a_completed_capture(camera_module):
    module, publishers = camera_module
    snapshot = _snapshot(module.reader, sequence=1, episode=0, tick=1, timestamp=100)
    module._on_snapshot(snapshot)
    assert module.reader.update()
    view = camera_view(module.reader, module.config.sensor, "g1")
    module._stop.set()

    module._publish_frame(module.reader.metadata, _rendered(100), *view)

    for publish in publishers.values():
        publish.assert_not_called()
    assert module.renderer_status()["discarded"] == 1
