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

from dataclasses import replace
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.sim2.backends.mjcf_visuals import load_visuals
from dimos.sim2.backends.mujoco import read_body_states
from dimos.sim2.messages import FrameMetadata
from dimos.sim2.physics import WorldState
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.camera.unreal.assets import geom_poses
from dimos.sim2.sensors.camera.unreal.client import RenderedFrame, RendererClient
from dimos.sim2.sensors.camera.unreal.module import UnrealCameraModule, camera_view, frame_messages
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.world.spec import SimulationSpec


@pytest.fixture
def scene():
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
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = {i: model.body(i).name.replace("robotg1_", "g1/") for i in range(model.nbody)}
    visuals = load_visuals(model, names, lambda name: name.replace("robotg1_", "g1/"), 0)
    return visuals, WorldState(
        FrameMetadata(0, 0, 0, 0.0), read_body_states(model, data, names), {}, {}
    )


def _snapshot(scene, *, sequence, episode, tick, timestamp):
    return replace(
        scene[1], metadata=FrameMetadata(sequence, episode, tick, tick * 0.005), wall_time=timestamp
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


def test_named_camera_retains_model_calibration_and_publishes_world_optical_pose(scene):
    calibration, position, optical = camera_view(
        *scene, Camera("front", "front_camera", width=1200, height=800), "g1"
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
    scene, robot_id, link, expected_position, expected_optical
):
    sensor = Camera(
        "observer",
        Mount(link, xyz=(0.4, 0.5, 0.6), rpy=(np.pi / 2, 0, 0)),
        width=320,
        height=240,
        fovy=90,
    )
    calibration, position, optical = camera_view(*scene, sensor, robot_id)

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
def camera_module(scene, mocker):
    # Keep the actual Module lifecycle/event loop; only RPC and output I/O are absent.
    mocker.patch.object(ZenohRPC, "start", autospec=True)
    mocker.patch.object(ZenohRPC, "serve_module_rpc", autospec=True)
    module = UnrealCameraModule(
        executable=Path("/unused/unreal"),
        rpc_transport=ZenohRPC,
        robot_id="g1",
        sensor=Camera("front", "front_camera", width=2, height=2),
    )
    simulation = mocker.Mock(spec=SimulationSpec)
    simulation.describe_visuals.side_effect = lambda: replace(
        scene[0], revision=(module._current_state() or scene[1]).scene_revision
    )
    mocker.patch.object(module, "_simulation", simulation, create=True)
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


@pytest.fixture
def native_renderer(mocker):
    process = mocker.patch(
        "dimos.sim2.sensors.camera.unreal.module.RendererProcess", autospec=True
    ).return_value
    client = mocker.create_autospec(RendererClient, instance=True)
    process.__enter__.return_value = client
    return client


@pytest.mark.parametrize(
    ("arrival", "expected_loads"), [("paused", 1), ("advance", 1), ("reset", 1), ("model", 2)]
)
def test_first_capture_uses_latest_state_after_asset_load_and_becomes_ready(
    camera_module, native_renderer, scene, arrival, expected_loads
):
    module, publishers = camera_module
    first = _snapshot(scene, sequence=1, episode=0, tick=40, timestamp=100.2)
    latest = first
    if arrival != "paused":
        latest = _snapshot(
            scene,
            sequence=2,
            episode=1 if arrival == "reset" else 0,
            tick=0 if arrival == "reset" else 50,
            timestamp=101,
        )
    if arrival == "model":
        latest = replace(latest, scene_revision=1)
    module._on_state(first)
    pending = iter([latest])

    def load_scene(path, *, timeout):
        frame = next(pending, None)
        if frame is not None:
            module._on_state(frame)

    native_renderer.load.side_effect = load_scene
    native_renderer.capture.side_effect = lambda command: _rendered(command["timestamp"])
    publishers["frame_metadata"].side_effect = lambda _: module._stop.set()

    module._run()

    assert native_renderer.load.call_count == expected_loads
    for load_call in native_renderer.load.call_args_list:
        assert 0 < load_call.kwargs["timeout"] <= module.config.startup_timeout
    native_renderer.reset.assert_called_once_with(latest.metadata.episode_id)
    command = native_renderer.capture.call_args.args[0]
    assert (
        command["episode"],
        command["sequence"],
        command["physics_tick"],
        command["timestamp"],
    ) == (
        latest.metadata.episode_id,
        latest.metadata.sequence,
        latest.metadata.physics_tick,
        latest.wall_time,
    )
    publishers["frame_metadata"].assert_called_once_with(latest.metadata)
    assert module._ready.get_nowait() is None
    assert module.renderer_status()["frames"] == 1


def test_initial_asset_load_uses_remaining_process_startup_budget(camera_module, scene, mocker):
    module, publishers = camera_module
    module.config.level = "/Game/Maps/Store.Store"
    module._on_state(_snapshot(scene, sequence=1, episode=0, tick=40, timestamp=100.2))
    clock = mocker.patch("dimos.sim2.sensors.camera.unreal.module.time.monotonic", return_value=100)
    native_renderer = mocker.create_autospec(RendererClient, instance=True)
    # Crossing the external renderer boundary consumes 40 seconds of the same budget.
    native_renderer.capture.side_effect = lambda command: _rendered(command["timestamp"])
    publishers["frame_metadata"].side_effect = lambda _: module._stop.set()

    def after_process_start(*args, **kwargs):
        clock.return_value = 140
        return native_renderer

    factory = mocker.patch("dimos.sim2.sensors.camera.unreal.module.RendererProcess", autospec=True)
    factory.return_value.__enter__.side_effect = after_process_start

    module._run()

    assert factory.call_args.kwargs["level"] == "/Game/Maps/Store.Store"
    assert native_renderer.load.call_args.kwargs["timeout"] == 140
    assert module.renderer_status()["frames"] == 1


def test_reset_received_during_capture_discards_old_episode_before_any_output(camera_module, scene):
    module, publishers = camera_module
    first = _snapshot(scene, sequence=1, episode=0, tick=40, timestamp=100.2)
    module._on_state(first)
    view = camera_view(scene[0], first, module.config.sensor, "g1")
    old_metadata = first.metadata

    reset = _snapshot(scene, sequence=2, episode=1, tick=0, timestamp=200)
    module._on_state(reset)
    # The in-flight frame retains its old epoch, even before the next capture.
    assert first.metadata is old_metadata
    assert module._current_state().metadata.episode_id == 1
    module._publish_frame(first, _rendered(100.2), *view)

    for publish in publishers.values():
        publish.assert_not_called()
    assert module.renderer_status()["discarded"] == 1
    new_view = camera_view(scene[0], reset, module.config.sensor, "g1")
    module._publish_frame(reset, _rendered(reset.wall_time), *new_view)

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


def test_stopped_module_never_publishes_a_completed_capture(camera_module, scene):
    module, publishers = camera_module
    snapshot = _snapshot(scene, sequence=1, episode=0, tick=1, timestamp=100)
    module._on_state(snapshot)
    view = camera_view(scene[0], snapshot, module.config.sensor, "g1")
    module._stop.set()

    module._publish_frame(snapshot, _rendered(100), *view)

    for publish in publishers.values():
        publish.assert_not_called()
    assert module.renderer_status()["discarded"] == 1


def test_rendered_placements_follow_neutral_body_state_without_native_model_updates(scene):
    visuals, initial = scene
    shifted = replace(
        initial,
        bodies={
            **initial.bodies,
            "g1/base": replace(initial.bodies["g1/base"], position=(4.0, -2.0, 3.0)),
        },
    )
    before = geom_poses(visuals, initial)
    after = geom_poses(visuals, shifted)
    assert_allclose(np.array(after[0]["position"]) - before[0]["position"], (300, 0, 0))
    sensor = Camera("front", "front_camera", width=600, height=400)
    _, position_before, _ = camera_view(visuals, initial, sensor, "g1")
    _, position_after, _ = camera_view(visuals, shifted, sensor, "g1")
    assert_allclose(position_after - position_before, (3, 0, 0))


def test_visual_revision_change_discards_capture_even_within_same_epoch(camera_module, scene):
    module, publishers = camera_module
    first = _snapshot(scene, sequence=1, episode=0, tick=1, timestamp=100)
    module._on_state(first)
    view = camera_view(scene[0], first, module.config.sensor, "g1")
    module._on_state(replace(first, metadata=replace(first.metadata, sequence=2), scene_revision=1))
    assert not module._publish_frame(first, _rendered(100), *view)
    publishers["color_image"].assert_not_called()
    assert module.renderer_status()["discarded"] == 1


def test_unreal_camera_import_needs_no_physics_engine():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import importlib.abc
import sys
from typing import get_type_hints
class RejectEngines(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco', 'robosuite', 'genesis'}:
            raise ImportError('physics engine leaked into Unreal worker: ' + fullname)
sys.meta_path.insert(0, RejectEngines())
from dimos.sim2.sensors.camera.unreal.module import UnrealCameraModule
from dimos.sim2.module import SimulationModule
from dimos.sim2.visuals import VisualScene
from dimos.sim2.world.spec import SimulationSpec
assert 'world_state' in UnrealCameraModule.__annotations__
assert get_type_hints(SimulationSpec.describe_visuals)['return'] is VisualScene
assert get_type_hints(SimulationModule.describe_visuals)['return'] is VisualScene
""",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
