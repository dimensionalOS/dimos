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

from __future__ import annotations

from threading import Event, Thread

import numpy as np
import pytest

from dimos.hardware.sensors.camera.stereo.module import StereoDepth
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.galaxea.r1pro.blueprints.perception.r1pro_head_perception import (
    r1pro_head_perception,
)
from dimos.robot.galaxea.r1pro.head_cameras import HeadLeftCamera, HeadRightCamera
from dimos.spec.utils import spec_annotation_compliance
from dimos.visualization.perception.scene_spec import PerceptionSceneSpec
from dimos.visualization.perception.viser_module import PerceptionViser


@pytest.fixture
def viewer(mocker):
    runtime = mocker.MagicMock()
    mocker.patch("dimos.visualization.perception.viser_module.ViserRuntime", return_value=runtime)
    module = PerceptionViser()
    scene = mocker.MagicMock()
    mocker.patch.object(module, "_scene", scene, create=True)
    module._root = mocker.MagicMock()
    module._status = mocker.MagicMock()
    module._rgb = mocker.MagicMock()
    module._depth = mocker.MagicMock()
    module._selection = mocker.MagicMock()
    module._details = mocker.MagicMock()
    status = dict(
        available=True,
        capture_ts=100.0,
        age_s=0.1,
        inference_s=0.05,
        frame_id="camera",
        quality=[],
        error=None,
        reason="",
    )
    scene.get_scan_status.return_value = status
    image = Image(np.zeros((32, 32, 3), np.uint8), ImageFormat.RGB, frame_id="camera", ts=100.0)
    depth = Image(np.ones((32, 32), np.float32), ImageFormat.DEPTH, frame_id="camera", ts=100.0)
    scene.get_scan_visualization.return_value = dict(
        status=status,
        overlay=image,
        depth=depth,
        points=np.zeros((10, 3)),
        objects=[
            dict(
                name="cup",
                object_id="one",
                center=[0.0, 0.0, 1.0],
                size=[0.1, 0.1, 0.2],
                point_count=10,
                points=np.zeros((10, 3)),
            )
        ],
    )
    try:
        yield module, scene, runtime
    finally:
        module.stop()


def test_refresh_renders_camera_geometry_and_expiry_hides_it(viewer):
    module, scene, runtime = viewer
    module.refresh()
    assert module._rgb.visible and module._depth.visible
    assert module._selection.options == ["None", "cup / one"]
    assert runtime.server.scene.add_box.call_args.kwargs["position"] == (0.0, 0.0, 1.0)
    assert runtime.server.scene.add_point_cloud.call_count == 2
    scene.get_scan_status.return_value["available"] = False
    module.refresh()
    assert module._root.visible is False
    assert module._rgb.visible is False and module._depth.visible is False
    assert module._selection.options == ["None"]


def test_scan_button_has_only_perception_rpc_and_rejects_overlap(viewer):
    module, scene, _ = viewer
    entered, release = Event(), Event()

    def scan(labels):
        assert labels == ["cup", "box"]
        entered.set()
        assert release.wait(2)

    scene.scan_scene.side_effect = scan
    thread = Thread(target=lambda: module.scan("cup, box"))
    thread.start()
    try:
        assert entered.wait(2)
        assert module.scan("cup") is False
    finally:
        release.set()
        thread.join(2)
    scene.scan_scene.assert_called_once_with(["cup", "box"])
    assert not thread.is_alive()


def test_error_clears_display(viewer):
    module, scene, _ = viewer
    scene.scan_scene.side_effect = ValueError("RGB-D capture is stale")
    assert module.scan("cup") is False
    assert "stale" in module._status.content
    assert module._root.visible is False


def test_stop_closes_listener_even_if_remote_rpc_is_pending(viewer, mocker):
    module, _, runtime = viewer
    thread = mocker.MagicMock()
    thread.is_alive.return_value = True
    module._thread = thread
    try:
        with pytest.raises(RuntimeError, match="listener closed"):
            module.stop()
        runtime.close.assert_called_once()
        with pytest.raises(RuntimeError, match="still running"):
            module.start()
    finally:
        module._thread = None


def test_blueprint_contains_exactly_perception_modules():
    assert {b.module for b in r1pro_head_perception.blueprints} == {
        HeadLeftCamera,
        HeadRightCamera,
        StereoDepth,
        ObjectSceneRegistrationModule,
        PerceptionViser,
    }
    assert spec_annotation_compliance(ObjectSceneRegistrationModule, PerceptionSceneSpec)
