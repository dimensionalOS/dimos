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

"""Real worker/transport checks for composed cameras sharing a rendering owner."""

from collections import defaultdict
from contextlib import ExitStack
from queue import Queue
import uuid

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sim_msgs import WorldState
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.assets.client import materialize_scene
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def camera_blueprint(world):
    def camera(name, hz):
        return SimCameraModule.blueprint(
            instance_name=name, sensor=CameraConfig(f"{name}/view", hz=hz)
        ).remappings(
            [
                (SimCameraModule, port, f"{name}/{port}")
                for port in ("color_image", "depth_image", "camera_info")
            ]
        )

    left, right = camera("left", 10), camera("right", 5)
    scene = UsdManager.blueprint(
        instance_name="scene",
        model=world,
        mounts={"left/camera": left, "right/camera": right},
    )
    return autoconnect(
        scene,
        left,
        right,
        CameraRenderModule.blueprint(
            scene_manager=scene, renderer=MujocoRenderer, cameras=("left", "right")
        ),
    ).global_config(
        viewer="none",
        transport="zenoh",
        n_workers=5,
        zenoh_domain="sim2-cameras-" + uuid.uuid4().hex,
    )


@pytest.fixture
def cameras(camera_blueprint, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    parsed = BlueprintConfigParser(camera_blueprint).parse()
    deployed = ModuleCoordinator.build(camera_blueprint, parsed_config=parsed)
    try:
        yield deployed
    finally:
        deployed.stop()


def test_two_mounted_cameras_share_one_scene_and_keep_topics_rates_and_timestamps(
    cameras, tmp_path
):
    render = cameras.get_instance(CameraRenderModule)
    wait_until(lambda: render.statistics().get("scene_loads") == 1, timeout=20)
    manager = cameras.get_instance("scene")
    scene = materialize_scene(
        manager.get_scene([])[1], manager.get_asset, cache=tmp_path / "reader"
    )
    messages = defaultdict(Queue)
    with ExitStack() as cleanup:
        backend = MujocoBackend()
        cleanup.callback(backend.close)
        backend.load(scene, 2_000_000)
        observations = cameras.transports[("observations", WorldState)]
        cleanup.callback(observations.stop)
        for name in ("left", "right"):
            for port, kind in (
                ("color_image", Image),
                ("depth_image", Image),
                ("camera_info", CameraInfo),
            ):
                transport = cameras.transports[(f"{name}/{port}", kind)]
                cleanup.callback(transport.stop)
                cleanup.callback(transport.subscribe(messages[name, port].put))
        samples = defaultdict(list)
        for step in range(5):
            bodies = backend.read_bodies()
            observations.publish(
                WorldState(
                    header=SampleHeader(
                        clock_id="camera-test", time_ns=step * 100_000_000, frame_id="world"
                    ),
                    model_revision=scene.revision,
                    bodies_length=len(bodies),
                    bodies=bodies,
                )
            )
            names = ("left", "right") if step % 2 == 0 else ("left",)
            for name in names:
                for port in ("color_image", "depth_image", "camera_info"):
                    msg = messages[name, port].get(timeout=10)
                    assert msg.frame_id == f"{name}/view"
                    assert msg.ts == pytest.approx(step / 10)
                    samples[name, port].append(msg)
    assert render.statistics()["scene_loads"] == 1
    assert render.statistics()["cameras"] == 2
    assert render.statistics()["error"] == ""
    assert [m.ts for m in samples["left", "color_image"]] == [0, 0.1, 0.2, 0.3, 0.4]
    assert [m.ts for m in samples["right", "color_image"]] == [0, 0.2, 0.4]
    assert cameras.get_instance("left").statistics()["acquisitions"] == 5
    assert cameras.get_instance("right").statistics()["acquisitions"] == 3
    assert samples["left", "color_image"][0].shape == (240, 320, 3)
    assert samples["right", "depth_image"][0].shape == (240, 320)
    np.testing.assert_allclose(
        samples["left", "camera_info"][0].K, [188.36, 0, 160, 0, 188.36, 120, 0, 0, 1]
    )


def test_renderer_cannot_take_a_camera_assigned_to_another_owner(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = autoconnect(
        SimCameraModule.blueprint(sensor=CameraConfig("camera"), renderer_name="other"),
        CameraRenderModule.blueprint(
            scene=world, renderer=MujocoRenderer, cameras=("SimCameraModule",)
        ),
    ).global_config(
        viewer="none", n_workers=1, zenoh_domain="sim2-wrong-renderer-" + uuid.uuid4().hex
    )
    with pytest.raises(ExceptionGroup) as error:
        ModuleCoordinator.build(blueprint)
    assert error.group_contains(ValueError, match="camera belongs to renderer")
