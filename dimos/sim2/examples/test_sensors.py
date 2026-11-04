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

from dataclasses import replace
import uuid

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def deployed_world(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    # Consumers know instance IDs, placements and drives; only the manager has models.
    scene = replace(world, instances=tuple(replace(i, model=None) for i in world.instances))
    blueprint = autoconnect(
        UrdfManager.blueprint(models={i.id: i.model for i in world.instances}),
        SimModule.blueprint(
            scene=scene,
            backend=MujocoBackend,
            motion_frames=("left/imu",),
            joint_ids=("left/slide_x",),
        ),
        SimImuModule.blueprint(sensor=ImuConfig("left/imu")),
        SimJointEncoderModule.blueprint(sensor=EncoderConfig(("left/slide_x",))),
        SimCameraModule.blueprint(
            scene=scene, sensor=CameraConfig("left/camera"), renderer=MujocoRenderer
        ),
        SimLidarModule.blueprint(
            scene=scene, sensor=LidarConfig("left/lidar"), raycaster=MujocoRaycaster
        ),
    ).global_config(
        viewer="none",
        transport="zenoh",
        n_workers=4,
        zenoh_domain="sim2-test-" + uuid.uuid4().hex,
        zenoh_scout_addr="224.0.0.227:7446",
    )
    deployment = ModuleCoordinator.build(blueprint)
    try:
        yield deployment
    finally:
        deployment.stop()


def test_separate_sensor_modules_run_reset_and_recover(deployed_world, command):
    sim = deployed_world.get_instance(SimModule)
    camera = deployed_world.get_instance(SimCameraModule)
    lidar = deployed_world.get_instance(SimLidarModule)
    wait_until(lambda: camera.statistics()["acquisitions"] >= 2, timeout=20)
    wait_until(lambda: lidar.statistics()["acquisitions"] >= 2, timeout=20)
    before = sim.inspect()
    h = before.header
    sim.submit(command(h, positions=(0.65,)))
    wait_until(lambda: poses_by_id(sim.inspect())["box"].position.x > 0.1, timeout=5)
    reset = sim.reset()
    assert reset.header.epoch == h.epoch + 1
    assert poses_by_id(reset)["box"].position.z == pytest.approx(0.055)
    count = camera.statistics()["acquisitions"]
    wait_until(lambda: camera.statistics()["acquisitions"] > count, timeout=5)
    assert camera.statistics()["error"] == lidar.statistics()["error"] == ""
    assert sim.status()["error"] == sim.status()["publication_error"] == ""
