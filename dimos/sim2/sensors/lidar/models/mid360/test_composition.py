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

"""The hardware Mid360's simulated binding preserves its public sensor ports."""

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.sim2.composition import attach_simulation
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.lidar.models.mid360.pattern import Mid360Pattern
from dimos.sim2.sensors.spec import Mount


def test_mid360_and_its_imu_keep_driver_names_and_public_ports(scene_path, mocker):
    mocker.patch(
        "dimos.sim2.sensors.lidar.models.mid360.model._pattern",
        return_value=Mid360Pattern(0, 0, 0, 0, np.asarray([[[1, 0, 0]]] * 4, dtype=float)),
    )
    blueprint = autoconnect(
        G1Connection.blueprint(control_mode="motors"),
        Mid360.blueprint(instance_name="scanner", frame_id="scan"),
    )

    result, _ = attach_simulation(
        blueprint, {}, GlobalConfig(simulation="mujoco", scene_package=str(scene_path))
    )

    world = next(atom for atom in result.blueprints if atom.module is SimulationModule)
    attached = next(sensor for owner, sensor in world.kwargs["sensors"] if sensor.name == "scanner")
    assert attached.imu.name == "scanner_imu"
    assert attached.imu.site == Mount("scan", xyz=(0.011, 0.02329, -0.04412))
    assert {
        port: stream
        for (atom, port), stream in result.remapping_map.items()
        if atom == "scanner" and port in ("lidar", "imu")
    } == {"lidar": "lidar", "imu": "imu"}
