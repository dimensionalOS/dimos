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

"""Compose one world and its sensor workers into a Blueprint."""

from collections.abc import Callable, Mapping
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.sim2.module import SimulationModule
from dimos.sim2.physics import PhysicsBackend
from dimos.sim2.sensors.camera.module import SimCameraModule, SimRGBDCameraModule
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.imu.module import SimImuModule
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.spec import RobotInstanceSpec


def simulation(
    *,
    environment: Callable[[], Any] | None = None,
    backend: Callable[[], PhysicsBackend] | None = None,
    robots: dict[str, RobotInstanceSpec] | None = None,
    sensors: tuple[tuple[str | None, Sensor], ...] = (),
    snapshot_hz: float = 60.0,
    module: type[SimulationModule] = SimulationModule,
    instance_name: str | None = None,
    names: Mapping[tuple[str | None, str], str] | None = None,
    robot_topics: dict[str, dict[str, str]] | None = None,
) -> Blueprint:
    """Expose an upstream environment and optional hardware/sensor stream bindings."""
    robots = robots or {}
    names = names or {}
    modules = [
        module.blueprint(
            environment=environment,
            backend=backend,
            robots=robots,
            sensors=sensors,
            snapshot_hz=snapshot_hz,
            instance_name=instance_name,
            robot_topics=robot_topics or {},
        )
    ]
    for robot_id, sensor in sensors:
        owner = robot_id or "world"
        named = names.get((robot_id, sensor.name))
        kwargs = dict(
            instance_name=named or f"{owner}_{sensor.name}", robot_id=robot_id, sensor=sensor
        )
        if isinstance(sensor, Camera):
            camera_module = SimRGBDCameraModule if sensor.depth else SimCameraModule
            blueprint = camera_module.blueprint(**kwargs)
        elif isinstance(sensor, Lidar):
            blueprint = LidarModule.blueprint(**kwargs)
        elif isinstance(sensor, Imu):
            blueprint = SimImuModule.blueprint(**kwargs)
        else:
            raise TypeError(f"unsupported sensor configuration: {type(sensor).__name__}")
        if named is not None:
            modules.append(blueprint)
            continue
        atom = blueprint.blueprints[0]
        blueprint = blueprint.remappings(
            [
                (atom.name, port.name, f"{owner}/{sensor.name}/{port.name}")
                for port in atom.streams
                if port.direction == "out" and port.name != "tf"
            ]
        )
        modules.append(blueprint)
    return autoconnect(*modules)
