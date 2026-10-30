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

"""Load only the selected physics implementation inside its owner worker."""

from functools import partial
from pathlib import Path

from dimos.sim2.physics import PhysicsBackend
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.spec import RobotInstanceSpec


def create_backend(
    name: str,
    robots: dict[str, RobotInstanceSpec],
    scene: Path,
    sensors: tuple[tuple[str | None, Sensor], ...] = (),
    snapshot_hz: float = 60.0,
) -> PhysicsBackend:
    if name == "mujoco":
        # These engines are optional and must not initialize in the coordinator.
        from dimos.sim2.backends.mujoco import MujocoBackend
        from dimos.sim2.environment import MotorScene
        from dimos.sim2.model import model_factory

        environment = partial(
            MotorScene,
            scene=scene,
            models={key: model_factory(robot, scene) for key, robot in robots.items()},
            sensors=sensors,
            timestep=min((robot.timestep for robot in robots.values()), default=0.005),
        )
        return MujocoBackend(environment, robots, sensors, snapshot_hz)
    if name == "genesis":
        from dimos.sim2.backends.genesis import GenesisBackend

        return GenesisBackend(robots, scene, sensors)
    raise ValueError(f"unsupported physics backend {name!r}; choose mujoco or genesis")
