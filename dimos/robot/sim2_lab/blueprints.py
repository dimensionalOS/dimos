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

"""The entire attachment is here: one world, connections, sensors and URDF owners."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.sim2_lab.connection import PusherConnection
from dimos.robot.sim2_lab.model import PUSHER, lab_world
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.module import SimModule
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.sensors import MujocoSensors

_world = lab_world()

sim2_lab = autoconnect(
    SimModule.blueprint(world=_world, backend=MujocoBackend),
    *(
        PusherConnection.blueprint(
            instance_name=instance.id, source=instance.id, model=PUSHER
        ).remappings([(instance.id, "position_command", instance.id + "/position_command")])
        for instance in _world.instances
        if instance.drives
    ),
    SimSensorModule.blueprint(world=_world, reader=MujocoSensors),
    *(
        UrdfManager.blueprint(instance_name=instance.id + "_urdf", model=instance.model).remappings(
            [
                (instance.id + "_urdf", "coordinator_joint_state", "joint_state"),
                (
                    instance.id + "_urdf",
                    "render_file_request",
                    instance.id + "/render_file_request",
                ),
                (instance.id + "_urdf", "file_info", instance.id + "/file_info"),
            ]
        )
        for instance in _world.instances
    ),
).global_config(viewer="none", transport="zenoh", n_workers=6)
