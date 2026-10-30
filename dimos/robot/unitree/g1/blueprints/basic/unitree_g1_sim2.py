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

"""Drivable G1, existing office desk assets and loose objects, through one shared world."""

from dataclasses import replace
import math
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.model import G1_SENSOR_MODEL, WALKING_DRIVES
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.module import SimModule
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.sensors import MujocoSensors
from dimos.sim2.types import Camera, Imu, Instance, Lidar, World
from dimos.utils.data import LfsPath

_ASSETS = Path(__file__).resolve().parents[6] / "assets"
_drives = tuple(replace(d, joint="g1/" + d.joint) for d in WALKING_DRIVES)
_desk = RobotModel.from_file(
    _ASSETS / "g1_desk/desk.urdf", package_paths={"g1_desk": LfsPath("g1_desk")}
)
_box = RobotModel.from_file(_ASSETS / "g1_desk/box.urdf")

G1_DESK_WORLD = World(
    instances=(
        Instance(
            "g1",
            G1_SENSOR_MODEL.with_prefix("g1/"),
            xyz=(0, 0, 0.793),
            floating=True,
            drives=_drives,
        ),
        Instance("desk", _desk, xyz=(2, 0, 0), rpy=(0, 0, math.pi / 2)),
        Instance("parcel", _box.with_prefix("parcel/"), xyz=(1, 1, 0.05), floating=True),
        Instance("desk_box", _box.with_prefix("desk_box/"), xyz=(2, 0.18, 0.72), floating=True),
    ),
    sensors=(Camera("g1/camera", hz=15.0), Lidar("g1/lidar", rays=720, packets=10), Imu("g1/imu")),
)

unitree_g1_sim2 = autoconnect(
    SimModule.blueprint(
        world=G1_DESK_WORLD, backend=MujocoBackend, control_joints=tuple(d.joint for d in _drives)
    ),
    G1Connection.blueprint(connection_type="sim2"),
    SimSensorModule.blueprint(world=G1_DESK_WORLD, reader=MujocoSensors, prefix="g1").remappings(
        [("simsensormodule", "lidar", "lidar_raw")]
    ),
    *(
        UrdfManager.blueprint(instance_name=i.id + "_urdf", model=i.model).remappings(
            [
                (i.id + "_urdf", "coordinator_joint_state", "joint_state"),
                (
                    i.id + "_urdf",
                    "render_file_request",
                    "render_file_request" if i.id == "g1" else i.id + "/render_file_request",
                ),
                (i.id + "_urdf", "file_info", "file_info" if i.id == "g1" else i.id + "/file_info"),
            ]
        )
        for i in G1_DESK_WORLD.instances
    ),
).global_config(viewer="none", transport="zenoh", n_workers=4)
