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

"""The sim2 Go2 with a Mid-360 on the real rig's mount, its fitted pattern and mount occlusion."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.sim2 import Go2Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.module import Simulation
from dimos.visualization.vis_module import vis_module

# The sensors as their own blueprints, because the manager mounts them by the instance each one
# deploys: a frame_id alone would name a module the manager cannot address.
_lidar = LivoxSim.blueprint(frame_id=Go2Sim.lidar_mount_frame, go2_occlusion=True)
_camera = SimCamera.blueprint(frame_id=Go2Sim.camera_mount_frame)

unitree_go2_sim2_mid360 = autoconnect(
    Simulation.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    Go2Sim.blueprint(),
    # One manager holding the whole robot: the Go2's own layer, with each sensor grafted onto the
    # frame it is bolted to. Without the mounts it answered for the robot alone, so the lidar and
    # the camera were nowhere in the stage a viewer draws -- the sensors published, and nothing
    # showed what was doing the sensing.
    UsdManager.blueprint(
        mounts={Go2Sim.lidar_mount_frame: _lidar, Go2Sim.camera_mount_frame: _camera}
    ).remappings([(UsdManager, "coordinator_joint_state", "joint_state")]),
    _lidar,
    _camera,
).global_config(n_workers=4, robot_model="unitree_go2")
