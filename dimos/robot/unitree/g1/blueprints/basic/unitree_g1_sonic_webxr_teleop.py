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

"""PICO WebXR full-body teleoperation of SONIC G1.

Run with:

    dimos --simulation mujoco run unitree-g1-sonic-webxr-teleop
    dimos --viewer none run unitree-g1-sonic-webxr-teleop --network-interface <robot-nic>

Press A+B+X+Y to start in planner mode, then A+X to toggle full-body POSE.
Press A+B+X+Y again to stop the teleop session without disabling SONIC.
Accepted POSE chunks appear under world/sonic_reference when Rerun is enabled.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_wbc import (
    _g1_sonic_control_blueprint,
    _g1_sonic_visualization,
)
from dimos.teleop.webxr.extensions import MobileVideoArmTeleopModule

unitree_g1_sonic_webxr_teleop = autoconnect(
    MobileVideoArmTeleopModule.blueprint(body_tracking_mode="required"),
    _g1_sonic_control_blueprint(
        task_type="g1_sonic_teleop",
        task_name="sonic_teleop",
        zmq_enabled=False,
    ),
    _g1_sonic_visualization(),
).global_config(robot_model="unitree_g1", n_workers=3)
