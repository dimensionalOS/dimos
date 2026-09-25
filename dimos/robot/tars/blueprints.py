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

"""TARS (MuJoCo sim via tars_sdk) driven through the ControlCoordinator as a twist base.

Usage:
    uv pip install -e experimental/tars_sdk
    dimos run coordinator-tars-sim                    # TARS in the Go2 office scene + viewer, /cmd_vel
    dimos run coordinator-tars-sim-keyboard-teleop    # + WASD pygame teleop (Linux; crashes on macOS)
    python -m dimos.robot.tars.demo_keyboard_cmd_vel  # macOS: run next to coordinator-tars-sim
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_tars_joints = make_twist_base_joints("tars")

_tars_hw = HardwareComponent(
    hardware_id="tars",
    hardware_type=HardwareType.BASE,
    joints=_tars_joints,
    adapter_type="tars",
    # same room as `dimos --simulation run unitree-go2`, in an open aisle facing +x
    adapter_kwargs={"scene": "office1", "spawn": (-4.5, 1.1, 0.0), "viewer": True},
)

_tars_vel_task = TaskConfig(
    name="vel_tars",
    type="velocity",
    joint_names=_tars_joints,
    priority=10,
)

coordinator_tars_sim = ControlCoordinator.blueprint(
    hardware=[_tars_hw],
    tasks=[_tars_vel_task],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

coordinator_tars_sim_keyboard_teleop = autoconnect(
    ControlCoordinator.blueprint(hardware=[_tars_hw], tasks=[_tars_vel_task]),
    KeyboardTeleop.blueprint(),
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])
