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

"""Composition contracts for the combined development stack."""

from dimos.core.transport_factory import zenoh_key_expr
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS
from dimos.simulation.behavior.blueprints import behavior_r1pro
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.types import TaskSelection


def test_combined_stack_keeps_task_scan_and_native_viewer():
    atom = next(a for a in behavior_r1pro.blueprints if a.module is BehaviorConnection)
    assert atom.kwargs["task"] == TaskSelection()
    assert atom.kwargs["publish_scan"] is True
    assert atom.kwargs["headless"] is False
    assert atom.kwargs["allow_task_changes"] is False
    for port in ("left_wrist_scan", "right_wrist_scan"):
        assert behavior_r1pro.remapping_map[(atom.name, port)] == "raw_scan"


def test_coordinator_transport_topics_match_adapter_factory():
    for port, message in (("motor_states", JointState), ("motor_command", MotorCommandArray)):
        spec = behavior_r1pro.transport_map[(port, message)]
        assert spec.args[0].key_expr == zenoh_key_expr(f"/r1pro/{port}", message.msg_name)


def test_only_upper_body_joints_are_executable_planning_groups():
    atom = next(a for a in behavior_r1pro.blueprints if a.module is ManipulationModule)
    model = atom.kwargs["model"]
    executable = {name for group in model.planning_groups for name in group.joint_names}
    assert executable == {f"r1pro/{name}" for name in UPPER_BODY_JOINTS}
    assert {"r1pro/base_x", "r1pro/base_y", "r1pro/base_yaw"} <= set(model.joint_names)
