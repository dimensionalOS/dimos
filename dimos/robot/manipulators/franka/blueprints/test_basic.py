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

"""The FR3 planner blueprint wires one connection, coordinator and planner on shared names."""

import pytest

from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.franka.blueprints.basic import fr3_planner_coordinator
from dimos.robot.manipulators.franka.config import FR3_FINGER_JOINT, fr3_joint_names
from dimos.robot.manipulators.franka.connection import FrankaConfig, FrankaConnection

# The FR3 model comes from franka_description on GitHub.
pytestmark = pytest.mark.self_hosted


def _parsed() -> tuple[object, BlueprintConfigParser]:
    blueprint = fr3_planner_coordinator.solidify()
    parsed = BlueprintConfigParser(blueprint).parse(environ={"FRANKACONNECTION__IP": "172.16.0.2"})
    return blueprint, parsed


def _atom(blueprint: object, module: type) -> object:
    return next(a for a in blueprint.active_blueprints if issubclass(a.module, module))  # type: ignore[attr-defined]


def test_the_connection_takes_its_ip_at_run_time_and_drives_the_arm_and_hand() -> None:
    blueprint, parsed = _parsed()
    config = FrankaConfig.model_validate(
        parsed.module_kwargs(_atom(blueprint, FrankaConnection).name)
    )
    assert config.ip == "172.16.0.2"
    assert list(config.joint_commands) == [*fr3_joint_names("arm"), f"arm/{FR3_FINGER_JOINT}"]


def test_coordinator_tasks_cover_the_connection_joints() -> None:
    blueprint, _ = _parsed()
    atom = _atom(blueprint, ControlCoordinator)
    (connection,) = atom.kwargs["connections"]  # type: ignore[attr-defined]
    commanded = {joint.name for joint in movable_joints(connection.model)}
    tasks = {task.name: task.joint_names for task in atom.kwargs["tasks"]}  # type: ignore[attr-defined]
    assert tasks["arm_gripper"] == [f"arm/{FR3_FINGER_JOINT}"]
    assert set(fr3_joint_names("arm")) | set(tasks["arm_gripper"]) == commanded


def test_the_planner_plans_the_seven_joints_to_the_hand_tcp() -> None:
    blueprint, parsed = _parsed()
    config = ManipulationModuleConfig.model_validate(
        parsed.module_kwargs(_atom(blueprint, ManipulationModule).name)
    )
    (group,) = config.model.planning_groups
    assert group.joint_names == tuple(fr3_joint_names("arm"))
    assert group.tip_link == "fr3_hand_tcp"
    assert config.model.gripper is not None
    assert config.model.gripper.finger_joints == {f"arm/{FR3_FINGER_JOINT}": (0.04, 0.0)}


@pytest.mark.parametrize("backend", ["roboplan", "drake"])
def test_both_planning_backends_load_the_fr3_with_the_tcp_where_the_robot_reports_it(
    backend: str,
) -> None:
    import math

    from dimos.manipulation.planning.spec.validation import prepare_robot_model
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.robot.manipulators.franka.config import make_fr3_model_config

    if backend == "roboplan":
        from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld as World
    else:
        from dimos.manipulation.planning.world.drake_world import (
            DrakeWorld as World,  # type: ignore[assignment]
        )

    config = make_fr3_model_config(robot="arm")
    world = World(enable_viz=False)
    world.load_model(prepare_robot_model(config))
    world.finalize()
    # A pose read from a real FR3 with the Franka Hand, and its O_T_EE translation.
    q = [math.radians(d) for d in [0.3, -40.2, -7.9, -142.2, -16.1, 90.9, 44.8]]
    joints = JointState({"name": config.joint_names, "position": q})
    ctx = world.get_live_context()
    world.set_joint_state(ctx, joints)
    tcp = world.get_ee_pose(ctx).position
    assert (tcp.x, tcp.y, tcp.z) == pytest.approx((0.292, -0.094, 0.407), abs=0.002)
    assert world.check_config_collision_free(joints)
