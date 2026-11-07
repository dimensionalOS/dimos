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

"""Policy rollout on one Metal: the policy contract, the CLI flags and the Quest takeover."""

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.imitation.policy.module import PolicyModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    DamiaoTeleopCoordinator,
)
from dimos.robot.manipulators.makermods_metal.blueprints.learning_rollout import (
    build_metal_rollout,
    metal_lerobot_rollout,
)
from dimos.robot.manipulators.makermods_metal.blueprints.teleop import METAL_WEBXR_TASK_NAME
from dimos.teleop.webxr.extensions import ArmTeleopModule

_JOINTS = [*[f"arm/joint{i}" for i in range(1, 7)], "arm/joint7"]


def _atom(blueprint: Blueprint, module_type: type) -> BlueprintAtom:
    return next(atom for atom in blueprint.blueprints if issubclass(atom.module, module_type))


def test_default_rollout_is_quest_free() -> None:
    blueprint = build_metal_rollout(
        checkpoint="checkpoint", task="pick up the cube", camera_device="/dev/video3", device="cuda"
    )
    modules = [atom.module for atom in blueprint.active_blueprints]
    coordinator = _atom(blueprint, ControlCoordinator)
    [trajectory] = coordinator.kwargs["tasks"]
    policy = _atom(blueprint, PolicyModule).kwargs

    assert coordinator.module is DamiaoCoordinator
    assert ArmTeleopModule not in modules
    assert ManipulationModule not in modules
    assert (trajectory.type, trajectory.joint_names) == ("trajectory", _JOINTS)
    assert (policy["policy_path"], policy["task"], policy["device"]) == (
        "checkpoint",
        "pick up the cube",
        "cuda",
    )
    assert _atom(blueprint, WebcamModule).kwargs["hardware"].camera_index == "/dev/video3"


def test_quest_rollout_adds_higher_priority_takeover() -> None:
    blueprint = build_metal_rollout(
        checkpoint="checkpoint", task="pick up the cube", quest_control=True
    )
    coordinator = _atom(blueprint, ControlCoordinator)
    tasks = coordinator.kwargs["tasks"]
    [trajectory] = [task for task in tasks if task.type == "trajectory"]
    teleop = next(task for task in tasks if task.name == METAL_WEBXR_TASK_NAME)
    gripper = next(task for task in tasks if task.name == "metal_gripper")

    assert coordinator.module is DamiaoTeleopCoordinator
    assert trajectory.joint_names == _JOINTS
    assert trajectory.priority < teleop.priority
    assert trajectory.priority < gripper.priority
    assert ArmTeleopModule in [atom.module for atom in blueprint.active_blueprints]
    assert _atom(blueprint, ManipulationModule).kwargs["gripper_tasks"] == {"arm": "metal_gripper"}


def test_rollout_takes_checkpoint_task_camera_and_can_port_from_the_cli() -> None:
    parsed = BlueprintConfigParser(metal_lerobot_rollout).parse(
        [
            "--can-port",
            "can0",
            "--policy.policy-path",
            "/checkpoints/metal-act",
            "--policy.task",
            "pick up the cube",
            "--policy.device",
            "cuda",
            "--wristcamera.hardware.camera-index",
            "/dev/video3",
        ],
        environ={},
    )
    policy = parsed.module_kwargs("policy")
    camera = parsed.module_kwargs("WristCamera")["hardware"]

    assert parsed.module_kwargs("metalconnection")["can_port"] == "can0"
    assert (policy["policy_path"], policy["task"], policy["device"]) == (
        "/checkpoints/metal-act",
        "pick up the cube",
        "cuda",
    )
    assert policy["robot_type"] == "makermods_metal"
    assert policy["joint_names"] == _JOINTS
    assert (policy["fps"], policy["image_width"], policy["image_height"]) == (30.0, 640, 480)
    assert (camera["camera_index"], camera["width"], camera["height"], camera["fps"]) == (
        "/dev/video3",
        640,
        480,
        30.0,
    )
