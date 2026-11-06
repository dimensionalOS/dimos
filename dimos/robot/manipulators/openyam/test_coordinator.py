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


import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.robot.manipulators.openyam.blueprints.learning_collection import openyam_teach_collection
from dimos.robot.manipulators.openyam.blueprints.learning_quest_collection import (
    openyam_quest_collection,
)
from dimos.robot.manipulators.openyam.blueprints.learning_rollout import build_openyam_rollout
from dimos.robot.manipulators.openyam.coordinator import (
    OpenYamCoordinator,
    OpenYamTeleopCoordinator,
)


@pytest.mark.parametrize(
    "blueprint",
    [
        openyam_teach_collection,
        openyam_quest_collection,
        build_openyam_rollout(),
        build_openyam_rollout(quest_control=True),
    ],
)
async def test_can_override_reaches_hardware_without_changing_teaching_gains(
    blueprint, mocker, tmp_path
):
    flags = ["--controlcoordinator.can-port", "follower_l"]
    if blueprint in (openyam_teach_collection, openyam_quest_collection):
        flags.extend(["--recorder.recording", str(tmp_path / "session"), "--episodes.task", "pick"])
    else:
        flags.extend(["--policy.policy-path", "checkpoint", "--policy.task", "pick"])
    parsed = BlueprintConfigParser(blueprint).parse(flags, environ={})
    atom = next(
        atom
        for atom in blueprint.active_blueprints
        if issubclass(atom.module, (OpenYamCoordinator, OpenYamTeleopCoordinator))
    )
    kwargs = {**atom.kwargs, **parsed.module_kwargs("ControlCoordinator")}
    original = atom.kwargs["hardware"][0]
    runtime = original.adapter_kwargs["runtime_config"]
    setup = mocker.patch.object(ControlCoordinator, "_setup_from_config")
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    coordinator = atom.module(**kwargs)
    try:
        coordinator._setup_from_config()
        configured = coordinator.config.hardware[0]
        assert configured.adapter_kwargs["runtime_config"].bus_devices == {"openyam": "follower_l"}
        assert (
            configured.adapter_kwargs["runtime_config"].passive_grippers == runtime.passive_grippers
        )
        assert configured.adapter_kwargs["runtime_config"].gravity_comp is runtime.gravity_comp
        assert configured.wb_config == original.wb_config
        if blueprint is openyam_teach_collection:
            assert configured.wb_config.kp == (0.0,) * 7
            assert configured.wb_config.kd == (2.0, 2.0, 2.0, 0.5, 0.5, 0.5, 0.0)
            assert configured.adapter_kwargs["runtime_config"].passive_grippers == ("gripper",)
        setup.assert_called_once_with()
    finally:
        coordinator.stop()
