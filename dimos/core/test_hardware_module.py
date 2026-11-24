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

from dataclasses import replace

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect, config_defaults
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig


class ArmConfig(HardwareModuleConfig):
    mounts: dict[str, str] = {"wrist": "/arm/link6"}


class Arm(HardwareModule):
    config: ArmConfig


class Camera(HardwareModule):
    @rpc
    def mounted_on(self) -> str | None:
        return self.config.mount


def _verify(blueprint: MetaBlueprint) -> None:
    HardwareModule.blueprint_checks(
        [
            replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
            for atom in blueprint.solidify().active_blueprints
        ]
    )


def test_mount_namespaces_under_the_mount_name() -> None:
    (atom,) = Camera.mount("wrist").active_blueprints
    assert atom.name == "wrist/camera"
    assert atom.kwargs["_mount"] == "wrist"


def test_one_unmounted_with_mounted_hardware_passes() -> None:
    _verify(autoconnect(Arm.blueprint(), Camera.mount("wrist")))


def test_two_unmounted_in_one_namespace_is_refused() -> None:
    with pytest.raises(ValueError, match=r"the top level: arm, camera"):
        _verify(autoconnect(Arm.blueprint(), Camera.blueprint()))


def test_each_namespace_gets_its_own_unmounted_hardware() -> None:
    robot = autoconnect(Arm.blueprint(), Camera.mount("wrist"))
    _verify(autoconnect(robot.namespace("left"), robot.namespace("right")))
    with pytest.raises(ValueError, match=r"left: left/arm, left/camera"):
        _verify(autoconnect(Arm.blueprint(), Camera.blueprint()).namespace("left"))


def test_build_passes_the_mount_to_the_module_and_refuses_two_unmounted() -> None:
    blueprint = autoconnect(Arm.blueprint(), Camera.mount("wrist")).solidify()
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        assert coordinator.get_instance("wrist/camera").mounted_on() == "wrist"
    finally:
        coordinator.stop()

    blueprint = autoconnect(Arm.blueprint(), Camera.blueprint()).solidify()
    with pytest.raises(ValueError, match="more than one unmounted HardwareModule"):
        ModuleCoordinator.build(
            blueprint,
            BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}}),
        )
