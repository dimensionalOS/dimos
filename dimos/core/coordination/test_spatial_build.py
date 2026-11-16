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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.spatial_module import SpatialModule, SpatialModuleConfig


class OdomConfig(SpatialModuleConfig):
    tf_frames_local: dict[str, str] = {"body": "base_link"}
    tf_frames_global: dict[str, str] = {"world": "world"}


class Odom(SpatialModule, Module):
    config: OdomConfig

    @rpc
    def frames(self) -> tuple[dict[str, str], dict[str, str]]:
        return self.config.tf_frames_local, self.config.tf_frames_global


def test_namespaced_local_frames_are_prefixed_and_global_frames_are_not() -> None:
    blueprint = autoconnect(
        Odom.blueprint().namespace("robot0"),
        Odom.blueprint(tf_frames_local={"body": "torso"}).namespace("inner").namespace("robot1"),
    )
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        assert coordinator.get_instance("robot0/odom").frames() == (
            {"body": "robot0/base_link"},
            {"world": "world"},
        )
        assert coordinator.get_instance("robot1/inner/odom").frames() == (
            {"body": "robot1/inner/torso"},
            {"world": "world"},
        )
    finally:
        coordinator.stop()


def test_unnamespaced_local_frames_are_untouched() -> None:
    blueprint = Odom.blueprint()
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        assert coordinator.get_instance(Odom).frames()[0] == {"body": "base_link"}
    finally:
        coordinator.stop()
