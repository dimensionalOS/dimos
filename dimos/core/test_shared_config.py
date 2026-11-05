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

from typing import Any

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule, shared_config


class _Collector(SharedConfigModule):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._received: ConfigSenders = ()
        self._built_after_receive = False

    def config_receive(self, config_senders: ConfigSenders) -> None:
        self._received = config_senders

    @rpc
    def build(self) -> None:
        super().build()
        self._built_after_receive = bool(self._received)

    @rpc
    def received(self) -> tuple[ConfigSenders, bool]:
        return tuple(sorted(self._received)), self._built_after_receive


class _OtherCollector(SharedConfigModule):
    pass


class _SensorConfig(ModuleConfig):
    fov: float = 90.0


class _Sensor(Module):
    config: _SensorConfig

    @shared_config(_Collector)
    def thing(self) -> dict[str, Any]:
        return {"fov": self.config.fov}

    @shared_config(_OtherCollector)
    def not_for_collector(self) -> dict[str, Any]:
        return {"wrong": True}


def test_receiver_gets_every_sender_before_build() -> None:
    blueprint = autoconnect(
        _Sensor.blueprint(fov=60.0),
        _Sensor.blueprint(instance_name="rear"),
        _Collector.blueprint(),
    )
    # A CLI-style override reaches the sender's config, so it reaches the receiver too.
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={}, overrides={"g": {"viewer": "none"}, "rear": {"fov": 120.0}}
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        received, built_after_receive = coordinator.get_instance(_Collector).received()
        assert received == (("_sensor", {"fov": 60.0}), ("rear", {"fov": 120.0}))
        assert built_after_receive
    finally:
        coordinator.stop()
