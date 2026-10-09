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
from dimos.core.module import Module, ModuleConfig
from dimos.hosted.fragment import with_host_config


class MountConfig(ModuleConfig):
    mount: str = "SF"
    iface: str = "eth0"


class Mounted(Module):
    config: MountConfig


def test_host_env_beats_controller_only_where_it_sets_something() -> None:
    blueprint = Mounted.blueprint()
    controller = BlueprintConfigParser(blueprint).parse(environ={"MOUNTED__IFACE": "wlan0"})
    merged = with_host_config(
        blueprint, controller, environ={"MOUNTED__MOUNT": "ATHENS", "VIEWER": "none"}
    )
    assert merged.module_kwargs("mounted")["mount"] == "ATHENS"
    assert merged.module_kwargs("mounted")["iface"] == "wlan0"
    assert merged.global_config["viewer"] == "none"
