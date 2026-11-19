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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.g1.connection import G1Config, G1Connection


@pytest.mark.parametrize("ip", [None, "", "   "])
def test_g1_rejects_missing_hardware_ip_without_suggesting_unsupported_modes(ip) -> None:
    with pytest.raises(ValueError, match="ROBOT_IP.*--g1connection.ip") as error:
        G1Config(g=GlobalConfig(robot_ip=ip, replay=False, simulation=""))
    assert "--replay" not in str(error.value)
    assert "--simulation" not in str(error.value)


@pytest.mark.parametrize("option", ["--robot-ip", "--g1connection.ip"])
def test_g1_accepts_global_and_scoped_ip_overrides(option) -> None:
    parsed = BlueprintConfigParser(G1Connection.blueprint()).parse(
        [option, "192.0.2.1"], environ={}
    )
    cfg = G1Config(
        g=GlobalConfig(**parsed.global_config_values()), **parsed.module_kwargs("g1connection")
    )
    assert cfg.ip == "192.0.2.1"
