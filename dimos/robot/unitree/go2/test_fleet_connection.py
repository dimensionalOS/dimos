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

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.go2.fleet_connection import FleetConnectionConfig, Go2FleetConnection


@pytest.mark.parametrize("robot_ips", [None, "", "   ", " , , "])
def test_missing_global_ips_give_configuration_guidance(robot_ips) -> None:
    with pytest.raises(ValueError, match="ROBOT_IPS.*--robot-ips"):
        FleetConnectionConfig(g=GlobalConfig(robot_ip=None, robot_ips=robot_ips))


@pytest.mark.parametrize("ips", [[], [""], [" ", ""]])
def test_empty_scoped_ips_give_configuration_guidance(ips) -> None:
    with pytest.raises(BlueprintConfigError, match="go2fleetconnection:.*--go2fleetconnection.ips"):
        BlueprintConfigParser(Go2FleetConnection.blueprint(ips=ips)).parse(environ={})


@pytest.mark.parametrize(
    ("cli", "environ", "pinned"),
    [
        (["--robot-ips", " 192.0.2.1, ,192.0.2.2, "], {}, {}),
        ([], {"ROBOT_IPS": " 192.0.2.1, ,192.0.2.2, "}, {}),
        ([], {}, {"ips": [" 192.0.2.1", "", "192.0.2.2 "]}),
        (["--go2fleetconnection.ips", '[" 192.0.2.1", "", "192.0.2.2 "]'], {}, {}),
    ],
)
def test_fleet_resolves_and_normalizes_ips_before_validating_primary(cli, environ, pinned) -> None:
    parsed = BlueprintConfigParser(Go2FleetConnection.blueprint(**pinned)).parse(
        cli, environ=environ
    )
    cfg = FleetConnectionConfig(
        g=GlobalConfig(**parsed.global_config_values()),
        **parsed.module_kwargs("go2fleetconnection"),
    )

    assert cfg.ip == "192.0.2.1"
    assert cfg.ips == ["192.0.2.1", "192.0.2.2"]


def test_fleet_preserves_explicit_primary_ip() -> None:
    cfg = FleetConnectionConfig(
        ip="192.0.2.3",
        ips=["192.0.2.1", "192.0.2.2"],
        g=GlobalConfig(robot_ip=None, replay=False, simulation=""),
    )
    assert cfg.ip == "192.0.2.3"


@pytest.mark.parametrize("ip", ["", "   "])
def test_fleet_rejects_blank_explicit_primary_ip(ip) -> None:
    with pytest.raises(ValueError, match="No robot IP specified"):
        FleetConnectionConfig(ip=ip, ips=["192.0.2.1"], g=GlobalConfig(replay=False, simulation=""))
