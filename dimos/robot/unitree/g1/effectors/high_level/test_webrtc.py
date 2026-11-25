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
from dimos.robot.unitree.g1.effectors.high_level.webrtc import (
    G1HighLevelWebRtc,
    G1HighLevelWebRtcConfig,
)


@pytest.mark.parametrize("ip", [None, "", "   "])
def test_missing_ip_names_the_scoped_setting(ip) -> None:
    with pytest.raises(ValueError, match="G1HIGHLEVELWEBRTC__IP.*--g1highlevelwebrtc.ip"):
        G1HighLevelWebRtcConfig(ip=ip)


def test_scoped_ip_reaches_high_level_config() -> None:
    parsed = BlueprintConfigParser(G1HighLevelWebRtc.blueprint()).parse(
        ["--g1highlevelwebrtc.ip", "192.0.2.1"], environ={}
    )
    assert parsed.module_kwargs("g1highlevelwebrtc")["ip"] == "192.0.2.1"
