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

"""Every built-in blueprint must survive the RPC codec.

`dimos shell` sends whole Blueprints to the daemon and the coordinator sends each
module's kwargs into its worker, so every config value a blueprint holds has to be
encodable (see dimos/protocol/encode/cbor.py).
"""

import pytest

from dimos.core.global_config import global_config
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.get_all_blueprints import OptionalDependencyError, load_blueprint
from dimos.robot.test_all_blueprints import SELF_HOSTED_BLUEPRINTS


@pytest.mark.parametrize(
    "name",
    [
        pytest.param(name, marks=pytest.mark.self_hosted)
        if name in SELF_HOSTED_BLUEPRINTS
        else name
        for name in sorted(all_blueprints)
    ],
)
def test_blueprint_roundtrips_through_the_codec(name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    # The multi-robot blueprints read ROBOT_IPS at import time.
    monkeypatch.setattr(global_config, "robot_ips", "192.0.2.10,192.0.2.11")
    try:
        blueprint = load_blueprint(name)
    except OptionalDependencyError as error:
        pytest.skip(f"{name} is not importable here: {error}")

    restored = decode(encode(blueprint))

    assert [atom.module for atom in restored.blueprints] == [
        atom.module for atom in blueprint.blueprints
    ]
    assert restored.transport_map.keys() == blueprint.transport_map.keys()
