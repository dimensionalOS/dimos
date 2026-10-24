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

import importlib

import pytest

from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.all_blueprints import all_blueprints


@pytest.mark.parametrize("name", sorted(all_blueprints))
def test_blueprint_roundtrips_through_the_codec(name: str) -> None:
    module_path, attr = all_blueprints[name].split(":")
    try:
        blueprint = getattr(importlib.import_module(module_path), attr)
    except Exception as error:  # an optional dependency or robot setting this machine lacks
        pytest.skip(f"{name} is not importable here: {error}")

    restored = decode(encode(blueprint))

    assert [atom.module for atom in restored.blueprints] == [
        atom.module for atom in blueprint.blueprints
    ]
    assert restored.transport_map.keys() == blueprint.transport_map.keys()
