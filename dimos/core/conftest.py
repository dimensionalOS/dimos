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

from collections.abc import Iterator

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.protocol.service.zenohservice import ZenohPeerSeed


@pytest.fixture(params=["lcm", "zenoh"])
def each_transport(request, monkeypatch) -> Iterator[str]:
    """Run each backend against an isolated, already-reachable test fabric."""
    monkeypatch.setattr(global_config, "transport", request.param)
    if request.param != "zenoh":
        yield request.param
        return

    seed = ZenohPeerSeed(global_config)
    seed.start()
    try:
        yield request.param
    finally:
        seed.stop()


@pytest.fixture
def dimos():
    client = ModuleCoordinator()
    client.start()
    try:
        yield client
    finally:
        client.stop()
