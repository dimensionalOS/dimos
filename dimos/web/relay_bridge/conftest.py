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

"""Fixtures shared by the RelayBridgeModule unit-test files."""

import pytest

from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.module_test_support import make_bridge


@pytest.fixture
def bridge(monkeypatch):
    module, clients = make_bridge(monkeypatch)
    try:
        yield module, clients
    finally:
        stop_module(module)
