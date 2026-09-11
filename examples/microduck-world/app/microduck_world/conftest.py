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

"""Hermetic module fixtures: replace only the external RPC boundary."""

from unittest.mock import Mock

import pytest
from dimos.protocol.rpc.zenohrpc import ZenohRPC


@pytest.fixture
def module_factory(monkeypatch):
    for method in ("__init__", "start", "serve_module_rpc", "stop"):
        monkeypatch.setattr(ZenohRPC, method, Mock(return_value=None))
    modules = []

    def create(cls, **kwargs):
        module = cls(rpc_transport=ZenohRPC, **kwargs)
        modules.append(module)
        return module

    try:
        yield create
    finally:
        for module in reversed(modules):
            module.stop()
