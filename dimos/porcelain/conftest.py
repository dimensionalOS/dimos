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

from __future__ import annotations

import pytest

from dimos.core.tests.stress_test_module import StressTestModule
from dimos.porcelain.dimos import Dimos


@pytest.fixture
def app():
    instance = Dimos()
    try:
        yield instance
    finally:
        instance.stop()


@pytest.fixture
def running_app():
    instance = Dimos(n_workers=1)
    instance.run(StressTestModule)
    try:
        yield instance
    finally:
        instance.stop()


@pytest.fixture
def client(running_app):
    port = running_app._coordinator.start_rpyc_service()
    instance = Dimos.connect(host="localhost", port=port)
    try:
        yield instance
    finally:
        instance.stop()
