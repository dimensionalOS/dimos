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

import threading

import pytest

from dimos.core.tests.stress_test_module import StressTestModule
from dimos.porcelain.dimos import Dimos


def _join_daemon_threads(timeout: float = 2.0) -> None:
    """Join any daemon threads spawned by event-loop threads."""
    for t in threading.enumerate():
        if t.daemon and "event-loop" in t.name:
            t.join(timeout=timeout)


@pytest.fixture
def app():
    instance = Dimos()
    try:
        yield instance
    finally:
        instance.stop()
        _join_daemon_threads()


@pytest.fixture
def running_app():
    instance = Dimos(n_workers=1)
    instance.run(StressTestModule)
    try:
        yield instance
    finally:
        instance.stop()
        _join_daemon_threads()


@pytest.fixture
def client(running_app):
    port = running_app._coordinator.start_rpyc_service()
    instance = Dimos.connect(host="localhost", port=port)
    try:
        yield instance
    finally:
        instance.stop()
