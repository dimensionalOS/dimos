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

import asyncio
from collections.abc import Iterator
import time

import pytest

from dimos.core.module import Module


class _Clocked(Module):
    pass


@pytest.fixture
def module() -> Iterator[_Clocked]:
    clocked = _Clocked()
    yield clocked
    clocked.stop()


def test_module_time_is_wall_time(module: _Clocked) -> None:
    before = time.time()
    now = module.time()
    assert before <= now <= time.time()


def test_module_sleep_waits_its_time(module: _Clocked) -> None:
    start = module.time()
    asyncio.run(module.sleep(0.05))
    assert module.time() - start >= 0.05
