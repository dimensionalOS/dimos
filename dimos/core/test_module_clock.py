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
import time

from dimos.core.module import Module


class _Plain(Module):
    pass


def test_a_module_stamps_with_wall_time_by_default() -> None:
    module = _Plain()
    try:
        before = time.time()
        stamp = module.time()
        after = time.time()
    finally:
        module.stop()

    assert before <= stamp <= after


def test_a_module_sleeps_in_wall_time_by_default() -> None:
    module = _Plain()
    try:
        started = time.monotonic()
        asyncio.run(module.sleep(0.05))
        slept = time.monotonic() - started
    finally:
        module.stop()

    assert slept >= 0.05
