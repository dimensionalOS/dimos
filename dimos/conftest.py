# Copyright 2025 Dimensional Inc.
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
import threading
import pytest


@pytest.fixture
def event_loop():
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


_seen_threads = set()
_seen_threads_lock = threading.RLock()

_skip_for = ["lcm", "heavy", "ros"]


@pytest.fixture(autouse=True)
def monitor_threads(request):
    # Skip monitoring for tests marked with specified markers
    if any(request.node.get_closest_marker(marker) for marker in _skip_for):
        yield
        return

    yield

    threads = [t for t in threading.enumerate() if t.name != "MainThread"]

    if not threads:
        return

    with _seen_threads_lock:
        new_leaks = [t for t in threads if t.ident not in _seen_threads]
        for t in threads:
            _seen_threads.add(t.ident)

    if not new_leaks:
        return

    thread_names = [t.name for f in new_leaks]

    pytest.fail(
        f"Non-closed threads before or during this test. The thread names: {thread_names}. "
        "Please look at the first test that fails and fix that."
    )
