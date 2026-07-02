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

"""Runtime-wiring tool for StreamModule, run on demand (not collected by the suite).

Reuses ``module_cases`` (the pipeline-style module classes) from the sibling
``test_module`` so the tool and the unit tests exercise the same modules.
"""

from __future__ import annotations

import threading

import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core.transport import pLCMTransport
from dimos.memory2.module import StreamModule
from dimos.memory2.test_module import module_cases


def _reset_thread_pool() -> None:
    """Shut down and replace the global RxPY thread pool so conftest thread-leak check passes."""
    import dimos.utils.threadpool as tp

    tp.scheduler.executor.shutdown(wait=True)
    tp.scheduler = ThreadPoolScheduler(max_workers=tp.get_max_workers())


@pytest.mark.parametrize("module_cls", module_cases)
def test_e2e_runtime_wiring(module_cls: type[StreamModule]) -> None:
    """Push data into In port, assert doubled data arrives on Out port."""
    module = module_cls()
    module.numbers.transport = pLCMTransport("/test/numbers")
    module.doubled.transport = pLCMTransport("/test/doubled")

    received: list[int] = []
    done = threading.Event()

    unsub = module.doubled.subscribe(lambda msg: (received.append(msg), done.set()))

    module.start()
    try:
        module.numbers.transport.publish(42)
        assert done.wait(timeout=5.0), f"Timed out, received={received}"
        assert received == [84]
    finally:
        unsub()
        module.stop()
        _reset_thread_pool()
        _reset_thread_pool()
