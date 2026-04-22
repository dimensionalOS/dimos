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

"""Grid tests for StreamModule — same e2e logic across all pipeline styles."""

from __future__ import annotations

from collections.abc import Iterator
import threading

import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import pLCMTransport
from dimos.memory2.module import StreamModule
from dimos.memory2.stream import Stream
from dimos.memory2.transform import Transformer
from dimos.memory2.type.observation import Observation

# -- Shared transformer ---------------------------------------------------


class Double(Transformer[int, int]):
    def __init__(self, factor: int = 2) -> None:
        self.factor = factor

    def __call__(self, upstream: Iterator[Observation[int]]) -> Iterator[Observation[int]]:
        for obs in upstream:
            yield obs.derive(data=obs.data * self.factor)


# -- Pipeline styles -------------------------------------------------------


class StaticStreamModule(StreamModule):
    """Pipeline as a static Stream chain on the class."""

    pipeline = Stream().transform(Double())
    numbers: In[int]
    doubled: Out[int]


class StaticTransformerModule(StreamModule):
    """Pipeline as a bare Transformer on the class."""

    pipeline = Double()
    numbers: In[int]
    doubled: Out[int]


class MethodPipelineConfig(ModuleConfig):
    factor: int = 2


class MethodPipelineModule(StreamModule):
    """Pipeline as a method with access to self.config."""

    config: MethodPipelineConfig

    def pipeline(self, stream: Stream) -> Stream:
        return stream.transform(Double(factor=self.config.factor))

    numbers: In[int]
    doubled: Out[int]


# -- Grid ------------------------------------------------------------------

module_cases = [
    pytest.param(StaticStreamModule, id="static-stream"),
    pytest.param(StaticTransformerModule, id="static-transformer"),
    pytest.param(MethodPipelineModule, id="method-pipeline"),
]


@pytest.mark.parametrize("module_cls", module_cases)
def test_blueprint_ports(module_cls: type[StreamModule]) -> None:
    """All pipeline styles produce a blueprint with the correct In/Out ports."""
    bp = module_cls.blueprint()

    assert len(bp.blueprints) == 1
    atom = bp.blueprints[0]
    stream_names = {s.name for s in atom.streams}
    assert "numbers" in stream_names
    assert "doubled" in stream_names


def _reset_thread_pool() -> None:
    """Shut down and replace the global RxPY thread pool so conftest thread-leak check passes."""
    import dimos.utils.threadpool as tp

    tp.scheduler.executor.shutdown(wait=True)
    tp.scheduler = ThreadPoolScheduler(max_workers=tp.get_max_workers())


@pytest.mark.tool
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
