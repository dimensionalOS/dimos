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

from pathlib import Path
import subprocess
import sys

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.experimental.isolated_python.example.contract import ExampleExternal
from dimos.experimental.isolated_python.example.support import Offset
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.utils.testing.waiting import wait_until


class Producer(Module):
    value: Out[Int32]

    @rpc
    def publish(self, value: int) -> None:
        self.value.publish(Int32(value))


class Consumer(Module):
    doubled: In[Int32]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._latest: int | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.doubled.subscribe(self._record)

    def _record(self, message: Int32) -> None:
        self._latest = message.data

    @rpc
    def latest(self) -> int | None:
        return self._latest


def test_isolated_python_rpc_refs_and_restart() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(
            ExampleExternal.blueprint(initial_multiplier=3),
            Offset.blueprint(),
            Producer.blueprint(),
            Consumer.blueprint(),
        )
    )
    try:
        external = coordinator.get_instance(ExampleExternal)
        producer = coordinator.get_instance(Producer)
        consumer = coordinator.get_instance(Consumer)
        assert external.get_multiplier() == 3
        assert external.get_adjusted_multiplier() == 13
        assert external.set_multiplier(5) == "External multiplier set to 5"
        assert external.get_multiplier() == 5
        producer.publish(4)
        wait_until(lambda: consumer.latest() == 20, timeout=5.0)
        assert consumer.latest() == 20

        coordinator.restart_module(ExampleExternal, reload_source=False)
        restarted = coordinator.get_instance(ExampleExternal)
        assert restarted.get_multiplier() == 3
        assert restarted.get_adjusted_multiplier() == 13
    finally:
        coordinator.stop()


def test_example_script_exits_after_printing_results() -> None:
    repository = Path(__file__).parents[3]

    result = subprocess.run(
        [sys.executable, "-m", "dimos.experimental.isolated_python.example.run"],
        cwd=repository,
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "external multiplier: 3" in result.stdout
    assert "adjusted multiplier: 13" in result.stdout
    assert "External multiplier set to 5" in result.stdout
    assert "restarted multiplier: 3" in result.stdout
