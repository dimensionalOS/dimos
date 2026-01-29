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

import pytest

from dimos.core import In, Module, Out, rpc
from dimos.core.worker_manager import WorkerManager
from dimos.msgs.geometry_msgs import Vector3


class SimpleModule(Module):
    output: Out[Vector3]
    input: In[Vector3]

    counter: int = 0

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def increment(self) -> int:
        self.counter += 1
        return self.counter

    @rpc
    def get_counter(self) -> int:
        return self.counter


class AnotherModule(Module):
    value: int = 100

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def add(self, n: int) -> int:
        self.value += n
        return self.value

    @rpc
    def get_value(self) -> int:
        return self.value


class ThirdModule(Module):
    multiplier: int = 1

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def multiply(self, n: int) -> int:
        self.multiplier *= n
        return self.multiplier

    @rpc
    def get_multiplier(self) -> int:
        return self.multiplier


@pytest.fixture
def worker_manager():
    manager = WorkerManager()
    try:
        yield manager
    finally:
        manager.close_all()


@pytest.mark.integration
def test_worker_manager_basic(worker_manager):
    module = worker_manager.deploy(SimpleModule)
    module.start()

    result = module.increment()
    assert result == 1

    result = module.increment()
    assert result == 2

    result = module.get_counter()
    assert result == 2

    module.stop()


@pytest.mark.integration
def test_worker_manager_multiple_different_modules(worker_manager):
    module1 = worker_manager.deploy(SimpleModule)
    module2 = worker_manager.deploy(AnotherModule)

    module1.start()
    module2.start()

    # Each module has its own state
    module1.increment()
    module1.increment()
    module2.add(10)

    assert module1.get_counter() == 2
    assert module2.get_value() == 110

    # Stop modules to clean up threads
    module1.stop()
    module2.stop()


@pytest.mark.integration
def test_worker_manager_parallel_deployment(worker_manager):
    modules = worker_manager.deploy_parallel(
        [
            (SimpleModule, (), {}),
            (AnotherModule, (), {}),
            (ThirdModule, (), {}),
        ]
    )

    assert len(modules) == 3
    module1, module2, module3 = modules

    # Start all modules
    module1.start()
    module2.start()
    module3.start()

    # Each module has its own state
    module1.increment()
    module2.add(50)
    module3.multiply(5)

    assert module1.get_counter() == 1
    assert module2.get_value() == 150
    assert module3.get_multiplier() == 5

    # Stop modules
    module1.stop()
    module2.stop()
    module3.stop()
