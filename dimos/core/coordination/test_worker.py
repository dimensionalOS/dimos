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

from typing import TYPE_CHECKING

import pytest

from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.core.resource_monitor.stats import WorkerStats


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
def create_worker_manager():
    manager = None

    def _create(n_workers):
        nonlocal manager
        g = GlobalConfig(n_workers=n_workers)
        manager = WorkerManagerPython(g=g)
        manager.start()
        return manager

    yield _create

    if manager is not None:
        manager.stop()


@pytest.mark.slow
def test_worker_manager_basic(create_worker_manager):
    worker_manager = create_worker_manager(n_workers=2)
    module = worker_manager.deploy(SimpleModule, global_config, {})
    module.start()

    result = module.increment()
    assert result == 1

    result = module.increment()
    assert result == 2

    result = module.get_counter()
    assert result == 2

    module.stop()


@pytest.mark.slow
def test_worker_manager_multiple_different_modules(create_worker_manager):
    worker_manager = create_worker_manager(n_workers=2)
    module1 = worker_manager.deploy(SimpleModule, global_config, {})
    module2 = worker_manager.deploy(AnotherModule, global_config, {})

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


@pytest.mark.slow
def test_worker_manager_parallel_deployment(create_worker_manager):
    worker_manager = create_worker_manager(n_workers=2)
    modules = worker_manager.deploy_parallel(
        [
            (SimpleModule, global_config, {}),
            (AnotherModule, global_config, {}),
            (ThirdModule, global_config, {}),
        ],
        {},
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


@pytest.mark.slow
def test_collect_stats(create_worker_manager):
    from dimos.core.resource_monitor.monitor import StatsMonitor

    manager = create_worker_manager(n_workers=2)
    module1 = manager.deploy(SimpleModule, global_config, {})
    module2 = manager.deploy(AnotherModule, global_config, {})
    module1.start()
    module2.start()

    # Use a capturing logger to collect stats via StatsMonitor
    captured: list[list[WorkerStats]] = []

    class CapturingLogger:
        def log_stats(self, coordinator, workers):
            captured.append(workers)

    monitor = StatsMonitor(manager, resource_logger=CapturingLogger(), interval=0.5)
    monitor.start()
    import time

    time.sleep(1.5)
    monitor.stop()

    assert len(captured) >= 1
    stats = captured[-1]
    assert len(stats) == 2

    for s in stats:
        assert s.alive is True
        assert s.pid > 0
        assert s.pss >= 0
        assert s.num_threads >= 1
        assert s.num_fds >= 0
        assert s.io_read_bytes >= 0
        assert s.io_write_bytes >= 0

    # At least one worker should report module names
    all_modules = [name for s in stats for name in s.modules]
    assert "SimpleModule" in all_modules
    assert "AnotherModule" in all_modules

    module1.stop()
    module2.stop()


@pytest.mark.slow
def test_worker_pool_modules_share_workers(create_worker_manager):
    manager = create_worker_manager(n_workers=1)
    module1 = manager.deploy(SimpleModule, global_config, {})
    module2 = manager.deploy(AnotherModule, global_config, {})

    module1.start()
    module2.start()

    # Verify isolated state
    module1.increment()
    module1.increment()
    module2.add(10)

    assert module1.get_counter() == 2
    assert module2.get_value() == 110

    # Verify only 1 worker process was used
    assert len(manager._workers) == 1
    assert manager._workers[0].module_count == 2

    module1.stop()
    module2.stop()


@pytest.fixture
def manager_and_modules():
    """Fixture that tracks deployed modules and stops them on teardown."""
    manager = None
    modules = []

    def _create(n_workers):
        nonlocal manager
        g = GlobalConfig(n_workers=n_workers)
        manager = WorkerManagerPython(g=g)
        manager.start()
        return manager, modules

    yield _create

    for m in reversed(modules):
        m.stop()
    if manager is not None:
        manager.stop()


@pytest.mark.slow
def test_health_check_alive_workers(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=2)
    module = manager.deploy(SimpleModule, global_config, {})
    modules.append(module)
    module.start()

    assert manager.health_check() is True


@pytest.mark.slow
def test_add_workers_grows_pool(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=1)
    manager.add_workers(2)

    assert len(manager._workers) == 3

    # Deploy on the expanded pool and verify it works
    module = manager.deploy(SimpleModule, global_config, {})
    modules.append(module)
    module.start()
    assert module.increment() == 1


@pytest.mark.slow
def test_load_balancing_distributes_modules(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=2)

    for _ in range(4):
        m = manager.deploy(SimpleModule, global_config, {})
        modules.append(m)
        m.start()

    # Each worker should have 2 modules (even distribution)
    counts = [w.module_count for w in manager._workers]
    assert counts == [2, 2]
