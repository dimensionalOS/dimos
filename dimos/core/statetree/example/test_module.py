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

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.statetree.example.module import DeliveryModule
from dimos.core.statetree.runtime import Status

FAST = {"speed": 2.0, "clearance_wait": 0.1, "tick_rate": 100.0}


@pytest.fixture
def local_delivery():
    module = DeliveryModule(**FAST)
    module.start()
    yield module
    module.stop()


@pytest.fixture
def endless_delivery():
    module = DeliveryModule(**FAST, route_length=1e9)
    module.start()
    yield module
    module.stop()


@pytest.fixture
def deployed_delivery():
    coordinator = ModuleCoordinator.build(DeliveryModule.blueprint(**FAST))
    yield coordinator.get_instance(DeliveryModule)
    coordinator.stop()


@pytest.fixture(params=["local_delivery", "deployed_delivery"])
def delivery(request):
    return request.getfixturevalue(request.param)


def test_module_ticks_its_tree_through_an_obstacle_to_success(delivery, wait_until):
    assert delivery.get_state().path == ("delivery", "idle")
    delivery.dispatch()
    wait_until(lambda: delivery.get_state().distance > 0, timeout=5, interval=0.01)
    delivery.report_obstacle()
    wait_until(lambda: delivery.get_state().status is Status.SUCCEEDED, timeout=5, interval=0.01)
    state = delivery.get_state()
    assert state.path == ()
    assert state.distance >= 2.0
    assert state.commanded_speed == 0.0
    assert state.trace == (
        "drive.enter",
        "drive.exit",
        "wait.enter",
        "wait.exit",
        "drive.enter",
        "drive.exit",
    )


def test_stopping_the_module_exits_the_running_task(endless_delivery, wait_until):
    endless_delivery.dispatch()
    wait_until(
        lambda: endless_delivery.get_state().commanded_speed == 2.0, timeout=5, interval=0.01
    )
    endless_delivery.stop()
    assert endless_delivery.commanded_speed == 0.0
    assert endless_delivery.trace == ["drive.enter", "drive.exit"]
