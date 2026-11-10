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

"""Run with python -m dimos.core.statetree.example.demo_statetree."""

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.statetree.example.module import DeliveryModule
from dimos.core.statetree.runtime import Status
from dimos.utils.testing.waiting import wait_until


def run_example() -> None:
    coordinator = ModuleCoordinator.build(DeliveryModule.blueprint())
    try:
        delivery = coordinator.get_instance(DeliveryModule)
        print(delivery.get_state())
        delivery.dispatch()
        wait_until(lambda: delivery.get_state().distance > 0.5, timeout=10)
        delivery.report_obstacle()
        wait_until(lambda: delivery.get_state().path == ("delivery", "wait"), timeout=10)
        print(delivery.get_state())
        wait_until(lambda: delivery.get_state().status is Status.SUCCEEDED, timeout=10)
        print(delivery.get_state())
    finally:
        coordinator.stop()


if __name__ == "__main__":
    run_example()
