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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from examples.external_python_module.contract import ExampleExternal
from examples.external_python_module.support import Offset


def run_example() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(ExampleExternal.blueprint(initial_multiplier=3), Offset.blueprint())
    )
    try:
        external = coordinator.get_instance(ExampleExternal)
        print("external multiplier:", external.get_multiplier())
        print("adjusted multiplier:", external.get_adjusted_multiplier())
        print(external.set_multiplier(5))
        coordinator.restart_module(ExampleExternal, reload_source=False)
        restarted = coordinator.get_instance(ExampleExternal)
        print("restarted multiplier:", restarted.get_multiplier())
    finally:
        coordinator.stop()


if __name__ == "__main__":
    run_example()
