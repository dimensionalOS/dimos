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

"""Start a separate DimOS world for launch validation, without replacing the live daemon."""

import signal
import threading

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from microduck_world.blueprints import cockpit_world


def main() -> None:
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    coordinator = ModuleCoordinator.build(cockpit_world)
    try:
        stopped.wait()
    finally:
        coordinator.stop()


if __name__ == "__main__":
    main()
