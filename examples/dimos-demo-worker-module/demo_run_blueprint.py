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

"""Run the demo Runtime Project blueprint from the main DimOS environment.

This script is intentionally outside ``src/``: run it with the repository's main
environment, and the blueprint will reconcile/spawn the module inside this
example project's locked Runtime Project environment.

Run from an environment where this example package is installed, or set
``PYTHONPATH=examples/dimos-demo-worker-module/src:.``.
"""

from __future__ import annotations

from pathlib import Path
import sys

from dimos_demo_worker_module.blueprint import demo_worker_runtime_blueprint
from dimos_demo_worker_module.contract import DemoWorkerModule

from dimos.core.coordination.module_coordinator import ModuleCoordinator

EXAMPLE_ROOT = Path(__file__).resolve().parent


def main() -> None:
    """Build the blueprint, call the runtime worker, and stop cleanly."""

    print(f"main python: {sys.executable}")
    print(f"runtime project: {EXAMPLE_ROOT}")
    print("building blueprint; this runs locked runtime reconciliation first...")

    coordinator = ModuleCoordinator.build(demo_worker_runtime_blueprint, {})
    try:
        module = coordinator.get_instance(DemoWorkerModule)
        print(f"worker python: {module.runtime_python()}")
        print(f"runtime-only dependency: {module.runtime_dependency_label()}")
        print(f"worker result: {module.transform(' hello from main venv ')}")
    finally:
        coordinator.stop()


if __name__ == "__main__":
    main()
