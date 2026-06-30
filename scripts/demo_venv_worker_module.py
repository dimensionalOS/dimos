#!/usr/bin/env python3
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

"""Run the demo venv-placed worker module blueprint."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.runtime_environment import PythonVenvRuntimeEnvironment

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO_MODULE_SRC = REPO_ROOT / "packages" / "dimos-demo-worker-module" / "src"


def _prepend_pythonpath(*paths: Path) -> None:
    existing_pythonpath = os.environ.get("PYTHONPATH")
    path_entries = [str(path) for path in paths]
    if existing_pythonpath:
        path_entries.append(existing_pythonpath)
    os.environ["PYTHONPATH"] = os.pathsep.join(path_entries)
    for path in reversed(paths):
        sys.path.insert(0, str(path))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "message", nargs="?", default="ok", help="Message to send through the demo RPC"
    )
    args = parser.parse_args()

    _prepend_pythonpath(DEMO_MODULE_SRC)

    from dimos_demo_worker_module import VENV_DEMO_ENV_NAME, VenvDemoConsumer, venv_demo_blueprint

    runtime = PythonVenvRuntimeEnvironment(
        name=VENV_DEMO_ENV_NAME,
        python_executable=Path(sys.executable),
        env={"PYTHONPATH": str(DEMO_MODULE_SRC)},
    )

    coordinator = ModuleCoordinator.build(
        venv_demo_blueprint(runtime),
        {"g": {"viewer": "none", "n_workers": 1}},
    )
    try:
        consumer = coordinator.get_instance(VenvDemoConsumer)
        if consumer is None:
            raise RuntimeError("VenvDemoConsumer was not deployed")
        print(consumer.consume_message(args.message))
    finally:
        coordinator.stop()


if __name__ == "__main__":
    main()
