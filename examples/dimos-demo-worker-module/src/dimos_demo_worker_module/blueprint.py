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

"""Blueprint snippet demonstrating Runtime Project registration and placement."""

import os
from pathlib import Path
import sys

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment, RuntimePlacement
from dimos_demo_worker_module.contract import DemoWorkerModule


PROJECT_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = PROJECT_ROOT.parents[1]
EXAMPLE_SRC = PROJECT_ROOT / "src"


def _worker_pythonpath() -> str:
    paths = [str(EXAMPLE_SRC), str(REPO_ROOT)]
    paths.extend(path for path in sys.path if path)
    if existing := os.environ.get("PYTHONPATH"):
        paths.append(existing)
    return os.pathsep.join(paths)

demo_worker_runtime = PythonProjectRuntimeEnvironment(
    name="demo-worker-runtime",
    project=PROJECT_ROOT,
    env={"PYTHONPATH": _worker_pythonpath()},
)

demo_worker_runtime_blueprint = (
    autoconnect(DemoWorkerModule.blueprint())
    .runtime_environments(demo_worker_runtime)
    .runtime_placements(
        {
            DemoWorkerModule: RuntimePlacement(
                runtime="demo-worker-runtime",
                implementation="dimos_demo_worker_module.runtime.DemoWorkerRuntimeModule",
            )
        }
    )
)
