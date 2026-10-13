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

"""Lightweight host declarations for ordinary and isolated Python consumers."""

import json
import os
from pathlib import Path
import sys

import packaging

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.experimental.isolated_python.module import IsolatedPythonModule
from dimos.experimental.isolated_python.package import PackageProject
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def record(message: Twist, kind: str) -> None:
    logger.info("Python package received Twist", kind=kind, packaging=packaging.__version__)
    directory = os.environ.get("DIMOS_PACKAGE_REPORT_DIR")
    if directory:
        path = Path(directory) / f"{kind}.json"
        path.write_text(
            json.dumps(
                {
                    "kind": kind,
                    "packaging": packaging.__version__,
                    "pid": os.getpid(),
                    "value": message.linear.x,
                    "python": sys.executable,
                }
            )
        )


class PythonObserver(Module):
    data: In[Twist]

    @rpc
    def start(self) -> None:
        super().start()
        self.data.subscribe(self._record)

    def _record(self, message: Twist) -> None:
        record(message, "python")


class IsolatedObserver(IsolatedPythonModule):
    data: In[Twist]
    package_project = PackageProject("dimos_package_python", "runtime", "dimos-package-python")
    implementation = "runtime:ObserverRuntime"
