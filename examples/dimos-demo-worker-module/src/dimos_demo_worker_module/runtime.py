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

"""Worker-only Runtime Implementation for the demo module."""

import sys

from dimos.core.core import rpc
from dimos_demo_worker_module.contract import DemoWorkerModule
from inflection import camelize


class DemoWorkerRuntimeModule(DemoWorkerModule):
    """Concrete implementation loaded inside the selected Python Runtime Project."""

    @rpc
    def transform(self, text: str) -> str:
        """Transform text in the worker runtime."""
        return f"demo-runtime:{text.strip().upper()}"

    @rpc
    def runtime_python(self) -> str:
        """Return the Python executable used by the worker runtime."""
        return sys.executable

    @rpc
    def runtime_dependency_label(self) -> str:
        """Return a label formatted with a runtime-only dependency."""
        return camelize("runtime_dependency")
