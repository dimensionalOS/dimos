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

"""Coordinator-import-safe Module Contract for the demo worker.

Keep this module dependency-light. It should define the stable RPC/stream shape
that blueprints and the coordinator need, but avoid importing packages that only
exist in the worker Runtime Project.
"""

from dimos.core.core import rpc
from dimos.core.module import Module


class DemoWorkerModule(Module):
    """Contract imported by the coordinator process."""

    @rpc
    def transform(self, text: str) -> str:
        """Transform text in the worker runtime."""
        raise NotImplementedError("Runtime placement must provide an implementation")

    @rpc
    def runtime_python(self) -> str:
        """Return the Python executable used by the worker runtime."""
        raise NotImplementedError("Runtime placement must provide an implementation")

    @rpc
    def runtime_dependency_label(self) -> str:
        """Return a label formatted with a runtime-only dependency."""
        raise NotImplementedError("Runtime placement must provide an implementation")
