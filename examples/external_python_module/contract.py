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

"""Host-side declaration contract for the external Python example."""

from __future__ import annotations

from typing import Protocol

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.external_python_module import ExternalPythonModule
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.spec.utils import Spec


class Config(ModuleConfig):
    """Configuration declared and owned by the external module contract."""

    initial_multiplier: int = 2


class ExampleExternal(ExternalPythonModule):
    """Declaration implemented by the isolated sibling runtime project."""

    implementation = "example_external.runtime:ExampleExternalRuntime"
    config: Config

    value: In[Int32]
    doubled: Out[Int32]

    @rpc
    def get_multiplier(self) -> int:
        """Return the multiplier used by the external implementation."""
        raise NotImplementedError

    @skill
    def set_multiplier(self, multiplier: int) -> str:
        """Set the multiplier used for values received on ``value``."""
        raise NotImplementedError


class ExampleExternalSpec(Spec, Protocol):
    """RPC contract consumed by a regular DimOS module."""

    @rpc
    def get_multiplier(self) -> int: ...
