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

"""Host-visible contract for the isolated Python example."""

from typing import Protocol

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.spec.utils import Spec


class OffsetSpec(Spec, Protocol):
    def get_offset(self) -> int: ...


class Config(IsolatedPythonModuleConfig):
    initial_multiplier: int = 2


class ExampleExternal(IsolatedPythonModule):
    """Multiply incoming integers in an isolated Python environment."""

    implementation = "example_external.runtime:ExampleExternalRuntime"
    config: Config

    _offset: OffsetSpec
    value: In[Int32]
    doubled: Out[Int32]

    @rpc
    def get_multiplier(self) -> int:
        """Return the current multiplier."""
        raise NotImplementedError

    @rpc
    def get_adjusted_multiplier(self) -> int:
        """Return the current multiplier plus the injected offset."""
        raise NotImplementedError

    @skill
    def set_multiplier(self, multiplier: int) -> str:
        """Set the multiplier used for incoming values."""
        raise NotImplementedError
