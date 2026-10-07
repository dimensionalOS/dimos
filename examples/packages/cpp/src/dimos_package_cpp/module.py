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

"""Installed cpp native module with typed ports."""

from pathlib import Path

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist


class CppPongConfig(NativeModuleConfig):
    executable: str = str(Path(__file__).resolve().parent / "bin" / "package_pong")
    stdin_config: bool = True
    sample_config: int = 42


class CppPong(NativeModule):
    config: CppPongConfig
    data: In[Twist]
    confirm: Out[Twist]
