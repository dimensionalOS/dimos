# Copyright 2025 Dimensional Inc.
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

"""Module introspection and rendering.

Renderers:
    - ansi: ANSI terminal output (default)
    - dot: Graphviz DOT format
"""

from dimos.core.introspection.module import ansi, dot
from dimos.core.introspection.module.info import (
    INTERNAL_RPCS,
    ModuleInfo,
    ParamInfo,
    RpcInfo,
    SkillInfo,
    StreamInfo,
    extract_module_info,
)
from dimos.core.introspection.module.render import render_module_io

__all__ = [
    "INTERNAL_RPCS",
    "ModuleInfo",
    "ParamInfo",
    "RpcInfo",
    "SkillInfo",
    "StreamInfo",
    "ansi",
    "dot",
    "extract_module_info",
    "render_module_io",
]
