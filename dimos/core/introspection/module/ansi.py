# Copyright 2025-2026 Dimensional Inc.
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

"""ANSI terminal renderer for module IO diagrams."""

from dimos.core import colors
from dimos.core.introspection.module.info import (
    ModuleInfo,
    ParamInfo,
    RpcInfo,
    SkillInfo,
    StreamInfo,
)


def render(info: ModuleInfo, color: bool = True) -> str:
    """Render module info as an ANSI terminal diagram.

    Args:
        info: ModuleInfo structure to render.
        color: Whether to include ANSI color codes.

    Returns:
        ASCII/Unicode diagram with optional ANSI colors.
    """
    # Color functions that become identity when color=False
    _green = colors.green if color else (lambda x: x)
    _blue = colors.blue if color else (lambda x: x)
    _yellow = colors.yellow if color else (lambda x: x)
    _cyan = colors.cyan if color else (lambda x: x)

    def _box(name: str) -> list[str]:
        return [
            "┌┴" + "─" * (len(name) + 1) + "┐",
            f"│ {name} │",
            "└┬" + "─" * (len(name) + 1) + "┘",
        ]

    def format_stream(stream: StreamInfo) -> str:
        return f"{_yellow(stream.name)}: {_green(stream.type_name)}"

    def format_param(param: ParamInfo) -> str:
        result = param.name
        if param.type_name:
            result += ": " + _green(param.type_name)
        if param.default:
            result += f" = {param.default}"
        return result

    def format_rpc(rpc: RpcInfo) -> str:
        params = ", ".join(format_param(p) for p in rpc.params)
        result = _blue(rpc.name) + f"({params})"
        if rpc.return_type:
            result += " -> " + _green(rpc.return_type)
        return result

    def format_skill(skill: SkillInfo) -> str:
        info_parts = []
        if skill.stream:
            info_parts.append(f"stream={skill.stream}")
        if skill.reducer:
            info_parts.append(f"reducer={skill.reducer}")
        if skill.output:
            info_parts.append(f"output={skill.output}")
        info = f" ({', '.join(info_parts)})" if info_parts else ""
        return _cyan(skill.name) + info

    # Build output
    lines = [
        *(f" ├─ {format_stream(s)}" for s in info.inputs),
        *_box(info.name),
        *(f" ├─ {format_stream(s)}" for s in info.outputs),
    ]

    if info.rpcs:
        lines.append(" │")
        for rpc in info.rpcs:
            lines.append(f" ├─ RPC {format_rpc(rpc)}")

    if info.skills:
        lines.append(" │")
        for skill in info.skills:
            lines.append(f" ├─ Skill {format_skill(skill)}")

    return "\n".join(lines)
