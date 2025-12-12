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

"""Module IO introspection and rendering."""

from collections.abc import Callable
import inspect
from typing import Any

from dimos.core import colors

# Internal RPCs to hide from io() output
INTERNAL_RPCS = {
    "dynamic_skills",
    "get_rpc_method_names",
    "set_rpc_method",
    "skills",
    "_io_instance",
}


def render_module_io(
    name: str,
    inputs: dict[str, Any],
    outputs: dict[str, Any],
    rpcs: dict[str, Callable],  # type: ignore[type-arg]
    color: bool = True,
) -> str:
    """Render module IO diagram.

    Args:
        name: Module class name.
        inputs: Dict of input stream name -> stream object or formatted string.
        outputs: Dict of output stream name -> stream object or formatted string.
        rpcs: Dict of RPC method name -> callable.
        color: Whether to include ANSI color codes.

    Returns:
        ASCII diagram showing module inputs, outputs, RPCs, and skills.
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

    def repr_rpc(fn: Callable) -> str:  # type: ignore[type-arg]
        sig = inspect.signature(fn)
        params = [p for pname, p in sig.parameters.items() if pname != "self"]

        param_strs = []
        for param in params:
            param_str = param.name
            if param.annotation != inspect.Parameter.empty:
                type_name = getattr(param.annotation, "__name__", str(param.annotation))
                param_str += ": " + _green(type_name)
            if param.default != inspect.Parameter.empty:
                param_str += f" = {param.default}"
            param_strs.append(param_str)

        return_annotation = ""
        if sig.return_annotation != inspect.Signature.empty:
            return_type = getattr(sig.return_annotation, "__name__", str(sig.return_annotation))
            return_annotation = " -> " + _green(return_type)

        return _blue(fn.__name__) + f"({', '.join(param_strs)})" + return_annotation

    def format_stream(stream: Any) -> str:
        # For instance streams, they have __str__ with colors baked in
        # For class-level, we pass pre-formatted strings
        if isinstance(stream, str):
            return stream
        # Instance stream - re-render without color if needed
        if not color:
            return f"{stream.name}: {stream.type.__name__}"
        return str(stream)

    # Separate skills from regular RPCs, and filter internal ones
    skills = {}
    regular_rpcs = {}
    for rpc_name, rpc_fn in rpcs.items():
        if rpc_name in INTERNAL_RPCS:
            continue
        if hasattr(rpc_fn, "_skill_config"):
            skills[rpc_name] = rpc_fn
        else:
            regular_rpcs[rpc_name] = rpc_fn

    ret = [
        *(f" ├─ {format_stream(stream)}" for stream in inputs.values()),
        *_box(name),
        *(f" ├─ {format_stream(stream)}" for stream in outputs.values()),
    ]

    if regular_rpcs:
        ret.append(" │")
        for rpc_fn in regular_rpcs.values():
            ret.append(f" ├─ RPC {repr_rpc(rpc_fn)}")

    if skills:
        ret.append(" │")
        for skill_fn in skills.values():
            cfg = skill_fn._skill_config
            info_parts = []
            if cfg.stream.name != "none":
                info_parts.append(f"stream={cfg.stream.name}")
            reducer_name = getattr(cfg.reducer, "__name__", str(cfg.reducer))
            if reducer_name != "latest":
                info_parts.append(f"reducer={reducer_name}")
            if cfg.output.name != "standard":
                info_parts.append(f"output={cfg.output.name}")
            info = f" ({', '.join(info_parts)})" if info_parts else ""
            ret.append(f" ├─ Skill {_cyan(skill_fn.__name__)}{info}")

    return "\n".join(ret)
