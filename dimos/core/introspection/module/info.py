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

"""Module introspection data structures."""

from collections.abc import Callable
from dataclasses import dataclass, field
import inspect
from typing import Any

# Internal RPCs to hide from io() output
INTERNAL_RPCS = {
    "dynamic_skills",
    "get_rpc_method_names",
    "set_rpc_method",
    "skills",
    "_io_instance",
}


@dataclass
class StreamInfo:
    """Information about a module stream (input or output)."""

    name: str
    type_name: str


@dataclass
class ParamInfo:
    """Information about an RPC parameter."""

    name: str
    type_name: str | None = None
    default: str | None = None


@dataclass
class RpcInfo:
    """Information about an RPC method."""

    name: str
    params: list[ParamInfo] = field(default_factory=list)
    return_type: str | None = None


@dataclass
class SkillInfo:
    """Information about a skill."""

    name: str
    stream: str | None = None  # None means "none"
    reducer: str | None = None  # None means "latest"
    output: str | None = None  # None means "standard"


@dataclass
class ModuleInfo:
    """Extracted information about a module's IO interface."""

    name: str
    inputs: list[StreamInfo] = field(default_factory=list)
    outputs: list[StreamInfo] = field(default_factory=list)
    rpcs: list[RpcInfo] = field(default_factory=list)
    skills: list[SkillInfo] = field(default_factory=list)


def extract_rpc_info(fn: Callable) -> RpcInfo:  # type: ignore[type-arg]
    """Extract RPC information from a callable."""
    sig = inspect.signature(fn)
    params = []

    for pname, p in sig.parameters.items():
        if pname == "self":
            continue
        type_name = None
        if p.annotation != inspect.Parameter.empty:
            type_name = getattr(p.annotation, "__name__", str(p.annotation))
        default = None
        if p.default != inspect.Parameter.empty:
            default = str(p.default)
        params.append(ParamInfo(name=pname, type_name=type_name, default=default))

    return_type = None
    if sig.return_annotation != inspect.Signature.empty:
        return_type = getattr(sig.return_annotation, "__name__", str(sig.return_annotation))

    return RpcInfo(name=fn.__name__, params=params, return_type=return_type)


def extract_skill_info(fn: Callable) -> SkillInfo:  # type: ignore[type-arg]
    """Extract skill information from a skill-decorated callable."""
    cfg = fn._skill_config  # type: ignore[attr-defined]

    stream = cfg.stream.name if cfg.stream.name != "none" else None
    reducer_name = getattr(cfg.reducer, "__name__", str(cfg.reducer))
    reducer = reducer_name if reducer_name != "latest" else None
    output = cfg.output.name if cfg.output.name != "standard" else None

    return SkillInfo(name=fn.__name__, stream=stream, reducer=reducer, output=output)


def extract_module_info(
    name: str,
    inputs: dict[str, Any],
    outputs: dict[str, Any],
    rpcs: dict[str, Callable],  # type: ignore[type-arg]
) -> ModuleInfo:
    """Extract module information into a ModuleInfo structure.

    Args:
        name: Module class name.
        inputs: Dict of input stream name -> stream object or formatted string.
        outputs: Dict of output stream name -> stream object or formatted string.
        rpcs: Dict of RPC method name -> callable.

    Returns:
        ModuleInfo with extracted data.
    """

    # Extract stream info
    def stream_info(stream: Any, stream_name: str) -> StreamInfo:
        if isinstance(stream, str):
            # Pre-formatted string like "name: Type" - parse it
            # Strip ANSI codes for parsing
            import re

            clean = re.sub(r"\x1b\[[0-9;]*m", "", stream)
            if ": " in clean:
                parts = clean.split(": ", 1)
                return StreamInfo(name=parts[0], type_name=parts[1])
            return StreamInfo(name=stream_name, type_name=clean)
        # Instance stream object
        return StreamInfo(name=stream.name, type_name=stream.type.__name__)

    input_infos = [stream_info(s, n) for n, s in inputs.items()]
    output_infos = [stream_info(s, n) for n, s in outputs.items()]

    # Separate skills from regular RPCs, filtering internal ones
    rpc_infos = []
    skill_infos = []

    for rpc_name, rpc_fn in rpcs.items():
        if rpc_name in INTERNAL_RPCS:
            continue
        if hasattr(rpc_fn, "_skill_config"):
            skill_infos.append(extract_skill_info(rpc_fn))
        else:
            rpc_infos.append(extract_rpc_info(rpc_fn))

    return ModuleInfo(
        name=name,
        inputs=input_infos,
        outputs=output_infos,
        rpcs=rpc_infos,
        skills=skill_infos,
    )
