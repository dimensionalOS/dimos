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

"""Live descriptions of the running system, and their keyed facts."""

from __future__ import annotations

from dataclasses import dataclass
import functools
import importlib
import importlib.metadata
import inspect
from pathlib import Path
import re
import subprocess
import sys
from typing import TYPE_CHECKING, cast

import dimos
from dimos.agents.harness.runtime.agent_module_manager import AgentModuleStatus
from dimos.agents.harness.runtime.constants import (
    INVENTORY_DOC_CHARS,
    INVENTORY_SECTION_LIMIT,
    JSON,
    LIFECYCLE_RPCS,
)
from dimos.core.coordination.blueprints import BlueprintAtom
from dimos.core.global_config import global_config
from dimos.core.module import Module

if TYPE_CHECKING:
    from dimos.core.introspection.module.info import ModuleInfo, RpcInfo, StreamInfo
    from dimos.core.rpc_client import RPCClient
    from dimos.porcelain.dimos import Dimos


@dataclass(frozen=True)
class Environment:
    python: str
    dimos_root: str
    dimos_version: str | None
    dimos_commit: str | None
    transport: str
    robot_ip: str | None
    robot_ips: str | None
    zenoh_connect: str | None
    zenoh_scouting: bool
    python_api_guide: str | None
    """docs/usage/python-api.md, when the checkout has it."""
    relay: dict[str, str] | None

    def to_json(self) -> dict[str, JSON]:
        return {
            "python": self.python,
            "dimos_root": self.dimos_root,
            "dimos_version": self.dimos_version,
            "dimos_commit": self.dimos_commit,
            "transport": self.transport,
            "robot_ip": self.robot_ip,
            "robot_ips": self.robot_ips,
            "zenoh_connect": self.zenoh_connect,
            "zenoh_scouting": self.zenoh_scouting,
            "python_api_guide": self.python_api_guide,
            "relay": None if self.relay is None else dict(self.relay),
        }


@dataclass(frozen=True)
class RpcEntry:
    name: str
    signature: str
    """As a caller writes it: name, parameters without self, return type."""
    doc: str | None

    def to_json(self) -> dict[str, JSON]:
        return {"name": self.name, "signature": self.signature, "doc": self.doc}


@dataclass(frozen=True)
class ModuleEntry:
    instance: str
    class_name: str | None
    path: str | None
    """The qualified import path of the module's class."""
    doc: str | None
    rpcs: list[RpcEntry]

    def to_json(self) -> dict[str, JSON]:
        return {
            "instance": self.instance,
            "class": self.class_name,
            "path": self.path,
            "doc": self.doc,
            "rpcs": [rpc.to_json() for rpc in self.rpcs],
        }


@dataclass(frozen=True)
class StreamEntry:
    module: str
    name: str
    direction: str | None
    type: str
    channel: str | None
    import_line: str | None = None
    """How to import the message type, when the module's class imports here."""

    def to_json(self) -> dict[str, JSON]:
        return {
            "module": self.module,
            "name": self.name,
            "direction": self.direction,
            "type": self.type,
            "channel": self.channel,
            "import": self.import_line,
        }


@dataclass(frozen=True)
class Runtime:
    modules: list[ModuleEntry]
    streams: list[StreamEntry]

    def to_json(self) -> dict[str, JSON]:
        return {
            "modules": [module.to_json() for module in self.modules],
            "streams": [stream.to_json() for stream in self.streams],
        }


@dataclass(frozen=True)
class ModuleDetail:
    module: str
    class_name: str | None
    path: str | None
    doc: str | None
    rpcs: list[RpcEntry]
    """With the first paragraph of each doc; lifecycle RPCs left out."""
    streams: list[StreamEntry]

    def to_json(self) -> dict[str, JSON]:
        return {
            "module": self.module,
            "class": self.class_name,
            "path": self.path,
            "doc": self.doc,
            "rpcs": [rpc.to_json() for rpc in self.rpcs],
            "streams": [stream.to_json() for stream in self.streams],
        }


@dataclass(frozen=True)
class RpcDetail:
    module: str
    rpc: RpcEntry
    """With the full doc."""

    def to_json(self) -> dict[str, JSON]:
        return {"module": self.module, "rpc": self.rpc.to_json()}


@dataclass(frozen=True)
class Fact:
    section: str
    key: str
    """Stable across calls while the system is unchanged, so clients can compare by key."""
    line: str

    def to_json(self) -> dict[str, JSON]:
        return {"section": self.section, "key": self.key, "line": self.line}


@dataclass(frozen=True)
class Inventory:
    environment: Environment
    runtime: Runtime
    agent_modules: list[AgentModuleStatus]

    def to_json(self) -> dict[str, JSON]:
        return {
            "environment": self.environment.to_json(),
            "runtime": self.runtime.to_json(),
            "agent_modules": [module.to_json() for module in self.agent_modules],
        }


@functools.cache
def _framework_docs() -> frozenset[str]:
    """Docstrings of the Module base classes.

    Module documentation is ``inspect.getdoc`` of the class, which inherits a base
    class docstring when the module has none; that text says nothing about the module.
    """
    return frozenset(doc for cls in Module.__mro__ if (doc := inspect.getdoc(cls)))


def _module_doc(text: str | None) -> str | None:
    return None if text in _framework_docs() else text


def _first_line(text: str | None) -> str | None:
    if not text:
        return None
    return text.strip().splitlines()[0]


def _paragraph(text: str | None) -> str | None:
    if not text:
        return None
    return " ".join(text.strip().split("\n\n")[0].split())


def _clip(text: str) -> str:
    return text if len(text) <= INVENTORY_DOC_CHARS else text[: INVENTORY_DOC_CHARS - 1] + "…"


def _doc_suffix(text: str | None) -> str:
    if not text:
        return ""
    return " — " + _clip(text)


def _signature(rpc: RpcInfo) -> str:
    if rpc.signature:
        # As a caller sees it: no self, and annotations unquoted when the module uses
        # postponed evaluation (string defaults keep their quotes).
        signature = re.sub(
            r"^\(self(, |\))", lambda match: "(" if match[1] == ", " else "()", rpc.signature
        )
        signature = re.sub(r"(: |-> )'([^']*)'", r"\1\2", signature)
        return f"{rpc.name}{signature}"
    params = ", ".join(
        param.name + (f": {param.type_name}" if param.type_name else "") for param in rpc.params
    )
    returns = f" -> {rpc.return_type}" if rpc.return_type else ""
    return f"{rpc.name}({params}){returns}"


def _stream(stream: StreamInfo, import_line: str | None = None) -> StreamEntry:
    return StreamEntry(
        module=stream.module_name or "",
        name=stream.name,
        direction=stream.direction,
        type=stream.type_name,
        channel=stream.channel,
        import_line=import_line,
    )


def _stream_imports(path: str | None) -> dict[str, str]:
    module_path, _, class_name = (path or "").rpartition(".")
    try:
        atom = BlueprintAtom.create(getattr(importlib.import_module(module_path), class_name), {})
    except Exception:
        return {}
    return {
        stream.name: f"from {stream.type.__module__} import {stream.type.__qualname__}"
        for stream in atom.streams
    }


def _find(app: Dimos, module: str) -> ModuleInfo:
    instance = app.get_module(module).remote_name
    info = next((item for item in app.list_modules() if item.name == instance), None)
    if info is None:
        raise KeyError(f"Module {module!r} is not running")
    return info


def relay_endpoint(app: Dimos) -> dict[str, str] | None:
    """The endpoint of the running bridge, including authored bridge subclasses."""
    for module in app.list_modules():
        if any(rpc.name == "relay_endpoint" for rpc in module.rpcs):
            endpoint: dict[str, str] | None = cast(
                "RPCClient", app.get_module(module.name)
            ).relay_endpoint()
            return endpoint
    return None


def describe_runtime(app: Dimos) -> Runtime:
    """Every module with its RPCs (signature, first doc line) and every wired stream."""
    return Runtime(
        modules=[
            ModuleEntry(
                instance=info.name,
                class_name=info.class_name,
                path=info.qualified_path,
                doc=_first_line(_module_doc(info.documentation)),
                rpcs=[
                    RpcEntry(rpc.name, _signature(rpc), _first_line(rpc.documentation))
                    for rpc in info.rpcs
                ],
            )
            for info in app.list_modules()
        ],
        streams=[_stream(stream) for stream in app.list_streams()],
    )


def describe_module(app: Dimos, module: str) -> ModuleDetail:
    """One module: its documentation, RPC signatures with the first paragraph of their
    documentation, and its streams with the import of each message type."""
    info = _find(app, module)
    imports = _stream_imports(info.qualified_path)
    return ModuleDetail(
        module=info.name,
        class_name=info.class_name,
        path=info.qualified_path,
        doc=_module_doc(info.documentation),
        rpcs=[
            RpcEntry(item.name, _signature(item), _paragraph(item.documentation))
            for item in info.rpcs
            if item.name not in LIFECYCLE_RPCS
        ],
        streams=[
            _stream(stream, imports.get(stream.name)) for stream in app.list_streams(info.name)
        ],
    )


def describe_rpc(app: Dimos, module: str, rpc: str) -> RpcDetail:
    """One RPC of one module, with its full documentation."""
    info = _find(app, module)
    found = next((item for item in info.rpcs if item.name == rpc), None)
    if found is None:
        raise KeyError(f"{info.name} has no RPC {rpc!r}")
    return RpcDetail(info.name, RpcEntry(found.name, _signature(found), found.documentation))


@functools.cache
def _commit() -> str | None:
    root = Path(dimos.__file__).resolve().parent.parent
    try:
        return subprocess.run(
            ["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=2,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def environment(relay: dict[str, str] | None = None) -> Environment:
    root = Path(dimos.__file__).resolve().parent.parent
    guide = root / "docs" / "usage" / "python-api.md"
    try:
        version: str | None = importlib.metadata.version("dimos")
    except importlib.metadata.PackageNotFoundError:
        version = None
    return Environment(
        python=sys.executable,
        dimos_root=str(root),
        dimos_version=version,
        dimos_commit=_commit(),
        transport=global_config.transport,
        robot_ip=global_config.robot_ip,
        robot_ips=global_config.robot_ips,
        zenoh_connect=global_config.zenoh_connect,
        zenoh_scouting=global_config.zenoh_scouting,
        python_api_guide=str(guide) if guide.exists() else None,
        relay=relay,
    )


def _limited(section: str, items: list[tuple[str, str]]) -> list[Fact]:
    shown = [Fact(section, key, line) for key, line in items[:INVENTORY_SECTION_LIMIT]]
    if len(items) > INVENTORY_SECTION_LIMIT:
        shown.append(
            Fact(
                section,
                f"more:{section}",
                f"… {len(items) - INVENTORY_SECTION_LIMIT} more; call inventory",
            )
        )
    return shown


def facts(inventory: Inventory) -> list[Fact]:
    """The inventory as keyed one-line facts in display order.

    A fact changes only when the state of the system changes: message ages are left
    out, so clients can compare facts by key to report what changed.
    """
    env = inventory.environment
    robot_ip = f" · robot_ip {env.robot_ip}" if env.robot_ip else ""
    guide = env.python_api_guide if env.python_api_guide is not None else "unavailable"
    result = [
        Fact(
            "Environment",
            "env:dimos",
            f"DimOS {env.dimos_version} @ {env.dimos_commit} · transport {env.transport}{robot_ip}",
        ),
        Fact("Environment", "env:python", f"Python: {env.python}"),
        Fact("Environment", "env:guide", f"Python API guide: {guide}"),
    ]
    modules = inventory.runtime.modules
    result += _limited(
        "Modules",
        [
            (
                f"module:{module.instance}",
                f"{module.instance} [{module.class_name}]{_doc_suffix(module.doc)}",
            )
            for module in modules
        ],
    )
    result += _limited(
        "RPCs",
        [
            (
                f"rpc:{module.instance}.{rpc.name}",
                f"{module.instance}.{rpc.signature}{_doc_suffix(rpc.doc)}",
            )
            for module in modules
            for rpc in module.rpcs
            if rpc.name not in LIFECYCLE_RPCS
        ],
    )
    result += _limited(
        "Streams",
        [
            (
                f"stream:{stream.module}.{stream.name}",
                f"{stream.module}.{stream.name} {stream.direction} {stream.type} "
                f"channel={stream.channel}",
            )
            for stream in inventory.runtime.streams
        ],
    )
    result += _limited(
        "Agent modules",
        [
            (
                f"agent_module:{item.name}",
                f"{item.name} [{item.class_name}] {item.state}"
                + ("" if item.revision is None else f" r{item.revision}")
                + (f" · last deploy failed: {_clip(item.last_error)}" if item.last_error else ""),
            )
            for item in inventory.agent_modules
        ],
    )
    return result


def module_text(detail: ModuleDetail) -> str:
    lines = [f"{detail.module} [{detail.class_name}] {detail.path}"]
    if detail.doc:
        lines += ["", detail.doc.strip()]
    lines += [
        "",
        f"RPCs ({len(detail.rpcs)}; call as app.get_module({detail.module!r}).RPC(...); "
        "lifecycle RPCs omitted; add rpc=NAME for the full documentation):",
    ]
    for rpc in detail.rpcs:
        lines.append(f"  {rpc.signature}")
        if rpc.doc:
            lines.append(f"      {rpc.doc}")
    lines += [
        "",
        f"Streams ({len(detail.streams)}; a module joins one with a port of the same name "
        "and type):",
    ]
    lines += [
        f"  {stream.module}.{stream.name} {stream.direction} {stream.type} "
        f"channel={stream.channel}" + (f" · {stream.import_line}" if stream.import_line else "")
        for stream in detail.streams
    ]
    return "\n".join(lines)


def rpc_text(detail: RpcDetail) -> str:
    return f"{detail.module}.{detail.rpc.signature}\n\n{detail.rpc.doc or 'No documentation.'}"
