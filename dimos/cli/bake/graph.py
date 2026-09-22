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

"""Wire the selected modules together the way autoconnect would.

Unlike autoconnect, a same-name type mismatch is an error rather than silence.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
from typing import Literal

from dimos.cli.bake.discovery import RegisteredModule, normalize_id
from dimos.cli.bake.errors import BakeError
from dimos.core.transport_factory import default_zenoh_qos_for, zenoh_key_expr

Kind = Literal["internal", "external_input", "external_output"]


@dataclass(frozen=True)
class PortRef:
    module: str
    port: str

    def __str__(self) -> str:
        return f"{self.module}.{self.port}"


@dataclass(frozen=True)
class Connection:
    """One logical channel and everything in this host attached to it."""

    name: str
    topic: str
    msg_type: str
    producers: tuple[PortRef, ...]
    consumers: tuple[PortRef, ...]
    suppressed: bool
    qos: Mapping[str, str] | None

    @property
    def kind(self) -> Kind:
        if self.producers and self.consumers:
            return "internal"
        return "external_output" if self.producers else "external_input"

    @property
    def remapped(self) -> bool:
        return any(ref.port != self.name for ref in self.producers + self.consumers)

    def to_json(self) -> dict[str, object]:
        return {
            "name": self.name,
            "topic": self.topic,
            "msg_type": self.msg_type,
            "kind": self.kind,
            "suppressed": self.suppressed,
            "producers": [{"module": p.module, "port": p.port} for p in self.producers],
            "consumers": [{"module": c.module, "port": c.port} for c in self.consumers],
            "qos": dict(self.qos) if self.qos else None,
        }


@dataclass(frozen=True)
class Graph:
    host: str
    modules: tuple[str, ...]
    connections: tuple[Connection, ...]
    warnings: tuple[str, ...]

    def topics(self) -> dict[str, dict[str, str]]:
        """Baked wiring: `{module: {port: topic}}`."""
        wiring: dict[str, dict[str, str]] = {m: {} for m in self.modules}
        for conn in self.connections:
            for ref in conn.producers + conn.consumers:
                wiring[ref.module][ref.port] = conn.topic
        return wiring

    def suppressed_topics(self) -> tuple[str, ...]:
        return tuple(c.topic for c in self.connections if c.suppressed)

    def qos(self) -> dict[str, dict[str, str]]:
        return {c.topic: dict(c.qos) for c in self.connections if c.qos}

    def fingerprint(self) -> str:
        """Identity of the wiring a host is baked with, stamped into its config.

        Covers what a stale config file would silently override.
        """
        material = json.dumps(
            {
                "host": self.host,
                "topics": self.topics(),
                "suppress": list(self.suppressed_topics()),
            },
            sort_keys=True,
        )
        return hashlib.sha256(material.encode()).hexdigest()[:16]

    def to_json(self) -> dict[str, object]:
        return {
            "host": self.host,
            "modules": list(self.modules),
            "connections": [c.to_json() for c in self.connections],
            "warnings": list(self.warnings),
        }


def parse_remap(value: str) -> tuple[tuple[str, str], str]:
    """Split one --remap argument into its target port and new channel name."""
    target, _, name = value.partition("=")
    module, _, port = target.partition(".")
    if not (module and port and name):
        raise BakeError(f"malformed --remap {value!r}; expected <module>.<port>=<name>")
    return (normalize_id(module), port), name


def _effective(remaps: Mapping[tuple[str, str], str], module: str, port: str) -> str:
    return remaps.get((module, port), port)


def _check_remap_targets(
    modules: Sequence[RegisteredModule], remaps: Mapping[tuple[str, str], str]
) -> None:
    known = {(m.id, port) for m in modules for port in m.ports}
    for module, port in remaps:
        if (module, port) not in known:
            raise BakeError(f"--remap names unknown port `{module}.{port}`")


def _resolve_suppress(entries: Iterable[str], connections: Sequence[Connection]) -> set[str]:
    """Match `--suppress` entries against channel names or full topics."""
    topics: set[str] = set()
    for entry in entries:
        match = [c for c in connections if entry in (c.name, c.topic)]
        if not match:
            raise BakeError(f"--suppress names `{entry}`, which no baked module touches")
        topics.update(c.topic for c in match)
    return topics


def build_graph(
    host: str,
    modules: Sequence[RegisteredModule],
    *,
    remaps: Mapping[tuple[str, str], str] | None = None,
    suppress: Iterable[str] = (),
) -> Graph:
    """Connect the modules by effective port name and classify every channel."""
    remaps = dict(remaps or {})
    _check_remap_targets(modules, remaps)

    types: dict[str, tuple[str, PortRef]] = {}
    producers: dict[str, list[PortRef]] = {}
    consumers: dict[str, list[PortRef]] = {}

    for module in modules:
        for port, msg_type in module.outputs.items():
            name = _effective(remaps, module.id, port)
            _record_type(types, name, msg_type, PortRef(module.id, port))
            producers.setdefault(name, []).append(PortRef(module.id, port))
        for port, msg_type in module.inputs.items():
            name = _effective(remaps, module.id, port)
            _record_type(types, name, msg_type, PortRef(module.id, port))
            consumers.setdefault(name, []).append(PortRef(module.id, port))

    warnings: list[str] = []
    for name, refs in producers.items():
        if len(refs) > 1:
            listed = ", ".join(str(r) for r in refs)
            warnings.append(f"{len(refs)} producers publish `{name}`: {listed}")

    def connection(name: str, suppressed: bool) -> Connection:
        msg_type, _ = types[name]
        qos = default_zenoh_qos_for(name, msg_type)
        return Connection(
            name=name,
            topic=zenoh_key_expr(name, msg_type),
            msg_type=msg_type,
            producers=tuple(producers.get(name, ())),
            consumers=tuple(consumers.get(name, ())),
            suppressed=suppressed,
            qos=qos.to_wire() if qos else None,
        )

    draft = [connection(name, False) for name in sorted(types)]
    wanted = _resolve_suppress(suppress, draft)
    connections = [connection(c.name, c.topic in wanted) for c in draft]
    for conn in connections:
        if conn.suppressed:
            _check_suppressible(conn, warnings)

    return Graph(
        host=host,
        modules=tuple(m.id for m in modules),
        connections=tuple(connections),
        warnings=tuple(warnings),
    )


def _record_type(
    types: dict[str, tuple[str, PortRef]], name: str, msg_type: str, ref: PortRef
) -> None:
    existing = types.get(name)
    if existing is None:
        types[name] = (msg_type, ref)
        return
    if existing[0] != msg_type:
        raise BakeError(
            f"`{name}` is {existing[0]} on {existing[1]} but {msg_type} on {ref}. "
            f"Two different messages cannot share a topic: rename one with "
            f"--remap {ref}=<other_name>"
        )


def _check_suppressible(conn: Connection, warnings: list[str]) -> None:
    if not conn.producers:
        raise BakeError(
            f"cannot suppress `{conn.topic}`: nothing in this host publishes it, "
            f"it is an external input to {', '.join(str(c) for c in conn.consumers)}"
        )
    if not conn.consumers:
        warnings.append(
            f"`{conn.topic}` is suppressed but no baked module consumes it, "
            "so it is now published to nobody"
        )


_SECTIONS: tuple[tuple[Kind, str], ...] = (
    ("internal", "Internal connections"),
    ("external_input", "External inputs (subscribed)"),
    ("external_output", "External outputs (published)"),
)


def render(graph: Graph) -> str:
    """The graph as printed before a build."""
    lines = [f"Host `{graph.host}`: {', '.join(graph.modules)}"]
    for kind, title in _SECTIONS:
        conns = [c for c in graph.connections if c.kind == kind]
        lines.append("")
        lines.append(f"{title}:")
        if not conns:
            lines.append("  (none)")
            continue
        for conn in conns:
            flags = "  [SUPPRESSED]" if conn.suppressed else ""
            flags += "  (remapped)" if conn.remapped else ""
            lines.append(f"  {conn.topic}  {conn.msg_type}{flags}")
            for ref in conn.producers:
                lines.append(f"      out  {ref}")
            for ref in conn.consumers:
                lines.append(f"      in   {ref}")
    if graph.warnings:
        lines.append("")
        lines.append("Warnings:")
        lines.extend(f"  {w}" for w in graph.warnings)
    return "\n".join(lines)
