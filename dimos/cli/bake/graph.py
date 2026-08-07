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

"""Wire the selected modules together the way ``autoconnect`` would.

Ports connect by name: same effective name means same topic. Where python's
autoconnect silently leaves a same-name/different-type pair unconnected, a bake
is a build artifact nobody re-reads, so the mismatch is an error instead.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Literal

from dimos.cli.bake import BakeError
from dimos.cli.bake.discovery import ModuleInfo, normalize_id

Kind = Literal["internal", "external_input", "external_output"]

_TOPIC_PREFIX = "dimos/"


def topic_for(name: str, msg_type: str) -> str:
    """The zenoh key a typed dimos channel lands on.

    ``transport_topic`` namespaces the channel under ``dimos/``; ``ZenohTopic``
    then appends the message type, and that full key is what a python
    NativeModule hands its native process. A baked host must agree, or a
    standalone run would sit on keys nobody else uses.
    """
    return f"{_TOPIC_PREFIX}{name.lstrip('/')}/{msg_type}"


def default_qos(name: str, msg_type: str) -> dict[str, str] | None:
    """Publisher QoS bake bakes in, mirroring ``default_zenoh_qos``."""
    from dimos.core.transport_factory import _LATEST_WINS_TYPES, _NEVER_DROP_CHANNELS
    from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, QOS_NEVER_DROP

    if msg_type in _LATEST_WINS_TYPES:
        return QOS_LATEST_WINS.to_wire()
    if name.lstrip("/") in _NEVER_DROP_CHANNELS:
        return QOS_NEVER_DROP.to_wire()
    return None


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

    def to_json(self) -> dict[str, Any]:
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
    # (module id, declared port) -> effective name
    remaps: Mapping[tuple[str, str], str]

    def topics(self) -> dict[str, dict[str, str]]:
        """Baked wiring: `{module: {port: topic}}`."""
        wiring: dict[str, dict[str, str]] = {m: {} for m in self.modules}
        for conn in self.connections:
            for ref in self.producers_and_consumers(conn):
                wiring[ref.module][ref.port] = conn.topic
        return wiring

    @staticmethod
    def producers_and_consumers(conn: Connection) -> tuple[PortRef, ...]:
        return conn.producers + conn.consumers

    def suppressed_topics(self) -> tuple[str, ...]:
        return tuple(c.topic for c in self.connections if c.suppressed)

    def qos(self) -> dict[str, dict[str, str]]:
        return {c.topic: dict(c.qos) for c in self.connections if c.qos}

    def to_json(self) -> dict[str, Any]:
        return {
            "host": self.host,
            "modules": list(self.modules),
            "connections": [c.to_json() for c in self.connections],
            "warnings": list(self.warnings),
        }


def parse_remap(value: str) -> tuple[tuple[str, str], str]:
    """`--remap mls_planner.global_map=surface_map` -> `(("mls_planner", "global_map"), ...)`."""
    target, _, name = value.partition("=")
    module, _, port = target.partition(".")
    if not (module and port and name):
        raise BakeError(f"malformed --remap {value!r}; expected <module>.<port>=<name>")
    return (normalize_id(module), port), name


def _effective(remaps: Mapping[tuple[str, str], str], module: str, port: str) -> str:
    return remaps.get((module, port), port)


def _check_remap_targets(
    modules: Sequence[ModuleInfo], remaps: Mapping[tuple[str, str], str]
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
    modules: Sequence[ModuleInfo],
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
        return Connection(
            name=name,
            topic=topic_for(name, msg_type),
            msg_type=msg_type,
            producers=tuple(producers.get(name, ())),
            consumers=tuple(consumers.get(name, ())),
            suppressed=suppressed,
            qos=default_qos(name, msg_type),
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
        remaps=remaps,
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
            f"Two different messages cannot share a topic — rename one with "
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
