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

"""Blueprint-side view of a `dimos bake` host binary.

One host process runs several native modules, so python drives it as a single
NativeModule whose ports are the union of its members'. Autoconnect,
`.remappings()` and `.namespace()` keep working because the union is expressed
as ordinary In/Out annotations; only the stdin blob is different, nesting one
section per member.

    GoNav = baked_host(
        "GoNav",
        executable="dist/go2-nav",
        members={"ray_tracing": RayTracingVoxelMap, "mls_planner": MLSPlannerNative},
    )
"""

from __future__ import annotations

from collections.abc import Mapping
import json
from typing import Any, get_args, get_origin, get_type_hints

from pydantic import Field, create_model

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out

# A member port and what it does on the host: producing wins, because a topic
# some member publishes is an output of the host as a whole.
_PRODUCING = (Out, IO)


class BakedHostConfig(NativeModuleConfig):
    """Config for a baked host. Per-member configs are `<member>_config` fields."""

    stdin_config: bool = True
    # Replaces the host's baked suppression list wholesale when set; `[]` makes
    # every internal hop externally visible again.
    suppress: list[str] | None = None


def _member_ports(member: type[NativeModule]) -> dict[str, Any]:
    """The member's declared In/Out/IO annotations, resolved to real types."""
    hints = get_type_hints(member)
    return {name: hint for name, hint in hints.items() if get_origin(hint) in (In, Out, IO)}


def _union_ports(
    members: Mapping[str, type[NativeModule]],
    remaps: Mapping[tuple[str, str], str],
) -> dict[str, Any]:
    """Host port annotations: one per effective name, Out if anything produces it."""
    ports: dict[str, Any] = {}
    msg_types: dict[str, tuple[type, str]] = {}
    for instance, member in members.items():
        for port, hint in _member_ports(member).items():
            name = remaps.get((instance, port), port)
            msg_type = get_args(hint)[0]
            seen = msg_types.get(name)
            if seen is not None and seen[0] is not msg_type:
                raise ValueError(
                    f"port `{name}` is {seen[0].__name__} on {seen[1]} but "
                    f"{msg_type.__name__} on {instance}.{port}; remap one of them"
                )
            msg_types[name] = (msg_type, f"{instance}.{port}")
            produces = get_origin(hint) in _PRODUCING
            if produces or name not in ports:
                ports[name] = Out[msg_type] if produces else hint  # type: ignore[valid-type]
    return ports


class BakedHost(NativeModule):
    """Base of every generated host class. Only the stdin blob differs."""

    config: BakedHostConfig

    # Filled in by `baked_host`.
    _members: Mapping[str, type[NativeModule]] = {}
    _remaps: Mapping[tuple[str, str], str] = {}

    def _member_topics(self, instance: str, topics: Mapping[str, str]) -> dict[str, str]:
        member = self._members[instance]
        resolved = {}
        for port in _member_ports(member):
            name = self._remaps.get((instance, port), port)
            if name in topics:
                resolved[port] = topics[name]
        return resolved

    def _stdin_blob(self, topics: dict[str, str]) -> bytes | None:
        if not self.config.stdin_config:
            return None
        sections: dict[str, Any] = {}
        for instance in self._members:
            member_config = getattr(self.config, f"{instance}_config")
            sections[instance] = {
                "topics": self._member_topics(instance, topics),
                "config": member_config.to_config_dict() or None,
            }
        blob: dict[str, Any] = {"modules": sections}
        qos = self._collect_output_qos()
        if qos:
            blob["qos"] = qos
        if self.config.suppress is not None:
            blob["suppress"] = list(self.config.suppress)
        return json.dumps(blob).encode() + b"\n"


def baked_host(
    name: str,
    executable: str,
    members: Mapping[str, type[NativeModule]],
    remaps: Mapping[tuple[str, str], str] | None = None,
    **config_defaults: Any,
) -> type[BakedHost]:
    """Build the NativeModule subclass that drives a baked host binary.

    `members` maps the baked module id to the python wrapper it replaces;
    `remaps` renames a member port, keyed by `(member, port)`, and must match
    the `--remap` the binary was baked with.
    """
    if not members:
        raise ValueError("a baked host needs at least one member module")
    remaps = dict(remaps or {})

    fields: dict[str, Any] = {"executable": (str, executable)}
    for instance, member in members.items():
        member_config = get_type_hints(member)["config"]
        fields[f"{instance}_config"] = (
            member_config,
            Field(default_factory=member_config),
        )
    fields.update({key: (type(value), value) for key, value in config_defaults.items()})
    config_cls = create_model(f"{name}Config", __base__=BakedHostConfig, **fields)

    namespace: dict[str, Any] = {
        "__annotations__": {"config": config_cls, **_union_ports(members, remaps)},
        "__doc__": f"Baked host `{name}`: {', '.join(members)}.",
        "__module__": __name__,
        "_members": dict(members),
        "_remaps": remaps,
    }
    return type(name, (BakedHost,), namespace)
