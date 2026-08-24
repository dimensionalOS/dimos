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

"""Blueprint-side view of a baked host binary: one process of several native
modules, driven as a single NativeModule with the union of their ports."""

from __future__ import annotations

from collections.abc import Mapping
import json
import sys
from typing import get_args, get_origin, get_type_hints

from pydantic import Field, create_model

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out


class BakedHostConfig(NativeModuleConfig):
    """Config for a baked host. Per-member configs are `<member>_config` fields."""

    stdin_config: bool = True
    # Replaces the host's baked suppression list wholesale when set. An empty
    # list makes every internal hop externally visible again.
    suppress: list[str] | None = None


def _member_ports(member: type[NativeModule]) -> dict[str, object]:
    """The member's declared In/Out/IO annotations, resolved to real types."""
    hints = get_type_hints(member)
    return {name: hint for name, hint in hints.items() if get_origin(hint) in (In, Out, IO)}


def _union_ports(
    members: Mapping[str, type[NativeModule]],
    remaps: Mapping[tuple[str, str], str],
) -> dict[str, object]:
    """Host port annotations, one per effective name.

    A member reading and writing one name keeps both directions. Otherwise a
    name anything publishes is an output of the whole host.
    """
    msg_types: dict[str, tuple[type, str]] = {}
    origins: dict[str, set[object]] = {}
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
            origins.setdefault(name, set()).add(get_origin(hint))

    ports: dict[str, object] = {}
    for name, (msg_type, _) in msg_types.items():
        kind = IO if IO in origins[name] else Out if Out in origins[name] else In
        ports[name] = kind[msg_type]  # type: ignore[index]
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

    def _argv(self, topics: dict[str, str]) -> list[str]:
        """A baked host takes its whole wiring on the launch line, so no flags.

        The binary rejects an unknown argument rather than ignoring it.
        """
        return [self.config.executable, *self.config.extra_args]

    def _stdin_blob(self, topics: dict[str, str]) -> bytes:
        sections: dict[str, object] = {}
        for instance in self._members:
            member_config = getattr(self.config, f"{instance}_config")
            sections[instance] = {
                "topics": self._member_topics(instance, topics),
                "config": member_config.to_config_dict() or None,
            }
        blob: dict[str, object] = {
            "modules": sections,
            "session": self._session().to_wire(),
        }
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
    **config_defaults: object,
) -> type[BakedHost]:
    """Build the NativeModule subclass that drives a baked host binary.

    Assign the result to `name` at module level: deploying to a worker pickles
    the class by that path.
    """
    if not members:
        raise ValueError("a baked host needs at least one member module")
    remaps = dict(remaps or {})

    caller = sys._getframe(1).f_globals.get("__name__", __name__)

    fields: dict[str, tuple[type, object]] = {"executable": (str, executable)}
    for instance, member in members.items():
        member_config = get_type_hints(member)["config"]
        fields[f"{instance}_config"] = (
            member_config,
            Field(default_factory=member_config),
        )
    fields.update({key: (type(value), value) for key, value in config_defaults.items()})
    config_cls = create_model(
        f"{name}Config",
        __base__=BakedHostConfig,
        __module__=caller,
        **fields,  # type: ignore[call-overload]
    )

    namespace: dict[str, object] = {
        "__annotations__": {"config": config_cls, **_union_ports(members, remaps)},
        "__doc__": f"Baked host `{name}`: {', '.join(members)}.",
        "__module__": caller,
        "_members": dict(members),
        "_remaps": remaps,
    }
    host_cls = type(name, (BakedHost,), namespace)
    # Nothing names the config class, so put it where pickle will look for it.
    setattr(sys.modules[caller], config_cls.__name__, config_cls)
    return host_cls
