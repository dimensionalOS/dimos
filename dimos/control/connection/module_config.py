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

"""The config contract of a connection module: what a tool reads and changes on a running robot.

A connection says what it can change in :class:`ConfigKeys`, reports how
each of its targets is driven now in :class:`ConfigState`, and takes changes
as a :class:`ConfigPatch`: only what a patch gives changes. A target is one
unit the connection drives, e.g. one arm of a two-arm robot. A patch applies
for the config session (``"session"``: undone when the session ends) or is
saved to the target's file and applied from it (``"persist"``). What took
effect comes back as :class:`ConfigApplied`.

The driver defines the keys of the values and checks them; these types check
only their shape. Plain pydantic models holding JSON-like values, so they
cross processes over RPC and import without any robot code.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from typing import Any, Literal

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig

#: The longest lease a config session may ask for, seconds.
MAX_CONFIG_LEASE_S = 30.0
#: The lease a config session gets unless it asks for another, seconds.
DEFAULT_CONFIG_LEASE_S = 2.0

#: ``"session"``: live until the config session ends. ``"persist"``: saved to
#: the target's file, then applied from it.
ConfigScope = Literal["session", "persist"]


def _dotted(values: Mapping[Any, Any], prefix: str = "") -> Iterator[str]:
    for key, value in values.items():
        name = f"{prefix}{key}"
        if isinstance(value, Mapping) and value:
            yield from _dotted(value, f"{name}.")
        else:
            yield name


class ConfigPatch(BaseConfig):
    """A change to how one target is driven; only what is given changes.

    Attributes:
        target: The unit of the connection it changes, e.g. an arm id.
        values: What to change, by key, nested mappings allowed, e.g.
            ``{"gains": {"joint2": {"kp": 60.0, "kd": 2.0}}}``. The driver
            defines the keys.
    """

    target: str
    values: dict[str, Any] = Field(default_factory=dict)

    def keys(self) -> list[str]:
        """Every value the patch gives, dotted, e.g. "gains.joint2.kp".

        A nested mapping is followed to its values; an empty one is named alone.
        """
        return list(_dotted(self.values))


class ConfigApplied(BaseConfig):
    """What one ``config_set`` did.

    Attributes:
        target: The target.
        scope: As asked.
        applied: The values as they took effect, after clamping, shaped as a
            patch's values.
        clamped: Keys whose value was clamped.
        unsupported: Keys the connection cannot change in this scope; they
            changed nothing.
        why: By key in ``unsupported``, why not.
        session_id: The config session the change belongs to, if one is live.
        lease_s: That session's lease, seconds.
        path: The file written (persist).
        applied_now: Whether the saved file was applied at once (persist).
        refused: Why nothing was written (persist).
        details: Anything else the driver reports about the change, by name.
    """

    target: str
    scope: ConfigScope
    applied: dict[str, Any] = Field(default_factory=dict)
    clamped: list[str] = Field(default_factory=list)
    unsupported: list[str] = Field(default_factory=list)
    why: dict[str, str] = Field(default_factory=dict)
    session_id: str | None = None
    lease_s: float | None = None
    path: str | None = None
    applied_now: bool = True
    refused: str | None = None
    details: dict[str, Any] = Field(default_factory=dict)


class AppliedFile(BaseConfig):
    """The file a target runs, as it was read.

    Attributes:
        path: The file.
        created: When it was written, ISO 8601, UTC. A string, since the RPC
            codec carries no datetime.
        sha256: The file's bytes as read, hex; a file rewritten since reads
            differently.
    """

    path: str
    created: str
    sha256: str


class ConfigState(BaseConfig):
    """How one target is driven now.

    Attributes:
        target: The target.
        values: How it is driven, by key, as the driver reports it.
        file: The file it runs, or ``None`` for its defaults.
        session_id: The live config session, if any.
    """

    target: str
    values: dict[str, Any] = Field(default_factory=dict)
    file: AppliedFile | None = None
    session_id: str | None = None


class ConfigKeys(BaseConfig):
    """What a connection lets a tool read and change.

    Attributes:
        source: The connection's robot name, the prefix of its joint state
            names ("{source}/{joint}").
        sessions: Whether it serves config sessions (``config_sessions``);
            without them nothing can be set.
        targets: Each target to the names under it, e.g. an arm id to its
            joints in motor order.
        readable: What ``config_get`` reports, by key.
        session: What a session can change, by key.
        persist: What can be saved, by key.
        persist_paths: Each target to the file ``persist`` writes.
        max_lease_s: The longest lease a session may ask for, seconds.
        features: Anything else the driver says it does, by name.
    """

    source: str
    sessions: bool
    targets: dict[str, list[str]] = Field(default_factory=dict)
    readable: list[str] = Field(default_factory=list)
    session: list[str] = Field(default_factory=list)
    persist: list[str] = Field(default_factory=list)
    persist_paths: dict[str, str] = Field(default_factory=dict)
    max_lease_s: float = MAX_CONFIG_LEASE_S
    features: dict[str, Any] = Field(default_factory=dict)
