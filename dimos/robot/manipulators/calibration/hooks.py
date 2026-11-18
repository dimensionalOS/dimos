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

"""What a calibration procedure needs from a running arm connection: the arm config contract.

Every ``ConnectionModule`` serves the contract of
:mod:`dimos.control.connection.arm_config`; :class:`ArmConfigHooks` is the
part a calibration tool calls. ``config_keys`` says what the connection can
read and change, ``config_get`` how each arm is driven now. With
``config_sessions`` on (calibration blueprints only), ``config_set`` with
scope "session" starts the one config session: while it lasts the
connection drops every other position command and the tool moves the arm
with ``config_command``; every ``config_*`` call renews its lease. When it
ends, for any reason, every live change is undone. ``config_set`` with
scope "persist" saves a calibration to the arm's file and applies it.

The arm's joint readings are not part of the contract: the tool reads them
from the connection's ``joint_state`` stream.
"""

from __future__ import annotations

from typing import Protocol, TypedDict, runtime_checkable

from dimos.control.connection.arm_config import (
    DEFAULT_CONFIG_LEASE_S,
    ArmConfigState,
    CommandResult,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    ConfigScope,
)
from dimos.control.connection.connection_module import ConnectionStatus


class AppliedCalibration(TypedDict):
    """The calibration file one arm runs, as ``DamiaoConnectionStatus`` reports it.

    Attributes:
        path: The file.
        created: When it was written, ISO 8601, UTC.
        gains: The joints it gave gains to.
        links: The links it gave masses to.
    """

    path: str
    created: str
    gains: list[str]
    links: list[str]


@runtime_checkable
class ArmConfigHooks(Protocol):
    """The arm config contract as a calibration tool drives it; see ``ConnectionModule``.

    Any ``config_*`` call that raises ends the config session.
    """

    def config_keys(self) -> ConfigKeys:
        """What the connection lets a calibration tool read and change."""
        ...

    def config_get(self) -> dict[str, ArmConfigState]:
        """How each arm is driven now, by arm id."""
        ...

    def config_set(
        self,
        patch: ConfigPatch,
        scope: ConfigScope = "session",
        force: bool = False,
        apply_now: bool = True,
        session_id: str | None = None,
        lease_s: float = DEFAULT_CONFIG_LEASE_S,
    ) -> ConfigApplied:
        """Change one arm for the config session (starting one if none is live), or save it."""
        ...

    def config_reset(self, keep_session: bool = False) -> None:
        """Undo every live change and end the config session, unless ``keep_session``."""
        ...

    def config_command(self, targets: dict[str, float], session_id: str) -> CommandResult:
        """Send position targets by joint name inside the config session."""
        ...

    def status(self) -> ConnectionStatus:
        """The connection's state."""
        ...
