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

"""What a calibration procedure needs from a running arm connection: the config contract.

Every ``ConnectionModule`` serves the contract of
:mod:`dimos.control.connection.module_config`; :class:`ArmConfigHooks` is the
part a calibration tool calls, with one target per arm id and the arm's keys
of :mod:`dimos.control.calibration.schema`: patches are built as
``ArmConfigPatch`` and sent with ``to_patch``, states read with
``ArmConfigState.from_state``. ``config_keys`` says what the connection can
read and change, ``config_get`` how each arm is driven now. With
``config_sessions`` on (calibration blueprints only), ``config_set`` with
scope "session" starts the one config session; each ``config_set`` that
carries its id renews its lease (an empty patch only renews it), reading and
``config_reset`` do not. When it ends, for any reason, every live change is
undone. ``config_set`` with scope "persist" saves a calibration to the arm's
file and applies it.

The arm is not moved through the contract, nor read: the tool's targets go
through the control coordinator's pass-through task to the connection's
``position_command``, as any position command does, and the tool reads the
arm from the connection's ``joint_state`` stream.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from dimos.control.connection.connection_module import ConnectionStatus
from dimos.control.connection.module_config import (
    DEFAULT_CONFIG_LEASE_S,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    ConfigScope,
    ConfigState,
)


@runtime_checkable
class ArmConfigHooks(Protocol):
    """The config contract as a calibration tool drives it; see ``ConnectionModule``.

    Any ``config_*`` call that raises ends the config session.
    """

    def config_keys(self) -> ConfigKeys:
        """What the connection lets a calibration tool read and change."""
        ...

    def config_get(self) -> dict[str, ConfigState]:
        """How each arm is driven now, by arm id (``ArmConfigState.from_state``)."""
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

    def status(self) -> ConnectionStatus:
        """The connection's state."""
        ...
