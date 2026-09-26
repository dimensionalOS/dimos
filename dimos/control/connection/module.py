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

"""The calls every robot driver answers, added to a module in one line.

A driver module mixes this in::

    class XArmConnection(Module, ConnectionRpcMixin):
        ...

and gets the two ports a driver talks on plus every lifecycle call, each
passed to the ``ConnectedHardware`` objects the module has registered. A
module running two pieces of hardware, such as a body and the wheels under
it, registers both.

Every lifecycle call takes an optional ``source`` naming one piece of hardware
and returns one answer per piece it acted on. Leave ``source`` out to act on
all of them. A ``source`` this module does not run returns no answers.
"""

from __future__ import annotations

from collections.abc import Callable

from reactivex.disposable import Disposable

from dimos.control.connection.connected_hardware import ConnectedHardware
from dimos.control.connection.status import ConnectionStatus, LifecycleAck
from dimos.control.contract.description import ControlDescription
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.control_msgs.ControlValues import ControlValues


class ConnectionRpcMixin:
    """Ports and lifecycle calls for a robot driver module. Mix into a ``Module``."""

    control_state: Out[ControlValues]
    control_command: In[ControlValues]

    def register_hardware(self, hardware: ConnectedHardware) -> None:
        """Make a piece of hardware answer this module's lifecycle calls.

        Also arranges for it to be stopped when the module stops, so a robot
        is brought to rest even if the driver's own ``stop`` forgets to.
        """
        registered = self._registered_hardware()
        registered.append(hardware)
        register_disposable = getattr(self, "register_disposable", None)
        if register_disposable is not None:
            register_disposable(Disposable(hardware.stop))

    @rpc
    def describe_control(self) -> list[ControlDescription]:
        """What each piece of hardware is, one description per piece."""
        return [hw.describe_control() for hw in self._registered_hardware()]

    @rpc
    def status(self) -> list[ConnectionStatus]:
        """A snapshot of each piece of hardware."""
        return [hw.status() for hw in self._registered_hardware()]

    @rpc
    def prepare_arm(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]:
        """Get hardware ready to move. See ``ConnectedHardware.prepare_arm``.

        Args:
            operation_id: A number for this request, so a retry gets the same
                answer.
            source: The one piece of hardware to act on, or ``None`` for all.
        """
        return self._each(source, lambda hw: hw.prepare_arm(operation_id))

    @rpc
    def commit_arm(
        self, operation_id: int, epoch: int, source: str | None = None
    ) -> list[LifecycleAck]:
        """Start obeying commands carrying ``epoch``. See
        ``ConnectedHardware.commit_arm``.

        Args:
            operation_id: A number for this request.
            epoch: The arming number commands will carry. Zero or more.
            source: The one piece of hardware to act on, or ``None`` for all.
        """
        return self._each(source, lambda hw: hw.commit_arm(operation_id, epoch))

    @rpc
    def abort_arm(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]:
        """Give up on arming, back to standby. See ``ConnectedHardware.abort_arm``."""
        return self._each(source, lambda hw: hw.abort_arm(operation_id))

    @rpc
    def safe_stop(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]:
        """Stop and stay stopped. See ``ConnectedHardware.safe_stop``."""
        return self._each(source, lambda hw: hw.safe_stop(operation_id))

    @rpc
    def clear_safe_stop(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]:
        """Release a stop, back to standby. See ``ConnectedHardware.clear_safe_stop``."""
        return self._each(source, lambda hw: hw.clear_safe_stop(operation_id))

    @rpc
    def estop(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]:
        """Stop in an emergency and stay stopped. See ``ConnectedHardware.estop``."""
        return self._each(source, lambda hw: hw.estop(operation_id))

    @rpc
    def clear_estop(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]:
        """Release an emergency stop, back to standby. See
        ``ConnectedHardware.clear_estop``."""
        return self._each(source, lambda hw: hw.clear_estop(operation_id))

    def _registered_hardware(self) -> list[ConnectedHardware]:
        registered: list[ConnectedHardware] | None = getattr(self, "_connected_hardware", None)
        if registered is None:
            registered = []
            self._connected_hardware = registered
        return registered

    def _each(
        self, source: str | None, call: Callable[[ConnectedHardware], LifecycleAck]
    ) -> list[LifecycleAck]:
        return [
            call(hw) for hw in self._registered_hardware() if source is None or hw.source == source
        ]
