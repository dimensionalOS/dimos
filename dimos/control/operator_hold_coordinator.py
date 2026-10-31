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

"""A coordinator an operator can hold and e-stop, with on/off streams for a page."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import ControlTask
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldStatus,
    OperatorHoldTask,
)
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OperatorHoldCoordinator(ControlCoordinator):
    """A ControlCoordinator with on/off streams for the hold and the e-stop.

    The e-stop is ControlCoordinator's own ``set_estop``. The hold is the "operator_hold" task, reached over ``task_invoke``
    ("request", "acknowledge", "get_status"); an e-stop turns it on and it
    stays on until acknowledged.

    ``operator_hold_status`` is True while the hold is on, sent each time the
    hold task sends its status. ``operator_estop_status`` is True while the
    e-stop is latched, sent at start and on every ``set_estop``.

    ``operator_estop`` takes a Bool, True to stop and False to clear: a page's
    stop that does not depend on an RPC round trip. ``estop`` and
    ``get_operator_status`` are the same for a page that talks RPC, in plain types.

    Combine it with another coordinator subclass by inheriting from both, e.g.
    ``class MyCoordinator(OperatorHoldCoordinator, TeleopControlCoordinator)``.
    """

    operator_hold_status: Out[Bool]
    operator_estop_status: Out[Bool]
    operator_estop: In[Bool]

    @rpc
    def add_task(
        self,
        task: ControlTask,
        task_type: str | None = None,
        stream_bind: dict[str, str] | None = None,
    ) -> bool:
        """Register a task; a hold task also sends its on/off on operator_hold_status."""
        added = super().add_task(task, task_type=task_type, stream_bind=stream_bind)
        if added and isinstance(task, OperatorHoldTask):
            task.set_status_publisher(
                lambda status: self.operator_hold_status.publish(Bool(data=status.on))
            )
        return added

    @rpc
    def start(self) -> None:
        """Start the coordinator, listen on operator_estop and send the current e-stop state."""
        super().start()
        try:
            self.register_disposable(
                Disposable(self.operator_estop.subscribe(self._on_estop_command))
            )
        except Exception:
            logger.warning(
                "Could not subscribe to operator_estop; e-stop works over RPC only", exc_info=True
            )
        self._publish_estop_status()

    @rpc
    def set_estop(self, estopped: bool) -> bool:
        """Stop (True) or clear (False) like ControlCoordinator, then send the state.

        Args:
            estopped: True to stop, False to clear.
        """
        try:
            return super().set_estop(estopped)
        finally:
            self._publish_estop_status()

    @rpc
    def estop(self, estopped: bool) -> dict[str, Any]:
        """``set_estop``, answering with what ``get_operator_status`` returns after it.

        Args:
            estopped: True to stop, False to clear.
        """
        self.set_estop(estopped)
        return self.get_operator_status()

    @rpc
    def get_operator_status(self) -> dict[str, Any]:
        """The hold and the e-stop latch, in plain types.

        Returns ``{"hold": {...OperatorHoldStatus fields}, "estopped": bool}``;
        ``hold`` is None when this coordinator runs no operator_hold task.
        """
        hold: dict[str, Any] | None = None
        try:
            status = self.task_invoke(OPERATOR_HOLD_TASK_NAME, "get_status", {})
        except Exception:
            logger.warning("Reading the operator hold failed", exc_info=True)
            status = None
        if isinstance(status, OperatorHoldStatus):
            hold = {**asdict(status), "unheld": list(status.unheld)}
        return {"hold": hold, "estopped": self._estopped}

    def _on_estop_command(self, msg: Bool) -> None:
        try:
            self.set_estop(bool(msg.data))
        except Exception:
            logger.exception("E-stop command failed", estopped=msg.data)

    def _publish_estop_status(self) -> None:
        try:
            self.operator_estop_status.publish(Bool(data=self._estopped))
        except Exception:
            logger.exception("Publishing e-stop status failed")
