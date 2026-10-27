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

"""A coordinator an operator can hold and e-stop, over streams or RPC."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import ControlTask
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    HoldRoute,
    OperatorEstopStatus,
    OperatorHoldCommand,
    OperatorHoldStatus,
    OperatorHoldTask,
)
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OperatorHoldCoordinator(ControlCoordinator):
    """Add the ports the operator_hold task card reads and reports on, plus an e-stop.

    ``operator_hold_command`` takes an OperatorHoldCommand that requests or
    acknowledges the hold. ``operator_hold_status`` carries every
    OperatorHoldStatus the hold task sends.

    ``operator_estop`` takes a Bool: True stops, False clears. Stop latches
    E-STOP on every task that has ``set_estop``, requests the hold (route
    "failsafe", reason "e-stop") so every joint is frozen where it is, and
    cancels the running trajectory. Clear only releases the E-STOP latch: the
    hold stays on until the operator acknowledges it. ``operator_estop_status``
    sends an OperatorEstopStatus once at start and on every ``set_estop``.

    A page that talks RPC instead of streams (a Desktop app through the dimos
    gateway) uses ``estop`` for the same stop and clear, and
    ``get_operator_status`` for the hold and the latch in plain types.

    Combine it with another coordinator subclass by inheriting from both, e.g.
    ``class MyCoordinator(OperatorHoldCoordinator, TeleopControlCoordinator)``.
    """

    operator_hold_command: In[OperatorHoldCommand]
    operator_hold_status: Out[OperatorHoldStatus]
    operator_estop: In[Bool]
    operator_estop_status: Out[OperatorEstopStatus]

    _estopped: bool = False

    @rpc
    def add_task(
        self,
        task: ControlTask,
        task_type: str | None = None,
        stream_bind: dict[str, str] | None = None,
    ) -> bool:
        """Register a task; a hold task's status also goes out on operator_hold_status."""
        added = super().add_task(task, task_type=task_type, stream_bind=stream_bind)
        if added and isinstance(task, OperatorHoldTask):
            task.set_status_publisher(self.operator_hold_status.publish)
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
                "Could not subscribe to operator_estop; e-stop works over the estop RPC only",
                exc_info=True,
            )
        self._publish_estop_status()

    @rpc
    def set_estop(self, estopped: bool) -> bool:
        """Latch (True) or clear (False) E-STOP on every task that has set_estop.

        The new state also goes out on operator_estop_status.
        """
        try:
            return super().set_estop(estopped)
        finally:
            self._estopped = estopped
            if not estopped:
                logger.info("E-STOP cleared at coordinator")
            self._publish_estop_status()

    @rpc
    def estop(self, estopped: bool) -> dict[str, Any]:
        """Stop (True) or clear (False) as one operator_estop message would.

        Stop latches E-STOP, requests the hold (route "failsafe", reason
        "e-stop") and cancels the running trajectory; clear releases the latch
        only, and the hold stays on until acknowledged. Returns what
        ``get_operator_status`` returns after it.
        """
        self._on_estop_command(Bool(data=estopped))
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
        """Stop (True) or clear (False) from one operator_estop message."""
        if msg.data:
            # Latch first. The hold is requested before the trajectory is
            # cancelled so no tick passes with nothing commanding the arm.
            steps: list[tuple[str, Callable[[], Any]]] = [
                ("set_estop", lambda: self.set_estop(True)),
                (
                    "request hold",
                    lambda: self.task_invoke(
                        OPERATOR_HOLD_TASK_NAME,
                        "request",
                        {"route": HoldRoute.FAILSAFE.value, "reason": "e-stop"},
                    ),
                ),
                ("cancel trajectory", self.cancel_trajectory),
            ]
        else:
            steps = [("set_estop", lambda: self.set_estop(False))]
        for name, step in steps:
            try:
                step()
            except Exception:
                logger.exception("E-stop step failed", step=name, estop=msg.data)

    def _publish_estop_status(self) -> None:
        try:
            self.operator_estop_status.publish(
                OperatorEstopStatus(estopped=self._estopped, stamp=time.time())
            )
        except Exception:
            logger.exception("Publishing e-stop status failed")
