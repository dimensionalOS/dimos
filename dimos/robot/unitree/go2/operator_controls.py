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

"""Local Cockpit Go2 actions and feedback over typed web channels."""

from dataclasses import dataclass
import threading
import time
from typing import Any

import reactivex as rx
from reactivex.disposable import Disposable
from unitree_webrtc_connect.constants import SPORT_CMD

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.utils.logging_config import setup_logger
from dimos.web.codecs import web_decoder

logger = setup_logger()

SPORT_ACTIONS = {
    name: SPORT_CMD[name] for name in ("StandDown", "RecoveryStand", "Hello", "Stretch")
}


@dataclass(frozen=True)
class OperatorCommand:
    id: str
    action: str
    level: int | None = None


@web_decoder("go2.operator.json.v1")
def decode_operator_command(value: Any) -> OperatorCommand:
    if not isinstance(value, dict) or set(value) - {"id", "action", "level"}:
        raise ValueError("Expected an operator command")
    request_id = value.get("id")
    action = value.get("action")
    if not isinstance(request_id, str) or not 1 <= len(request_id) <= 80:
        raise ValueError("Invalid command id")
    if not isinstance(action, str) or action not in {*SPORT_ACTIONS, "StandReady", "light"}:
        raise ValueError("Unsupported Go2 action")
    level = value.get("level")
    if action == "light":
        if type(level) is not int or not 0 <= level <= 10:
            raise ValueError("Light level must be an integer from 0 to 10")
    elif level is not None:
        raise ValueError("Only light accepts a level")
    return OperatorCommand(request_id, action, level)


@dataclass(frozen=True)
class OperatorState:
    battery: int | None
    light_requested: int | None
    last_action: str | None
    ts: float


@dataclass(frozen=True)
class OperatorResult:
    id: str
    action: str
    ok: bool
    message: str


class Go2OperatorControls(Module):
    go2: GO2Connection
    go2_operator_command: In[OperatorCommand]
    go2_operator_state: Out[OperatorState]
    go2_operator_result: Out[OperatorResult]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._command_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._light_requested: int | None = None
        self._last_action: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.go2_operator_command.subscribe(self._command)))
        self.register_disposable(rx.interval(1.0).subscribe(lambda _: self._publish_state()))

    def _publish_state(self) -> None:
        try:
            battery = self.go2.battery_soc()
            with self._state_lock:
                state = OperatorState(
                    battery, self._light_requested, self._last_action, time.time()
                )
            self.go2_operator_state.publish(state)
        except Exception:
            logger.warning("Operator telemetry unavailable", exc_info=True)

    def _command(self, command: OperatorCommand) -> None:
        if not self._command_lock.acquire(blocking=False):
            self.go2_operator_result.publish(
                OperatorResult(command.id, command.action, False, "Another action is running")
            )
            return
        try:
            if command.action == "light":
                ok = command.level is not None and self.go2.set_light(command.level)
            elif command.action == "StandReady":
                ok = self.go2.standup() and self.go2.balance_stand()
            elif command.action in SPORT_ACTIONS:
                ok = self.go2.sport_command(SPORT_ACTIONS[command.action])
            else:
                ok = False
            if ok:
                with self._state_lock:
                    if command.action == "light":
                        self._light_requested = command.level
                    else:
                        self._last_action = command.action
            self.go2_operator_result.publish(
                OperatorResult(
                    command.id,
                    command.action,
                    bool(ok),
                    "Robot API accepted" if ok else "Robot API rejected",
                )
            )
        except Exception:
            logger.exception("Operator command failed", action=command.action)
            self.go2_operator_result.publish(
                OperatorResult(
                    command.id, command.action, False, "Robot API failed; outcome unknown"
                )
            )
        finally:
            self._command_lock.release()
