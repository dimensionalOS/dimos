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

"""DDS stock firmware device owned by G1Connection."""

from __future__ import annotations

from enum import IntEnum
import json
import time
from typing import TYPE_CHECKING, Any

from dimos.control.contract.convert import twist_from_values

if TYPE_CHECKING:
    from dimos.robot.unitree.g1.connection import G1ConnectionConfig


class FsmState(IntEnum):
    ZERO_TORQUE = 0
    DAMP = 1
    SIT = 3
    AI_MODE = 200
    LIE_TO_STANDUP = 702
    SQUAT_STANDUP_TOGGLE = 706


def _boost_above_deadzone(value: float, minimum: float) -> float:
    if value == 0.0 or abs(value) >= minimum:
        return value
    return minimum if value > 0 else -minimum


def _require_success(code: int | None, operation: str) -> None:
    if code not in (None, 0):
        raise RuntimeError(f"G1 DDS {operation} failed with code {code}")


class G1DdsDevice:
    _standup_step_delay = 3.0

    def __init__(self, config: G1ConnectionConfig) -> None:
        self._config = config
        self._loco: Any = None
        self._motion_switcher: Any = None
        self._get_fsm_api: int | None = None

    def start(self) -> None:
        # Unitree DDS is an optional hardware dependency.
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        if self._config.network_interface:
            ChannelFactoryInitialize(0, self._config.network_interface)
        else:
            ChannelFactoryInitialize(0)
        self._motion_switcher = MotionSwitcherClient()
        self._motion_switcher.SetTimeout(self._config.motion_switcher_timeout)
        self._motion_switcher.Init()
        self._loco = LocoClient()
        self._loco.SetTimeout(self._config.loco_client_timeout)
        self._loco.Init()
        self._get_fsm_api = ROBOT_API_ID_LOCO_GET_FSM_ID
        self._loco._RegistApi(self._get_fsm_api, 0)
        code, _ = self._motion_switcher.SelectMode(self._config.connection_mode)
        _require_success(code, "select motion mode")

    def _connected(self) -> Any:
        if self._loco is None:
            raise RuntimeError("G1 DDS device is not connected")
        return self._loco

    def write(self, values: dict[str, float]) -> None:
        twist = twist_from_values(values, f"{self._config.source}/base")
        vx = _boost_above_deadzone(twist.linear.x, self._config.min_effective_linear_velocity)
        vy = _boost_above_deadzone(twist.linear.y, self._config.min_effective_linear_velocity)
        wz = _boost_above_deadzone(twist.angular.z, self._config.min_effective_angular_velocity)
        _require_success(self._connected().Move(vx, vy, wz, continous_move=True), "move")

    def halt(self) -> None:
        if self._loco is not None:
            _require_success(self._loco.StopMove(), "halt")

    def stop(self) -> None:
        self._loco = None
        self._motion_switcher = None
        self._get_fsm_api = None

    def _fsm(self) -> int:
        code, data = self._connected()._Call(self._get_fsm_api, "{}")
        _require_success(code, "read state")
        result = json.loads(data) if isinstance(data, str) else data
        value = result.get("data") if isinstance(result, dict) else result
        if not isinstance(value, int):
            raise RuntimeError("G1 DDS state response contains no FSM id")
        return value

    def get_state(self) -> str:
        fsm_id = self._fsm()
        try:
            return FsmState(fsm_id).name
        except ValueError:
            return f"UNKNOWN_{fsm_id}"

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        if topic not in ("rt/api/sport/request", "rt/api/loco/request"):
            raise NotImplementedError(f"G1 DDS does not implement vendor requests on {topic!r}")
        loco = self._connected()
        api_id = data.get("api_id")
        parameter = data.get("parameter", {})
        if api_id == 7101:
            code = loco.SetFsmId(parameter["data"])
        elif api_id == 7105:
            velocity = parameter["velocity"]
            code = loco.SetVelocity(*velocity, parameter.get("duration", 1.0))
        else:
            raise NotImplementedError(f"G1 DDS does not implement API {api_id!r}")
        _require_success(code, "vendor request")
        return {"code": 0}

    def stand_up(self) -> bool:
        loco = self._connected()
        if self._config.ai_standup:
            fsm = self._fsm()
            if fsm == FsmState.ZERO_TORQUE:
                _require_success(loco.SetFsmId(FsmState.DAMP), "enable damping")
                time.sleep(self._standup_step_delay / 3)
                fsm = self._fsm()
            if fsm != FsmState.AI_MODE:
                _require_success(loco.SetFsmId(FsmState.AI_MODE), "select AI mode")
                time.sleep(self._standup_step_delay / 2)
        else:
            _require_success(loco.SetFsmId(FsmState.DAMP), "enable damping")
            time.sleep(self._standup_step_delay / 3)
        _require_success(loco.SetFsmId(FsmState.SQUAT_STANDUP_TOGGLE), "stand up")
        time.sleep(self._standup_step_delay)
        return True

    def lie_down(self) -> bool:
        loco = self._connected()
        _require_success(loco.StandUp2Squat(), "squat")
        time.sleep(self._standup_step_delay / 3)
        _require_success(loco.Damp(), "enable damping")
        return True
