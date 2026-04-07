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

"""Tests for G1 high-level control modules (DDS SDK and WebRTC)."""

from __future__ import annotations

from enum import IntEnum
import json
import sys
from typing import Any
from unittest.mock import MagicMock, call, patch

import pytest


# Stub out unitree_sdk2py so we can import dds_sdk without the real SDK
def _install_sdk_stubs() -> dict[str, MagicMock]:
    stubs: dict[str, MagicMock] = {}
    for mod_name in [
        "unitree_sdk2py",
        "unitree_sdk2py.comm",
        "unitree_sdk2py.comm.motion_switcher",
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client",
        "unitree_sdk2py.core",
        "unitree_sdk2py.core.channel",
        "unitree_sdk2py.g1",
        "unitree_sdk2py.g1.loco",
        "unitree_sdk2py.g1.loco.g1_loco_api",
        "unitree_sdk2py.g1.loco.g1_loco_client",
    ]:
        mock = MagicMock()
        stubs[mod_name] = mock
        sys.modules[mod_name] = mock

    # Wire up named attributes the module actually imports
    api_mod = stubs["unitree_sdk2py.g1.loco.g1_loco_api"]
    api_mod.ROBOT_API_ID_LOCO_GET_FSM_ID = 7001
    api_mod.ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002
    api_mod.ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003

    client_mod = stubs["unitree_sdk2py.g1.loco.g1_loco_client"]
    client_mod.LocoClient = MagicMock

    switcher_mod = stubs["unitree_sdk2py.comm.motion_switcher.motion_switcher_client"]
    switcher_mod.MotionSwitcherClient = MagicMock

    channel_mod = stubs["unitree_sdk2py.core.channel"]
    channel_mod.ChannelFactoryInitialize = MagicMock()

    return stubs


# Stub out unitree_webrtc_connect too
def _install_webrtc_stubs() -> dict[str, MagicMock]:
    stubs: dict[str, MagicMock] = {}
    for mod_name in [
        "unitree_webrtc_connect",
        "unitree_webrtc_connect.constants",
        "unitree_webrtc_connect.webrtc_driver",
    ]:
        mock = MagicMock()
        stubs[mod_name] = mock
        sys.modules[mod_name] = mock

    constants = stubs["unitree_webrtc_connect.constants"]
    constants.RTC_TOPIC = "rt/topic"
    constants.SPORT_CMD = "sport_cmd"
    # VUI_COLOR is used both as a type and a value (VUI_COLOR.RED) in connection.py
    constants.VUI_COLOR = MagicMock()

    driver = stubs["unitree_webrtc_connect.webrtc_driver"]
    driver.UnitreeWebRTCConnection = MagicMock
    driver.WebRTCConnectionMethod = MagicMock()

    return stubs


_sdk_stubs = _install_sdk_stubs()
_webrtc_stubs = _install_webrtc_stubs()

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import (
    FsmState,
    G1HighLevelDdsSdk,
    G1HighLevelDdsSdkConfig,
)
from dimos.robot.unitree.g1.effectors.high_level.webrtc import (
    _ARM_COMMANDS,
    _MODE_COMMANDS,
    G1_ARM_CONTROLS,
    G1_MODE_CONTROLS,
    G1HighLevelWebRtc,
    G1HighLevelWebRtcConfig,
)

# FsmState enum tests


class TestFsmState:
    def test_is_int_enum(self) -> None:
        assert issubclass(FsmState, IntEnum)

    def test_values(self) -> None:
        assert FsmState.ZERO_TORQUE == 0  # type: ignore[comparison-overlap]
        assert FsmState.DAMP == 1  # type: ignore[comparison-overlap]
        assert FsmState.SIT == 3  # type: ignore[comparison-overlap]
        assert FsmState.AI_MODE == 200  # type: ignore[comparison-overlap]
        assert FsmState.LIE_TO_STANDUP == 702  # type: ignore[comparison-overlap]
        assert FsmState.SQUAT_STANDUP_TOGGLE == 706  # type: ignore[comparison-overlap]

    def test_name_lookup(self) -> None:
        assert FsmState(0).name == "ZERO_TORQUE"
        assert FsmState(1).name == "DAMP"
        assert FsmState(200).name == "AI_MODE"
        assert FsmState(706).name == "SQUAT_STANDUP_TOGGLE"

    def test_int_comparison(self) -> None:
        assert FsmState.DAMP == 1  # type: ignore[comparison-overlap]
        assert FsmState.AI_MODE != 0  # type: ignore[comparison-overlap]

    def test_unknown_value_raises(self) -> None:
        with pytest.raises(ValueError):
            FsmState(999)

    def test_iteration(self) -> None:
        names = [s.name for s in FsmState]
        assert "ZERO_TORQUE" in names
        assert "AI_MODE" in names
        assert len(names) == 6


# Config tests


class TestDdsSdkConfig:
    def test_defaults(self) -> None:
        cfg = G1HighLevelDdsSdkConfig()
        assert cfg.ip is None
        assert cfg.network_interface == "eth0"
        assert cfg.connection_mode == "ai"
        assert cfg.ai_standup is True
        assert cfg.motion_switcher_timeout == 5.0
        assert cfg.loco_client_timeout == 10.0
        assert cfg.cmd_vel_timeout == 0.2

    def test_override(self) -> None:
        cfg = G1HighLevelDdsSdkConfig(
            ip="192.168.1.1",
            ai_standup=False,
            cmd_vel_timeout=0.5,
        )
        assert cfg.ip == "192.168.1.1"
        assert cfg.ai_standup is False
        assert cfg.cmd_vel_timeout == 0.5


class TestWebRtcConfig:
    def test_defaults(self) -> None:
        cfg = G1HighLevelWebRtcConfig()
        assert cfg.ip is None
        assert cfg.connection_mode == "ai"


# DDS SDK module tests (mocked)


def _make_dds_module(**config_overrides: Any) -> G1HighLevelDdsSdk:
    """Create a G1HighLevelDdsSdk with mocked internals."""
    gc = MagicMock()
    with patch.object(G1HighLevelDdsSdk, "__init__", lambda self, *a, **kw: None):
        mod = G1HighLevelDdsSdk.__new__(G1HighLevelDdsSdk)

    mod.config = G1HighLevelDdsSdkConfig(**config_overrides)
    mod._global_config = gc
    mod._stop_timer = None
    mod._running = False
    mod._mode_selected = False
    mod.motion_switcher = MagicMock()
    mod.loco_client = MagicMock()
    mod._standup_step_delay = 0.0  # no real sleeps in tests
    return mod


class TestDdsSdkGetState:
    def test_known_fsm(self) -> None:
        mod = _make_dds_module()
        mod.loco_client._Call.return_value = (0, json.dumps({"data": 0}))
        assert mod.get_state() == "ZERO_TORQUE"

    def test_ai_mode_fsm(self) -> None:
        mod = _make_dds_module()
        mod.loco_client._Call.return_value = (0, json.dumps({"data": 200}))
        assert mod.get_state() == "AI_MODE"

    def test_unknown_fsm(self) -> None:
        mod = _make_dds_module()
        mod.loco_client._Call.return_value = (0, json.dumps({"data": 999}))
        assert mod.get_state() == "UNKNOWN_999"

    def test_query_failed(self) -> None:
        mod = _make_dds_module()
        mod.loco_client._Call.return_value = (1, None)
        assert mod.get_state() == "Unknown (query failed)"

    def test_call_raises(self) -> None:
        mod = _make_dds_module()
        mod.loco_client._Call.side_effect = RuntimeError("timeout")
        assert mod.get_state() == "Unknown (query failed)"


class TestDdsSdkStandUp:
    def test_ai_standup_from_zero_torque(self) -> None:
        mod = _make_dds_module(ai_standup=True)
        mod.loco_client._Call.return_value = (0, json.dumps({"data": FsmState.ZERO_TORQUE}))
        result = mod.stand_up()
        assert result is True
        calls = mod.loco_client.SetFsmId.call_args_list
        assert calls[0] == call(FsmState.DAMP)
        assert calls[1] == call(FsmState.AI_MODE)
        assert calls[2] == call(FsmState.SQUAT_STANDUP_TOGGLE)

    def test_ai_standup_already_ai_mode(self) -> None:
        mod = _make_dds_module(ai_standup=True)
        mod.loco_client._Call.return_value = (0, json.dumps({"data": FsmState.AI_MODE}))
        result = mod.stand_up()
        assert result is True
        calls = mod.loco_client.SetFsmId.call_args_list
        # Should skip DAMP and AI_MODE, go straight to toggle
        assert len(calls) == 1
        assert calls[0] == call(FsmState.SQUAT_STANDUP_TOGGLE)

    def test_normal_standup(self) -> None:
        mod = _make_dds_module(ai_standup=False)
        result = mod.stand_up()
        assert result is True
        calls = mod.loco_client.SetFsmId.call_args_list
        assert calls[0] == call(FsmState.DAMP)
        assert calls[1] == call(FsmState.SQUAT_STANDUP_TOGGLE)

    def test_standup_exception(self) -> None:
        mod = _make_dds_module(ai_standup=False)
        mod.loco_client.SetFsmId.side_effect = RuntimeError("comms lost")
        result = mod.stand_up()
        assert result is False


class TestDdsSdkLieDown:
    def test_lie_down(self) -> None:
        mod = _make_dds_module()
        result = mod.lie_down()
        assert result is True
        mod.loco_client.StandUp2Squat.assert_called_once()
        mod.loco_client.Damp.assert_called_once()

    def test_lie_down_exception(self) -> None:
        mod = _make_dds_module()
        mod.loco_client.StandUp2Squat.side_effect = RuntimeError("err")
        result = mod.lie_down()
        assert result is False


class TestDdsSdkMove:
    def test_move_with_duration(self) -> None:
        mod = _make_dds_module()
        mod.loco_client.SetVelocity.return_value = 0
        twist = Twist(linear=Vector3(1.0, 0.5, 0), angular=Vector3(0, 0, 0.3))
        result = mod.move(twist, duration=2.0)
        assert result is True
        mod.loco_client.SetVelocity.assert_called_once_with(1.0, 0.5, 0.3, 2.0)

    def test_move_with_duration_error_code(self) -> None:
        mod = _make_dds_module()
        mod.loco_client.SetVelocity.return_value = -1
        twist = Twist(linear=Vector3(1.0, 0, 0), angular=Vector3(0, 0, 0))
        result = mod.move(twist, duration=1.0)
        assert result is False

    def test_move_continuous(self) -> None:
        mod = _make_dds_module()
        twist = Twist(linear=Vector3(0.5, 0, 0), angular=Vector3(0, 0, 0.1))
        result = mod.move(twist)
        assert result is True
        mod.loco_client.Move.assert_called_once_with(0.5, 0, 0.1, continous_move=True)
        # Timer should have been started
        assert mod._stop_timer is not None
        mod._stop_timer.cancel()
        mod._stop_timer.join()  # wait for thread to finish

    def test_move_exception(self) -> None:
        mod = _make_dds_module()
        mod.loco_client.SetVelocity.side_effect = RuntimeError("err")
        twist = Twist(linear=Vector3(1.0, 0, 0), angular=Vector3(0, 0, 0))
        result = mod.move(twist, duration=1.0)
        assert result is False


class TestDdsSdkPublishRequest:
    def test_set_fsm_id(self) -> None:
        mod = _make_dds_module()
        mod.loco_client.SetFsmId.return_value = 0
        result = mod.publish_request("topic", {"api_id": 7101, "parameter": {"data": 200}})
        assert result == {"code": 0}
        mod.loco_client.SetFsmId.assert_called_once_with(200)

    def test_set_velocity(self) -> None:
        mod = _make_dds_module()
        mod.loco_client.SetVelocity.return_value = 0
        result = mod.publish_request(
            "topic",
            {"api_id": 7105, "parameter": {"velocity": [1.0, 0.5, 0.2], "duration": 3.0}},
        )
        assert result == {"code": 0}
        mod.loco_client.SetVelocity.assert_called_once_with(1.0, 0.5, 0.2, 3.0)

    def test_unsupported_api(self) -> None:
        mod = _make_dds_module()
        result = mod.publish_request("topic", {"api_id": 9999})
        assert result["code"] == -1
        assert result["error"] == "unsupported_api"

    def test_exception(self) -> None:
        mod = _make_dds_module()
        mod.loco_client.SetFsmId.side_effect = RuntimeError("boom")
        result = mod.publish_request("topic", {"api_id": 7101, "parameter": {"data": 1}})
        assert result["code"] == -1
        assert "boom" in result["error"]


# WebRTC module tests (mocked)


def _make_webrtc_module(**config_overrides: Any) -> G1HighLevelWebRtc:
    with patch.object(G1HighLevelWebRtc, "__init__", lambda self, *a, **kw: None):
        mod = G1HighLevelWebRtc.__new__(G1HighLevelWebRtc)

    mod.config = G1HighLevelWebRtcConfig(**config_overrides)
    mod._global_config = MagicMock()
    mod.connection = MagicMock()
    return mod


class TestWebRtcConstants:
    def test_arm_controls_structure(self) -> None:
        for name, id_, desc in G1_ARM_CONTROLS:
            assert isinstance(name, str)
            assert isinstance(id_, int)
            assert isinstance(desc, str)

    def test_mode_controls_structure(self) -> None:
        for name, id_, desc in G1_MODE_CONTROLS:
            assert isinstance(name, str)
            assert isinstance(id_, int)
            assert isinstance(desc, str)

    def test_arm_commands_dict(self) -> None:
        assert "Handshake" in _ARM_COMMANDS
        assert "CancelAction" in _ARM_COMMANDS
        assert len(_ARM_COMMANDS) == len(G1_ARM_CONTROLS)

    def test_mode_commands_dict(self) -> None:
        assert "WalkMode" in _MODE_COMMANDS
        assert "RunMode" in _MODE_COMMANDS
        assert len(_MODE_COMMANDS) == len(G1_MODE_CONTROLS)


class TestWebRtcGetState:
    def test_connected(self) -> None:
        mod = _make_webrtc_module()
        assert mod.get_state() == "Connected (WebRTC)"

    def test_not_connected(self) -> None:
        mod = _make_webrtc_module()
        mod.connection = None
        assert mod.get_state() == "Not connected"


class TestWebRtcMove:
    def test_move_delegates(self) -> None:
        mod = _make_webrtc_module()
        mod.connection.move.return_value = True  # type: ignore[union-attr]
        twist = Twist(linear=Vector3(1.0, 0, 0), angular=Vector3(0, 0, 0))
        assert mod.move(twist, duration=2.0) is True
        mod.connection.move.assert_called_once_with(twist, 2.0)  # type: ignore[union-attr]


class TestWebRtcStandUp:
    def test_stand_up_delegates(self) -> None:
        mod = _make_webrtc_module()
        mod.connection.standup.return_value = True  # type: ignore[union-attr]
        assert mod.stand_up() is True
        mod.connection.standup.assert_called_once()  # type: ignore[union-attr]


class TestWebRtcLieDown:
    def test_lie_down_delegates(self) -> None:
        mod = _make_webrtc_module()
        mod.connection.liedown.return_value = True  # type: ignore[union-attr]
        assert mod.lie_down() is True
        mod.connection.liedown.assert_called_once()  # type: ignore[union-attr]


class TestWebRtcPublishRequest:
    def test_delegates(self) -> None:
        mod = _make_webrtc_module()
        mod.connection.publish_request.return_value = {"code": 0}  # type: ignore[union-attr]
        result = mod.publish_request("topic", {"api_id": 7101})
        assert result == {"code": 0}


class TestWebRtcArmCommand:
    def test_valid_command(self) -> None:
        mod = _make_webrtc_module()
        mod.connection.publish_request.return_value = {"code": 0}  # type: ignore[union-attr]
        result = mod.execute_arm_command("Handshake")
        assert "successfully" in result

    def test_invalid_command(self) -> None:
        mod = _make_webrtc_module()
        result = mod.execute_arm_command("NotARealCommand")
        assert "no" in result.lower() or "There's" in result


class TestWebRtcModeCommand:
    def test_valid_command(self) -> None:
        mod = _make_webrtc_module()
        mod.connection.publish_request.return_value = {"code": 0}  # type: ignore[union-attr]
        result = mod.execute_mode_command("WalkMode")
        assert "successfully" in result

    def test_invalid_command(self) -> None:
        mod = _make_webrtc_module()
        result = mod.execute_mode_command("FlyMode")
        assert "no" in result.lower() or "There's" in result


# FSM State Machine model + transition tests


class FsmSimulator:
    """Models the valid FSM transitions of the Unitree G1.

    Used to verify that stand_up / lie_down issue commands in a
    valid order.
    """

    VALID_TRANSITIONS: dict[FsmState, set[FsmState]] = {
        FsmState.ZERO_TORQUE: {FsmState.DAMP},
        FsmState.DAMP: {FsmState.AI_MODE, FsmState.SQUAT_STANDUP_TOGGLE, FsmState.ZERO_TORQUE},
        FsmState.SIT: {FsmState.DAMP, FsmState.SQUAT_STANDUP_TOGGLE},
        FsmState.AI_MODE: {FsmState.SQUAT_STANDUP_TOGGLE, FsmState.DAMP, FsmState.ZERO_TORQUE},
        FsmState.LIE_TO_STANDUP: {FsmState.DAMP, FsmState.SIT},
        FsmState.SQUAT_STANDUP_TOGGLE: {
            FsmState.DAMP,
            FsmState.AI_MODE,
            FsmState.SIT,
            FsmState.SQUAT_STANDUP_TOGGLE,
        },
    }

    def __init__(self, initial: FsmState = FsmState.ZERO_TORQUE) -> None:
        self.state = initial
        self.history: list[FsmState] = [initial]

    def transition(self, target: FsmState) -> None:
        # Self-transitions are no-ops on the real robot
        if target == self.state:
            self.history.append(target)
            return
        valid = self.VALID_TRANSITIONS.get(self.state, set())
        if target not in valid:
            raise ValueError(
                f"Invalid transition: {self.state.name} -> {target.name}. "
                f"Valid targets: {[s.name for s in valid]}"
            )
        self.state = target
        self.history.append(target)


def _make_dds_with_fsm_sim(
    initial_state: FsmState, *, ai_standup: bool = True
) -> tuple[G1HighLevelDdsSdk, FsmSimulator]:
    """Build a DDS module whose loco_client tracks an FsmSimulator."""
    sim = FsmSimulator(initial_state)
    mod = _make_dds_module(ai_standup=ai_standup)

    def mock_set_fsm_id(fsm_id: int) -> int:
        sim.transition(FsmState(fsm_id))
        return 0

    def mock_call(api_id: int, payload: str) -> tuple[int, str]:
        return (0, json.dumps({"data": int(sim.state)}))

    mod.loco_client.SetFsmId.side_effect = mock_set_fsm_id
    mod.loco_client._Call.side_effect = mock_call

    # StandUp2Squat is the high-level SDK wrapper around SQUAT_STANDUP_TOGGLE
    def mock_standup2squat() -> None:
        sim.transition(FsmState.SQUAT_STANDUP_TOGGLE)

    def mock_damp() -> None:
        sim.transition(FsmState.DAMP)

    mod.loco_client.StandUp2Squat.side_effect = mock_standup2squat
    mod.loco_client.Damp.side_effect = mock_damp

    return mod, sim


class TestFsmSimulator:
    def test_valid_transition(self) -> None:
        sim = FsmSimulator(FsmState.ZERO_TORQUE)
        sim.transition(FsmState.DAMP)
        assert sim.state == FsmState.DAMP

    def test_invalid_transition_raises(self) -> None:
        sim = FsmSimulator(FsmState.ZERO_TORQUE)
        with pytest.raises(ValueError, match="Invalid transition"):
            sim.transition(FsmState.AI_MODE)

    def test_history_tracking(self) -> None:
        sim = FsmSimulator(FsmState.ZERO_TORQUE)
        sim.transition(FsmState.DAMP)
        sim.transition(FsmState.AI_MODE)
        assert sim.history == [FsmState.ZERO_TORQUE, FsmState.DAMP, FsmState.AI_MODE]


class TestStandUpTransitions:
    def test_ai_standup_from_zero_torque_valid_transitions(self) -> None:
        mod, sim = _make_dds_with_fsm_sim(FsmState.ZERO_TORQUE, ai_standup=True)
        assert mod.stand_up() is True
        assert sim.history == [
            FsmState.ZERO_TORQUE,
            FsmState.DAMP,
            FsmState.AI_MODE,
            FsmState.SQUAT_STANDUP_TOGGLE,
        ]

    def test_ai_standup_from_damp_valid_transitions(self) -> None:
        mod, sim = _make_dds_with_fsm_sim(FsmState.DAMP, ai_standup=True)
        assert mod.stand_up() is True
        assert sim.history == [
            FsmState.DAMP,
            FsmState.AI_MODE,
            FsmState.SQUAT_STANDUP_TOGGLE,
        ]

    def test_ai_standup_already_in_ai_mode(self) -> None:
        mod, sim = _make_dds_with_fsm_sim(FsmState.AI_MODE, ai_standup=True)
        assert mod.stand_up() is True
        assert sim.history == [FsmState.AI_MODE, FsmState.SQUAT_STANDUP_TOGGLE]

    def test_normal_standup_from_zero_torque_invalid(self) -> None:
        """Normal standup tries DAMP first, which is valid from ZERO_TORQUE."""
        mod, sim = _make_dds_with_fsm_sim(FsmState.ZERO_TORQUE, ai_standup=False)
        assert mod.stand_up() is True
        assert sim.history == [
            FsmState.ZERO_TORQUE,
            FsmState.DAMP,
            FsmState.SQUAT_STANDUP_TOGGLE,
        ]

    def test_normal_standup_from_damp(self) -> None:
        mod, sim = _make_dds_with_fsm_sim(FsmState.DAMP, ai_standup=False)
        assert mod.stand_up() is True
        assert sim.history == [
            FsmState.DAMP,
            # DAMP -> DAMP is not in valid transitions, but SetFsmId
            # is called unconditionally; the real robot handles this as a no-op.
            # Our sim models it as valid since the robot stays in DAMP.
            FsmState.DAMP,
            FsmState.SQUAT_STANDUP_TOGGLE,
        ]


class TestLieDownTransitions:
    def test_lie_down_from_standing(self) -> None:
        """Assumes the robot is in SQUAT_STANDUP_TOGGLE (standing) state."""
        mod, sim = _make_dds_with_fsm_sim(FsmState.SQUAT_STANDUP_TOGGLE)
        assert mod.lie_down() is True
        # StandUp2Squat toggles -> SQUAT_STANDUP_TOGGLE, then Damp -> DAMP
        assert sim.history == [
            FsmState.SQUAT_STANDUP_TOGGLE,
            FsmState.SQUAT_STANDUP_TOGGLE,
            FsmState.DAMP,
        ]

    def test_lie_down_from_ai_mode(self) -> None:
        mod, sim = _make_dds_with_fsm_sim(FsmState.AI_MODE)
        assert mod.lie_down() is True
        assert FsmState.DAMP in sim.history
