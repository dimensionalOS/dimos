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

import asyncio
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from threading import RLock
from types import SimpleNamespace
from typing import Any

from fastapi.testclient import TestClient
import numpy as np
import pytest

from dimos.agents.mcp.mcp_server import handle_request
import dimos.agents.skills.seat_guide as seat_guide_module
from dimos.agents.skills.seat_guide import (
    CameraSeatObservationProvider,
    CameraSeatSceneConfig,
    PersonObservation,
    SeatGuidePlanner,
    SeatGuideRequestSpec,
    SeatGuideSkillContainer,
    SeatObservation,
    SeatSceneObservation,
    SyntheticSeatObservationProvider,
    SyntheticSeatSceneConfig,
    _flatten_people,
    _flatten_seats,
    _parse_people,
    _parse_seats,
    is_seat_guide_preview_request,
    parse_seat_guide_intent,
)
from dimos.agents.system_prompt import SYSTEM_PROMPT
import dimos.agents.web_human_input as web_human_input_module
from dimos.agents.web_human_input import WebInput, _create_whisper_node
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState
from dimos.web.robot_web_interface import RobotWebInterface

REPO_ROOT = Path(__file__).parents[3]
ACCEPTANCE_LOG_VERIFIER = REPO_ROOT / "bin" / "demo_seat_guide_verify_acceptance_log"
HARDWARE_ACCEPTANCE_SCRIPT = REPO_ROOT / "bin" / "demo_seat_guide_hardware_acceptance"
HARDWARE_BRINGUP_SCRIPT = REPO_ROOT / "bin" / "demo_seat_guide_hardware_bringup"
SMOKE_SCRIPT = REPO_ROOT / "bin" / "demo_seat_guide_smoke"
REPLAY_SMOKE_SCRIPT = REPO_ROOT / "bin" / "demo_seat_guide_replay_smoke"
SEAT_GUIDE_DOC = REPO_ROOT / "docs" / "agents" / "seat_guide_modules.md"
SEAT_GUIDE_SCRIPTS = [
    SMOKE_SCRIPT,
    REPLAY_SMOKE_SCRIPT,
    HARDWARE_BRINGUP_SCRIPT,
    HARDWARE_ACCEPTANCE_SCRIPT,
    ACCEPTANCE_LOG_VERIFIER,
]


class FakeNavigation:
    def __init__(
        self,
        *,
        accepts_goal: bool = True,
        raises_on_goal: bool = False,
        state: NavigationState = NavigationState.IDLE,
        goal_reached: bool = False,
    ) -> None:
        self.accepts_goal = accepts_goal
        self.raises_on_goal = raises_on_goal
        self.state = state
        self.goal_reached = goal_reached
        self.goal: PoseStamped | None = None

    def set_goal(self, goal: PoseStamped) -> bool:
        if self.raises_on_goal:
            raise RuntimeError("planner unavailable")
        self.goal = goal
        return self.accepts_goal

    def get_state(self) -> NavigationState:
        return self.state

    def is_goal_reached(self) -> bool:
        return self.goal_reached

    def cancel_goal(self) -> bool:
        return True


class FakeSeatObservationProvider:
    def __init__(self, scene: SeatSceneObservation) -> None:
        self.scene = scene

    def get_seat_scene(self) -> SeatSceneObservation:
        return self.scene


class CountingSeatObservationProvider(FakeSeatObservationProvider):
    def __init__(self, scene: SeatSceneObservation) -> None:
        super().__init__(scene)
        self.calls = 0

    def get_seat_scene(self) -> SeatSceneObservation:
        self.calls += 1
        return super().get_seat_scene()


class FakeSpeaker:
    def __init__(self, *, raises: bool = False) -> None:
        self.spoken: list[tuple[str, bool]] = []
        self.raises = raises

    def speak(self, text: str, blocking: bool = True) -> str:
        if self.raises:
            raise RuntimeError("audio device unavailable")
        self.spoken.append((text, blocking))
        return f"Spoke: {text}"


class FakeSeatGuideRequest:
    def __init__(self, *, raises: bool = False) -> None:
        self.requests: list[str] = []
        self.preview_requests: list[str] = []
        self.raises = raises

    def handle_seat_request(self, text: str) -> str:
        self.requests.append(text)
        if self.raises:
            raise RuntimeError("seat guide unavailable")
        return "handled"

    def preview_seat_request(self, text: str) -> str:
        self.preview_requests.append(text)
        if self.raises:
            raise RuntimeError("seat guide unavailable")
        return "previewed"


class FakeHumanTransport:
    def __init__(self) -> None:
        self.published: list[str] = []

    def publish(self, text: str) -> None:
        self.published.append(text)


class FakeWebInterface:
    port = 5555
    audio_subject = SimpleNamespace()


class FakeThread:
    def __init__(self, *, alive: bool) -> None:
        self.alive = alive

    def is_alive(self) -> bool:
        return self.alive


class FakeAgentResponses:
    def __init__(self) -> None:
        self.published: list[str] = []

    def on_next(self, text: str) -> None:
        self.published.append(text)


class FakeLogger:
    def __init__(self) -> None:
        self.info_calls: list[tuple[str, dict[str, Any]]] = []

    def info(self, event: str, **kwargs: Any) -> None:
        self.info_calls.append((event, kwargs))


class FakeVlModel:
    def __init__(self, detections_by_query: dict[str, list[SimpleNamespace]]) -> None:
        self._detections_by_query = detections_by_query
        self.queries: list[str] = []

    def query_detections(self, image: Image, query: str) -> SimpleNamespace:
        self.queries.append(query)
        return SimpleNamespace(detections=self._detections_by_query.get(query, []))


class OdomMutatingFakeVlModel(FakeVlModel):
    def __init__(
        self,
        detections_by_query: dict[str, list[SimpleNamespace]],
        provider: CameraSeatObservationProvider,
    ) -> None:
        super().__init__(detections_by_query)
        self._provider = provider

    def query_detections(self, image: Image, query: str) -> SimpleNamespace:
        self._provider._on_odom(
            PoseStamped(
                frame_id="map",
                position=Vector3(100.0, 200.0, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 1.0)),
            )
        )
        return super().query_detections(image, query)


class RecordingNavigation(Module):
    _last_goal: PoseStamped | None = None

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        self._last_goal = goal
        return True

    @rpc
    def get_state(self) -> NavigationState:
        return NavigationState.IDLE

    @rpc
    def is_goal_reached(self) -> bool:
        return False

    @rpc
    def cancel_goal(self) -> bool:
        return True

    @rpc
    def get_last_goal_xy(self) -> tuple[float, float] | None:
        if self._last_goal is None:
            return None
        return self._last_goal.position.x, self._last_goal.position.y


class RecordingSpeaker(Module):
    _spoken: list[str]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._spoken = []

    @rpc
    def speak(self, text: str, blocking: bool = True) -> str:
        self._spoken.append(text)
        return f"Spoke: {text}"

    @rpc
    def get_spoken(self) -> list[str]:
        return self._spoken


def test_planner_selects_nearest_empty_seat() -> None:
    planner = SeatGuidePlanner(occupied_radius_m=0.75, aisle_offset_m=0.5)
    seats = [
        SeatObservation("occupied_near", x=1.0, y=0.0, yaw=0.0),
        SeatObservation("empty_far", x=5.0, y=0.0, yaw=0.0),
        SeatObservation("empty_near", x=2.0, y=0.0, yaw=math.pi / 2),
    ]
    people = [PersonObservation(x=1.2, y=0.1)]

    result = planner.find_empty_seat(seats, people, robot_x=0.0, robot_y=0.0)

    assert result is not None
    assert result.seat.seat_id == "empty_near"
    assert result.goal_x == pytest.approx(2.0)
    assert result.goal_y == pytest.approx(0.5)
    assert result.goal_yaw == pytest.approx(math.pi / 2)
    assert planner.occupancy_counts(seats, people) == (2, 1)


def test_planner_returns_none_when_all_seats_are_occupied() -> None:
    planner = SeatGuidePlanner()
    seats = [
        SeatObservation("left", x=0.0, y=0.0),
        SeatObservation("right", x=1.0, y=0.0),
    ]
    people = [
        PersonObservation(x=0.1, y=0.0),
        PersonObservation(x=1.1, y=0.0),
    ]

    assert planner.find_empty_seat(seats, people) is None


def test_skill_sets_navigation_goal_for_empty_seat() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation

    message = skill.find_empty_seat(
        seats=[0.0, 0.0, 0.0, 2.0, 0.0, math.pi],
        people=[0.2, 0.1],
        robot_x=0.0,
        robot_y=0.0,
    )

    assert "seat_2" in message
    assert fake_navigation.goal is not None
    assert fake_navigation.goal.frame_id == "map"
    assert fake_navigation.goal.position.x == pytest.approx(1.35)
    assert fake_navigation.goal.position.y == pytest.approx(0.0)
    assert "goal_sequence=1" in skill.seat_guide_navigation_status()


def test_navigation_status_ignores_stale_goal_reached_until_reset_seen() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation(goal_reached=True)
    skill._navigation = fake_navigation

    message = skill.find_empty_seat(seats=[1.0, 0.0, 0.0], people=[])

    assert "Navigating to" in message
    assert skill.seat_guide_navigation_status() == (
        "SeatGuide navigation status: navigation=IDLE; goal_reached=false; "
        "goal_sequence=1; completion_reset=waiting_for_false."
    )

    fake_navigation.goal_reached = False
    assert skill.seat_guide_navigation_status() == (
        "SeatGuide navigation status: navigation=IDLE; goal_reached=false; "
        "goal_sequence=1."
    )

    fake_navigation.goal_reached = True
    assert skill.seat_guide_navigation_status() == (
        "SeatGuide navigation status: navigation=IDLE; goal_reached=true; "
        "goal_sequence=1."
    )


def test_skill_reports_navigation_failure() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._navigation = FakeNavigation(accepts_goal=False)

    message = skill.find_empty_seat(seats=[1.0, 0.0, 0.0], people=[])

    assert message == "Found empty seat seat_1, but failed to start navigation."
    assert "goal_sequence=0" in skill.seat_guide_navigation_status()


def test_skill_reports_navigation_exception() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation(raises_on_goal=True)
    skill._navigation = fake_navigation

    message = skill.find_empty_seat(seats=[1.0, 0.0, 0.0], people=[])

    assert (
        message
        == "Found empty seat seat_1, but navigation raised an error: planner unavailable."
    )
    assert fake_navigation.goal is None


def test_skill_refuses_to_override_active_navigation_goal() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation(state=NavigationState.FOLLOWING_PATH)
    skill._navigation = fake_navigation

    message = skill.find_empty_seat(seats=[1.0, 0.0, 0.0], people=[])

    assert message == (
        "Found empty seat seat_1, but navigation is not ready for a new goal: "
        "navigation=FOLLOWING_PATH."
    )
    assert fake_navigation.goal is None


def test_skill_continues_when_speech_feedback_raises() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._speaker = FakeSpeaker(raises=True)

    message = skill.find_empty_seat(seats=[1.0, 0.0, 0.0], people=[])

    assert message == (
        "I found an empty seat seat_1. Please follow me to the chair beside the table. "
        "Navigating to (1.65, 0.00)."
    )
    assert fake_navigation.goal is not None


def test_skill_uses_connected_observation_provider() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("empty", x=1.0, y=0.0, yaw=0.0),
            ],
            people=[PersonObservation(x=0.1, y=0.0)],
            robot_x=0.0,
            robot_y=0.0,
        )
    )

    message = skill.find_empty_seat_from_scene(require_live_perception=False)

    assert "seat_2" in message
    assert fake_navigation.goal is not None
    assert fake_navigation.goal.position.x == pytest.approx(1.65)


def test_skill_reports_missing_observation_provider() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._seat_observation_provider = None

    assert skill.find_empty_seat_from_scene() == "No seat observation provider is connected."


def test_skill_reports_no_visible_seats_separately() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_speaker = FakeSpeaker()
    skill._navigation = FakeNavigation()
    skill._speaker = fake_speaker

    message = skill.find_empty_seat(seats=[], people=[])

    assert message == (
        "I cannot see any seats yet. Please face the conference table or calibrate "
        "the room layout."
    )
    assert fake_speaker.spoken == [(message, False)]


def test_handle_seat_request_delegates_to_scene_provider() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("empty", x=1.0, y=0.0, yaw=0.0),
            ],
            people=[PersonObservation(x=0.1, y=0.0)],
            source="camera",
        )
    )

    message = skill.handle_seat_request("Please help me find an empty seat")

    assert "seat_2" in message
    assert fake_navigation.goal is not None
    assert fake_navigation.goal.position.x == pytest.approx(1.65)


def test_handle_seat_request_speaks_feedback_when_speaker_is_connected() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    fake_speaker = FakeSpeaker()
    skill._navigation = fake_navigation
    skill._speaker = fake_speaker
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("empty", x=1.0, y=0.0, yaw=0.0),
            ],
            people=[PersonObservation(x=0.1, y=0.0)],
            source="camera",
        )
    )

    skill.handle_seat_request("Please help me find an empty seat")

    assert fake_speaker.spoken == [
        ("I found an empty seat seat_2. Please follow me to the chair beside the table.", False)
    ]


def test_handle_seat_request_requires_live_camera_by_default() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    fake_speaker = FakeSpeaker()
    skill._navigation = fake_navigation
    skill._speaker = fake_speaker
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("fallback", x=1.0, y=0.0, yaw=0.0)],
            people=[],
            source="configured_fallback",
        )
    )

    message = skill.handle_seat_request("Please help me find an empty seat")

    assert message == (
        "SeatGuide requires live camera perception before navigation; "
        "source=configured_fallback; seats=1; people=0; robot=(0.00, 0.00); "
        "next=use require_live_perception=false only for explicit fallback calibration."
    )
    assert fake_navigation.goal is None
    assert fake_speaker.spoken == [(message, False)]


def test_handle_seat_request_reports_camera_detection_error_next_step() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("fallback", x=1.0, y=0.0, yaw=0.0)],
            people=[PersonObservation(x=0.0, y=0.0)],
            robot_x=-1.0,
            robot_y=2.0,
            source="camera_detection_error",
        )
    )

    message = skill.handle_seat_request("Please help me find an empty seat")

    assert message == (
        "SeatGuide requires live camera perception before navigation; "
        "source=camera_detection_error; seats=1; people=1; robot=(-1.00, 2.00); "
        "next=check VLM/API key setup and logs."
    )
    assert fake_navigation.goal is None


def test_handle_seat_request_can_explicitly_allow_fallback_calibration() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("fallback", x=1.0, y=0.0, yaw=0.0)],
            people=[],
            source="configured_fallback",
        )
    )

    message = skill.handle_seat_request(
        "Please help me find an empty seat",
        require_live_perception=False,
    )

    assert "seat_1" in message
    assert fake_navigation.goal is not None
    assert fake_navigation.goal.position.x == pytest.approx(1.65)


def test_handle_seat_request_rejects_unrelated_text() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)

    message = skill.handle_seat_request("what time is the meeting")

    assert message == "I did not hear a request to find an empty seat."


def test_preview_seat_request_runs_preflight_without_navigating() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    fake_speaker = FakeSpeaker()
    skill._navigation = fake_navigation
    skill._speaker = fake_speaker
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("empty", x=2.0, y=0.0, yaw=0.0)],
            people=[],
            robot_x=0.0,
            robot_y=0.0,
            source="camera",
        )
    )

    message = skill.preview_seat_request("预检帮我找一个空位")

    assert "SeatGuide preflight ready" in message
    assert fake_navigation.goal is None
    assert fake_speaker.spoken == [(message, False)]


def test_seat_guide_status_describes_current_scene() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("seat_1", x=1.0, y=2.0, yaw=0.5)],
            people=[PersonObservation(x=1.1, y=2.0)],
            robot_x=-1.0,
            robot_y=0.0,
            source="camera",
        )
    )

    message = skill.seat_guide_status()

    assert message == (
        "SeatGuide scene source=camera: 1 seats [seat_1=(1.00, 2.00, yaw=0.50)], "
        "1 people [(1.10, 2.00)], robot=(-1.00, 0.00)."
    )


def test_seat_guide_status_reports_missing_provider() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._seat_observation_provider = None

    assert (
        skill.seat_guide_status()
        == "SeatGuide status: no seat observation provider is connected."
    )


def test_preview_empty_seat_goal_does_not_navigate() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("empty", x=2.0, y=0.0, yaw=0.0),
            ],
            people=[PersonObservation(x=0.1, y=0.0)],
            robot_x=0.0,
            robot_y=0.0,
            source="camera",
        )
    )

    message = skill.preview_empty_seat_goal()

    assert message == (
        "SeatGuide preview source=camera: selected empty "
        "empty=1 occupied=1 "
        "seat=(2.00, 0.00, yaw=0.00) goal=(2.65, 0.00, yaw=0.00)."
    )
    assert fake_navigation.goal is None


def test_preview_empty_seat_goal_reports_no_seats() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(seats=[], people=[], source="no_camera_image")
    )

    assert (
        skill.preview_empty_seat_goal()
        == "SeatGuide preview source=no_camera_image: no seats visible or configured."
    )


def test_preview_empty_seat_goal_reports_no_empty_seat() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0)],
            people=[PersonObservation(x=0.1, y=0.0)],
            source="camera",
        )
    )

    assert (
        skill.preview_empty_seat_goal()
        == "SeatGuide preview source=camera: no empty seat available; empty=0 occupied=1."
    )


def test_seat_guide_preflight_reports_ready_without_navigating() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._speaker = FakeSpeaker()
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("empty", x=2.0, y=0.0, yaw=0.0),
            ],
            people=[PersonObservation(x=0.1, y=0.0)],
            robot_x=0.0,
            robot_y=0.0,
            source="camera",
        )
    )

    message = skill.seat_guide_preflight()

    assert message == (
        "SeatGuide preflight ready: navigation=IDLE; perception=camera seats=2 people=1; "
        "empty=1 occupied=1; selected=empty; "
        "goal=(2.65, 0.00, yaw=0.00); speaker=connected."
    )
    assert fake_navigation.goal is None


def test_seat_guide_preflight_reports_no_go_without_provider() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._navigation = FakeNavigation()
    skill._seat_observation_provider = None
    skill._speaker = None

    assert (
        skill.seat_guide_preflight()
        == "SeatGuide preflight no-go: navigation=IDLE; perception=missing; speaker=missing."
    )


def test_seat_guide_preflight_reports_no_go_without_visible_seats() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._navigation = FakeNavigation()
    skill._speaker = None
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(seats=[], people=[], source="camera_detection_error")
    )

    assert (
        skill.seat_guide_preflight()
        == "SeatGuide preflight no-go: navigation=IDLE; perception=camera_detection_error "
        "no seats; speaker=missing."
    )


def test_seat_guide_preflight_reports_no_empty_seat_counts() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._navigation = FakeNavigation()
    skill._speaker = None
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("left", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("right", x=1.0, y=0.0, yaw=0.0),
            ],
            people=[
                PersonObservation(x=0.1, y=0.0),
                PersonObservation(x=1.1, y=0.0),
            ],
            source="camera",
        )
    )

    assert skill.seat_guide_preflight() == (
        "SeatGuide preflight no-go: navigation=IDLE; perception=camera; "
        "no empty seat; empty=0 occupied=2; speaker=missing."
    )


def test_seat_guide_preflight_reports_no_go_when_navigation_is_busy() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation(state=NavigationState.FOLLOWING_PATH)
    skill._navigation = fake_navigation
    skill._speaker = None
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("empty", x=2.0, y=0.0, yaw=0.0)],
            people=[],
            source="camera",
        )
    )

    message = skill.seat_guide_preflight()

    assert message == (
        "SeatGuide preflight no-go: navigation=FOLLOWING_PATH; "
        "perception=camera seats=1 people=0; empty=1 occupied=0; selected=empty; "
        "goal=(2.65, 0.00, yaw=0.00); speaker=missing."
    )
    assert fake_navigation.goal is None


def test_seat_guide_preflight_requires_live_camera_by_default() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._speaker = None
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("fallback", x=2.0, y=0.0, yaw=0.0)],
            people=[],
            source="configured_fallback",
        )
    )

    message = skill.seat_guide_preflight()

    assert message == (
        "SeatGuide preflight no-go: navigation=IDLE; perception=configured_fallback "
        "is not live camera; seats=1 people=0; speaker=missing."
    )
    assert fake_navigation.goal is None


def test_seat_guide_preflight_can_explicitly_allow_fallback_calibration() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._speaker = None
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("fallback", x=2.0, y=0.0, yaw=0.0)],
            people=[],
            source="configured_fallback",
        )
    )

    message = skill.seat_guide_preflight(require_live_perception=False)

    assert "SeatGuide preflight ready" in message
    assert "perception=configured_fallback" in message
    assert fake_navigation.goal is None


def test_seat_guide_readiness_report_combines_no_motion_checks() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._speaker = FakeSpeaker()
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("empty", x=2.0, y=0.0, yaw=0.0),
            ],
            people=[PersonObservation(x=0.1, y=0.0)],
            robot_x=0.0,
            robot_y=0.0,
            source="camera",
        )
    )

    message = skill.seat_guide_readiness_report()

    assert message.startswith("SeatGuide readiness report: SeatGuide scene source=camera")
    assert "SeatGuide preflight ready" in message
    assert "empty=1 occupied=1" in message
    assert "SeatGuide preview source=camera" in message
    assert fake_navigation.goal is None


def test_seat_guide_readiness_report_keeps_fallback_no_go_by_default() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._speaker = None
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("fallback", x=2.0, y=0.0, yaw=0.0)],
            people=[],
            source="configured_fallback",
        )
    )

    message = skill.seat_guide_readiness_report()

    assert "SeatGuide preflight no-go" in message
    assert "perception=configured_fallback is not live camera" in message
    assert "SeatGuide preview source=configured_fallback" in message
    assert fake_navigation.goal is None


def test_seat_guide_readiness_report_reads_scene_once() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    provider = CountingSeatObservationProvider(
        SeatSceneObservation(
            seats=[SeatObservation("empty", x=2.0, y=0.0, yaw=0.0)],
            people=[],
            source="camera",
        )
    )
    skill._navigation = FakeNavigation()
    skill._speaker = None
    skill._seat_observation_provider = provider

    skill.seat_guide_readiness_report()

    assert provider.calls == 1


def test_seat_guide_navigation_status_reports_goal_reached() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._navigation = FakeNavigation(goal_reached=True)

    assert skill.seat_guide_navigation_status() == (
        "SeatGuide navigation status: navigation=IDLE; goal_reached=true; goal_sequence=0."
    )


def test_seat_guide_navigation_status_reports_busy_goal_not_reached() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._navigation = FakeNavigation(
        state=NavigationState.FOLLOWING_PATH,
        goal_reached=False,
    )

    assert skill.seat_guide_navigation_status() == (
        "SeatGuide navigation status: navigation=FOLLOWING_PATH; "
        "goal_reached=false; goal_sequence=0."
    )


def test_seat_guide_navigation_status_reports_missing_navigation() -> None:
    skill = SeatGuideSkillContainer.__new__(SeatGuideSkillContainer)
    skill._navigation = None

    assert skill.seat_guide_navigation_status() == (
        "SeatGuide navigation status: navigation=missing; goal_reached=unknown; "
        "goal_sequence=0."
    )


def test_web_input_routes_seat_voice_text_directly_to_seat_guide() -> None:
    web_input = WebInput.__new__(WebInput)
    fake_seat_guide = FakeSeatGuideRequest()
    fake_transport = FakeHumanTransport()
    fake_agent_responses = FakeAgentResponses()
    web_input._seat_guide = fake_seat_guide
    web_input._human_transport = fake_transport
    web_input._agent_responses = fake_agent_responses

    web_input._route_text("帮我找一个空位")

    assert fake_seat_guide.requests == ["帮我找一个空位"]
    assert fake_seat_guide.preview_requests == []
    assert fake_transport.published == []
    assert fake_agent_responses.published == ["handled"]


def test_web_input_logs_live_seat_guide_route_for_voice_bringup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    web_input = WebInput.__new__(WebInput)
    fake_seat_guide = FakeSeatGuideRequest()
    fake_transport = FakeHumanTransport()
    fake_agent_responses = FakeAgentResponses()
    fake_logger = FakeLogger()
    web_input._seat_guide = fake_seat_guide
    web_input._human_transport = fake_transport
    web_input._agent_responses = fake_agent_responses
    monkeypatch.setattr(web_human_input_module, "logger", fake_logger)

    web_input._route_text("帮我找一个空位")

    assert fake_logger.info_calls == [
        ("WebInput received text", {"text": "帮我找一个空位"}),
        ("WebInput routing text to SeatGuide live request", {"text": "帮我找一个空位"}),
    ]


def test_web_input_routes_preview_seat_voice_text_without_navigation_request(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    web_input = WebInput.__new__(WebInput)
    fake_seat_guide = FakeSeatGuideRequest()
    fake_transport = FakeHumanTransport()
    fake_agent_responses = FakeAgentResponses()
    fake_logger = FakeLogger()
    web_input._seat_guide = fake_seat_guide
    web_input._human_transport = fake_transport
    web_input._agent_responses = fake_agent_responses
    monkeypatch.setattr(web_human_input_module, "logger", fake_logger)

    web_input._route_text("预检帮我找一个空位")

    assert fake_seat_guide.preview_requests == ["预检帮我找一个空位"]
    assert fake_seat_guide.requests == []
    assert fake_transport.published == []
    assert fake_agent_responses.published == ["previewed"]
    assert fake_logger.info_calls == [
        ("WebInput received text", {"text": "预检帮我找一个空位"}),
        (
            "WebInput routing text to SeatGuide preview",
            {"text": "预检帮我找一个空位"},
        ),
    ]


def test_web_input_submit_query_http_route_reaches_seat_guide_preview() -> None:
    web_input = WebInput.__new__(WebInput)
    fake_seat_guide = FakeSeatGuideRequest()
    fake_transport = FakeHumanTransport()
    fake_agent_responses = FakeAgentResponses()
    web_input._seat_guide = fake_seat_guide
    web_input._human_transport = fake_transport
    web_input._agent_responses = fake_agent_responses

    interface = RobotWebInterface(port=5555)
    subscription = interface.query_stream.subscribe(web_input._route_text)

    try:
        response = TestClient(interface.app).post(
            "/submit_query", data={"query": "预检帮我找一个空位"}
        )
    finally:
        subscription.dispose()

    assert response.status_code == 200
    assert response.json() == {"success": True, "message": "Query received"}
    assert fake_seat_guide.preview_requests == ["预检帮我找一个空位"]
    assert fake_seat_guide.requests == []
    assert fake_transport.published == []
    assert fake_agent_responses.published == ["previewed"]


def test_web_input_upload_audio_http_route_emits_audio_event(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    audio_events: list[Any] = []
    audio_subject = web_human_input_module.rx.subject.Subject()
    audio_subject.subscribe(audio_events.append)

    decoded_audio = np.array([0.1, -0.1, 0.0], dtype=np.float32)
    monkeypatch.setattr(
        "dimos.web.dimos_interface.api.server.FastAPIServer._decode_audio",
        staticmethod(lambda raw: (decoded_audio, 16000)),
    )

    interface = RobotWebInterface(port=5555, audio_subject=audio_subject)
    response = TestClient(interface.app).post(
        "/upload_audio",
        files={"file": ("seat-guide.webm", b"browser audio", "audio/webm")},
    )

    assert response.status_code == 200
    assert response.json() == {"success": True}
    assert len(audio_events) == 1
    event = audio_events[0]
    assert event.sample_rate == 16000
    assert event.channels == 1
    assert np.array_equal(event.data, decoded_audio)


def test_web_input_upload_audio_requires_configured_voice_subject() -> None:
    interface = RobotWebInterface(port=5555, audio_subject=None)
    response = TestClient(interface.app).post(
        "/upload_audio",
        files={"file": ("seat-guide.webm", b"browser audio", "audio/webm")},
    )

    assert response.status_code == 400
    assert response.json() == {
        "success": False,
        "message": "Voice input not configured",
    }


def test_web_input_upload_audio_rejects_decode_failures(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    audio_events: list[Any] = []
    audio_subject = web_human_input_module.rx.subject.Subject()
    audio_subject.subscribe(audio_events.append)
    monkeypatch.setattr(
        "dimos.web.dimos_interface.api.server.FastAPIServer._decode_audio",
        staticmethod(lambda raw: (None, None)),
    )

    interface = RobotWebInterface(port=5555, audio_subject=audio_subject)
    response = TestClient(interface.app).post(
        "/upload_audio",
        files={"file": ("seat-guide.webm", b"not audio", "audio/webm")},
    )

    assert response.status_code == 400
    assert response.json() == {"success": False, "message": "Unable to decode audio"}
    assert audio_events == []


def test_web_input_falls_back_to_agent_path_when_seat_guide_route_fails() -> None:
    web_input = WebInput.__new__(WebInput)
    fake_seat_guide = FakeSeatGuideRequest(raises=True)
    fake_transport = FakeHumanTransport()
    fake_agent_responses = FakeAgentResponses()
    web_input._seat_guide = fake_seat_guide
    web_input._human_transport = fake_transport
    web_input._agent_responses = fake_agent_responses

    web_input._route_text("帮我找一个空位")

    assert fake_seat_guide.requests == ["帮我找一个空位"]
    assert fake_transport.published == ["帮我找一个空位"]
    assert fake_agent_responses.published == []


def test_web_input_keeps_unrelated_voice_text_on_agent_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    web_input = WebInput.__new__(WebInput)
    fake_seat_guide = FakeSeatGuideRequest()
    fake_transport = FakeHumanTransport()
    fake_logger = FakeLogger()
    web_input._seat_guide = fake_seat_guide
    web_input._human_transport = fake_transport
    monkeypatch.setattr(web_human_input_module, "logger", fake_logger)

    web_input._route_text("what time is the meeting")

    assert fake_seat_guide.requests == []
    assert fake_transport.published == ["what time is the meeting"]
    assert fake_logger.info_calls == [
        ("WebInput received text", {"text": "what time is the meeting"}),
        ("WebInput routing text to agent path", {"text": "what time is the meeting"}),
    ]


def test_web_input_status_reports_not_started_state() -> None:
    web_input = WebInput.__new__(WebInput)
    web_input._web_interface = None
    web_input._thread = None
    web_input._seat_guide = None
    web_input._agent_responses = None
    web_input._stt_node = None
    web_input._stt_error = None
    web_input._human_transport = None

    assert web_input.web_input_status() == (
        "WebInput status: web=not_started; thread=not_running; "
        "seat_route=agent_only; responses=missing; voice_upload=missing; "
        "stt=missing; human_transport=missing; url=unavailable."
    )


def test_web_input_status_reports_seat_guide_direct_route() -> None:
    web_input = WebInput.__new__(WebInput)
    web_input._web_interface = FakeWebInterface()
    web_input._thread = FakeThread(alive=True)
    web_input._seat_guide = FakeSeatGuideRequest()
    web_input._agent_responses = FakeAgentResponses()
    web_input._stt_node = SimpleNamespace()
    web_input._stt_error = None
    web_input._human_transport = FakeHumanTransport()

    assert web_input.web_input_status() == (
        "WebInput status: web=started; thread=running; "
        "seat_route=seat_guide_direct; responses=connected; "
        "voice_upload=connected; stt=connected; human_transport=connected; "
        "url=http://localhost:5555."
    )


def test_web_input_status_reports_stt_initialization_error() -> None:
    web_input = WebInput.__new__(WebInput)
    web_input._web_interface = FakeWebInterface()
    web_input._thread = FakeThread(alive=True)
    web_input._seat_guide = FakeSeatGuideRequest()
    web_input._agent_responses = FakeAgentResponses()
    web_input._stt_node = None
    web_input._stt_error = "RuntimeError: whisper missing"
    web_input._human_transport = FakeHumanTransport()

    assert web_input.web_input_status() == (
        "WebInput status: web=started; thread=running; "
        "seat_route=seat_guide_direct; responses=connected; "
        "voice_upload=connected; stt=error(RuntimeError: whisper missing); "
        "human_transport=connected; url=http://localhost:5555."
    )


def test_web_input_status_reports_missing_browser_voice_upload() -> None:
    web_input = WebInput.__new__(WebInput)
    web_input._web_interface = SimpleNamespace(port=5555, audio_subject=None)
    web_input._thread = FakeThread(alive=True)
    web_input._seat_guide = FakeSeatGuideRequest()
    web_input._agent_responses = FakeAgentResponses()
    web_input._stt_node = SimpleNamespace()
    web_input._stt_error = None
    web_input._human_transport = FakeHumanTransport()

    assert web_input.web_input_status() == (
        "WebInput status: web=started; thread=running; "
        "seat_route=seat_guide_direct; responses=connected; "
        "voice_upload=missing; stt=connected; human_transport=connected; "
        "url=http://localhost:5555."
    )


def test_web_input_whisper_auto_detects_language(monkeypatch: pytest.MonkeyPatch) -> None:
    created_modelopts: list[dict[str, Any] | None] = []

    class FakeWhisperNode:
        def __init__(
            self,
            model: str = "base",
            modelopts: dict[str, Any] | None = None,
        ) -> None:
            created_modelopts.append(modelopts)

    monkeypatch.setitem(
        sys.modules,
        "dimos.stream.audio.stt.node_whisper",
        SimpleNamespace(WhisperNode=FakeWhisperNode),
    )

    _create_whisper_node()

    assert created_modelopts == [{"fp16": False}]
    assert "language" not in created_modelopts[0]


def test_autoconnect_injects_scene_provider_and_navigation() -> None:
    blueprint = autoconnect(
        SeatGuideSkillContainer.blueprint(),
        SyntheticSeatObservationProvider.blueprint(
            seats=[0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
            people=[0.1, 0.0],
            robot_x=0.0,
            robot_y=0.0,
        ),
        RecordingNavigation.blueprint(),
    )
    coordinator = ModuleCoordinator.build(blueprint, {"g": {"viewer": "none"}})

    try:
        seat_guide = coordinator.get_instance(SeatGuideSkillContainer)
        navigation = coordinator.get_instance(RecordingNavigation)

        message = seat_guide.handle_seat_request(
            "Please find me an empty seat",
            require_live_perception=False,
        )

        assert "seat_2" in message
        assert navigation.get_last_goal_xy() == pytest.approx((2.65, 0.0))
    finally:
        coordinator.stop()


def test_autoconnect_uses_runtime_configured_synthetic_scene() -> None:
    blueprint = autoconnect(
        SeatGuideSkillContainer.blueprint(),
        SyntheticSeatObservationProvider.blueprint(
            seats=[0.0, 0.0, 0.0],
            people=[],
            robot_x=0.0,
            robot_y=0.0,
        ),
        RecordingNavigation.blueprint(),
    )
    coordinator = ModuleCoordinator.build(blueprint, {"g": {"viewer": "none"}})

    try:
        seat_guide = coordinator.get_instance(SeatGuideSkillContainer)
        provider = coordinator.get_instance(SyntheticSeatObservationProvider)
        navigation = coordinator.get_instance(RecordingNavigation)

        provider.set_seat_scene(
            seats=[0.0, 0.0, 0.0, 4.0, 0.0, 0.0],
            people=[0.1, 0.0],
            robot_x=0.0,
            robot_y=0.0,
        )
        message = seat_guide.handle_seat_request(
            "Please find me an empty seat",
            require_live_perception=False,
        )

        assert "seat_2" in message
        assert navigation.get_last_goal_xy() == pytest.approx((4.65, 0.0))
    finally:
        coordinator.stop()


def test_autoconnect_injects_speaker_for_feedback() -> None:
    blueprint = autoconnect(
        SeatGuideSkillContainer.blueprint(),
        SyntheticSeatObservationProvider.blueprint(
            seats=[0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
            people=[0.1, 0.0],
            robot_x=0.0,
            robot_y=0.0,
        ),
        RecordingNavigation.blueprint(),
        RecordingSpeaker.blueprint(),
    )
    coordinator = ModuleCoordinator.build(blueprint, {"g": {"viewer": "none"}})

    try:
        seat_guide = coordinator.get_instance(SeatGuideSkillContainer)
        speaker = coordinator.get_instance(RecordingSpeaker)

        seat_guide.handle_seat_request(
            "Please find me an empty seat",
            require_live_perception=False,
        )

        assert speaker.get_spoken() == [
            "I found an empty seat seat_2. Please follow me to the chair beside the table."
        ]
    finally:
        coordinator.stop()


def test_seat_guide_exposes_agent_friendly_skills() -> None:
    skill = SeatGuideSkillContainer()
    try:
        skill_infos = {info.func_name: json.loads(info.args_schema) for info in skill.get_skills()}
    finally:
        skill._close_module()

    assert "handle_seat_request" in skill_infos
    assert "preview_seat_request" in skill_infos
    assert "find_empty_seat_from_scene" in skill_infos
    assert "seat_guide_preflight" in skill_infos
    assert "seat_guide_readiness_report" in skill_infos
    assert "seat_guide_navigation_status" in skill_infos
    assert "preview_empty_seat_goal" in skill_infos
    assert "seat_guide_status" in skill_infos
    assert "camera_seat_provider_status" not in skill_infos

    request_schema = skill_infos["handle_seat_request"]
    assert "spoken or typed request" in request_schema["description"]
    assert request_schema["properties"] == {
        "text": {"title": "Text", "type": "string"},
        "require_live_perception": {
            "default": True,
            "title": "Require Live Perception",
            "type": "boolean",
        },
    }
    assert request_schema["required"] == ["text"]

    preview_request_schema = skill_infos["preview_seat_request"]
    assert "without moving" in preview_request_schema["description"]
    assert preview_request_schema["properties"] == {"text": {"title": "Text", "type": "string"}}
    assert preview_request_schema["required"] == ["text"]

    scene_schema = skill_infos["find_empty_seat_from_scene"]
    assert "observation provider" in scene_schema["description"]
    assert scene_schema["properties"] == {
        "require_live_perception": {
            "default": True,
            "title": "Require Live Perception",
            "type": "boolean",
        }
    }

    preflight_schema = skill_infos["seat_guide_preflight"]
    assert "no-motion" in preflight_schema["description"]
    assert preflight_schema["properties"] == {
        "require_live_perception": {
            "default": True,
            "title": "Require Live Perception",
            "type": "boolean",
        }
    }

    readiness_schema = skill_infos["seat_guide_readiness_report"]
    assert "readiness checks" in readiness_schema["description"]
    assert readiness_schema["properties"] == {
        "require_live_perception": {
            "default": True,
            "title": "Require Live Perception",
            "type": "boolean",
        }
    }

    navigation_status_schema = skill_infos["seat_guide_navigation_status"]
    assert "navigation goal has completed" in navigation_status_schema["description"]
    assert navigation_status_schema.get("properties", {}) == {}

    preview_schema = skill_infos["preview_empty_seat_goal"]
    assert "without moving" in preview_schema["description"]
    assert preview_schema.get("properties", {}) == {}

    status_schema = skill_infos["seat_guide_status"]
    assert "without navigating" in status_schema["description"]
    assert status_schema.get("properties", {}) == {}


def test_web_input_exposes_bringup_status_skill() -> None:
    web_input = WebInput()
    try:
        skill_infos = {
            info.func_name: json.loads(info.args_schema) for info in web_input.get_skills()
        }
    finally:
        web_input._close_module()

    assert "web_input_status" in skill_infos
    status_schema = skill_infos["web_input_status"]
    assert "voice and text routing readiness" in status_schema["description"]
    assert status_schema.get("properties", {}) == {}


def test_camera_provider_exposes_bringup_status_skill() -> None:
    provider = CameraSeatObservationProvider()
    try:
        skill_infos = {
            info.func_name: json.loads(info.args_schema) for info in provider.get_skills()
        }
    finally:
        provider._close_module()

    assert "camera_seat_provider_status" in skill_infos
    status_schema = skill_infos["camera_seat_provider_status"]
    assert "perception readiness" in status_schema["description"]
    assert status_schema.get("properties", {}) == {}


def test_seat_guide_mcp_request_flow_without_go2() -> None:
    skill = SeatGuideSkillContainer()
    fake_navigation = FakeNavigation()
    skill._navigation = fake_navigation
    skill._seat_observation_provider = FakeSeatObservationProvider(
        SeatSceneObservation(
            seats=[
                SeatObservation("occupied", x=0.0, y=0.0, yaw=0.0),
                SeatObservation("empty", x=2.0, y=0.0, yaw=0.0),
            ],
            people=[PersonObservation(x=0.1, y=0.0)],
            robot_x=0.0,
            robot_y=0.0,
            source="configured_fallback",
        )
    )

    try:
        skills = skill.get_skills()
        rpc_calls = {info.func_name: getattr(skill, info.func_name) for info in skills}

        response = asyncio.run(
            handle_request({"method": "tools/list", "id": 1}, skills, rpc_calls)
        )
        tool_names = {tool["name"] for tool in response["result"]["tools"]}
        assert {
            "seat_guide_status",
            "preview_seat_request",
            "seat_guide_preflight",
            "seat_guide_readiness_report",
            "seat_guide_navigation_status",
            "preview_empty_seat_goal",
            "handle_seat_request",
        } <= tool_names

        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 2,
                    "params": {"name": "seat_guide_status", "arguments": {}},
                },
                skills,
                rpc_calls,
            )
        )
        assert "source=configured_fallback" in response["result"]["content"][0]["text"]

        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 3,
                    "params": {
                        "name": "seat_guide_preflight",
                        "arguments": {"require_live_perception": False},
                    },
                },
                skills,
                rpc_calls,
            )
        )
        assert "SeatGuide preflight ready" in response["result"]["content"][0]["text"]
        assert fake_navigation.goal is None

        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 4,
                    "params": {
                        "name": "seat_guide_readiness_report",
                        "arguments": {"require_live_perception": False},
                    },
                },
                skills,
                rpc_calls,
            )
        )
        assert "SeatGuide readiness report" in response["result"]["content"][0]["text"]
        assert fake_navigation.goal is None

        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 5,
                    "params": {"name": "preview_empty_seat_goal", "arguments": {}},
                },
                skills,
                rpc_calls,
            )
        )
        assert "goal=(2.65, 0.00" in response["result"]["content"][0]["text"]
        assert fake_navigation.goal is None

        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 6,
                    "params": {
                        "name": "preview_seat_request",
                        "arguments": {"text": "预检帮我找一个空位"},
                    },
                },
                skills,
                rpc_calls,
            )
        )
        assert "is not live camera" in response["result"]["content"][0]["text"]
        assert fake_navigation.goal is None

        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 7,
                    "params": {
                        "name": "handle_seat_request",
                        "arguments": {
                            "text": "帮我找一个空位",
                            "require_live_perception": False,
                        },
                    },
                },
                skills,
                rpc_calls,
            )
        )
        assert "seat_2" in response["result"]["content"][0]["text"]
        assert fake_navigation.goal is not None
        assert fake_navigation.goal.position.x == pytest.approx(2.65)

        fake_navigation.goal_reached = True
        response = asyncio.run(
            handle_request(
                {
                    "method": "tools/call",
                    "id": 8,
                    "params": {"name": "seat_guide_navigation_status", "arguments": {}},
                },
                skills,
                rpc_calls,
            )
        )
        assert "goal_reached=true" in response["result"]["content"][0]["text"]
    finally:
        skill._close_module()


def test_seat_guide_go2_blueprints_include_real_runtime_modules() -> None:
    from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_seat_guide import (
        unitree_go2_seat_guide,
    )
    from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_seat_guide_agentic import (
        unitree_go2_seat_guide_agentic,
    )

    agentic_modules = {atom.module.__name__ for atom in unitree_go2_seat_guide_agentic.blueprints}
    direct_modules = {atom.module.__name__ for atom in unitree_go2_seat_guide.blueprints}

    assert {
        "GO2Connection",
        "SpatialMemory",
        "McpServer",
        "McpClient",
        "CameraSeatObservationProvider",
        "SeatGuideSkillContainer",
        "WebInput",
        "SpeakSkill",
    } <= agentic_modules
    assert {
        "GO2Connection",
        "SpatialMemory",
        "McpServer",
        "CameraSeatObservationProvider",
        "SeatGuideSkillContainer",
        "WebInput",
        "SpeakSkill",
    } <= direct_modules
    assert "McpClient" not in direct_modules


def test_go2_seat_guide_blueprints_wire_web_input_to_seat_guide() -> None:
    from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_seat_guide import (
        unitree_go2_seat_guide,
    )
    from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_seat_guide_agentic import (
        unitree_go2_seat_guide_agentic,
    )

    for blueprint in (unitree_go2_seat_guide, unitree_go2_seat_guide_agentic):
        web_input_atom = next(
            atom for atom in blueprint.blueprints if atom.module is WebInput
        )
        seat_guide_refs = [
            ref for ref in web_input_atom.module_refs if ref.name == "_seat_guide"
        ]

        assert len(seat_guide_refs) == 1
        assert seat_guide_refs[0].spec is SeatGuideRequestSpec
        assert seat_guide_refs[0].optional


def test_synthetic_observation_provider_returns_configured_scene() -> None:
    provider = SyntheticSeatObservationProvider.__new__(SyntheticSeatObservationProvider)
    provider.config = SyntheticSeatSceneConfig(
        seats=[0.0, 1.0, 0.0, 2.0, 1.0, 0.5],
        people=[0.1, 1.0],
        robot_x=-1.0,
        robot_y=1.0,
    )

    scene = provider.get_seat_scene()

    assert scene.seats == [
        SeatObservation("seat_1", 0.0, 1.0, 0.0),
        SeatObservation("seat_2", 2.0, 1.0, 0.5),
    ]
    assert scene.people == [PersonObservation(0.1, 1.0)]
    assert scene.robot_x == -1.0
    assert scene.robot_y == 1.0
    assert scene.source == "configured_fallback"


def test_synthetic_observation_provider_runtime_override() -> None:
    provider = SyntheticSeatObservationProvider.__new__(SyntheticSeatObservationProvider)
    provider.config = SyntheticSeatSceneConfig(
        seats=[0.0, 0.0, 0.0],
        people=[],
        robot_x=0.0,
        robot_y=0.0,
    )
    provider._scene_override = None

    message = provider.set_seat_scene(
        seats=[1.0, 2.0, 0.0, 3.0, 2.0, 0.0],
        people=[1.1, 2.0],
        robot_x=-1.0,
        robot_y=2.0,
    )
    scene = provider.get_seat_scene()

    assert message == "Configured 2 seats and 1 person."
    assert scene.seats == [
        SeatObservation("seat_1", 1.0, 2.0, 0.0),
        SeatObservation("seat_2", 3.0, 2.0, 0.0),
    ]
    assert scene.people == [PersonObservation(1.1, 2.0)]
    assert scene.robot_x == -1.0
    assert scene.robot_y == 2.0
    assert scene.source == "runtime_override"

    assert provider.clear_seat_scene_override() == "Cleared synthetic seat scene override."
    assert provider.get_seat_scene().seats == [SeatObservation("seat_1", 0.0, 0.0, 0.0)]


def test_camera_observation_provider_detects_scene_from_image() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[9.0, 9.0, 0.0],
        people=[],
        robot_x=0.0,
        robot_y=0.0,
        chair_distance_m=2.0,
        lateral_span_m=4.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = PoseStamped(
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    fake_vl_model = FakeVlModel(
        {
            "chair": [
                SimpleNamespace(name="chair", bbox=(10.0, 20.0, 30.0, 80.0)),
                SimpleNamespace(name="chair", bbox=(70.0, 20.0, 90.0, 80.0)),
            ],
            "person": [
                SimpleNamespace(name="person", bbox=(12.0, 10.0, 32.0, 90.0)),
            ],
        }
    )
    provider._vl_model = fake_vl_model

    scene = provider.get_seat_scene()

    assert fake_vl_model.queries == ["chair", "person"]
    assert [seat.seat_id for seat in scene.seats] == ["seat_1", "seat_2"]
    assert [seat.x for seat in scene.seats] == pytest.approx([2.0, 2.0])
    assert [seat.y for seat in scene.seats] == pytest.approx([-1.2, 1.2])
    assert [seat.yaw for seat in scene.seats] == pytest.approx([0.0, 0.0])
    assert [person.x for person in scene.people] == pytest.approx([2.0])
    assert [person.y for person in scene.people] == pytest.approx([-1.12])
    assert scene.source == "camera"


def test_camera_observation_provider_projects_detections_from_latest_odom() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[],
        people=[],
        robot_x=0.0,
        robot_y=0.0,
        chair_distance_m=2.0,
        lateral_span_m=4.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = PoseStamped(
        frame_id="map",
        position=Vector3(10.0, 20.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2)),
    )
    provider._vl_model = FakeVlModel(
        {
            "chair": [SimpleNamespace(name="chair", bbox=(40.0, 20.0, 60.0, 80.0))],
            "person": [SimpleNamespace(name="person", bbox=(90.0, 20.0, 100.0, 80.0))],
        }
    )

    scene = provider.get_seat_scene()

    assert scene.robot_x == pytest.approx(10.0)
    assert scene.robot_y == pytest.approx(20.0)
    assert len(scene.seats) == 1
    assert scene.seats[0].seat_id == "seat_1"
    assert scene.seats[0].x == pytest.approx(10.0)
    assert scene.seats[0].y == pytest.approx(22.0)
    assert scene.seats[0].yaw == pytest.approx(math.pi / 2)
    assert len(scene.people) == 1
    assert scene.people[0].x == pytest.approx(8.2)
    assert scene.people[0].y == pytest.approx(22.0)


def test_camera_observation_provider_uses_one_odom_snapshot_per_detection() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[],
        people=[],
        robot_x=0.0,
        robot_y=0.0,
        chair_distance_m=2.0,
        lateral_span_m=4.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = PoseStamped(
        frame_id="map",
        position=Vector3(10.0, 20.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    provider._vl_model = OdomMutatingFakeVlModel(
        {
            "chair": [SimpleNamespace(name="chair", bbox=(40.0, 20.0, 60.0, 80.0))],
            "person": [],
        },
        provider,
    )

    scene = provider.get_seat_scene()

    assert scene.robot_x == pytest.approx(10.0)
    assert scene.robot_y == pytest.approx(20.0)
    assert scene.seats[0].x == pytest.approx(12.0)
    assert scene.seats[0].y == pytest.approx(20.0)
    assert provider._latest_odom.x == pytest.approx(100.0)


def test_camera_observation_provider_clamps_detection_bbox_to_image_width() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[],
        people=[],
        robot_x=0.0,
        robot_y=0.0,
        chair_distance_m=2.0,
        lateral_span_m=4.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = PoseStamped(
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    provider._vl_model = FakeVlModel(
        {
            "chair": [
                SimpleNamespace(name="chair", bbox=(-20.0, 20.0, 20.0, 80.0)),
                SimpleNamespace(name="chair", bbox=(80.0, 20.0, 140.0, 80.0)),
                SimpleNamespace(name="chair", bbox=(80.0, 20.0, 20.0, 80.0)),
            ],
            "person": [],
        }
    )

    scene = provider.get_seat_scene()

    assert [seat.x for seat in scene.seats] == pytest.approx([2.0, 2.0, 2.0])
    assert [seat.y for seat in scene.seats] == pytest.approx([-1.6, 1.6, 0.0])


def test_camera_observation_provider_falls_back_without_image() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[1.0, 2.0, 0.0],
        people=[1.1, 2.0],
        robot_x=-1.0,
        robot_y=2.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = None
    provider._latest_odom = None

    scene = provider.get_seat_scene()

    assert scene.seats == [SeatObservation("seat_1", 1.0, 2.0, 0.0)]
    assert scene.people == [PersonObservation(1.1, 2.0)]
    assert scene.robot_x == -1.0
    assert scene.robot_y == 2.0
    assert scene.source == "no_camera_image"


def test_camera_observation_provider_requires_odom_for_camera_source() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[1.0, 2.0, 0.0],
        people=[1.1, 2.0],
        robot_x=-1.0,
        robot_y=2.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = None
    provider._vl_model = FakeVlModel({"chair": [], "person": []})

    scene = provider.get_seat_scene()

    assert scene.seats == [SeatObservation("seat_1", 1.0, 2.0, 0.0)]
    assert scene.people == [PersonObservation(1.1, 2.0)]
    assert scene.robot_x == -1.0
    assert scene.robot_y == 2.0
    assert scene.source == "camera_no_odom"
    assert provider._vl_model.queries == []


def test_camera_observation_provider_rejects_stale_image(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(seat_guide_module.time, "time", lambda: 200.0)
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(max_input_age_s=5.0)
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(
        np.zeros((100, 100, 3), dtype=np.uint8), ts=190.0
    )
    provider._latest_odom = PoseStamped(
        ts=199.0,
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    provider._vl_model = FakeVlModel(
        {"chair": [SimpleNamespace(bbox=(40, 20, 60, 80))], "person": []}
    )

    scene = provider.get_seat_scene()

    assert scene.source == "stale_camera_image"
    assert provider._vl_model.queries == []


def test_camera_observation_provider_rejects_stale_odom(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(seat_guide_module.time, "time", lambda: 200.0)
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(max_input_age_s=5.0)
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(
        np.zeros((100, 100, 3), dtype=np.uint8), ts=199.0
    )
    provider._latest_odom = PoseStamped(
        ts=190.0,
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    provider._vl_model = FakeVlModel(
        {"chair": [SimpleNamespace(bbox=(40, 20, 60, 80))], "person": []}
    )

    scene = provider.get_seat_scene()

    assert scene.source == "stale_camera_odom"
    assert provider._vl_model.queries == []


def test_camera_seat_provider_status_reports_missing_runtime_inputs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("ALIBABA_API_KEY", raising=False)
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[1.0, 2.0, 0.0],
        people=[1.1, 2.0],
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = None
    provider._latest_odom = None

    assert provider.camera_seat_provider_status() == (
        "CameraSeatObservationProvider status: image=missing; image_fresh=missing; "
        "odom=missing; odom_fresh=missing; detection_model=qwen; "
        "credential=missing; override=inactive; "
        "configured_fallback_seats=1; configured_fallback_people=1."
    )


def test_camera_seat_provider_status_reports_live_inputs_and_credentials(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("ALIBABA_API_KEY", "test-key")
    monkeypatch.setattr(seat_guide_module.time, "time", lambda: 101.0)
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(seats=[], people=[])
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(
        np.zeros((120, 160, 3), dtype=np.uint8), ts=100.0
    )
    provider._latest_odom = PoseStamped(
        ts=100.0,
        frame_id="map",
        position=Vector3(1.0, 2.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.5)),
    )

    assert provider.camera_seat_provider_status() == (
        "CameraSeatObservationProvider status: image=160x120; "
        "image_fresh=true; odom=(1.00, 2.00, yaw=0.50); "
        "odom_fresh=true; detection_model=qwen; "
        "credential=present; override=inactive; configured_fallback_seats=0; "
        "configured_fallback_people=0."
    )


def test_camera_seat_provider_status_reports_runtime_override() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(seats=[], people=[])
    provider._scene_override = SeatSceneObservation(
        seats=[SeatObservation("manual", 1.0, 0.0, 0.0)],
        people=[],
        source="runtime_override",
    )
    provider._scene_lock = RLock()
    provider._latest_image = None
    provider._latest_odom = None

    assert "override=active" in provider.camera_seat_provider_status()


def test_camera_observation_provider_runtime_override_source() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig()
    provider._scene_override = None
    provider._scene_lock = RLock()

    provider.set_seat_scene(
        seats=[1.0, 2.0, 0.0],
        people=[1.1, 2.0],
        robot_x=-1.0,
        robot_y=2.0,
    )
    scene = provider.get_seat_scene()

    assert scene.source == "runtime_override"
    assert scene.seats == [SeatObservation("seat_1", 1.0, 2.0, 0.0)]


def test_camera_observation_provider_reports_empty_camera_detection_source() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[1.0, 2.0, 0.0],
        people=[],
        robot_x=-1.0,
        robot_y=2.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = PoseStamped(
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    provider._vl_model = FakeVlModel({"chair": [], "person": []})

    scene = provider.get_seat_scene()

    assert scene.seats == [SeatObservation("seat_1", 1.0, 2.0, 0.0)]
    assert scene.source == "camera_no_seats_detected"


def test_camera_observation_provider_reports_detection_error_source() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[1.0, 2.0, 0.0],
        people=[],
        robot_x=-1.0,
        robot_y=2.0,
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = PoseStamped(
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )

    def raise_detection_error(
        image: Image, odom: PoseStamped | None = None
    ) -> SeatSceneObservation:
        raise RuntimeError("vlm unavailable")

    provider._detect_scene_from_image = raise_detection_error

    scene = provider.get_seat_scene()

    assert scene.seats == [SeatObservation("seat_1", 1.0, 2.0, 0.0)]
    assert scene.source == "camera_detection_error"


def test_camera_observation_provider_reports_missing_qwen_key_as_detection_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("ALIBABA_API_KEY", raising=False)
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig(
        seats=[1.0, 2.0, 0.0],
        people=[],
        robot_x=-1.0,
        robot_y=2.0,
        detection_model="qwen",
    )
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = Image.from_numpy(np.zeros((100, 100, 3), dtype=np.uint8))
    provider._latest_odom = PoseStamped(
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    provider._vl_model = None

    scene = provider.get_seat_scene()

    assert scene.seats == [SeatObservation("seat_1", 1.0, 2.0, 0.0)]
    assert scene.source == "camera_detection_error"


def test_camera_observation_provider_does_not_fake_default_scene_without_image() -> None:
    provider = CameraSeatObservationProvider.__new__(CameraSeatObservationProvider)
    provider.config = CameraSeatSceneConfig()
    provider._scene_override = None
    provider._scene_lock = RLock()
    provider._latest_image = None

    scene = provider.get_seat_scene()

    assert scene.seats == []
    assert scene.people == []
    assert scene.source == "no_camera_image"


def test_parse_flat_observation_lists() -> None:
    assert _parse_seats([1.0, 2.0, 0.1, 3.0, 4.0, 0.2]) == [
        SeatObservation("seat_1", 1.0, 2.0, 0.1),
        SeatObservation("seat_2", 3.0, 4.0, 0.2),
    ]
    assert _parse_people([1.0, 2.0, 3.0, 4.0]) == [
        PersonObservation(1.0, 2.0),
        PersonObservation(3.0, 4.0),
    ]
    assert _flatten_seats([SeatObservation("ignored", 1.0, 2.0, 0.1)]) == [1.0, 2.0, 0.1]
    assert _flatten_people([PersonObservation(1.0, 2.0)]) == [1.0, 2.0]


def test_parse_flat_observation_lists_reject_bad_lengths() -> None:
    with pytest.raises(ValueError, match="triples"):
        _parse_seats([1.0, 2.0])

    with pytest.raises(ValueError, match="pairs"):
        _parse_people([1.0])


@pytest.mark.parametrize(
    "text",
    [
        "Please find me an empty seat",
        "guide me to a chair",
        "帮我找一个空位",
        "带我去座位",
        "我要找到一个位置",
        "帮我找到附近的位置, 然后找到一个空位置",
    ],
)
def test_parse_seat_guide_intent_accepts_seat_requests(text: str) -> None:
    intent = parse_seat_guide_intent(text)

    assert intent.should_find_seat
    assert intent.normalized_text


@pytest.mark.parametrize(
    "text",
    [
        "preview find me an empty seat",
        "preflight guide me to a chair",
        "预检帮我找一个空位",
        "测试带我去座位",
        "不要动, 先帮我找一个位置",
    ],
)
def test_parse_seat_guide_preview_request_detects_no_motion_requests(text: str) -> None:
    assert is_seat_guide_preview_request(text)


@pytest.mark.parametrize(
    "text",
    [
        "",
        "start patrol",
        "what is on the table",
        "会议什么时候开始",
    ],
)
def test_parse_seat_guide_intent_rejects_other_requests(text: str) -> None:
    assert not parse_seat_guide_intent(text).should_find_seat


def _complete_acceptance_transcript() -> str:
    return """Hardware run registry: /tmp/dimos/runs/hardware.json
Hardware run mode: hardware.
Hardware blueprint: unitree-go2-seat-guide-agentic
WebInput status: web=started; thread=running; seat_route=seat_guide_direct; responses=connected; voice_upload=connected; stt=connected; human_transport=connected; url=http://localhost:5555.
Using WebInput URL: http://localhost:5555
{"modules": {"CameraSeatObservationProvider": ["camera_seat_provider_status"], "SeatGuideSkillContainer": ["seat_guide_status"], "WebInput": ["web_input_status"], "SpeakSkill": ["speech_status"]}}
image=160x120
image_fresh=true
credential=present
odom=(1.00, 2.00, yaw=0.50)
odom_fresh=true
override=inactive
configured_fallback_seats=0
configured_fallback_people=0
tts=ready
audio_output=connected
+ dimos mcp call speak --json-args {"text": "SeatGuide audio check. I can guide you to an empty seat.", "blocking": true}
Spoke: SeatGuide audio check. I can guide you to an empty seat.
Audio output confirmation:
Operator audio confirmation: HEARD
SeatGuide scene source=camera: 2 seats [seat_1=(1.00, 2.00, yaw=0.00)], 0 people [none], robot=(1.00, 2.00).
SeatGuide preflight ready: navigation=IDLE; perception=camera seats=2 people=0; empty=2 occupied=0; selected=seat_1; goal=(1.65, 2.00, yaw=0.00); speaker=connected.
SeatGuide preview source=camera: selected seat_1 empty=2 occupied=0 seat=(1.00, 2.00, yaw=0.00) goal=(1.65, 2.00, yaw=0.00).
Captured WebInput agent_responses stream
Manual no-motion voice gate:
Press Enter here when ready.
Click the microphone button and say: 预检帮我找一个空位
Captured WebInput voice agent_responses stream
WebInput received text text=预检帮我找一个空位
WebInput received text text=预检帮我找一个空位
WebInput received text text=帮我找一个空位
WebInput routing text to SeatGuide preview text=预检帮我找一个空位
Capturing DimOS log snapshot after no-motion checks
No-motion checks completed.
Operator confirmation: LIVE
Live voice navigation gate:
Press Enter here when ready.
Say: 帮我找一个空位
Captured live WebInput voice agent_responses stream
WebInput routing text to SeatGuide live request text=帮我找一个空位
Navigating to
goal_sequence=1
Checking SeatGuide navigation completion
goal_reached=true
SeatGuide navigation goal reached
Capturing DimOS log snapshot after live request
"""


def test_acceptance_log_verifier_accepts_complete_hardware_transcript(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(_complete_acceptance_transcript())

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0
    assert "contains all required evidence" in result.stdout


def test_acceptance_log_verifier_accepts_earlier_stale_goal_reached_status(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "Operator confirmation: LIVE\n",
            "goal_reached=true\nOperator confirmation: LIVE\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0
    assert "contains all required evidence" in result.stdout


@pytest.mark.parametrize(
    ("missing_text", "expected_error"),
    [
        ("Hardware run mode: hardware.\n", "hardware run mode"),
        ("Hardware run registry: /tmp/dimos/runs/hardware.json\n", "hardware run registry"),
        (
            "Hardware blueprint: unitree-go2-seat-guide-agentic\n",
            "SeatGuide hardware blueprint",
        ),
        ("voice_upload=connected; ", "WebInput browser audio upload route"),
        ("stt=connected; ", "WebInput speech-to-text pipeline"),
        ("image=160x120\n", "camera image readiness"),
        ("image_fresh=true\n", "fresh camera image readiness"),
        ("odom_fresh=true\n", "fresh odometry readiness"),
        ("override=inactive\n", "camera runtime override disabled"),
        ("configured_fallback_seats=0\n", "camera fallback seats disabled"),
        ("configured_fallback_people=0\n", "camera fallback people disabled"),
        (
            '"SeatGuideSkillContainer": ["seat_guide_status"], ',
            "SeatGuide planner/navigation module",
        ),
        (
            "Capturing DimOS log snapshot after no-motion checks\n",
            "no-motion DimOS log snapshot",
        ),
        ("No-motion checks completed.\n", "no-motion completion marker"),
        ("Manual no-motion voice gate:\n", "browser microphone no-motion gate"),
        ("Press Enter here when ready.\n", "browser microphone readiness prompts"),
        (
            "Click the microphone button and say: 预检帮我找一个空位\n",
            "browser microphone no-motion spoken phrase",
        ),
        ("Live voice navigation gate:\n", "browser microphone live gate"),
        ("Say: 帮我找一个空位\n", "browser microphone live spoken phrase"),
        (
            "WebInput routing text to SeatGuide live request text=帮我找一个空位\n",
            "live WebInput SeatGuide route",
        ),
        (
            "WebInput received text text=预检帮我找一个空位\n",
            "WebInput recognized text events",
        ),
        (
            "Capturing DimOS log snapshot after live request\n",
            "live DimOS log snapshot",
        ),
        ("speaker=connected", "SeatGuide speaker wiring"),
        (
            "Spoke: SeatGuide audio check. I can guide you to an empty seat.\n",
            "TTS audio check completion",
        ),
        ("Operator audio confirmation: HEARD\n", "operator heard TTS confirmation"),
        ("goal_reached=true\n", "navigation completion"),
    ],
)
def test_acceptance_log_verifier_rejects_missing_required_evidence(
    tmp_path: Path,
    missing_text: str,
    expected_error: str,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(_complete_acceptance_transcript().replace(missing_text, "", 1))

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert expected_error in result.stderr


def test_acceptance_log_verifier_rejects_missing_occupancy_counts(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript()
        .replace("empty=2 occupied=0; ", "")
        .replace("empty=2 occupied=0 ", "")
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "SeatGuide occupancy counts" in result.stderr


def test_acceptance_log_verifier_rejects_wrong_recognized_live_phrase(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "WebInput received text text=帮我找一个空位\n",
            "WebInput received text text=帮我找一个垃圾桶\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "recognized live SeatGuide phrase" in result.stderr


def test_acceptance_log_verifier_rejects_wrong_routed_preview_phrase(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "WebInput routing text to SeatGuide preview text=预检帮我找一个空位\n",
            "WebInput routing text to SeatGuide preview text=预检帮我找一个垃圾桶\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "no-motion WebInput SeatGuide phrase route" in result.stderr


def test_acceptance_log_verifier_rejects_live_before_no_motion_completion(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "No-motion checks completed.\nOperator confirmation: LIVE\n",
            "Operator confirmation: LIVE\nNo-motion checks completed.\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "no-motion before live order" in result.stderr


def test_acceptance_log_verifier_rejects_no_motion_completion_before_snapshot(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "Capturing DimOS log snapshot after no-motion checks\nNo-motion checks completed.\n",
            "No-motion checks completed.\nCapturing DimOS log snapshot after no-motion checks\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "no-motion snapshot before completion order" in result.stderr


def test_acceptance_log_verifier_rejects_no_motion_speech_before_readiness_prompt(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "Press Enter here when ready.\nClick the microphone button and say: 预检帮我找一个空位\n",
            "Click the microphone button and say: 预检帮我找一个空位\nPress Enter here when ready.\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "no-motion readiness before speech order" in result.stderr


def test_acceptance_log_verifier_rejects_live_speech_before_live_readiness_prompt(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "Press Enter here when ready.\nSay: 帮我找一个空位\n",
            "Say: 帮我找一个空位\nPress Enter here when ready.\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "live readiness before speech order" in result.stderr


def test_acceptance_log_verifier_rejects_navigation_before_live_route(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "WebInput routing text to SeatGuide live request text=帮我找一个空位\nNavigating to\n",
            "Navigating to\nWebInput routing text to SeatGuide live request text=帮我找一个空位\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "live route before navigation order" in result.stderr


def test_acceptance_log_verifier_rejects_audio_confirmation_before_tts_completion(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "Spoke: SeatGuide audio check. I can guide you to an empty seat.\n"
            "Audio output confirmation:\n"
            "Operator audio confirmation: HEARD\n",
            "Audio output confirmation:\n"
            "Operator audio confirmation: HEARD\n"
            "Spoke: SeatGuide audio check. I can guide you to an empty seat.\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "TTS completion before operator audio confirmation order" in result.stderr


def test_acceptance_log_verifier_rejects_goal_reached_before_polling(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript().replace(
            "Navigating to\ngoal_sequence=1\nChecking SeatGuide navigation completion\ngoal_reached=true\n",
            "Navigating to\ngoal_reached=true\ngoal_sequence=1\nChecking SeatGuide navigation completion\n",
            1,
        )
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "polling before completion order" in result.stderr


def test_acceptance_log_verifier_rejects_direct_mcp_live_request(
    tmp_path: Path,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(
        _complete_acceptance_transcript()
        + '\n+ dimos mcp call handle_seat_request --json-args \'{"text": "帮我找一个空位"}\'\n'
    )

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert "direct MCP live SeatGuide call" in result.stderr


@pytest.mark.parametrize(
    ("forbidden_text", "expected_error"),
    [
        (
            '+ dimos mcp call set_seat_scene --json-args \'{"seats": [0, 0, 0], "people": []}\'',
            "fallback seat scene calibration",
        ),
        (
            "+ dimos mcp call clear_seat_scene_override",
            "fallback seat scene override clearing",
        ),
        (
            'dimos mcp call seat_guide_preflight --json-args \'{"require_live_perception": false}\'',
            "fallback live-perception bypass",
        ),
        (
            'dimos mcp call seat_guide_preflight --json-args \'{"require_live_perception":false}\'',
            "fallback live-perception bypass",
        ),
        (
            "dimos mcp call seat_guide_preflight --arg require_live_perception=false",
            "fallback live-perception bypass",
        ),
    ],
)
def test_acceptance_log_verifier_rejects_fallback_calibration_evidence(
    tmp_path: Path,
    forbidden_text: str,
    expected_error: str,
) -> None:
    log_file = tmp_path / "acceptance.log"
    log_file.write_text(_complete_acceptance_transcript() + f"\n{forbidden_text}\n")

    result = subprocess.run(
        ["bash", str(ACCEPTANCE_LOG_VERIFIER), str(log_file)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 3
    assert expected_error in result.stderr


def _extract_bash_function(source: str, name: str) -> str:
    lines = source.splitlines()
    start = lines.index(f"{name}() {{")
    body = [lines[start]]
    for line in lines[start + 1 :]:
        body.append(line)
        if line == "}":
            return "\n".join(body)
    raise AssertionError(f"Could not extract bash function {name}")


def _run_hardware_registry_guard(
    tmp_path: Path,
    *,
    run_id: str,
    registry: dict[str, Any],
) -> subprocess.CompletedProcess[str]:
    state_dir = tmp_path / "state"
    registry_dir = state_dir / "dimos" / "runs"
    registry_dir.mkdir(parents=True)
    (registry_dir / f"{run_id}.json").write_text(json.dumps(registry))
    log_file = tmp_path / "acceptance.log"

    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "guard.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "log"),
                _extract_bash_function(script_source, "extract_run_id"),
                _extract_bash_function(
                    script_source, "require_hardware_run_registry"
                ),
                'log_file="$1"',
                'XDG_STATE_HOME="$2"',
                f"require_hardware_run_registry $'  Run ID:    {run_id}\\n'",
            ]
        )
    )

    return subprocess.run(
        ["bash", str(wrapper), str(log_file), str(state_dir)],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_smoke_registry_guard(
    tmp_path: Path,
    *,
    run_id: str,
    registry: dict[str, Any],
) -> subprocess.CompletedProcess[str]:
    state_dir = tmp_path / "state"
    registry_dir = state_dir / "dimos" / "runs"
    registry_dir.mkdir(parents=True)
    (registry_dir / f"{run_id}.json").write_text(json.dumps(registry))

    script_source = SMOKE_SCRIPT.read_text()
    wrapper = tmp_path / "smoke_guard.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "extract_run_id"),
                _extract_bash_function(
                    script_source, "require_seat_guide_run_registry"
                ),
                'XDG_STATE_HOME="$1"',
                f"require_seat_guide_run_registry $'  Run ID:    {run_id}\\n'",
            ]
        )
    )

    return subprocess.run(
        ["bash", str(wrapper), str(state_dir)],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_web_input_url_extract(tmp_path: Path, status_text: str) -> str:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "extract_url.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "extract_web_input_url"),
                'extract_web_input_url "$1"',
            ]
        )
    )
    result = subprocess.run(
        ["bash", str(wrapper), status_text],
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def _run_goal_sequence_extract(tmp_path: Path, status_text: str) -> str:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "extract_goal_sequence.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "extract_goal_sequence"),
                'extract_goal_sequence "$1"',
            ]
        )
    )
    result = subprocess.run(
        ["bash", str(wrapper), status_text],
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def _run_goal_completion_check(
    tmp_path: Path,
    *,
    previous_sequence: int,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "goal_completed.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "extract_goal_sequence"),
                _extract_bash_function(
                    script_source, "seat_guide_goal_completed_after_sequence"
                ),
                'seat_guide_goal_completed_after_sequence "$1" "$2"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), str(previous_sequence), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_preflight_ready_check(
    tmp_path: Path,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "preflight_ready.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(
                    script_source, "seat_guide_preflight_ready_for_hardware"
                ),
                'seat_guide_preflight_ready_for_hardware "$1"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_web_input_ready_check(
    tmp_path: Path,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "web_input_ready.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "web_input_ready_for_seat_guide"),
                'web_input_ready_for_seat_guide "$1"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_web_input_no_go_details(
    tmp_path: Path,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "web_input_details.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "log"),
                _extract_bash_function(script_source, "log_web_input_no_go_details"),
                'log_file="$1"',
                'log_web_input_no_go_details "$2"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), str(tmp_path / "web_input.log"), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_camera_provider_ready_check(
    tmp_path: Path,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "camera_ready.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(
                    script_source, "camera_provider_ready_for_hardware"
                ),
                'camera_provider_ready_for_hardware "$1"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_camera_provider_no_go_details(
    tmp_path: Path,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "camera_details.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "log"),
                _extract_bash_function(
                    script_source, "log_camera_provider_no_go_details"
                ),
                'log_file="$1"',
                'log_camera_provider_no_go_details "$2"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), str(tmp_path / "camera.log"), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_speech_no_go_details(
    tmp_path: Path,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "speech_details.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "log"),
                _extract_bash_function(script_source, "log_speech_no_go_details"),
                'log_file="$1"',
                'log_speech_no_go_details "$2"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), str(tmp_path / "speech.log"), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_seat_guide_no_go_details(
    tmp_path: Path,
    status_text: str,
) -> subprocess.CompletedProcess[str]:
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "seat_guide_details.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "log"),
                _extract_bash_function(script_source, "log_seat_guide_no_go_details"),
                'log_file="$1"',
                'log_seat_guide_no_go_details "$2"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), str(tmp_path / "seat_guide.log"), status_text],
        check=False,
        text=True,
        capture_output=True,
    )


def _run_stream_wait_check(
    tmp_path: Path,
    stream_text: str,
    expected_text: str,
    start_offset: int = 0,
) -> subprocess.CompletedProcess[str]:
    stream_file = tmp_path / "stream.txt"
    stream_file.write_text(stream_text)
    script_source = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    wrapper = tmp_path / "stream_wait.sh"
    wrapper.write_text(
        "\n".join(
            [
                "set -euo pipefail",
                _extract_bash_function(script_source, "wait_for_stream_text"),
                'wait_for_stream_text "$1" "$2" 0 "$3"',
            ]
        )
    )
    return subprocess.run(
        ["bash", str(wrapper), str(stream_file), expected_text, str(start_offset)],
        check=False,
        text=True,
        capture_output=True,
    )


@pytest.mark.parametrize(
    ("status_text", "expected_url"),
    [
        (
            "WebInput status: web=started; url=http://localhost:5555.",
            "http://localhost:5555",
        ),
        (
            "WebInput status: web=started; url=http://127.0.0.1:6001.",
            "http://127.0.0.1:6001",
        ),
        (
            "WebInput status: web=not_started; url=unavailable.",
            "",
        ),
    ],
)
def test_hardware_acceptance_extracts_web_input_url(
    tmp_path: Path,
    status_text: str,
    expected_url: str,
) -> None:
    assert _run_web_input_url_extract(tmp_path, status_text) == expected_url


@pytest.mark.parametrize(
    ("status_text", "expected_sequence"),
    [
        (
            "SeatGuide navigation status: navigation=IDLE; goal_reached=false; goal_sequence=0.",
            "0",
        ),
        (
            "old goal_sequence=1\nnew goal_sequence=2",
            "2",
        ),
        (
            "SeatGuide navigation status: navigation=missing; goal_reached=unknown.",
            "",
        ),
    ],
)
def test_hardware_acceptance_extracts_goal_sequence(
    tmp_path: Path,
    status_text: str,
    expected_sequence: str,
) -> None:
    assert _run_goal_sequence_extract(tmp_path, status_text) == expected_sequence


@pytest.mark.parametrize(
    ("previous_sequence", "status_text", "expected_returncode"),
    [
        (
            1,
            "SeatGuide navigation status: navigation=IDLE; goal_reached=true; goal_sequence=2.",
            0,
        ),
        (
            1,
            "SeatGuide navigation status: navigation=IDLE; goal_reached=true; goal_sequence=1.",
            1,
        ),
        (
            1,
            "SeatGuide navigation status: navigation=FOLLOWING_PATH; goal_reached=false; goal_sequence=2.",
            1,
        ),
        (
            1,
            "SeatGuide navigation status: navigation=missing; goal_reached=unknown.",
            1,
        ),
    ],
)
def test_hardware_acceptance_goal_completion_requires_new_reached_goal(
    tmp_path: Path,
    previous_sequence: int,
    status_text: str,
    expected_returncode: int,
) -> None:
    result = _run_goal_completion_check(
        tmp_path,
        previous_sequence=previous_sequence,
        status_text=status_text,
    )

    assert result.returncode == expected_returncode


@pytest.mark.parametrize(
    ("status_text", "expected_returncode"),
    [
        (
            "SeatGuide preflight ready: navigation=IDLE; perception=camera seats=2 people=0; selected=seat_1; goal=(1.65, 2.00, yaw=0.00); speaker=connected.",
            0,
        ),
        (
            "SeatGuide preflight ready: navigation=FOLLOWING_PATH; perception=camera seats=2 people=0; selected=seat_1; goal=(1.65, 2.00, yaw=0.00); speaker=connected.",
            1,
        ),
        (
            "SeatGuide preflight ready: navigation=IDLE; perception=camera seats=2 people=0; selected=seat_1; goal=(1.65, 2.00, yaw=0.00); speaker=missing.",
            1,
        ),
        (
            "SeatGuide preflight no-go: navigation=IDLE; perception=camera no seats; speaker=connected.",
            1,
        ),
    ],
)
def test_hardware_acceptance_preflight_requires_navigation_and_speaker_ready(
    tmp_path: Path,
    status_text: str,
    expected_returncode: int,
) -> None:
    result = _run_preflight_ready_check(tmp_path, status_text)

    assert result.returncode == expected_returncode


@pytest.mark.parametrize(
    ("status_text", "expected_returncode"),
    [
        (
            "WebInput status: web=started; thread=running; seat_route=seat_guide_direct; responses=connected; voice_upload=connected; stt=connected; human_transport=connected; url=http://localhost:5555.",
            0,
        ),
        (
            "WebInput status: web=started; thread=not_running; seat_route=seat_guide_direct; responses=connected; voice_upload=connected; stt=connected; human_transport=connected; url=http://localhost:5555.",
            1,
        ),
        (
            "WebInput status: web=started; thread=running; seat_route=agent_only; responses=connected; voice_upload=connected; stt=connected; human_transport=connected; url=http://localhost:5555.",
            1,
        ),
        (
            "WebInput status: web=started; thread=running; seat_route=seat_guide_direct; responses=missing; voice_upload=connected; stt=connected; human_transport=connected; url=http://localhost:5555.",
            1,
        ),
        (
            "WebInput status: web=started; thread=running; seat_route=seat_guide_direct; responses=connected; voice_upload=missing; stt=connected; human_transport=connected; url=http://localhost:5555.",
            1,
        ),
        (
            "WebInput status: web=started; thread=running; seat_route=seat_guide_direct; responses=connected; voice_upload=connected; stt=missing; human_transport=connected; url=http://localhost:5555.",
            1,
        ),
        (
            "WebInput status: web=started; thread=running; seat_route=seat_guide_direct; responses=connected; voice_upload=connected; stt=error(RuntimeError: whisper missing); human_transport=connected; url=http://localhost:5555.",
            1,
        ),
        (
            "WebInput status: web=started; thread=running; seat_route=seat_guide_direct; responses=connected; voice_upload=connected; stt=connected; human_transport=missing; url=http://localhost:5555.",
            1,
        ),
    ],
)
def test_hardware_acceptance_web_input_requires_complete_voice_route(
    tmp_path: Path,
    status_text: str,
    expected_returncode: int,
) -> None:
    result = _run_web_input_ready_check(tmp_path, status_text)

    assert result.returncode == expected_returncode


def test_hardware_acceptance_web_input_no_go_details_are_actionable(
    tmp_path: Path,
) -> None:
    status_text = (
        "WebInput status: web=not_started; thread=not_running; seat_route=agent_only; "
        "responses=missing; voice_upload=missing; stt=missing; human_transport=missing; url=unavailable."
    )

    result = _run_web_input_no_go_details(tmp_path, status_text)

    assert result.returncode == 0
    assert "WebInput server is not started" in result.stdout
    assert "server thread is not running" in result.stdout
    assert "not directly wired to SeatGuide" in result.stdout
    assert "response stream is missing" in result.stdout
    assert "browser audio upload endpoint is not connected" in result.stdout
    assert "speech-to-text pipeline is unavailable" in result.stdout
    assert "fallback transport is missing" in result.stdout


@pytest.mark.parametrize(
    ("status_text", "expected_returncode"),
    [
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=true; odom=(1.00, 2.00, yaw=0.50); odom_fresh=true; detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=0; configured_fallback_people=0.",
            0,
        ),
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=true; odom=(1.00, 2.00, yaw=0.50); odom_fresh=true; detection_model=qwen; credential=missing; override=inactive; configured_fallback_seats=0; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: image=missing; image_fresh=missing; odom=(1.00, 2.00, yaw=0.50); odom_fresh=true; detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=0; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=true; odom=missing; odom_fresh=missing; detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=0; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=false; odom=(1.00, 2.00, yaw=0.50); odom_fresh=true; detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=0; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=true; odom=(1.00, 2.00, yaw=0.50); odom_fresh=false; detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=0; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=0; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=true; odom=(1.00, 2.00, yaw=0.50); odom_fresh=true; detection_model=qwen; credential=present; override=active; configured_fallback_seats=0; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=true; odom=(1.00, 2.00, yaw=0.50); odom_fresh=true; detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=1; configured_fallback_people=0.",
            1,
        ),
        (
            "CameraSeatObservationProvider status: image=160x120; image_fresh=true; odom=(1.00, 2.00, yaw=0.50); odom_fresh=true; detection_model=qwen; credential=present; override=inactive; configured_fallback_seats=0; configured_fallback_people=1.",
            1,
        ),
    ],
)
def test_hardware_acceptance_camera_provider_requires_live_inputs(
    tmp_path: Path,
    status_text: str,
    expected_returncode: int,
) -> None:
    result = _run_camera_provider_ready_check(tmp_path, status_text)

    assert result.returncode == expected_returncode


def test_hardware_acceptance_camera_provider_no_go_details_are_actionable(
    tmp_path: Path,
) -> None:
    status_text = (
        "CameraSeatObservationProvider status: image=missing; image_fresh=false; "
        "odom=missing; odom_fresh=false; detection_model=qwen; credential=missing; "
        "override=active; "
        "configured_fallback_seats=1; configured_fallback_people=1."
    )

    result = _run_camera_provider_no_go_details(tmp_path, status_text)

    assert result.returncode == 0
    assert "ALIBABA_API_KEY" in result.stdout
    assert "Camera image is missing" in result.stdout
    assert "Camera image is stale" in result.stdout
    assert "Odometry is missing" in result.stdout
    assert "Odometry is stale" in result.stdout
    assert "Runtime seat-scene override is active" in result.stdout
    assert "Configured fallback seats/people are non-zero" in result.stdout


def test_hardware_acceptance_speech_no_go_details_are_actionable(
    tmp_path: Path,
) -> None:
    status_text = (
        "SpeakSkill status: tts=unavailable; reason=OPENAI_API_KEY is not set; "
        "audio_output=missing; background_speech_threads=0."
    )

    result = _run_speech_no_go_details(tmp_path, status_text)

    assert result.returncode == 0
    assert "OPENAI_API_KEY" in result.stdout
    assert "Audio output is not connected" in result.stdout


def test_hardware_acceptance_seat_guide_no_go_details_are_actionable(
    tmp_path: Path,
) -> None:
    status_text = (
        "SeatGuide readiness report: SeatGuide scene source=stale_camera_image: "
        "no seats visible or configured; 0 people detected. | "
        "SeatGuide preflight no-go: navigation=FOLLOWING_PATH; "
        "perception=stale_camera_odom no seats; speaker=missing. | "
        "SeatGuide preflight no-go: perception=camera_detection_error no seats. | "
        "SeatGuide preview source=configured_fallback: no empty seat available."
    )

    result = _run_seat_guide_no_go_details(tmp_path, status_text)

    assert result.returncode == 0
    assert "Navigation is busy" in result.stdout
    assert "speaker wiring is missing" in result.stdout
    assert "cannot see chairs" in result.stdout
    assert "camera frames are stale" in result.stdout
    assert "odometry is stale" in result.stdout
    assert "camera detection failed" in result.stdout
    assert "fallback/calibrated coordinates" in result.stdout
    assert "none are empty" in result.stdout


def test_hardware_acceptance_logs_seat_guide_details_for_preflight_failures() -> None:
    script = HARDWARE_ACCEPTANCE_SCRIPT.read_text()

    assert "log_seat_guide_no_go_details()" in script
    assert "seat_guide_status did not report live camera perception" in script
    assert script.count('log_seat_guide_no_go_details "${') >= 4


def test_hardware_acceptance_has_actionable_mcp_tools_failure_message() -> None:
    script = HARDWARE_ACCEPTANCE_SCRIPT.read_text()

    assert 'if ! tools="$(run_dimos mcp list-tools 2>&1)"; then' in script
    assert "Hardware acceptance no-go: MCP tools are unavailable." in script
    assert "Hardware acceptance no-go: missing MCP tool" in script
    assert "SeatGuide, WebInput, camera provider, and SpeakSkill modules" in script
    assert "unitree-go2-seat-guide-agentic and includes McpServer" in script
    assert 'require_tool "${tools}" "speak"' in script
    assert "SeatGuide audio check. I can guide you to an empty seat." in script
    assert "Operator audio confirmation" in script
    assert 'require_output_contains "${audio_check_output}" "Spoke: SeatGuide audio check"' in script
    assert "Transcript saved to: ${log_file}" in script


def test_hardware_acceptance_missing_stack_reports_transcript_path() -> None:
    script = HARDWARE_ACCEPTANCE_SCRIPT.read_text()
    missing_stack_block = script.split(
        'if grep -q "No running DimOS instance" <<<"${status_output}"; then',
        maxsplit=1,
    )[1].split("exit 2", maxsplit=1)[0]

    assert "No running DimOS stack found." in missing_stack_block
    assert "dimos run unitree-go2-seat-guide-agentic --robot-ip" in missing_stack_block
    assert 'log "Transcript saved to: ${log_file}"' in missing_stack_block


@pytest.mark.parametrize(
    ("stream_text", "expected_text", "expected_returncode"),
    [
        ("event: message\ndata: SeatGuide preflight ready\n", "SeatGuide preflight ready", 0),
        ("event: message\ndata: Navigating to (1.00, 2.00)\n", "Navigating to", 0),
        ("event: message\ndata: still thinking\n", "Navigating to", 1),
    ],
)
def test_hardware_acceptance_waits_for_webinput_stream_text(
    tmp_path: Path,
    stream_text: str,
    expected_text: str,
    expected_returncode: int,
) -> None:
    result = _run_stream_wait_check(tmp_path, stream_text, expected_text)

    assert result.returncode == expected_returncode


def test_hardware_acceptance_stream_wait_ignores_text_before_start_offset(
    tmp_path: Path,
) -> None:
    stale_text = "event: message\ndata: Navigating to stale goal\n"
    result = _run_stream_wait_check(
        tmp_path,
        stale_text + "event: message\ndata: still waiting\n",
        "Navigating to",
        start_offset=len(stale_text),
    )

    assert result.returncode == 1


def test_hardware_acceptance_stream_gates_use_offset_wait_result() -> None:
    script = HARDWARE_ACCEPTANCE_SCRIPT.read_text()

    assert 'grep -q "SeatGuide preflight ready" "${stream_file}"' not in script
    assert 'grep -q "Navigating to" "${stream_file}"' not in script
    assert script.count('if [[ "${stream_matched}" != "1" ]]; then') == 3


def test_hardware_acceptance_text_stream_failures_cleanup() -> None:
    script = HARDWARE_ACCEPTANCE_SCRIPT.read_text()

    post_failure_block = script.split(
        'log "Hardware acceptance no-go: WebInput /submit_query request failed."',
        maxsplit=1,
    )[1].split("exit 3", maxsplit=1)[0]
    text_wait_failure_block = script.split(
        'log "Hardware acceptance no-go: WebInput text route did not publish a ready SeatGuide preview response."',
        maxsplit=1,
    )[1].split("exit 3", maxsplit=1)[0]

    assert "stop_stream" in post_failure_block
    assert 'rm -f "${stream_file}"' in post_failure_block
    assert 'rm -f "${stream_file}"' in text_wait_failure_block


def test_hardware_acceptance_has_interrupt_stream_cleanup_trap() -> None:
    script = HARDWARE_ACCEPTANCE_SCRIPT.read_text()

    assert "cleanup_active_stream()" in script
    assert "trap cleanup_active_stream EXIT" in script
    assert "trap 'cleanup_active_stream; exit 130' INT" in script
    assert "trap 'cleanup_active_stream; exit 143' TERM" in script
    assert script.count('active_stream_file="${stream_file}"') == 3
    assert script.count('active_stream_pid="${stream_pid}"') == 3


def test_hardware_acceptance_auto_verifies_transcript_after_live_request() -> None:
    script = HARDWARE_ACCEPTANCE_SCRIPT.read_text()

    assert 'acceptance_log_verifier="${script_dir}/demo_seat_guide_verify_acceptance_log"' in script
    assert "verify_acceptance_log()" in script
    assert 'log "+ ${acceptance_log_verifier} ${log_file}"' in script
    live_tail = script.split(
        'capture_dimos_log "Capturing DimOS log snapshot after live request..."',
        maxsplit=1,
    )[1]
    assert live_tail.index("verify_acceptance_log") < live_tail.index(
        'log "Live request sent. Continue monitoring with: dimos log -f"'
    )


@pytest.mark.parametrize(
    "blueprint",
    ["unitree-go2-seat-guide", "unitree-go2-seat-guide-agentic"],
)
def test_hardware_acceptance_registry_guard_accepts_hardware_run(
    tmp_path: Path,
    blueprint: str,
) -> None:
    result = _run_hardware_registry_guard(
        tmp_path,
        run_id="hardware",
        registry={
            "run_id": "hardware",
            "blueprint": blueprint,
            "cli_args": [blueprint],
            "config_overrides": {"replay": False, "simulation": ""},
            "original_argv": ["dimos", "run", blueprint],
        },
    )

    assert result.returncode == 0
    assert "Hardware run mode: hardware." in result.stdout
    assert f"Hardware blueprint: {blueprint}" in result.stdout


@pytest.mark.parametrize(
    ("registry", "expected_output"),
    [
        (
            {"cli_args": ["--replay=true"], "config_overrides": {}},
            "replay mode",
        ),
        (
            {"cli_args": [], "config_overrides": {"replay": True}},
            "replay mode",
        ),
        (
            {"cli_args": [], "config_overrides": {}, "replay": True},
            "replay mode",
        ),
        (
            {"cli_args": ["--simulation=dimsim"], "config_overrides": {}},
            "simulation mode",
        ),
        (
            {"cli_args": [], "config_overrides": {"simulation": "customsim"}},
            "simulation mode",
        ),
        (
            {"cli_args": [], "config_overrides": {"simulation": True}},
            "simulation mode",
        ),
        (
            {"cli_args": [], "config_overrides": {}, "simulation": True},
            "simulation mode",
        ),
        (
            {
                "blueprint": "unitree-go2-agentic",
                "cli_args": ["unitree-go2-agentic"],
                "config_overrides": {"replay": False, "simulation": ""},
            },
            "not a SeatGuide Go2 blueprint",
        ),
    ],
)
def test_hardware_acceptance_registry_guard_rejects_non_hardware_runs(
    tmp_path: Path,
    registry: dict[str, Any],
    expected_output: str,
) -> None:
    registry = {
        "run_id": "not-hardware",
        "original_argv": ["dimos", "run", "unitree-go2-seat-guide-agentic"],
        **registry,
    }

    result = _run_hardware_registry_guard(
        tmp_path,
        run_id="not-hardware",
        registry=registry,
    )

    assert result.returncode == 3
    assert expected_output in result.stdout


@pytest.mark.parametrize(
    "tool_name",
    ["seat_guide_status", "preview_empty_seat_goal"],
)
def test_no_motion_smoke_calls_scene_and_goal_preview(tool_name: str) -> None:
    script = SMOKE_SCRIPT.read_text()

    assert f'require_tool "${{tools}}" "{tool_name}"' in script
    assert f"run_dimos mcp call {tool_name}" in script


def test_no_motion_smoke_requires_web_input_voice_path_ready() -> None:
    script = SMOKE_SCRIPT.read_text()

    assert "require_output_contains()" in script
    assert "web_input_output=" in script
    for expected in [
        "web=started",
        "thread=running",
        "seat_route=seat_guide_direct",
        "responses=connected",
        "voice_upload=connected",
        "stt=connected",
        "human_transport=connected",
    ]:
        assert f'require_output_contains "${{web_input_output}}" "{expected}"' in script


def test_no_motion_smoke_has_actionable_missing_stack_and_mcp_messages() -> None:
    script = SMOKE_SCRIPT.read_text()

    assert 'status_output="$(run_dimos status)"' in script
    assert "No running DimOS stack found." in script
    assert "dimos --replay run unitree-go2-seat-guide-agentic --daemon" in script
    assert "dimos run unitree-go2-seat-guide-agentic --robot-ip" in script
    assert "SeatGuide smoke no-go: MCP tools are unavailable." in script
    assert "SeatGuide smoke no-go: missing MCP tool" in script
    assert "SeatGuide, WebInput, camera provider, and SpeakSkill modules" in script
    assert "includes McpServer" in script


@pytest.mark.parametrize(
    "blueprint",
    ["unitree-go2-seat-guide", "unitree-go2-seat-guide-agentic"],
)
def test_no_motion_smoke_registry_guard_accepts_seat_guide_stack(
    tmp_path: Path,
    blueprint: str,
) -> None:
    result = _run_smoke_registry_guard(
        tmp_path,
        run_id="seat-guide-smoke",
        registry={
            "run_id": "seat-guide-smoke",
            "blueprint": blueprint,
            "original_argv": ["dimos", "run", blueprint],
        },
    )

    assert result.returncode == 0


def test_no_motion_smoke_registry_guard_rejects_general_go2_stack(
    tmp_path: Path,
) -> None:
    result = _run_smoke_registry_guard(
        tmp_path,
        run_id="general-go2",
        registry={
            "run_id": "general-go2",
            "blueprint": "unitree-go2-agentic",
            "original_argv": ["dimos", "run", "unitree-go2-agentic"],
        },
    )

    assert result.returncode == 3
    assert "not a SeatGuide Go2 blueprint" in result.stderr


def test_replay_smoke_starts_seat_guide_stack_and_runs_no_motion_smoke() -> None:
    script = REPLAY_SMOKE_SCRIPT.read_text()

    assert "run_dimos --replay run unitree-go2-seat-guide-agentic --daemon" in script
    assert "demo_seat_guide_smoke" in script
    assert "unitree-go2-agentic" not in script.replace(
        "unitree-go2-seat-guide-agentic", ""
    )


def test_hardware_bringup_starts_real_stack_then_runs_smoke_and_acceptance() -> None:
    script = HARDWARE_BRINGUP_SCRIPT.read_text()

    assert 'robot_ip="${SEAT_GUIDE_ROBOT_IP:-192.168.123.161}"' in script
    assert 'run_dimos run unitree-go2-seat-guide-agentic --robot-ip "${robot_ip}" --daemon' in script
    assert "demo_seat_guide_smoke" in script
    assert "demo_seat_guide_hardware_acceptance" in script
    assert "unitree-go2-agentic" not in script.replace(
        "unitree-go2-seat-guide-agentic", ""
    )


def test_hardware_bringup_requires_real_perception_and_speech_credentials() -> None:
    script = HARDWARE_BRINGUP_SCRIPT.read_text()

    assert 'ALIBABA_API_KEY' in script
    assert 'OPENAI_API_KEY' in script
    assert "SeatGuide bring-up no-go: ALIBABA_API_KEY is not set." in script
    assert "SeatGuide bring-up no-go: OPENAI_API_KEY is not set." in script


def test_hardware_bringup_allows_existing_stack_and_smoke_skip() -> None:
    script = HARDWARE_BRINGUP_SCRIPT.read_text()

    assert "--skip-start" in script
    assert "--skip-smoke" in script
    assert "Using the currently running DimOS stack." in script
    assert "Skipping no-motion smoke checks." in script


@pytest.mark.parametrize("script_path", SEAT_GUIDE_SCRIPTS)
def test_seat_guide_demo_scripts_are_directly_executable(script_path: Path) -> None:
    assert script_path.read_text().startswith("#!/usr/bin/env bash")
    assert os.access(script_path, os.X_OK)


def test_seat_guide_doc_does_not_recommend_rejected_general_go2_stack() -> None:
    doc = SEAT_GUIDE_DOC.read_text()

    assert "bin/demo_seat_guide_hardware_bringup --robot-ip" in doc
    assert "dimos run unitree-go2-seat-guide-agentic --robot-ip" in doc
    assert "dimos --replay run unitree-go2-seat-guide-agentic --daemon" in doc
    assert "dimos run unitree-go2-agentic --robot-ip" not in doc


def test_seat_guide_doc_has_parallel_hardware_day_checklist() -> None:
    doc = SEAT_GUIDE_DOC.read_text()

    assert "### Parallel hardware-day checklist" in doc
    for track in [
        "Voice intake",
        "Perception",
        "Planner",
        "Navigation",
        "Speech feedback",
        "Acceptance evidence",
    ]:
        assert f"| {track} |" in doc
    assert "bin/demo_seat_guide_verify_acceptance_log <log>" in doc


def test_seat_guide_doc_describes_smoke_webinput_gate() -> None:
    doc = SEAT_GUIDE_DOC.read_text()

    smoke_section = doc.split("bin/demo_seat_guide_smoke", maxsplit=1)[0].rsplit(
        "Run the no-motion smoke script", maxsplit=1
    )[1]
    for expected in [
        "web=started",
        "thread=running",
        "seat_route=seat_guide_direct",
        "responses=connected",
        "voice_upload=connected",
        "stt=connected",
        "human_transport=connected",
    ]:
        assert expected in smoke_section


def test_seat_guide_doc_keeps_direct_mcp_out_of_live_bringup_commands() -> None:
    doc = SEAT_GUIDE_DOC.read_text()
    no_motion_section = doc.split("Run the real voice path:", maxsplit=1)[0]

    assert "Run the no-motion readiness path" in no_motion_section
    assert "dimos mcp call handle_seat_request --json-args" not in no_motion_section
    assert "verifier rejects that path" in doc


def test_go2_system_prompt_mentions_seat_guide_flow() -> None:
    assert "handle_seat_request" in SYSTEM_PROMPT
    assert "seat_guide_status" in SYSTEM_PROMPT
    assert "camera_seat_provider_status" in SYSTEM_PROMPT
    assert "web_input_status" in SYSTEM_PROMPT
    assert "speech_status" in SYSTEM_PROMPT
    assert "preview_empty_seat_goal" in SYSTEM_PROMPT
    assert "seat_guide_navigation_status" in SYSTEM_PROMPT
    assert "goal_reached=true" in SYSTEM_PROMPT
    assert "STT pipeline is connected" in SYSTEM_PROMPT
    assert "set_seat_scene" in SYSTEM_PROMPT
    assert "Do not claim live chair/person perception is active" in SYSTEM_PROMPT
