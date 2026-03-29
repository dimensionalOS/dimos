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

"""
Agentic integration test for the ``unitree_g1_agentic_sim`` blueprint.

Builds the **exact same modules** as the production agentic blueprint —
ROSNav, NavigationSkillContainer, spatial memory, object tracking,
perceive loop, person follow, speak, web input — with only two changes:

  1. Agent is replaced by a ``FilteredAgent`` that skips DockerModuleProxy
     proxies in ``on_system_modules`` (they can't survive pickle across
     the forkserver boundary) and uses a ``MockModel`` fixture for
     deterministic, offline-capable LLM responses.
  2. An ``AgentTestRunner`` and ``OdomRecorder`` are added for test
     orchestration and assertions.

This validates the full production blueprint builds, starts, wires all
RPC methods / skills / streams correctly, and can execute a natural-
language navigation command end-to-end through the agent → skill →
nav stack → Unity sim pipeline.

Requires:
    - Docker with BuildKit
    - NVIDIA GPU with drivers
    - X11 display (real or virtual)

Run:
    pytest dimos/navigation/rosnav/test_rosnav_agentic.py -m slow -s
"""

import json
import math
import os
from pathlib import Path
import threading
from typing import Any

from langchain_core.messages import HumanMessage
from langchain_core.messages.base import BaseMessage
import pytest
from reactivex.disposable import Disposable

from dimos.agents.agent import Agent
from dimos.agents.agent_test_runner import AgentTestRunner
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.docker_module import DockerModuleProxy
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In
from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.perception.object_tracker import ObjectTracking
from dimos.perception.perceive_loop_skill import PerceiveLoopSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.g1.blueprints.perceptive.unitree_g1_rosnav_sim import (
    unitree_g1_rosnav_sim,
)
from dimos.robot.unitree.go2.connection import _camera_info_static

FIXTURE_DIR = Path(__file__).parent / "fixtures"

# Timeouts
ODOM_WAIT_SEC = 60  # Docker + Unity startup can be slow
NAV_TIMEOUT_SEC = 180  # Agent → skill → ROS nav → arrival


class FilteredAgent(Agent):
    """Agent that filters DockerModuleProxy proxies from on_system_modules.

    DockerModuleProxy proxies hold host-process LCMRPC connections that don't
    survive pickle serialization across the forkserver worker boundary.
    Worker-side modules (NavigationSkillContainer, etc.) discover their
    own skills and connect to Docker RPCs via ``rpc_calls`` — so filtering
    Docker proxies out of the agent's module list is safe.
    """

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        worker_modules = [m for m in modules if not isinstance(m, DockerModuleProxy)]
        super().on_system_modules(worker_modules)


class OdomRecorder(Module):
    """Lightweight odom recorder for test assertions."""

    odom: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._poses: list[PoseStamped] = []
        self._first_odom = threading.Event()
        self._moved_event = threading.Event()
        self._start_pose: PoseStamped | None = None

    @rpc
    def start(self) -> None:
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._poses.append(msg)
            if len(self._poses) == 1:
                self._first_odom.set()
            if self._start_pose is not None and not self._moved_event.is_set():
                dx = msg.position.x - self._start_pose.position.x
                dy = msg.position.y - self._start_pose.position.y
                if math.sqrt(dx * dx + dy * dy) > 0.3:
                    self._moved_event.set()

    @rpc
    def wait_for_odom(self, timeout: float = 60.0) -> bool:
        return self._first_odom.wait(timeout)

    @rpc
    def wait_for_movement(self, timeout: float = 120.0) -> bool:
        return self._moved_event.wait(timeout)

    @rpc
    def mark_start(self) -> None:
        with self._lock:
            if self._poses:
                self._start_pose = self._poses[-1]

    @rpc
    def get_start_pose(self) -> PoseStamped | None:
        with self._lock:
            return self._start_pose

    @rpc
    def get_latest_pose(self) -> PoseStamped | None:
        with self._lock:
            return self._poses[-1] if self._poses else None

    @rpc
    def get_odom_count(self) -> int:
        with self._lock:
            return len(self._poses)

    @rpc
    def stop(self) -> None:
        pass


def _distance_2d(a: PoseStamped, b: PoseStamped) -> float:
    return math.sqrt((a.position.x - b.position.x) ** 2 + (a.position.y - b.position.y) ** 2)


def _ensure_fixture(name: str, responses: list[dict]) -> Path:
    """Create a MockModel fixture file if it doesn't exist."""
    fixture_path = FIXTURE_DIR / name
    fixture_path.parent.mkdir(parents=True, exist_ok=True)
    if not fixture_path.exists():
        fixture_path.write_text(json.dumps({"responses": responses}, indent=2) + "\n")
    return fixture_path


def _build_agentic_sim_test(
    fixture_path: Path,
    messages: list[BaseMessage],
    system_prompt: str | None = None,
) -> tuple:
    """Build the test blueprint and return (coordinator, recorder, history, finished_event)."""

    agent_kwargs: dict[str, Any] = {}
    if system_prompt:
        agent_kwargs["system_prompt"] = system_prompt
    if bool(os.getenv("RECORD")) or fixture_path.exists():
        agent_kwargs["model_fixture"] = str(fixture_path)

    # Tap agent messages for assertions
    history: list[BaseMessage] = []
    finished_event = threading.Event()
    agent_transport = pLCMTransport("/agent")
    finished_transport = pLCMTransport("/finished")
    agent_transport.subscribe(lambda msg: history.append(msg))
    finished_transport.subscribe(lambda _: finished_event.set())

    # Build the EXACT same modules as unitree_g1_agentic_sim, but with:
    #   - FilteredAgent instead of Agent (handles DockerModuleProxy pickle issue)
    #   - model_fixture for deterministic testing
    #   - AgentTestRunner for driving messages
    #   - OdomRecorder for position assertions
    blueprint = autoconnect(
        # From unitree_g1_rosnav_sim
        unitree_g1_rosnav_sim,
        # From unitree_g1_agentic_sim (all production modules)
        NavigationSkillContainer.blueprint(),  # NavigationSkillContainer
        PersonFollowSkillContainer.blueprint(
            camera_info=_camera_info_static()
        ),  # PersonFollowSkill
        SpatialMemory.blueprint(),  # SpatialMemory
        ObjectTracking.blueprint(frame_id="camera_link"),  # ObjectTracking
        PerceiveLoopSkill.blueprint(),  # PerceiveLoopSkill
        WebInput.blueprint(),  # WebHumanInput
        SpeakSkill.blueprint(),  # SpeakSkill
        # Test overrides
        FilteredAgent.blueprint(**agent_kwargs),  # Replaces agent()
        AgentTestRunner.blueprint(messages=messages),  # Test driver
        OdomRecorder.blueprint(),  # Position tracking
    ).global_config(viewer="none", n_workers=8)

    coordinator = blueprint.build()
    return (
        coordinator,
        coordinator.get_instance(OdomRecorder),
        history,
        finished_event,
        agent_transport,
        finished_transport,
    )


@pytest.mark.slow
def test_agentic_sim_navigate_to_coordinates():
    """Full unitree_g1_agentic_sim stack: agent triggers exploration.

    The MockModel fixture instructs the agent to call ``begin_exploration``
    which triggers the WavefrontFrontierExplorer to autonomously drive the
    robot to explore unmapped areas. The test verifies the robot moves.

    This validates the full end-to-end pipeline:
      Agent → skill call → NavigationSkillContainer → NavigationInterface →
      ROSNav (Docker) → ROS2 nav stack → Unity sim → odom update
    """

    fixture = _ensure_fixture(
        "test_agentic_sim_navigate.json",
        [
            {
                "content": "",
                "tool_calls": [
                    {
                        "name": "begin_exploration",
                        "args": {},
                        "id": "call_explore_001",
                        "type": "tool_call",
                    }
                ],
            },
            {
                "content": "I've started autonomous exploration. The robot is now moving around to map the environment.",
                "tool_calls": [],
            },
        ],
    )

    coordinator, recorder, history, finished_event, agent_tp, finished_tp = _build_agentic_sim_test(
        fixture,
        messages=[HumanMessage("Start exploring the environment.")],
        system_prompt=(
            "You are a robot assistant. Use begin_exploration to make the "
            "robot explore autonomously. Execute commands immediately."
        ),
    )

    try:
        # Wait for sim
        assert recorder.wait_for_odom(ODOM_WAIT_SEC), "No odom — Unity sim not running"
        recorder.mark_start()
        start = recorder.get_start_pose()
        assert start is not None
        print(f"\n  Start: ({start.position.x:.2f}, {start.position.y:.2f})")

        # Wait for agent to finish
        agent_done = finished_event.wait(NAV_TIMEOUT_SEC)

        # Check tool calls
        tool_calls = [tc for msg in history if hasattr(msg, "tool_calls") for tc in msg.tool_calls]
        print(f"  Tool calls: {[tc['name'] for tc in tool_calls]}")

        if agent_done:
            print("  Agent finished processing")
        else:
            print(f"  ⚠️  Agent still processing after {NAV_TIMEOUT_SEC}s")

        # Wait for movement — exploration may take a few seconds to start
        recorder.wait_for_movement(60)
        end = recorder.get_latest_pose()
        assert end is not None

        displacement = _distance_2d(start, end)
        print(f"  End: ({end.position.x:.2f}, {end.position.y:.2f})")
        print(f"  Displacement: {displacement:.2f}m")
        print(f"  Odom messages: {recorder.get_odom_count()}")

        # Check agent response
        texts = [
            m
            for m in history
            if hasattr(m, "content") and m.content and not getattr(m, "tool_calls", None)
        ]
        if texts:
            print(f"  Agent: {texts[-1].content[:120]}")

        # Assertions
        explore_calls = [tc for tc in tool_calls if tc["name"] == "begin_exploration"]
        assert len(explore_calls) >= 1, (
            f"Agent didn't call begin_exploration. Tools: {[tc['name'] for tc in tool_calls]}"
        )
        assert displacement > 0.3, f"Robot only moved {displacement:.2f}m during exploration"
        print("  ✅ PASSED: agentic exploration command")

    finally:
        agent_tp.stop()
        finished_tp.stop()
        coordinator.stop()


@pytest.mark.slow
def test_agentic_sim_stop_navigation():
    """Agent issues stop command — verifies stop_navigation skill works."""

    fixture = _ensure_fixture(
        "test_agentic_sim_stop.json",
        [
            {
                "content": "",
                "tool_calls": [
                    {
                        "name": "stop_navigation",
                        "args": {},
                        "id": "call_stop_001",
                        "type": "tool_call",
                    }
                ],
            },
            {
                "content": "I've stopped the robot.",
                "tool_calls": [],
            },
        ],
    )

    coordinator, recorder, history, finished_event, agent_tp, finished_tp = _build_agentic_sim_test(
        fixture,
        messages=[HumanMessage("Stop moving right now.")],
        system_prompt=(
            "You are a robot assistant. You can stop the robot with stop_navigation(). "
            "Execute commands immediately."
        ),
    )

    try:
        assert recorder.wait_for_odom(ODOM_WAIT_SEC), "No odom — Unity sim not running"
        print(f"\n  Odom flowing ({recorder.get_odom_count()} messages)")

        # The agent should call stop_navigation and finish quickly
        agent_done = finished_event.wait(60)

        tool_calls = [tc for msg in history if hasattr(msg, "tool_calls") for tc in msg.tool_calls]
        print(f"  Tool calls: {[tc['name'] for tc in tool_calls]}")

        texts = [
            m
            for m in history
            if hasattr(m, "content") and m.content and not getattr(m, "tool_calls", None)
        ]
        if texts:
            print(f"  Agent: {texts[-1].content[:120]}")

        assert agent_done, "Agent did not finish processing stop command"
        stop_calls = [tc for tc in tool_calls if tc["name"] == "stop_navigation"]
        assert len(stop_calls) >= 1, (
            f"Agent didn't call stop_navigation. Tools: {[tc['name'] for tc in tool_calls]}"
        )
        print("  ✅ PASSED: agentic stop navigation")

    finally:
        agent_tp.stop()
        finished_tp.stop()
        coordinator.stop()
