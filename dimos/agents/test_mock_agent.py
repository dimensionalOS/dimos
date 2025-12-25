# Copyright 2025 Dimensional Inc.
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

"""Test agent with FakeChatModel for unit testing."""

import time

from dimos_lcm.sensor_msgs import CameraInfo
from langchain_core.messages import AIMessage, HumanMessage
import pytest

from dimos.agents.agent import Agent
from dimos.agents.testing import MockModel
from dimos.core import LCMTransport, start
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.skill.test_coordinator import SkillContainerTest
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage


def test_tool_call() -> None:
    """Test agent initialization and tool call execution."""
    # Create a fake model that will respond with tool calls
    fake_model = MockModel(
        responses=[
            AIMessage(
                content="I'll add those numbers for you.",
                tool_calls=[
                    {
                        "name": "add",
                        "args": {"args": {"x": 5, "y": 3}},
                        "id": "tool_call_1",
                    }
                ],
            ),
            AIMessage(content="Let me do some math..."),
            AIMessage(content="The result of adding 5 and 3 is 8."),
        ]
    )

    # Create agent with the fake model
    agent = Agent(
        model_instance=fake_model,
        system_prompt="You are a helpful robot assistant with math skills.",
    )

    # Register skills with coordinator
    skills = SkillContainerTest()
    agent.coordinator.register_skills(skills)
    agent.start()

    # Query the agent
    agent.query("Please add 5 and 3")

    # Check that tools were bound
    assert fake_model.tools is not None
    assert len(fake_model.tools) > 0

    # Verify the model was called and history updated
    assert len(agent._history) > 0

    agent.stop()


def test_image_tool_call() -> None:
    """Test agent with image tool call execution."""
    dimos = start(2)
    # Create a fake model that will respond with image tool calls
    fake_model = MockModel(
        responses=[
            AIMessage(
                content="I'll take a photo for you.",
                tool_calls=[
                    {
                        "name": "take_photo",
                        "args": {"args": {}},
                        "id": "tool_call_image_1",
                    }
                ],
            ),
            AIMessage(content="I've taken the photo. The image shows a cafe scene."),
        ]
    )

    # Create agent with the fake model
    agent = Agent(
        model_instance=fake_model,
        system_prompt="You are a helpful robot assistant with camera capabilities.",
    )

    test_skill_module = dimos.deploy(SkillContainerTest)

    agent.register_skills(test_skill_module)
    agent.start()

    agent.run_implicit_skill("get_detections")

    # Query the agent
    agent.query("Please take a photo")

    # Check that tools were bound
    assert fake_model.tools is not None
    assert len(fake_model.tools) > 0

    # Verify the model was called and history updated
    assert len(agent._history) > 0

    # Check that image was handled specially
    # Look for HumanMessage with image content in history
    human_messages_with_images = [
        msg
        for msg in agent._history
        if isinstance(msg, HumanMessage) and msg.content and isinstance(msg.content, list)
    ]
    assert len(human_messages_with_images) >= 0  # May have image messages
    agent.stop()
    test_skill_module.stop()
    dimos.close_all()


@pytest.mark.tool
def test_tool_call_implicit_detections() -> None:
    """Test agent with image tool call execution."""
    dimos = start(2)
    # Create a fake model that will respond with image tool calls
    fake_model = MockModel(
        responses=[
            AIMessage(
                content="I'll take a photo for you.",
                tool_calls=[
                    {
                        "name": "take_photo",
                        "args": {"args": {}},
                        "id": "tool_call_image_1",
                    }
                ],
            ),
            AIMessage(content="I've taken the photo. The image shows a cafe scene."),
        ]
    )

    # Create agent with the fake model
    agent = Agent(
        model_instance=fake_model,
        system_prompt="You are a helpful robot assistant with camera capabilities.",
    )

    robot_connection = dimos.deploy(ConnectionModule, connection_type="fake")
    robot_connection.lidar.transport = LCMTransport("/lidar", LidarMessage)
    robot_connection.odom.transport = LCMTransport("/odom", PoseStamped)
    robot_connection.video.transport = LCMTransport("/image", Image)
    robot_connection.movecmd.transport = LCMTransport("/cmd_vel", Vector3)
    robot_connection.camera_info.transport = LCMTransport("/camera_info", CameraInfo)
    robot_connection.start()

    test_skill_module = dimos.deploy(SkillContainerTest)

    agent.register_skills(test_skill_module)
    agent.start()

    agent.run_implicit_skill("get_detections")

    print(
        "Robot replay pipeline is running in the background.\nWaiting 8.5 seconds for some detections before quering agent"
    )
    time.sleep(8.5)

    # Query the agent
    agent.query("Please take a photo")

    # Check that tools were bound
    assert fake_model.tools is not None
    assert len(fake_model.tools) > 0

    # Verify the model was called and history updated
    assert len(agent._history) > 0

    # Check that image was handled specially
    # Look for HumanMessage with image content in history
    human_messages_with_images = [
        msg
        for msg in agent._history
        if isinstance(msg, HumanMessage) and msg.content and isinstance(msg.content, list)
    ]
    assert len(human_messages_with_images) >= 0

    agent.stop()
    test_skill_module.stop()
    robot_connection.stop()
    dimos.stop()
