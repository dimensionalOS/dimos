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


from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.annotation import skill
from dimos.core.module import Module
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


class Adder(Module):
    @skill
    def add(self, x: int, y: int) -> str:
        """adds x and y."""
        return str(x + y)


@pytest.mark.integration
@pytest.mark.parametrize("dask", [False, True])
def test_can_call_tool(dask, agent_setup):
    history = agent_setup(
        blueprints=[Adder.blueprint()],
        messages=[HumanMessage("What is 33333 + 100? Use the tool.")],
        dask=dask,
    )

    assert "33433" in history[-1].content


class UserRegistration(Module):
    def __init__(self):
        super().__init__()
        self._first_call = True
        self._use_upper = False

    @skill
    def register_user(self, name: str) -> str:
        """registers a user by name."""

        # If the agent calls with "paul" or "Paul", always say it's the wrong way
        # to force it to try again.

        if self._first_call:
            self._first_call = False
            self._use_upper = not name[0].isupper()

        if self._use_upper and not name[0].isupper():
            return ValueError("Names must start with an uppercase letter.")
        if not self._use_upper and name[0].isupper():
            return ValueError("The names must only use lowercase letters.")

        global _correct_name_registered
        _correct_name_registered = True
        return "User name registered successfully."


@pytest.mark.integration
@pytest.mark.parametrize("dask", [False, True])
def test_can_call_again_on_error(dask, agent_setup):
    history = agent_setup(
        blueprints=[UserRegistration.blueprint()],
        messages=[
            HumanMessage(
                "Register a user named 'Paul'. If there are errors, just try again until you succeed."
            )
        ],
        dask=dask,
    )

    assert any(message.content == "User name registered successfully." for message in history)


class MultipleTools(Module):
    def __init__(self):
        super().__init__()
        self._people = {"Ben": "office", "Bob": "garage"}

    @skill
    def register_person(self, name: str) -> str:
        """Registers a person by name."""
        if name.lower() == "john":
            self._people[name] = "kitchen"
        elif name.lower() == "jane":
            self._people[name] = "living room"
        return f"'{name}' has been registered."

    @skill
    def locate_person(self, name: str) -> str:
        """Locates a person by name."""
        if name not in self._people:
            known_people = list(self._people.keys())
            return (
                f"Error: '{name}' is not registered. People cannot be located until they've "
                f"been registered in the system. People known so far: {', '.join(known_people)}. "
                "Use register_person to register a person."
            )
        return f"'{name}' is located at '{self._people[name]}'."


class NavigationSkill(Module):
    @skill
    def go_to_location(self, description: str) -> str:
        """Go to a location by a description."""
        if description.strip().lower() not in ["kitchen", "living room"]:
            return f"Error: Unknown location description: '{description}'."
        return f"Going to the {description}."


@pytest.mark.integration
def test_multiple_tool_calls_with_multiple_messages(agent_setup):
    history = agent_setup(
        blueprints=[MultipleTools.blueprint(), NavigationSkill.blueprint()],
        messages=[
            HumanMessage(
                "You are a robot assistant. Move to the location where John is. Don't ask me for feedback, just go there."
            ),
            HumanMessage("Nice job. You did it. Now go to the location where Jane is."),
        ],
    )

    # Collect all go_to_location calls from the history
    go_to_location_calls = []
    for message in history:
        if hasattr(message, "tool_calls"):
            for tool_call in message.tool_calls:
                if tool_call["name"] == "go_to_location":
                    go_to_location_calls.append(tool_call)

    # Find the index of the second HumanMessage to split first/second prompt
    second_human_idx = None
    human_count = 0
    for i, message in enumerate(history):
        if isinstance(message, HumanMessage):
            human_count += 1
            if human_count == 2:
                second_human_idx = i
                break

    # Collect go_to_location calls before and after the second prompt
    calls_after_first_prompt = []
    calls_after_second_prompt = []
    for i, message in enumerate(history):
        if hasattr(message, "tool_calls"):
            for tool_call in message.tool_calls:
                if tool_call["name"] == "go_to_location":
                    if i < second_human_idx:
                        calls_after_first_prompt.append(tool_call)
                    else:
                        calls_after_second_prompt.append(tool_call)

    # After the first prompt, go_to_location should be called with "kitchen"
    assert len(calls_after_first_prompt) == 1
    assert "kitchen" in calls_after_first_prompt[0]["args"]["description"].lower()

    # After the second prompt, go_to_location should be called with "living room"
    assert len(calls_after_second_prompt) == 1
    assert "living room" in calls_after_second_prompt[0]["args"]["description"].lower()

    # There should be exactly two go_to_location calls total
    assert len(go_to_location_calls) == 2


@pytest.mark.integration
def test_prompt(agent_setup):
    history = agent_setup(
        blueprints=[],
        messages=[HumanMessage("What is your name?")],
        system_prompt="You are a helpful assistant named Johnny.",
    )

    assert "Johnny" in history[-1].content


class Visualizer(Module):
    @skill
    def take_a_picture(self) -> Image:
        """Takes a picture."""
        return Image.from_file(get_data("cafe-smol.jpg")).to_rgb()


@pytest.mark.integration
def test_image(agent_setup):
    history = agent_setup(
        blueprints=[Visualizer.blueprint()],
        messages=[
            HumanMessage(
                "What do you see? Take a picture using your camera and describe it. "
                "Please mention one of the words which best match the image: "
                "'stadium', 'cafe', 'battleship'."
            )
        ],
        system_prompt="You are a helpful assistant that can use a camera to take pictures.",
    )

    response = history[-1].content.lower()
    assert "cafe" in response
    assert "stadium" not in response
    assert "battleship" not in response
