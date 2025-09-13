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


def test_what_is_your_name(create_potato_agent):
    agent = create_potato_agent(fixture="test_what_is_your_name.json")
    response = agent.query("hi there, please tell me what's your name?")
    assert "Mr. Potato" in response


def test_how_much_is_124181112_plus_124124(create_potato_agent):
    agent = create_potato_agent(fixture="test_how_much_is_124181112_plus_124124.json")

    response = agent.query("how much is 124181112 + 124124?")
    assert "124,305,236" in response

    response = agent.query("how much is one billion minus 1 million, in digits please")
    assert "999,000,000" in response


def test_what_do_you_see_in_this_picture(create_potato_agent):
    agent = create_potato_agent(fixture="test_what_do_you_see_in_this_picture.json")

    response = agent.query("take a photo and tell me what do you see")
    assert "group of people sitting at an outdoor café" in response