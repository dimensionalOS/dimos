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

from collections.abc import Callable, Iterator

import pytest

from dimos.core.transport import pLCMTransport
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.msgs.std_msgs.Bool import Bool


def _pose(x: float, y: float, theta: float) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, 0),
        orientation=Quaternion.from_euler(make_vector3(0, 0, theta)),
        frame_id="map",
    )


@pytest.fixture
def lcm_spy() -> Iterator[LcmSpy]:
    lcm_spy = LcmSpy()
    lcm_spy.start()
    yield lcm_spy
    lcm_spy.stop()


@pytest.fixture
def follow_points(lcm_spy: LcmSpy):
    def fun(*, points: list[tuple[float, float, float]], fail_message: str) -> None:
        topic = "/goal_reached#std_msgs.Bool"
        lcm_spy.save_topic(topic)

        for x, y, theta in points:
            lcm_spy.publish("/goal_request#geometry_msgs.PoseStamped", _pose(x, y, theta))
            lcm_spy.wait_for_message_result(
                topic,
                Bool,
                predicate=lambda v: bool(v),
                fail_message=fail_message,
                timeout=60.0,
            )

    yield fun


@pytest.fixture
def start_blueprint() -> Iterator[Callable[[str], DimosCliCall]]:
    dimos_robot_call = DimosCliCall()

    def set_name_and_start(demo_name: str) -> DimosCliCall:
        dimos_robot_call.demo_name = demo_name
        dimos_robot_call.start()
        return dimos_robot_call

    yield set_name_and_start

    dimos_robot_call.stop()


@pytest.fixture
def human_input():
    transport = pLCMTransport("/human_input")
    transport.lcm.start()

    def send_human_input(message: str) -> None:
        transport.publish(message)

    yield send_human_input

    transport.lcm.stop()
