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

from collections.abc import Generator

import pytest

from dimos.constants import LCM_MAX_CHANNEL_NAME_LENGTH
from dimos.protocol.rpc import LCMRPC


@pytest.fixture
def lcmrpc() -> Generator[LCMRPC, None, None]:
    ret = LCMRPC()
    ret.start()
    yield ret
    ret.stop()


def test_short_name(lcmrpc) -> None:
    actual = lcmrpc.topicgen("Hello/say", req_or_res=True)
    assert actual.topic == "/rpc/Hello/say/res"


def test_long_name(lcmrpc) -> None:
    long = "GreatyLongComplexExampleClassNameForTestingStuff/create"
    long_topic = lcmrpc.topicgen(long, req_or_res=True).topic
    assert long_topic == "/rpc/2cudPuFGMJdWxM5KZb/res"

    less_long = long[:-1]
    less_long_topic = lcmrpc.topicgen(less_long, req_or_res=True).topic
    assert less_long_topic == "/rpc/GreatyLongComplexExampleClassNameForTestingStuff/creat/res"

    assert len(less_long_topic) == LCM_MAX_CHANNEL_NAME_LENGTH
