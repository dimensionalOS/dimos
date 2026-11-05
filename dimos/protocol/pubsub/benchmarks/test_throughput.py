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

import pytest

from dimos.protocol.pubsub.benchmarks.cases import Case, memory_msggen, memory_pubsub_channel
from dimos.protocol.pubsub.benchmarks.throughput import measure
from dimos.protocol.pubsub.impl.memory import Memory


@pytest.mark.parametrize("drop_messages", [False, True])
def test_measure_counts_delivery_and_loss(drop_messages, mocker):
    if drop_messages:
        mocker.patch.object(Memory, "publish")
    result = measure(
        Case(memory_pubsub_channel, memory_msggen),
        msg_size=64,
        max_messages=5,
        receive_timeout=0,
    )

    assert result.msgs_sent == 5
    assert result.msgs_received == (0 if drop_messages else 5)
    assert result.loss_pct == (100 if drop_messages else 0)
