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

from typing import Any

import pytest

from dimos.simulation.dimsim.agent_turn_control import (
    CANCEL_ACTIVE_TURN,
    publish_turn_cancellation,
)


class FakeControlTransport:
    def __init__(self, publish_error: Exception | None = None) -> None:
        self.publish_error = publish_error
        self.started = False
        self.stopped = False
        self.messages: list[Any] = []

    def start(self) -> None:
        self.started = True

    def publish(self, message: Any) -> None:
        self.messages.append(message)
        if self.publish_error is not None:
            raise self.publish_error

    def stop(self) -> None:
        self.stopped = True


def test_publish_turn_cancellation_is_correlated_and_cleans_up() -> None:
    transport = FakeControlTransport()

    publish_turn_cancellation(transport, "run-123")

    assert transport.messages == [
        {
            "type": CANCEL_ACTIVE_TURN,
            "runId": "run-123",
        }
    ]
    assert transport.started
    assert transport.stopped


def test_publish_turn_cancellation_cleans_up_after_publish_failure() -> None:
    transport = FakeControlTransport(RuntimeError("publish failed"))

    with pytest.raises(RuntimeError, match="publish failed"):
        publish_turn_cancellation(transport, "run-123")

    assert transport.stopped


def test_publish_turn_cancellation_rejects_empty_run_id() -> None:
    transport = FakeControlTransport()

    with pytest.raises(ValueError, match="run_id"):
        publish_turn_cancellation(transport, "")

    assert not transport.started
