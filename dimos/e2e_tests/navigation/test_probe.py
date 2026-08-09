# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable
from typing import Any

from dimos.core.stream import Stream, Transport
from dimos.e2e_tests.navigation.probe import StreamProbe


class _FakeTransport(Transport[int]):
    def __init__(self) -> None:
        self.callback: Callable[[int], Any] | None = None
        self.stopped = False

    def broadcast(self, selfstream: Stream[int] | None, value: int) -> None:
        del selfstream
        if self.callback is not None:
            self.callback(value)

    def subscribe(
        self,
        callback: Callable[[int], Any],
        selfstream: Stream[int] | None = None,
    ) -> Callable[[], None]:
        del selfstream
        self.callback = callback

        def unsubscribe() -> None:
            self.callback = None

        return unsubscribe

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.stopped = True


def test_probe_waits_after_marker_and_keeps_sequence() -> None:
    transport = _FakeTransport()
    probe = StreamProbe("value", int, transport=transport)
    probe.start()
    transport.broadcast(None, 1)
    marker = probe.mark()
    transport.broadcast(None, 2)
    transport.broadcast(None, 3)

    observation = probe.wait_for(
        lambda value: value > 2,
        after=marker,
        timeout=1.0,
        failure_message="missing value",
    )
    probe.stop()

    assert observation.value == 3
    assert observation.sequence == 3
    assert transport.callback is None
    assert transport.stopped
