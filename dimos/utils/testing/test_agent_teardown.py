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

from dimos.utils.testing.agent_teardown import teardown_agent_setup


class _Stopper:
    """Stand-in for the coordinator / a transport: records its stop() call."""

    def __init__(self, calls: list[str], name: str, *, raises: bool = False) -> None:
        self._calls = calls
        self._name = name
        self._raises = raises

    def stop(self) -> None:
        self._calls.append(self._name)
        if self._raises:
            raise RuntimeError(f"{self._name} boom")


def _unsub(calls: list[str], name: str, *, raises: bool = False):
    def _call() -> None:
        calls.append(name)
        if raises:
            raise RuntimeError(f"{name} boom")

    return _call


def test_runs_every_step_in_order() -> None:
    calls: list[str] = []
    coordinator = _Stopper(calls, "coordinator")
    transports = [_Stopper(calls, "t0"), _Stopper(calls, "t1")]
    unsubs = [_unsub(calls, "u0"), _unsub(calls, "u1")]

    teardown_agent_setup(coordinator, transports, unsubs)

    assert calls == ["coordinator", "t0", "t1", "u0", "u1"]


def test_continues_when_coordinator_stop_raises() -> None:
    # The whole point of the helper: a failing step must not skip the rest,
    # so transports/unsubs still get cleaned up and no exception propagates.
    calls: list[str] = []
    coordinator = _Stopper(calls, "coordinator", raises=True)
    transports = [_Stopper(calls, "t0")]
    unsubs = [_unsub(calls, "u0")]

    teardown_agent_setup(coordinator, transports, unsubs)

    assert calls == ["coordinator", "t0", "u0"]


def test_continues_when_a_transport_raises() -> None:
    calls: list[str] = []
    transports = [_Stopper(calls, "t0", raises=True), _Stopper(calls, "t1")]
    unsubs = [_unsub(calls, "u0", raises=True), _unsub(calls, "u1")]

    # coordinator=None must be skipped without error; later steps still run.
    teardown_agent_setup(None, transports, unsubs)

    assert calls == ["t0", "t1", "u0", "u1"]


def test_none_coordinator_and_empty_collections() -> None:
    teardown_agent_setup(None, [], [])  # must be a no-op, not raise
