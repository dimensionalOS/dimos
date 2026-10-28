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

from dataclasses import replace
import threading
import time
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module
from dimos.hosted import deploy
from dimos.hosted.protocol import DeploymentStatus, HostDescriptor


class Left(Module):
    pass


class Right(Module):
    pass


def _host(host_id: str) -> HostDescriptor:
    return HostDescriptor(host_id, host_id, {}, "r", ())


def test_deployed_holds_the_run_token_and_starts_and_stops_hosts_side_by_side(
    mocker: MockerFixture,
) -> None:
    mocker.patch("dimos.hosted.deploy.code_revision", return_value="r")
    blueprint = autoconnect(Left.blueprint().hosted(host="a"), Right.blueprint().hosted(host="b"))
    rpc = mocker.MagicMock()
    # Both start calls, then both stop calls, must be in flight at once, or a barrier breaks.
    both_started = threading.Barrier(2, timeout=5)
    both_stopped = threading.Barrier(2, timeout=5)
    calls: list[str] = []

    def call_sync(name: str, arguments: Any, rpc_timeout: float) -> tuple[Any, Any]:
        calls.append(name)
        if name.endswith("/start"):
            both_started.wait()
            return DeploymentStatus("running", "run-1", 1, "/logs", None), mocker.Mock()
        both_stopped.wait()
        return DeploymentStatus("absent", None, None, None, None), mocker.Mock()

    rpc.call_sync.side_effect = call_sync

    with deploy.deployed(
        blueprint,
        rpc,
        run_id="run-1",
        local_host_id=None,
        descriptors=(_host("a"), _host("b")),
    ) as placement:
        assert placement == {"left": "a", "right": "b"}
        rpc.session.liveliness.return_value.declare_token.assert_called_once_with(
            "dimos/runs/run-1/live"
        )
        token = rpc.session.liveliness.return_value.declare_token.return_value
        token.undeclare.assert_not_called()

    assert sorted(calls) == ["hosts/a/start", "hosts/a/stop", "hosts/b/start", "hosts/b/stop"]
    # Nothing to fetch before a start: no describe round trip.
    assert not any(name.endswith("/describe") for name in calls)
    assert not both_stopped.broken
    token.undeclare.assert_called_once_with()


def _deploy(rpc: Any, blueprint: Any, descriptors: tuple[HostDescriptor, ...]) -> Any:
    return deploy.deployed(
        blueprint, rpc, run_id="run-1", local_host_id=None, descriptors=descriptors
    )


def test_a_start_is_given_the_hosts_own_limit(mocker: MockerFixture) -> None:
    mocker.patch("dimos.hosted.deploy.code_revision", return_value="r")
    rpc = mocker.MagicMock()
    timeouts: dict[str, float] = {}

    def call_sync(name: str, arguments: Any, rpc_timeout: float) -> tuple[Any, Any]:
        timeouts[name] = rpc_timeout
        return DeploymentStatus("running", "run-1", 1, "/logs", None), mocker.Mock()

    rpc.call_sync.side_effect = call_sync
    slow = replace(_host("a"), start_timeout=600.0)

    with _deploy(rpc, Left.blueprint().hosted(host="a"), (slow,)):
        pass

    assert timeouts["hosts/a/start"] == 600.0


def test_a_start_answered_with_starting_is_followed_until_it_is_running(
    mocker: MockerFixture,
) -> None:
    mocker.patch("dimos.hosted.deploy.code_revision", return_value="r")
    rpc = mocker.MagicMock()
    states = iter(["starting", "running"])
    calls: list[str] = []

    def call_sync(name: str, arguments: Any, rpc_timeout: float) -> tuple[Any, Any]:
        calls.append(name)
        state = "starting" if name.endswith("/start") else next(states)
        return DeploymentStatus(state, "run-1", 1, "/logs", None), mocker.Mock()  # type: ignore[arg-type]

    rpc.call_sync.side_effect = call_sync

    with deploy.deployed(
        Left.blueprint().hosted(host="a"),
        rpc,
        run_id="run-1",
        local_host_id=None,
        descriptors=(_host("a"),),
        start_poll=0.0,
    ) as placement:
        assert placement == {"left": "a"}

    assert calls == ["hosts/a/start", "hosts/a/status", "hosts/a/status", "hosts/a/stop"]


def test_nothing_to_place_is_said_so(mocker: MockerFixture) -> None:
    mocker.patch("dimos.hosted.deploy.code_revision", return_value="r")
    blueprint = Left.blueprint().hosted(host="a").disabled_modules(Left)

    with (
        pytest.raises(ValueError, match="Nothing to place"),
        _deploy(mocker.MagicMock(), blueprint, (_host("a"),)),
    ):
        pass


def test_a_failed_start_stops_the_run_without_waiting_for_the_other_starts(
    mocker: MockerFixture,
) -> None:
    mocker.patch("dimos.hosted.deploy.code_revision", return_value="r")
    blueprint = autoconnect(Left.blueprint().hosted(host="a"), Right.blueprint().hosted(host="b"))
    rpc = mocker.MagicMock()
    b_started, release_b = threading.Event(), threading.Event()
    stopped: list[str] = []

    def call_sync(name: str, arguments: Any, rpc_timeout: float) -> tuple[Any, Any]:
        if name == "hosts/a/start":
            assert b_started.wait(5.0)
            raise RuntimeError("no such module")
        if name == "hosts/b/start":
            # Stays in flight until the test lets go, well past the failure on a.
            b_started.set()
            release_b.wait(10.0)
            return DeploymentStatus("running", "run-1", 1, "/logs", None), mocker.Mock()
        stopped.append(name)
        return DeploymentStatus("absent", None, None, None, None), mocker.Mock()

    rpc.call_sync.side_effect = call_sync

    started_at = time.monotonic()
    with (
        pytest.raises(RuntimeError, match="no such module"),
        _deploy(rpc, blueprint, (_host("a"), _host("b"))),
    ):
        pass
    elapsed = time.monotonic() - started_at
    release_b.set()

    assert elapsed < 5.0
    assert sorted(stopped) == ["hosts/a/stop", "hosts/b/stop"]
