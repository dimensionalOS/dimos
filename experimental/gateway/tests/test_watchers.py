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

import asyncio
from collections.abc import Callable
from pathlib import Path
import time
from typing import Any

from fastapi.testclient import TestClient
import pytest
import watchfiles
import watchfiles.main

from experimental.gateway import events
from experimental.gateway.state import State

# long enough that only a file event or the child exiting can explain a prompt publish
NO_STALE_CHECK_S = 60.0
# a short batch so the tests answer in well under a second
TEST_DEBOUNCE_MS = 50


async def until(check: Callable[[], bool], seconds: float = 5.0) -> None:
    deadline = time.monotonic() + seconds
    while not check():
        assert time.monotonic() < deadline, "timed out"
        await asyncio.sleep(0.02)


@pytest.fixture
def watchers(monkeypatch: pytest.MonkeyPatch) -> list[Any]:
    """The OS file watchers awatch has opened: a change made after one opens is seen."""
    opened: list[Any] = []
    open_watcher = watchfiles.main.RustNotify

    def recorded(*args: Any) -> Any:
        opened.append(open_watcher(*args))
        return opened[-1]

    monkeypatch.setattr(watchfiles.main, "RustNotify", recorded)
    return opened


def test_a_blueprint_file_added_under_dimos_robot_relists(
    tmp_path: Path, watchers: list[Any]
) -> None:
    robot_dir = tmp_path / "dimos" / "robot"
    robot_dir.mkdir(parents=True)
    (robot_dir / "old_robot.py").write_text("blueprint = None\n")
    reported: list[tuple[list[str], list[str]]] = []
    listings: list[list[str]] = []

    async def relist() -> list[str]:
        listings.append(sorted(path.stem for path in robot_dir.glob("*.py")))
        return listings[-1]

    async def scenario() -> None:
        watcher = asyncio.create_task(
            events.watch_blueprints(
                tmp_path,
                [tmp_path / "missing-site-packages"],
                relist,
                lambda added, removed: reported.append((added, removed)),
                debounce_ms=TEST_DEBOUNCE_MS,
            )
        )
        # the first listing is what a change is compared with
        await until(lambda: bool(watchers and listings))
        (robot_dir / "new_robot.py").write_text("blueprint = None\n")
        await until(lambda: bool(reported))
        watcher.cancel()

    asyncio.run(scenario())
    assert reported == [(["new_robot"], [])]


def test_only_sources_and_installs_wake_the_blueprint_watcher() -> None:
    added = watchfiles.Change.added
    assert events.is_blueprint_source(added, "/dimos/robot/go2/blueprints.py")
    assert events.is_blueprint_source(added, "/dimos/robot/new_robot")
    assert not events.is_blueprint_source(added, "/dimos/robot/__pycache__/go2.cpython-312.pyc")
    assert not events.is_blueprint_source(added, "/dimos/robot/notes.txt")
    assert events.is_installed_package(added, "/site-packages/dimos_extra-1.0.dist-info")
    assert not events.is_installed_package(added, "/site-packages/numpy")


def launch_phases(sent: list[tuple[str, Any]]) -> list[str]:
    return [payload["launch"]["phase"] for _, payload in sent if payload.get("launch")]


def watch_a_launch(
    client: TestClient,
    state: State,
    sent: list[tuple[str, Any]],
    watchers: list[Any],
    blueprint: str,
    phase: str,
) -> None:
    async def scenario() -> None:
        watcher = asyncio.create_task(
            events.watch_launch(
                state.bus, stale_check_s=NO_STALE_CHECK_S, debounce_ms=TEST_DEBOUNCE_MS
            )
        )
        await until(lambda: bool(watchers))
        await asyncio.to_thread(client.post, "/active/launch", json={"blueprint": blueprint})
        await until(lambda: phase in launch_phases(sent))
        watcher.cancel()

    asyncio.run(scenario())


def test_a_registered_run_is_published_as_running(
    client: TestClient, state: State, sent: list[tuple[str, Any]], watchers: list[Any]
) -> None:
    watch_a_launch(client, state, sent, watchers, "demo", "running")


def test_a_launch_whose_child_exits_is_published_as_failed(
    client: TestClient, state: State, sent: list[tuple[str, Any]], watchers: list[Any]
) -> None:
    watch_a_launch(client, state, sent, watchers, "fail", "failed")
