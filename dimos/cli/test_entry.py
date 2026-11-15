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

from collections.abc import Callable
from importlib.metadata import distribution
import sys

import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import global_config
import dimos.visualization.rerun.bridge as bridge


@pytest.fixture
def rerun_bridge_entrypoint() -> Callable[[], None]:
    return next(
        entry for entry in distribution("dimos").entry_points if entry.name == "rerun-bridge"
    ).load()


@pytest.mark.parametrize(
    ("args", "exit_code", "message"),
    [
        (["--help"], 0, "--rerunbridgemodule.host"),
        (["--rerun-open", "invalid"], 2, "Error:"),
    ],
)
def test_rerun_bridge_exits_without_starting_bridge(
    rerun_bridge_entrypoint: Callable[[], None],
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    capsys: pytest.CaptureFixture[str],
    args: list[str],
    exit_code: int,
    message: str,
) -> None:
    run_bridge = mocker.patch.object(bridge, "run_bridge")
    monkeypatch.setattr(sys, "argv", ["rerun-bridge", *args])

    with pytest.raises(SystemExit) as exited:
        rerun_bridge_entrypoint()

    assert exited.value.code == exit_code
    captured = capsys.readouterr()
    assert message in captured.out + captured.err
    run_bridge.assert_not_called()


def test_rerun_bridge_forwards_bridge_and_global_options(
    rerun_bridge_entrypoint: Callable[[], None],
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    run_bridge = mocker.patch.object(bridge, "run_bridge")
    monkeypatch.setattr(
        sys,
        "argv",
        ["rerun-bridge", "--rerun-open", "none", "--memory-limit", "4GB", "--transport", "lcm"],
    )

    with pytest.raises(SystemExit) as exited:
        rerun_bridge_entrypoint()

    assert exited.value.code == 0
    run_bridge.assert_called_once_with(rerun_open="none", memory_limit="4GB", rerun_web=True)
    assert global_config.transport == "lcm"
