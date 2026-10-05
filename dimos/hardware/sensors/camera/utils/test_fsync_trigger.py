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

from __future__ import annotations

import subprocess
from typing import Any

import pytest

from dimos.hardware.sensors.camera.utils import fsync_trigger


@pytest.mark.parametrize(
    "failure",
    [subprocess.CalledProcessError(-11, "trigger"), subprocess.TimeoutExpired("trigger", 10)],
)
def test_a_failed_trigger_leaves_the_cameras_free_running(
    monkeypatch: pytest.MonkeyPatch, failure: Exception
) -> None:
    """Capture must still start when the vendor SDK crashes or hangs."""

    def run(*_: Any, **__: Any) -> None:
        raise failure

    monkeypatch.setattr(fsync_trigger.os.path, "exists", lambda _: True)
    monkeypatch.setattr(fsync_trigger.subprocess, "run", run)

    fsync_trigger.trigger_gmsl_cameras(30, 0x0F)  # does not raise


def test_no_sdk_skips_the_trigger(monkeypatch: pytest.MonkeyPatch) -> None:
    """Off the robot there is no SDK, so nothing is launched."""

    def run(*_: Any, **__: Any) -> None:
        raise AssertionError("launched without an SDK")

    monkeypatch.setattr(fsync_trigger.os.path, "exists", lambda _: False)
    monkeypatch.setattr(fsync_trigger.subprocess, "run", run)

    fsync_trigger.trigger_gmsl_cameras(30, 0x0F)


def test_trigger_runs_in_its_own_process_at_the_given_rate(monkeypatch: pytest.MonkeyPatch) -> None:
    launched: list[list[str]] = []
    monkeypatch.setattr(fsync_trigger.os.path, "exists", lambda _: True)
    monkeypatch.setattr(fsync_trigger.subprocess, "run", lambda argv, **_: launched.append(argv))

    fsync_trigger.trigger_gmsl_cameras(30, 0x0F)

    assert launched == [[fsync_trigger.sys.executable, "-m", fsync_trigger.__name__, "30", "15"]]
