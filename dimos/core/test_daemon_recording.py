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

from contextlib import contextmanager

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
from dimos.core.daemon import run_coordinator
from dimos.core.global_config import global_config
from dimos.experimental.memory import rust_cli_recorder
from dimos.memory import tap
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@pytest.fixture
def recording_coordinator(mocker, monkeypatch):
    monkeypatch.setenv(DIMOS_RUN_ID_ENV, "recording-test")
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record_topics", "*")
    monkeypatch.setattr(global_config, "replay", False)
    coordinator = mocker.Mock(spec=ModuleCoordinator)
    coordinator.health_check.return_value = True
    coordinator.transports = {("odom", PoseStamped): mocker.Mock()}
    mocker.patch.object(ModuleCoordinator, "build", return_value=coordinator)
    mocker.patch("dimos.core.coordination.process_lifecycle.spawn_watchdog")
    mocker.patch("dimos.core.daemon.install_signal_handlers")
    mocker.patch.object(rust_cli_recorder, "make_plan")
    return coordinator


def test_rust_recorder_is_ready_before_daemon_readiness(recording_coordinator, mocker, tmp_path):
    events = []
    session = mocker.patch.object(rust_cli_recorder, "RustRecordingSession").return_value
    session.start.side_effect = lambda: events.append("recorder ready")
    session.stop.side_effect = lambda: events.append("recorder stopped")
    recording_coordinator.loop.side_effect = lambda: events.append("loop")

    run_coordinator(
        mocker.Mock(),
        mocker.Mock(),
        run_id="recording-test",
        log_dir=tmp_path,
        on_ready=lambda _: events.append("daemon ready"),
    )

    assert events == ["recorder ready", "daemon ready", "loop", "recorder stopped"]


def test_rust_build_failure_fails_startup_before_readiness(recording_coordinator, mocker, tmp_path):
    prepare = mocker.patch.object(
        rust_cli_recorder,
        "prepare_rust_recorder",
        side_effect=RuntimeError("Rust recorder build failed: Nix is unavailable"),
    )
    on_ready = mocker.Mock()

    with pytest.raises(RuntimeError, match="Rust recorder build failed: Nix is unavailable"):
        run_coordinator(
            mocker.Mock(),
            mocker.Mock(),
            run_id="recording-test",
            log_dir=tmp_path,
            on_ready=on_ready,
        )

    prepare.assert_called_once_with()
    on_ready.assert_not_called()
    recording_coordinator.loop.assert_not_called()
    recording_coordinator.stop.assert_called_once_with()


def test_python_recorder_preserves_readiness_order(
    recording_coordinator, mocker, monkeypatch, tmp_path
):
    monkeypatch.setattr(global_config, "record_engine", "python")
    events = []

    @contextmanager
    def recording(coordinator):
        events.append("recording")
        yield

    mocker.patch.object(tap, "recording", side_effect=recording)
    recording_coordinator.loop.side_effect = lambda: events.append("loop")

    run_coordinator(
        mocker.Mock(),
        mocker.Mock(),
        run_id="recording-test",
        log_dir=tmp_path,
        on_ready=lambda _: events.append("daemon ready"),
    )

    assert events == ["daemon ready", "recording", "loop"]
