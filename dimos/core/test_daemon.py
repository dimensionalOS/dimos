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

import os
from pathlib import Path
import re
import signal
import sys
from unittest import mock

import pytest

# ---------------------------------------------------------------------------
# Registry tests
# ---------------------------------------------------------------------------
from dimos.core import run_registry
from dimos.core.run_registry import (
    RunEntry,
    check_port_conflicts,
    cleanup_stale,
    generate_run_id,
    list_runs,
)


@pytest.fixture()
def tmp_registry(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Redirect the registry to a temp dir for test isolation."""
    monkeypatch.setattr("dimos.core.run_registry.REGISTRY_DIR", tmp_path)
    return tmp_path


def _make_entry(
    run_id: str = "20260306-120000-test",
    pid: int | None = None,
    grpc_port: int = 9877,
) -> RunEntry:
    return RunEntry(
        run_id=run_id,
        pid=pid if pid is not None else os.getpid(),
        blueprint="test",
        started_at="2026-03-06T12:00:00Z",
        log_dir="/tmp/test-logs",
        cli_args=["test"],
        config_overrides={},
        grpc_port=grpc_port,
    )


class TestRunEntryCRUD:
    """test_run_entry_save_load_remove — full CRUD cycle."""

    def test_run_entry_save_load_remove(self, tmp_registry: Path):
        entry = _make_entry()
        entry.save()

        loaded = RunEntry.load(entry.registry_path)
        assert loaded.run_id == entry.run_id
        assert loaded.pid == entry.pid
        assert loaded.blueprint == entry.blueprint
        assert loaded.grpc_port == entry.grpc_port

        entry.remove()
        assert not entry.registry_path.exists()

    def test_save_creates_directory(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
        nested = tmp_path / "a" / "b" / "c"
        monkeypatch.setattr("dimos.core.run_registry.REGISTRY_DIR", nested)
        entry = _make_entry()
        entry.save()
        assert entry.registry_path.exists()

    def test_remove_idempotent(self, tmp_registry: Path):
        entry = _make_entry()
        entry.remove()  # file doesn't exist — should not raise
        entry.save()
        entry.remove()
        entry.remove()  # already gone — still fine


class TestGenerateRunId:
    """test_generate_run_id_format — timestamp + sanitized blueprint name."""

    def test_generate_run_id_format(self):
        rid = generate_run_id("unitree-go2")
        # Pattern: YYYYMMDD-HHMMSS-<name>
        assert re.match(r"^\d{8}-\d{6}-unitree-go2$", rid), f"unexpected format: {rid}"

    def test_sanitizes_slashes(self):
        rid = generate_run_id("path/to/bp")
        assert "/" not in rid

    def test_sanitizes_spaces(self):
        rid = generate_run_id("my blueprint")
        assert " " not in rid


class TestCleanupStale:
    """Stale PID cleanup tests."""

    def test_cleanup_stale_removes_dead_pids(self, tmp_registry: Path):
        # PID 2_000_000_000 is almost certainly not alive
        entry = _make_entry(pid=2_000_000_000)
        entry.save()
        assert entry.registry_path.exists()

        removed = cleanup_stale()
        assert removed == 1
        assert not entry.registry_path.exists()

    def test_cleanup_stale_keeps_alive_pids(self, tmp_registry: Path):
        # Our own PID is alive
        entry = _make_entry(pid=os.getpid())
        entry.save()

        removed = cleanup_stale()
        assert removed == 0
        assert entry.registry_path.exists()

    def test_cleanup_corrupt_file(self, tmp_registry: Path):
        bad = tmp_registry / "corrupt.json"
        bad.write_text("not json{{{")
        removed = cleanup_stale()
        assert removed == 1
        assert not bad.exists()


class TestPortConflicts:
    """Port conflict detection."""

    def test_port_conflict_detection(self, tmp_registry: Path):
        entry = _make_entry(pid=os.getpid(), grpc_port=9877)
        entry.save()

        conflict = check_port_conflicts(grpc_port=9877)
        assert conflict is not None
        assert conflict.run_id == entry.run_id

    def test_port_conflict_no_false_positive(self, tmp_registry: Path):
        entry = _make_entry(pid=os.getpid(), grpc_port=8001)
        entry.save()

        conflict = check_port_conflicts(grpc_port=9877)
        assert conflict is None


# ---------------------------------------------------------------------------
# Health check tests
# ---------------------------------------------------------------------------

from dimos.core.module_coordinator import ModuleCoordinator


def _mock_worker(pid: int | None = 1234, worker_id: int = 0):
    """Create a mock Worker with a controllable pid."""
    w = mock.MagicMock()
    w.worker_id = worker_id
    w.pid = pid
    return w


def _mock_coordinator(workers: list | None = None) -> ModuleCoordinator:
    """Create a ModuleCoordinator with mocked internals and controllable workers."""
    coord = mock.MagicMock(spec=ModuleCoordinator)
    # Bind the real health_check method so it runs actual logic
    coord.health_check = ModuleCoordinator.health_check.__get__(coord)
    if workers is not None:
        coord.workers = workers
        coord.n_workers = len(workers)
    else:
        coord.workers = []
        coord.n_workers = 0
    return coord


class TestHealthCheck:
    """health_check verifies all workers are alive after synchronous build."""

    def test_all_healthy(self):
        workers = [_mock_worker(pid=os.getpid(), worker_id=i) for i in range(3)]
        coord = _mock_coordinator(workers)
        assert coord.health_check() is True

    def test_dead_worker(self):
        dead = _mock_worker(pid=None, worker_id=0)
        coord = _mock_coordinator([dead])
        assert coord.health_check() is False

    def test_no_workers(self):
        coord = _mock_coordinator(workers=[])
        assert coord.health_check() is False

    def test_partial_death(self):
        w1 = _mock_worker(pid=os.getpid(), worker_id=0)
        w2 = _mock_worker(pid=os.getpid(), worker_id=1)
        w3 = _mock_worker(pid=None, worker_id=2)
        coord = _mock_coordinator([w1, w2, w3])
        assert coord.health_check() is False


# ---------------------------------------------------------------------------
# Daemon tests
# ---------------------------------------------------------------------------

from dimos.core.daemon import daemonize, install_signal_handlers


class TestDaemonize:
    """test_daemonize_creates_log_dir."""

    def test_daemonize_creates_log_dir(self, tmp_path: Path):
        log_dir = tmp_path / "nested" / "logs"
        assert not log_dir.exists()

        # We can't actually double-fork in tests (child would continue running
        # pytest), so we mock os.fork to return >0 both times (parent path).
        with mock.patch("os.fork", return_value=1), pytest.raises(SystemExit):
            # Parent calls os._exit(0) which we let raise
            with mock.patch("os._exit", side_effect=SystemExit(0)):
                daemonize(log_dir)

        assert log_dir.exists()


class TestSignalHandler:
    """test_signal_handler_cleans_registry."""

    def test_signal_handler_cleans_registry(self, tmp_registry: Path):
        entry = _make_entry()
        entry.save()
        assert entry.registry_path.exists()

        coord = mock.MagicMock()
        with mock.patch("signal.signal") as mock_signal:
            install_signal_handlers(entry, coord)
            # Capture the handler closure registered for SIGTERM
            handler = mock_signal.call_args_list[0][0][1]

        with pytest.raises(SystemExit):
            handler(signal.SIGTERM, None)

        # Registry file should be cleaned up
        assert not entry.registry_path.exists()
        # Coordinator should have been stopped
        coord.stop.assert_called_once()

    def test_signal_handler_tolerates_stop_error(self, tmp_registry: Path):
        entry = _make_entry()
        entry.save()

        coord = mock.MagicMock()
        coord.stop.side_effect = RuntimeError("boom")
        with mock.patch("signal.signal") as mock_signal:
            install_signal_handlers(entry, coord)
            handler = mock_signal.call_args_list[0][0][1]

        with pytest.raises(SystemExit):
            handler(signal.SIGTERM, None)

        # Entry still removed even if stop() throws
        assert not entry.registry_path.exists()


# ---------------------------------------------------------------------------
# dimos status tests
# ---------------------------------------------------------------------------


class TestStatusCommand:
    """Tests for `dimos status` CLI command."""

    def test_status_no_instances(self, tmp_path, monkeypatch):
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path / "runs")
        entries = list_runs(alive_only=True)
        assert entries == []

    def test_status_shows_alive_instance(self, tmp_path, monkeypatch):
        reg_dir = tmp_path / "runs"
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", reg_dir)

        entry = RunEntry(
            run_id="20260306-120000-test",
            pid=os.getpid(),  # our own PID — alive
            blueprint="test",
            started_at="2026-03-06T12:00:00Z",
            log_dir=str(tmp_path / "logs"),
            cli_args=["test"],
            config_overrides={},
        )
        entry.save()

        entries = list_runs(alive_only=True)
        assert len(entries) == 1
        assert entries[0].run_id == "20260306-120000-test"
        assert entries[0].pid == os.getpid()

    def test_status_filters_dead(self, tmp_path, monkeypatch):
        reg_dir = tmp_path / "runs"
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", reg_dir)

        entry = RunEntry(
            run_id="20260306-120000-dead",
            pid=99999999,  # fake PID, not alive
            blueprint="dead",
            started_at="2026-03-06T12:00:00Z",
            log_dir=str(tmp_path / "logs"),
            cli_args=["dead"],
            config_overrides={},
        )
        entry.save()

        entries = list_runs(alive_only=True)
        assert len(entries) == 0


# ---------------------------------------------------------------------------
# dimos stop tests
# ---------------------------------------------------------------------------


class TestStopCommand:
    """Tests for `dimos stop` CLI command."""

    def test_stop_sends_sigterm(self, tmp_path, monkeypatch):
        """Verify stop sends SIGTERM to the correct PID."""
        reg_dir = tmp_path / "runs"
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", reg_dir)

        killed_pids = []
        killed_signals = []

        def mock_kill(pid, sig):
            killed_pids.append(pid)
            killed_signals.append(sig)
            raise ProcessLookupError  # pretend it died immediately

        monkeypatch.setattr(os, "kill", mock_kill)

        entry = RunEntry(
            run_id="20260306-120000-test",
            pid=12345,
            blueprint="test",
            started_at="2026-03-06T12:00:00Z",
            log_dir=str(tmp_path / "logs"),
            cli_args=["test"],
            config_overrides={},
        )
        entry.save()

        # Import the stop helper
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from dimos.core.run_registry import stop_entry

        stop_entry(entry, force=False)

        assert 12345 in killed_pids
        import signal

        assert signal.SIGTERM in killed_signals
        # Registry entry should be removed
        assert not entry.registry_path.exists()

    def test_stop_force_sends_sigkill(self, tmp_path, monkeypatch):
        """Verify --force sends SIGKILL."""
        reg_dir = tmp_path / "runs"
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", reg_dir)

        killed_signals = []

        def mock_kill(pid, sig):
            killed_signals.append(sig)
            raise ProcessLookupError

        monkeypatch.setattr(os, "kill", mock_kill)

        entry = RunEntry(
            run_id="20260306-120000-test",
            pid=12345,
            blueprint="test",
            started_at="2026-03-06T12:00:00Z",
            log_dir=str(tmp_path / "logs"),
            cli_args=["test"],
            config_overrides={},
        )
        entry.save()

        from dimos.core.run_registry import stop_entry

        stop_entry(entry, force=True)

        import signal

        assert signal.SIGKILL in killed_signals
        assert not entry.registry_path.exists()

    def test_stop_cleans_registry_on_already_dead(self, tmp_path, monkeypatch):
        """If process is already dead, just clean registry."""
        reg_dir = tmp_path / "runs"
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", reg_dir)

        def mock_kill(pid, sig):
            raise ProcessLookupError

        monkeypatch.setattr(os, "kill", mock_kill)

        entry = RunEntry(
            run_id="20260306-120000-dead",
            pid=99999999,
            blueprint="dead",
            started_at="2026-03-06T12:00:00Z",
            log_dir=str(tmp_path / "logs"),
            cli_args=["dead"],
            config_overrides={},
        )
        entry.save()
        assert entry.registry_path.exists()

        from dimos.core.run_registry import stop_entry

        stop_entry(entry, force=False)
        assert not entry.registry_path.exists()

    def test_stop_escalates_to_sigkill_after_timeout(self, tmp_path, monkeypatch):
        """If SIGTERM doesn't kill within 5s, escalates to SIGKILL."""
        reg_dir = tmp_path / "runs"
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", reg_dir)

        signals_sent = []

        def mock_kill(pid, sig):
            signals_sent.append(sig)
            # Don't raise — process "survives"

        monkeypatch.setattr(os, "kill", mock_kill)

        # Make is_pid_alive always return True (process won't die)
        monkeypatch.setattr(run_registry, "is_pid_alive", lambda pid: True)

        # Speed up the wait loop
        monkeypatch.setattr("time.sleep", lambda x: None)

        entry = RunEntry(
            run_id="20260306-120000-stubborn",
            pid=12345,
            blueprint="stubborn",
            started_at="2026-03-06T12:00:00Z",
            log_dir=str(tmp_path / "logs"),
            cli_args=["stubborn"],
            config_overrides={},
        )
        entry.save()

        from dimos.core.run_registry import stop_entry

        stop_entry(entry, force=False)

        import signal

        assert signal.SIGTERM in signals_sent
        assert signal.SIGKILL in signals_sent
        assert not entry.registry_path.exists()

    def test_get_most_recent_returns_latest(self, tmp_path, monkeypatch):
        """Verify get_most_recent returns the most recently created entry."""
        reg_dir = tmp_path / "runs"
        monkeypatch.setattr(run_registry, "REGISTRY_DIR", reg_dir)
        monkeypatch.setattr(run_registry, "is_pid_alive", lambda pid: True)

        entry1 = RunEntry(
            run_id="20260306-100000-first",
            pid=os.getpid(),
            blueprint="first",
            started_at="2026-03-06T10:00:00Z",
            log_dir=str(tmp_path / "logs1"),
            cli_args=["first"],
            config_overrides={},
        )
        entry1.save()

        entry2 = RunEntry(
            run_id="20260306-110000-second",
            pid=os.getpid(),
            blueprint="second",
            started_at="2026-03-06T11:00:00Z",
            log_dir=str(tmp_path / "logs2"),
            cli_args=["second"],
            config_overrides={},
        )
        entry2.save()

        from dimos.core.run_registry import get_most_recent

        latest = get_most_recent(alive_only=True)
        assert latest is not None
        assert latest.run_id == "20260306-110000-second"
