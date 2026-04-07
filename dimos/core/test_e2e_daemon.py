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

from datetime import datetime, timezone
import json
import os
import signal
import time

import pytest
from typer.testing import CliRunner

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import global_config
from dimos.core.module import Module
import dimos.core.run_registry as _reg
from dimos.core.run_registry import (
    RunEntry,
    cleanup_stale,
    get_most_recent,
    list_runs,
)
from dimos.core.stream import Out
from dimos.robot.cli.dimos import main


class PingModule(Module):
    data: Out[str]

    def start(self):
        super().start()


class PongModule(Module):
    data: Out[str]

    def start(self):
        super().start()


@pytest.fixture(autouse=True)
def _ci_env(monkeypatch):
    """Set CI=1 to skip sysctl interactive prompt — scoped per test, not module."""
    monkeypatch.setenv("CI", "1")


@pytest.fixture(autouse=True)
def _clean_registry(tmp_path, monkeypatch):
    """Redirect registry to a temp dir for test isolation."""
    test_dir = tmp_path / "runs"
    test_dir.mkdir()
    monkeypatch.setattr(_reg, "REGISTRY_DIR", test_dir)
    yield test_dir


@pytest.fixture()
def coordinator():
    """Build a PingPong blueprint (1 worker) and yield the coordinator."""
    global_config.update(viewer="none", n_workers=1)
    bp = autoconnect(PingModule.blueprint(), PongModule.blueprint())
    coord = ModuleCoordinator.build(bp)
    yield coord
    coord.stop()


@pytest.fixture()
def coordinator_2w():
    """Build a PingPong blueprint with 2 workers."""
    global_config.update(viewer="none", n_workers=2)
    bp = autoconnect(PingModule.blueprint(), PongModule.blueprint())
    coord = ModuleCoordinator.build(bp)
    yield coord
    coord.stop()


@pytest.fixture()
def registry_entry():
    """Create and save a registry entry. Removes on teardown."""
    run_id = f"test-{datetime.now(timezone.utc).strftime('%H%M%S%f')}"
    entry = RunEntry(
        run_id=run_id,
        pid=os.getpid(),
        blueprint="ping-pong-test",
        started_at=datetime.now(timezone.utc).isoformat(),
        log_dir="/tmp/dimos-e2e-test",
        cli_args=["ping-pong"],
        config_overrides={"n_workers": 1},
    )
    entry.save()
    yield entry
    entry.remove()


@pytest.mark.slow
class TestDaemonE2E:
    """End-to-end daemon lifecycle with real workers."""

    def test_single_worker_lifecycle(self, coordinator, registry_entry):
        """Build -> health check -> registry -> status (1 worker)."""
        assert coordinator.n_modules == 2

        assert coordinator.health_check(), "Health check should pass"

        runs = list_runs(alive_only=True)
        assert len(runs) == 1
        assert runs[0].run_id == registry_entry.run_id

        latest = get_most_recent(alive_only=True)
        assert latest is not None
        assert latest.run_id == registry_entry.run_id

    def test_multiple_workers(self, coordinator_2w):
        """Build with 2 workers — both should be alive."""
        assert coordinator_2w.health_check(), "Health check should pass"

    def test_health_check_detects_dead_worker(self, coordinator):
        """Kill a worker process — health check should fail."""
        py_mgr = next(
            m for m in coordinator._managers.values() if isinstance(m, WorkerManagerPython)
        )
        worker = py_mgr.workers[0]
        worker_pid = worker.pid
        assert worker_pid is not None

        os.kill(worker_pid, signal.SIGKILL)
        for _ in range(50):
            try:
                os.kill(worker_pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)

        assert not coordinator.health_check(), "Health check should FAIL"

    def test_registry_entry_details(self, coordinator):
        """Verify all fields are correctly persisted in the JSON registry."""
        run_id = "detail-test-001"
        entry = RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint="ping-pong-detail",
            started_at="2026-03-06T12:00:00+00:00",
            log_dir="/tmp/dimos-detail-test",
            cli_args=["--replay", "ping-pong"],
            config_overrides={"n_workers": 1, "viewer": "none"},
        )
        entry.save()

        raw = json.loads(entry.registry_path.read_text())
        assert raw["run_id"] == run_id
        assert raw["pid"] == os.getpid()
        assert raw["blueprint"] == "ping-pong-detail"
        assert raw["started_at"] == "2026-03-06T12:00:00+00:00"
        assert raw["log_dir"] == "/tmp/dimos-detail-test"
        assert raw["cli_args"] == ["--replay", "ping-pong"]
        assert raw["config_overrides"] == {"n_workers": 1, "viewer": "none"}

        runs = list_runs()
        assert len(runs) == 1
        loaded = runs[0]
        assert loaded.run_id == run_id
        assert loaded.cli_args == ["--replay", "ping-pong"]

        entry.remove()

    def test_stale_cleanup(self, coordinator, registry_entry):
        """Stale entries (dead PIDs) should be removed by cleanup_stale."""
        stale = RunEntry(
            run_id="stale-dead-pid",
            pid=99999999,
            blueprint="ghost",
            started_at=datetime.now(timezone.utc).isoformat(),
            log_dir="/tmp/ghost",
            cli_args=[],
            config_overrides={},
        )
        stale.save()

        assert len(list_runs(alive_only=False)) == 2

        removed = cleanup_stale()
        assert removed == 1

        remaining = list_runs(alive_only=False)
        assert len(remaining) == 1
        assert remaining[0].run_id == registry_entry.run_id


@pytest.fixture()
def live_blueprint():
    """Build PingPong and register. Yields (coord, entry). Cleans up on teardown."""
    global_config.update(viewer="none", n_workers=1)
    bp = autoconnect(PingModule.blueprint(), PongModule.blueprint())
    coord = ModuleCoordinator.build(bp)
    run_id = f"e2e-cli-{datetime.now(timezone.utc).strftime('%H%M%S%f')}"
    entry = RunEntry(
        run_id=run_id,
        pid=os.getpid(),
        blueprint="ping-pong",
        started_at=datetime.now(timezone.utc).isoformat(),
        log_dir="/tmp/dimos-e2e-cli",
        cli_args=["ping-pong"],
        config_overrides={"n_workers": 1},
    )
    entry.save()
    yield coord, entry
    coord.stop()
    entry.remove()


@pytest.mark.slow
class TestCLIWithRealBlueprint:
    """Exercise dimos status and dimos stop against a live DimOS blueprint."""

    def test_status_shows_live_blueprint(self, live_blueprint):
        _coord, entry = live_blueprint
        result = CliRunner().invoke(main, ["status"])

        assert result.exit_code == 0
        assert entry.run_id in result.output
        assert "ping-pong" in result.output
        assert str(os.getpid()) in result.output

    def test_status_shows_live_entry_via_registry(self, live_blueprint):
        _coord, entry = live_blueprint

        runs = list_runs(alive_only=True)
        matching = [r for r in runs if r.run_id == entry.run_id]
        assert len(matching) == 1

    def test_stop_kills_real_workers(self, live_blueprint):
        coord, _entry = live_blueprint
        py_mgr = next(m for m in coord._managers.values() if isinstance(m, WorkerManagerPython))
        worker_pids = [w.pid for w in py_mgr.workers if w.pid]
        assert len(worker_pids) >= 1

        coord.stop()

        for wpid in worker_pids:
            for _ in range(50):
                try:
                    os.kill(wpid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            else:
                pytest.fail(f"Worker PID {wpid} still alive after coord.stop()")
