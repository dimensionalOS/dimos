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

from datetime import datetime, timedelta, timezone
import subprocess
import time
from typing import TYPE_CHECKING

import pytest
from typer.testing import CliRunner

from dimos.core import run_registry
from dimos.core.run_registry import RunEntry
from dimos.robot.cli.dimos import main

if TYPE_CHECKING:
    from pathlib import Path


@pytest.fixture(autouse=True)
def _tmp_registry(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Redirect registry to a temp dir for test isolation."""
    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path)
    yield tmp_path


@pytest.fixture()
def sleeper():
    """Start a sleep subprocess, kill it on teardown."""
    procs: list[subprocess.Popen] = []

    def _make():
        p = subprocess.Popen(["sleep", "300"])
        procs.append(p)
        return p

    yield _make
    for p in procs:
        try:
            p.kill()
        except OSError:
            pass
        try:
            p.wait(timeout=2)
        except Exception:
            pass


def _entry(run_id: str, pid: int, blueprint: str = "test", **kwargs) -> RunEntry:
    defaults = dict(
        started_at=datetime.now(timezone.utc).isoformat(),
        log_dir="/tmp/dimos-test",
        cli_args=[blueprint],
        config_overrides={},
    )
    defaults.update(kwargs)
    e = RunEntry(run_id=run_id, pid=pid, blueprint=blueprint, **defaults)
    e.save()
    return e


# ---------------------------------------------------------------------------
# STATUS
# ---------------------------------------------------------------------------


class TestStatusCLI:
    """Tests for `dimos status` command."""

    def test_status_no_instances(self):
        result = CliRunner().invoke(main, ["status"])
        assert result.exit_code == 0
        assert "No running" in result.output

    def test_status_shows_running_instance(self, sleeper):
        proc = sleeper()
        _entry("status-test-001", proc.pid, blueprint="unitree-go2")

        result = CliRunner().invoke(main, ["status"])
        assert result.exit_code == 0
        assert "status-test-001" in result.output
        assert str(proc.pid) in result.output
        assert "unitree-go2" in result.output

    def test_status_shows_uptime_minutes(self, sleeper):
        proc = sleeper()
        ago = (datetime.now(timezone.utc) - timedelta(minutes=7, seconds=30)).isoformat()
        _entry("uptime-min", proc.pid, started_at=ago)

        result = CliRunner().invoke(main, ["status"])
        assert "7m" in result.output

    def test_status_shows_uptime_hours(self, sleeper):
        proc = sleeper()
        ago = (datetime.now(timezone.utc) - timedelta(hours=3, minutes=22)).isoformat()
        _entry("uptime-hrs", proc.pid, started_at=ago)

        result = CliRunner().invoke(main, ["status"])
        assert "3h 22m" in result.output

    def test_status_shows_log_dir(self, sleeper):
        proc = sleeper()
        _entry("log-dir-test", proc.pid, log_dir="/tmp/custom-logs")

        result = CliRunner().invoke(main, ["status"])
        assert "/tmp/custom-logs" in result.output

    def test_status_shows_blueprint(self, sleeper):
        proc = sleeper()
        _entry("bp-test", proc.pid, blueprint="unitree-g1")

        result = CliRunner().invoke(main, ["status"])
        assert "unitree-g1" in result.output

    def test_status_filters_dead_pids(self):
        _entry("dead-one", pid=2_000_000_000)

        result = CliRunner().invoke(main, ["status"])
        assert "No running" in result.output


# ---------------------------------------------------------------------------
# STOP
# ---------------------------------------------------------------------------


class TestStopCLI:
    """Tests for `dimos stop` command."""

    def test_stop_no_instances(self):
        result = CliRunner().invoke(main, ["stop"])
        assert result.exit_code == 1

    @pytest.mark.slow
    def test_stop_default_most_recent(self, sleeper):
        proc = sleeper()
        entry = _entry("stop-default", proc.pid)

        result = CliRunner().invoke(main, ["stop"])
        assert result.exit_code == 0
        assert "Stopping" in result.output
        assert "stop-default" in result.output
        # Poll for registry cleanup
        for _ in range(30):
            if not entry.registry_path.exists():
                break
            time.sleep(0.1)
        assert not entry.registry_path.exists()

    @pytest.mark.slow
    def test_stop_force_sends_sigkill(self, sleeper):
        proc = sleeper()
        _entry("force-kill", proc.pid)

        result = CliRunner().invoke(main, ["stop", "--force"])
        assert result.exit_code == 0
        assert "SIGKILL" in result.output
        for _ in range(30):
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        assert proc.poll() is not None

    @pytest.mark.slow
    def test_stop_sigterm_kills_process(self, sleeper):
        """Verify SIGTERM actually terminates the target process."""
        proc = sleeper()
        _entry("sigterm-verify", proc.pid)

        result = CliRunner().invoke(main, ["stop"])
        assert "SIGTERM" in result.output
        for _ in range(100):  # up to 10s
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        assert proc.poll() is not None, "Process should be dead after SIGTERM"
