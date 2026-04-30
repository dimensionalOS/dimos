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

import pytest

from dimos.core.run_registry import RunEntry, get_most_recent_rpyc_port


@pytest.fixture(autouse=True)
def _use_tmp_registry(tmp_path, monkeypatch):
    """Redirect the registry directory to a temp dir for every test."""
    monkeypatch.setattr("dimos.core.run_registry.REGISTRY_DIR", tmp_path)


@pytest.fixture(autouse=True)
def _all_pids_alive(mocker):
    """Treat every PID as alive by default; individual tests can override."""
    mocker.patch("dimos.core.run_registry.is_pid_alive", return_value=True)


@pytest.fixture()
def make_entry(tmp_path):
    """Factory fixture: create and persist a RunEntry, return it."""

    def _make(run_id="run-1", rpyc_port=18812, **kwargs):
        defaults = dict(
            pid=9999,
            blueprint="bp",
            started_at="2026-01-01T00:00:00",
            log_dir=str(tmp_path / "logs"),
        )
        defaults.update(kwargs)
        entry = RunEntry(run_id=run_id, rpyc_port=rpyc_port, **defaults)
        entry.save()
        return entry

    return _make


def test_returns_port_of_most_recent_run(make_entry):
    make_entry(run_id="20260101-000000-old", rpyc_port=10001)
    make_entry(run_id="20260102-000000-new", rpyc_port=10002)

    assert get_most_recent_rpyc_port() == 10002


def test_returns_port_for_specific_run_id(make_entry):
    make_entry(run_id="target", rpyc_port=10001)
    make_entry(run_id="other", rpyc_port=10002)

    assert get_most_recent_rpyc_port(run_id="target") == 10001


def test_raises_when_no_running_instances():
    with pytest.raises(RuntimeError, match="No running DimOS instance"):
        get_most_recent_rpyc_port()


def test_raises_when_run_id_not_found(make_entry):
    make_entry(run_id="exists")

    with pytest.raises(RuntimeError, match="No running DimOS instance with run_id"):
        get_most_recent_rpyc_port(run_id="nope")


def test_raises_when_rpyc_port_is_zero(make_entry):
    make_entry(rpyc_port=0)

    with pytest.raises(RuntimeError, match="no rpyc_port"):
        get_most_recent_rpyc_port()


def test_skips_dead_processes(make_entry, mocker):
    alive = make_entry(run_id="alive", rpyc_port=10001, pid=1)
    make_entry(run_id="dead", rpyc_port=10002, pid=2)

    mocker.patch(
        "dimos.core.run_registry.is_pid_alive",
        side_effect=lambda pid: pid == alive.pid,
    )

    assert get_most_recent_rpyc_port() == 10001
