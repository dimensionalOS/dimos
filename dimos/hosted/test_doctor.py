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

from pathlib import Path
import subprocess

import pytest
from pytest_mock import MockerFixture

from dimos.hosted import doctor, machine
from dimos.hosted.config import HostConfig
from dimos.hosted.discovery import Probe
from dimos.hosted.doctor import CHECKS, Check, Result, Skipped, run
from dimos.hosted.protocol import HostDescriptor


def test_doctor_runner_outcomes() -> None:
    state = {"ok": False}

    def boom() -> str | None:
        raise OSError("gone")

    results = run(
        [
            Check("fine", lambda: None),
            Check("broken", lambda: "bad"),
            Check("fixable", lambda: None if state["ok"] else "off", lambda: state.update(ok=True)),
            Check("raising", boom),
        ],
        fix=True,
    )
    assert [(r.ok, r.fix_note) for r in results] == [
        (True, None),
        (False, "no automatic fix"),
        (True, "fixed"),
        (False, "no automatic fix"),
    ]
    assert results[3].error == "OSError: gone"


def test_doctor_runner_without_fix_only_checks() -> None:
    fixed = []
    (result,) = run([Check("d", lambda: "off", lambda: fixed.append(1))])
    assert not result.ok and result.fix_note is None and not fixed


def test_a_failing_warning_is_reported_not_fatal() -> None:
    (result,) = run([Check("d", lambda: "off", warning=True)])
    assert not result.ok and result.warning


def test_fixes_install_before_they_start_or_restart() -> None:
    order = [c.description for c in CHECKS]
    assert order.index("The installed systemd unit matches this checkout") < order.index(
        "The local Host is running and answers on its router"
    )
    assert order.index("The local Host is running and answers on its router") < order.index(
        "The running Host serves this checkout's code revision"
    )


def test_a_loopback_only_host_is_flagged_unreachable(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(doctor, "HostConfig", lambda: HostConfig(listen="tcp/127.0.0.1:7447"))
    assert "Only this machine" in (doctor._reachable() or "")
    monkeypatch.setattr(
        doctor, "HostConfig", lambda: HostConfig(listen="tcp/127.0.0.1:7447,iface/eth0")
    )
    assert doctor._reachable() is None


@pytest.mark.parametrize("fix", [False, True])
def test_skipped_check_is_neither_passed_nor_fixed(mocker: MockerFixture, fix: bool) -> None:
    repair = mocker.Mock()

    results = run([Check("dependent", lambda: Skipped("prerequisite missing"), repair)], fix=fix)

    assert results == [Result("dependent", ok=False, skip_reason="prerequisite missing")]
    repair.assert_not_called()


def test_check_skipped_after_fix_is_not_reported_as_passed(mocker: MockerFixture) -> None:
    check = mocker.Mock(side_effect=["stale", Skipped("no local Host")])
    repair = mocker.Mock()

    results = run([Check("revision", check, repair)], fix=True)

    assert results == [Result("revision", ok=False, fix_note="fixed", skip_reason="no local Host")]
    repair.assert_called_once_with()


@pytest.mark.parametrize("fix", [False, True])
@pytest.mark.parametrize(
    ("local", "reason"),
    [
        (None, "no local Host"),
        (
            machine.HostRecord("host", "lab", 123, (), "tcp/127.0.0.1:7447"),
            "the local Host does not answer",
        ),
    ],
)
def test_running_revision_is_skipped_without_an_answering_host(
    mocker: MockerFixture, local: machine.HostRecord | None, reason: str, fix: bool
) -> None:
    mocker.patch.object(machine, "local_host", return_value=local)
    mocker.patch.object(doctor, "probe", return_value=None)
    restart = mocker.patch.object(machine, "restart")
    check = next(
        c
        for c in CHECKS
        if c.description == "The running Host serves this checkout's code revision"
    )

    results = run([check], fix=fix)

    assert results == [Result(check.description, ok=False, skip_reason=reason)]
    restart.assert_not_called()


@pytest.mark.parametrize(
    ("revision", "error"),
    [("checkout", None), ("outdated", "Host serves outdated, checkout is checkout")],
)
def test_running_revision_is_evaluated_when_host_answers(
    mocker: MockerFixture, revision: str, error: str | None
) -> None:
    local = machine.HostRecord("host", "lab", 123, (), "tcp/127.0.0.1:7447")
    host = HostDescriptor(local.host_id, local.name, {}, revision, ())
    mocker.patch.object(machine, "local_host", return_value=local)
    mocker.patch.object(
        doctor, "probe", return_value=Probe(local.client_endpoint, "router", (host,))
    )
    mocker.patch.object(machine, "code_revision", return_value="checkout")
    check = next(
        c
        for c in CHECKS
        if c.description == "The running Host serves this checkout's code revision"
    )

    assert run([check]) == [Result(check.description, ok=error is None, error=error)]


@pytest.mark.parametrize(
    ("linger", "ok", "error", "fix_note"),
    [
        ("yes", True, None, None),
        ("no", False, "run: sudo loginctl enable-linger tester", "no automatic fix"),
    ],
)
def test_fix_evaluates_dependent_checks_after_installing_unit(
    mocker: MockerFixture,
    tmp_path: Path,
    linger: str,
    ok: bool,
    error: str | None,
    fix_note: str | None,
) -> None:
    path = tmp_path / machine.UNIT_NAME
    mocker.patch.object(machine, "unit_path", return_value=path)
    mocker.patch.object(machine, "enabled", return_value=True)
    mocker.patch.object(doctor.getpass, "getuser", return_value="tester")
    loginctl = mocker.patch.object(
        doctor.subprocess,
        "run",
        return_value=subprocess.CompletedProcess([], 0, stdout=f"Linger={linger}\n"),
    )

    def install() -> Path:
        path.write_text(machine.render_unit(machine.dimos_executable(), machine.DIMOS_PROJECT_ROOT))
        return path

    repair = mocker.patch.object(machine, "install", side_effect=install)
    descriptions = (
        "The dimos-host systemd user unit is installed and enabled",
        "The installed systemd unit matches this checkout",
        "Lingering is on, so the unit starts at boot without a login",
    )
    checks = [check for check in CHECKS if check.description in descriptions]

    results = run(checks, fix=True)

    assert results == [
        Result(descriptions[0], ok=True, fix_note="fixed"),
        Result(descriptions[1], ok=True),
        Result(
            descriptions[2],
            ok=ok,
            error=error,
            fix_note=fix_note,
            warning=True,
        ),
    ]
    repair.assert_called_once_with()
    loginctl.assert_called_once_with(
        ["loginctl", "show-user", "tester", "-p", "Linger"], capture_output=True, text=True
    )
