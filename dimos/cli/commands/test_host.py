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

from functools import partial
from pathlib import Path

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.cli.commands.host import doctor as host_doctor, host_app
from dimos.hosted import doctor, machine


def test_doctor_reports_missing_unit_checks_as_skipped(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    descriptions = (
        "The dimos-host systemd user unit is installed and enabled",
        "The installed systemd unit matches this checkout",
        "Lingering is on, so the unit starts at boot without a login",
    )
    checks = [check for check in doctor.CHECKS if check.description in descriptions]
    mocker.patch.object(doctor, "run", side_effect=partial(doctor.run, checks))
    mocker.patch.object(machine, "unit_path", return_value=tmp_path / machine.UNIT_NAME)

    result = CliRunner().invoke(host_app, ["doctor"])

    assert result.exit_code == 1
    assert result.output.splitlines() == [
        f"FAIL  {descriptions[0]}  (not installed)",
        f"SKIP  {descriptions[1]}  (unit not installed)",
        f"SKIP  {descriptions[2]}  (unit not installed)",
    ]


@pytest.mark.parametrize("tty", [False, True])
def test_doctor_skipped_checks_are_nonfatal(
    mocker: MockerFixture, capsys: pytest.CaptureFixture[str], tty: bool
) -> None:
    results = doctor.run(
        [doctor.Check("dependent", lambda: doctor.Skipped("prerequisite missing"))]
    )
    mocker.patch.object(doctor, "run", return_value=results)
    mocker.patch("sys.stdout.isatty", return_value=tty)

    host_doctor(fix=False)

    assert capsys.readouterr().out == "SKIP  dependent  (prerequisite missing)\n"
