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

from dimos.hosted.doctor import CHECKS, Check, run


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
