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

from io import StringIO

from rich.console import Console

from dimos.cli.network.display import curve, print_report
from dimos.cli.network.model import Report, Settings


def test_ascii_non_tty_report_has_legible_status_and_no_ansi():
    report = Report(
        "a" * 32,
        "host",
        "/bin/dimos",
        Settings(),
        "1.10.1",
        status="completed",
        cleanup="confirmed",
    )
    stream = StringIO()
    console = Console(file=stream, width=80, height=25, force_terminal=False, no_color=True)
    print_report(report, console)
    output = stream.getvalue()
    assert "completed" in output
    assert "Cleanup: confirmed" in output
    assert "not_requested" in output
    assert "\x1b[" not in output


def test_curve_includes_idle_latency_in_axis_scale():
    report = Report("a" * 32, "host", "/bin/dimos", Settings(), "1.10.1")
    report.baseline = {"p95_ms": 100}
    output = curve(report, "p95_ms").plain
    assert "110.0" in output
    assert "R" in output
    assert "50 Mbps" in output
