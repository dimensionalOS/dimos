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

"""HTTP probe failures and asymmetric timing preserve their measurement scope."""

import requests

from dimos.diagnostics.probe import exchange


def test_probe_reports_rtt_and_offset_bounds_without_assuming_symmetry(requests_mock, monkeypatch):
    ticks = iter([0, 50])
    monkeypatch.setattr("dimos.diagnostics.probe.time.monotonic_ns", lambda: next(ticks))
    monkeypatch.setattr("dimos.diagnostics.probe.clock_domain", lambda: "A")
    requests_mock.get(
        "https://probe.invalid/clock",
        json={"schema": "dimos.clock/v1", "clock_domain": "B", "received_ns": 130, "sent_ns": 140},
    )
    with requests.Session() as session:
        result = exchange(session, "https://probe.invalid/clock")
    assert (result["rtt_ns"], result["server_work_ns"]) == (50, 10)
    assert (result["mapping"]["lower_ns"], result["mapping"]["upper_ns"]) == (90, 130)
    assert result["scope"] == "HTTP application round trip"


def test_probe_timeout_is_a_failure_not_a_zero_latency_sample(requests_mock):
    requests_mock.get("https://probe.invalid/clock", exc=requests.Timeout)
    with requests.Session() as session:
        result = exchange(session, "https://probe.invalid/clock")
    assert result["status"] == "error"
    assert "rtt_ns" not in result
