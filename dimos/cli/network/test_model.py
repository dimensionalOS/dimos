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

from dataclasses import replace

import pytest

from dimos.cli.network.model import RTT, Settings, meets_thresholds, percentile


def test_percentiles_and_timeouts_are_separate():
    stats = RTT([1, 2, 10, 20], timeouts=2).summary()
    assert stats["p50_ms"] == 6
    assert stats["p95_ms"] == pytest.approx(18.5)
    assert stats["timeouts"] == 2
    assert stats["replies"] == 4
    assert percentile([], 0.99) is None


@pytest.mark.parametrize(
    "field,value",
    [
        ("max_mbps", 0),
        ("max_mbps", float("nan")),
        ("max_seconds", 601),
        ("max_bytes", 2**31),
        ("payload_bytes", 12),
        ("probe_hz", 201),
        ("max_rtt_p95_ms", -1),
        ("min_goodput_mbps", 51),
        ("max_missing_pct", 101),
    ],
)
def test_invalid_settings_fail_before_process_start(field, value):
    with pytest.raises(ValueError):
        replace(Settings(), **{field: value}).validate()


def test_no_default_verdict_and_timeouts_prevent_latency_pass():
    step = {"receiver": {"goodput_mbps": 20, "missing_pct": 0}, "rtt": {"p95_ms": 2, "timeouts": 1}}
    assert not meets_thresholds(step, Settings())
    assert not meets_thresholds(step, Settings(max_rtt_p95_ms=10))
    assert meets_thresholds(step, Settings(min_goodput_mbps=15))
