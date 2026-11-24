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

import json

import pytest

from dimos.mapping.wifi.coverage import CoverageMap
from dimos.mapping.wifi.survey import SurveyMap
from dimos.msgs.sensor_msgs.SurveyMetrics import SurveyMetrics, SurveySample
from dimos.msgs.sensor_msgs.WifiScan import WifiAccessPoint, WifiScan
from dimos.protocol.encode.cbor import decode, encode


@pytest.fixture
def survey():
    coverage = CoverageMap(cell_size=1)
    for ts in range(7, 13):
        coverage.add_pose(ts, "map", (2.2, 3.2, 0.2))
    grid = SurveyMap(coverage, max_samples=4)
    grid.add_wifi(
        WifiScan(9, "wlan", [WifiAccessPoint(9, "aa:bb:cc:dd:ee:01", "office", 5500, -60, True)])
    )
    return grid


def test_measurement_roundtrip_cell_statistics_and_cached_report_deduplication(survey):
    batch = SurveyMetrics(
        10, "wlan", [SurveySample(10, "ping/router", {"rtt_ms": 20, "probe_loss_pct": 0}, 0.1)]
    )
    assert decode(encode(batch)) == batch
    rows = survey.add(batch)
    assert (rows[0]["position"], rows[0]["bssid"], rows[0]["reason"]) == (
        (2.2, 3.2, 0.2),
        "aa:bb:cc:dd:ee:01",
        None,
    )
    assert survey.add(batch) == []
    survey.add(
        SurveyMetrics(
            11, "wlan", [SurveySample(11, "ping/router", {"rtt_ms": 80, "probe_loss_pct": 0})]
        )
    )
    cell = survey.cells("rtt_ms", "ping/router")[0]
    assert (cell["cell"], cell["count"], cell["mean"], cell["p50"], cell["p95"]) == (
        (2, 3, 0),
        2,
        50,
        50,
        77,
    )


@pytest.mark.parametrize(
    "sample,reason",
    [
        (SurveySample(10, "camera", {"received_fps": 0}, time_uncertainty_s=1), "clock_uncertain"),
        (SurveySample(1, "camera", {"received_fps": 0}), "stale_measurement"),
        (SurveySample(10, "camera", {}), "no_measurement"),
        (
            SurveySample(10, "camera", {}, labels={"error": "clock_unavailable"}),
            "clock_unavailable",
        ),
        (SurveySample(float("nan"), "camera", {"received_fps": float("inf")}), "invalid_timestamp"),
    ],
)
def test_unusable_measurements_are_preserved_but_never_painted(survey, sample, reason):
    row = survey.add(SurveyMetrics(10, "wlan", [sample]))[0]
    assert row["reason"] == reason
    assert survey.cells("received_fps", "camera") == []
    assert json.loads(json.dumps(row, allow_nan=False))["reason"] == reason


def test_missing_pose_and_windows_crossing_cells_are_not_assigned_to_latest_pose():
    coverage = CoverageMap(cell_size=1)
    grid = SurveyMap(coverage)
    sample = SurveySample(11, "camera", {"received_fps": 10}, duration_s=1)
    assert grid.add(SurveyMetrics(11, "wlan", [sample]))[0]["reason"] == "missing_pose"
    grid.clear()
    coverage.add_pose(10, "map", (0.2, 0.2, 0.2))
    coverage.add_pose(11, "map", (2.2, 0.2, 0.2))
    assert grid.add(SurveyMetrics(11, "wlan", [sample]))[0]["reason"] == "measurement_spans_cells"
    assert grid.cells("received_fps", "camera") == []


def test_loss_counts_include_timeouts_but_not_probe_errors(survey):
    survey.add(
        SurveyMetrics(
            10, "wlan", [SurveySample(10, "ping/router", {"rtt_ms": 20, "probe_loss_pct": 0})]
        )
    )
    survey.add(
        SurveyMetrics(11, "wlan", [SurveySample(11, "ping/router", {"probe_loss_pct": 100})])
    )
    survey.add(
        SurveyMetrics(
            12, "wlan", [SurveySample(12, "ping/router", {}, labels={"error": "permission_denied"})]
        )
    )
    assert survey.cells("probe_loss_pct", "ping/router")[0]["mean"] == 50
    assert survey.cells("rtt_ms", "ping/router")[0]["count"] == 1


def test_radio_filter_viewer_identity_and_bounded_retention(survey):
    survey.add(
        SurveyMetrics(
            10,
            "wlan",
            [
                SurveySample(10, "camera/a", {"received_fps": 5}),
                SurveySample(10, "camera/b", {"received_fps": 30}),
            ],
        )
    )
    survey.add_wifi(
        WifiScan(
            10.5, "wlan", [WifiAccessPoint(10.5, "aa:bb:cc:dd:ee:02", "office", 5500, -50, True)]
        )
    )
    survey.add(SurveyMetrics(11, "wlan", [SurveySample(11, "camera/a", {"received_fps": 20})]))
    assert survey.cells("received_fps", "camera/a", "aa:bb:cc:dd:ee:01")[0]["mean"] == 5
    assert survey.cells("received_fps", "camera/a", "aa:bb:cc:dd:ee:02")[0]["mean"] == 20
    assert survey.cells("received_fps", "camera/b")[0]["mean"] == 30
    survey.add(
        SurveyMetrics(
            12,
            "wlan",
            [
                SurveySample(12, "camera/a", {"received_fps": 25}),
                SurveySample(12, "camera/b", {"received_fps": 35}),
            ],
        )
    )
    assert len(survey.samples) == 4
    assert survey.cells("received_fps", "camera/a")[0]["mean"] == 22.5
    epoch = survey.epoch
    survey.clear()
    assert survey.epoch != epoch
    assert survey.layers() == []
