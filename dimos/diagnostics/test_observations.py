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

"""Survey evidence and resource history share the same bounded incident bundle."""

import threading

from dimos.core.resource_monitor.stats import ProcessStats
from dimos.diagnostics.analysis import load_bundle
from dimos.diagnostics.collector import Collector
from dimos.diagnostics.observations import record_survey, record_wifi
from dimos.diagnostics.report import report_data
from dimos.msgs.sensor_msgs.SurveyMetrics import SurveyMetrics, SurveySample
from dimos.msgs.sensor_msgs.WifiScan import WifiAccessPoint, WifiScan


def test_survey_capture_preserves_measurement_time_and_distinguishes_unknown_from_loss(
    trace_runtime,
):
    samples = [
        SurveySample(100, "ping/router", {"rtt_ms": 20, "probe_loss_pct": 0}, 0.02),
        SurveySample(101, "ping/router", {"probe_loss_pct": 100}, 1),
        SurveySample(102, "ping/router", {}, labels={"error": "Operation not permitted"}),
        SurveySample(103, "ping/router", {}),
        SurveySample(104, "host", {"tx_mbps": 4}, 1),
    ]
    record_survey(SurveyMetrics(105, "wifi", samples))
    trace_runtime.close()
    manifests, events = load_bundle(trace_runtime.collector.directory)
    assert {m["run_id"] for m in manifests} == {"run"}
    observations = [e for e in events if e["event"] == "survey.measurement"]
    assert [e["observation"]["ts"] for e in observations] == [100, 101, 102, 103, 104]
    assert observations[-1]["observation"]["values"] == {"tx_mbps": 4}
    assert {e["process"] for e in observations} == {trace_runtime.process}
    path = report_data(manifests, events)["network"]["paths"][0]
    assert (path["replies"], path["missing_replies"], path["probe_errors"]) == (1, 1, 2)
    assert path["p95_ms"] == 20


def test_radio_capture_preserves_cached_age_and_does_not_invent_association(trace_runtime):
    record_wifi(
        WifiScan(100, "wifi", [WifiAccessPoint(98, "aa:bb:cc:dd:ee:ff", "office", 5200, -60)])
    )
    trace_runtime.close()
    manifests, events = load_bundle(trace_runtime.collector.directory)
    observation = next(e for e in events if e["event"] == "survey.wifi")
    assert observation["observation"]["ts"] == 98
    radio = report_data(manifests, events)["network"]["radios"][0]
    assert radio["latest"]["bssid"] is None
    assert radio["latest"]["signal_dbm"] is None
    assert radio["latest"]["coverage"]["retry_counters"] is False
    assert radio["association_changes"] is None


def test_dtop_process_sample_survives_in_incident_history(tmp_path, mocker):
    sampled = threading.Event()
    stats = ProcessStats(pid=123, alive=True, cpu_percent=25, rss=1000000)
    collect = mocker.patch("dimos.diagnostics.host.collect_process_stats", return_value=stats)
    original = Collector.emit

    def observe(self, event):
        original(self, event)
        if event["event"] == "host.process":
            sampled.set()

    mocker.patch.object(Collector, "emit", observe)
    collector = Collector(tmp_path, {"process": "worker", "clock_domain": "test"})
    try:
        assert sampled.wait(2), "collector did not sample process resources"
    finally:
        collector.close()
    _, events = load_bundle(tmp_path)
    history = [e for e in events if e["event"] == "host.process"]
    assert collect.call_count >= 1
    assert len(history) >= 1
    assert history[0]["metrics"]["cpu_percent"] == 25
    assert history[0]["metrics"]["rss"] == 1000000
    assert history[0]["process"] == "worker"
