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

from dimos.diagnostics.infrastructure import normalize_ap
from dimos.diagnostics.report import report_data, write_report


def test_ap_collector_keeps_per_client_downlink_counters_separate():
    probes = [
        {
            "command": ["iw", "dev", "wlan0", "info"],
            "returncode": 0,
            "stdout": "Interface wlan0\n addr 11:22:33:44:55:66\n",
        },
        {
            "command": ["iw", "dev", "wlan0", "station", "dump"],
            "returncode": 0,
            "stdout": "Station aa:bb:cc:dd:ee:01 (on wlan0)\n tx packets: 100\n tx retries: 12\nStation aa:bb:cc:dd:ee:02 (on wlan0)\n tx packets: 200\n tx retries: 30\n",
        },
    ]
    observation = normalize_ap(
        {"probes": probes, "received_ns": 10, "sent_ns": 20, "clock_domain": "ap/boot/monotonic"},
        "ap-host",
        "wlan0",
        1,
        30,
    )
    assert observation["status"] == "ok"
    assert [link["radio"]["counters"]["tx_retries"] for link in observation["links"]] == [12, 30]
    assert [link["target"] for link in observation["links"]] == [
        "station/aa:bb:cc:dd:ee:01",
        "station/aa:bb:cc:dd:ee:02",
    ]
    assert all("AP TX is client downlink" in link["evidence"] for link in observation["links"])
    assert observation["coverage"]["per_packet_queue_delay"] is False


def test_report_draws_observed_radio_and_probes_and_marks_bad_process(tmp_path):
    manifests = [
        {"process": "worker", "host": "test-host", "health": {"capture_window_complete": False}}
    ]
    events = [
        {
            "event": "network.wifi",
            "host": "test-host",
            "radio": {
                "interface": "wlan0",
                "bssid": "11:22:33:44:55:66",
                "source": "iw",
                "mono_ns": 1,
                "coverage": {"retry_counters": False},
            },
        },
        {
            "event": "network.path_probe",
            "host": "test-host",
            "measurement": {
                "address": "192.0.2.1",
                "role": "default_gateway",
                "status": "ok",
                "rtt_ms": 12,
                "scope": "ICMP round trip",
            },
        },
        {
            "event": "ik.start",
            "process": "worker",
            "span_id": "1",
            "clock_domain": "clock",
            "mono_ns": 0,
        },
        {
            "event": "ik.end",
            "process": "worker",
            "span_id": "1",
            "clock_domain": "clock",
            "mono_ns": 80_000_000,
        },
    ]
    profile = {"schema": "dimos.budgets/v1", "stages": {"ik": {"p99_ms": 10, "min_samples": 1}}}
    report = report_data(manifests, events, profile)
    worker = next(n for n in report["graph"]["nodes"] if n["kind"] == "process")
    assert worker["budget_breaches"] == ["ik"]
    assert report["network"]["paths"][0]["p95_ms"] == 12
    assert any(e["kind"] == "wireless_association" for e in report["graph"]["edges"])
    assert report["evaluation"]["status"] == "fail"
    output = tmp_path / "report.html"
    write_report(output, manifests, events, profile)
    assert "__REPORT_DATA__" not in output.read_text()
    assert "test-host" in output.read_text()
