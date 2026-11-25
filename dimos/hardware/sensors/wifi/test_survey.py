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

import asyncio
from contextlib import aclosing
from unittest.mock import AsyncMock

import pytest

from dimos.hardware.sensors.wifi.desktop_metrics import DesktopMetrics
from dimos.hardware.sensors.wifi.probes import HostCounters, probe
from dimos.hardware.sensors.wifi.survey import NetworkSurvey
from dimos.msgs.sensor_msgs.SurveyMetrics import SurveySample


@pytest.fixture(autouse=True)
def linux_probe_platform(mocker):
    mocker.patch("dimos.hardware.sensors.wifi.probes.platform.system", return_value="Linux")


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "code,stdout,stderr,values",
    [
        (
            0,
            b"64 bytes: icmp_seq=1 ttl=64 time=12.4 ms",
            b"",
            {"rtt_ms": 12.4, "probe_loss_pct": 0},
        ),
        (1, b"100% packet loss", b"", {"probe_loss_pct": 100}),
        (2, b"", b"Operation not permitted", {}),
    ],
)
async def test_ping_distinguishes_delay_loss_and_probe_failure(
    mocker, code, stdout, stderr, values
):
    process = mocker.Mock(returncode=code)
    process.communicate = AsyncMock(return_value=(stdout, stderr))
    spawn = mocker.patch(
        "dimos.hardware.sensors.wifi.probes.asyncio.create_subprocess_exec", return_value=process
    )
    sample = await probe("testwifi", "router", "192.0.2.1")
    assert sample.values == values
    assert sample.labels.get("error", "") == stderr.decode()
    assert spawn.call_args.args[:5] == ("ping", "-n", "-I", "testwifi", "-c")


@pytest.mark.asyncio
async def test_cancelled_ping_is_killed_and_reaped(mocker):
    process = mocker.Mock(returncode=None)
    process.communicate = AsyncMock(side_effect=asyncio.CancelledError)
    process.wait = AsyncMock()
    mocker.patch(
        "dimos.hardware.sensors.wifi.probes.asyncio.create_subprocess_exec", return_value=process
    )
    with pytest.raises(asyncio.CancelledError):
        await probe("testwifi", "router", "192.0.2.1")
    process.kill.assert_called_once()
    process.wait.assert_awaited_once()


def test_interface_rate_uses_monotonic_elapsed_and_counter_reset_is_unknown():
    before = HostCounters(100, 10, 100, 200, 1000, 1000)
    after = HostCounters(200, 12, 104, 208, 2001000, 1001000)
    assert after.since(before, "wifi").values == {"tx_mbps": 8, "rx_mbps": 4, "cpu_pct": 50}
    reset = HostCounters(201, 13, 104, 208, 1, 1)
    assert reset.since(after, "wifi").values == {}


@pytest.fixture
def report():
    return {
        "clients": [
            {
                "clientId": "headset",
                "at": 100000,
                "receivedAt": 100220,
                "connection": "connected",
                "visibility": "visible",
                "runId": "test-run",
                "link": {
                    "rttMs": 600,
                    "pair": {"currentRoundTripTime": 0.02},
                    "gateway": {"clock": {"offsetMs": 200, "rttMs": 40}},
                },
                "cameras": [
                    {
                        "key": "left",
                        "sourceFps": 30,
                        "receivedFps": 5,
                        "displayedFps": None,
                        "lastDisplayedAgoMs": 300,
                    }
                ],
            }
        ]
    }


def test_desktop_clock_alignment_probe_semantics_and_deduplication(report):
    adapter = DesktopMetrics()
    samples = adapter.read(report, 100.3)
    link = next(s for s in samples if s.source == "viewer/headset")
    assert (link.ts, link.time_uncertainty_s) == (100.2, 0.02)
    assert link.values == {"application_rtt_ms": 600, "rtc_rtt_ms": 20}
    duplicate = adapter.read(report, 101)
    assert [(s.source, s.values) for s in duplicate] == [
        ("viewer/headset/telemetry", {"report_arrival_age_ms": 780})
    ]
    report["clients"][0]["at"] = 102000
    camera = next(s for s in adapter.read(report, 102.3) if s.source == "viewer/headset/left")
    assert camera.values == {"source_fps": 30, "received_fps": 5}
    assert camera.duration_s == 2


def test_stale_disconnected_or_unsynchronized_browser_stats_are_not_measurements(report):
    report["clients"][0]["connection"] = "lost"
    samples = DesktopMetrics().read(report, 100.3)
    assert not any("rtt_ms" in s.values or "received_fps" in s.values for s in samples)
    assert samples[-1].labels["error"] == "viewer_disconnected"
    report["clients"][0]["link"]["gateway"]["clock"] = None
    assert DesktopMetrics().read(report, 100.3)[-1].labels["error"] == "clock_unavailable"
    report["clients"][0]["link"]["gateway"]["clock"] = {"offsetMs": 200, "rttMs": 40}
    assert DesktopMetrics().read(report, 120)[-1].labels["error"] == "stale_report"


@pytest.mark.asyncio
async def test_sampler_publishes_one_batch_and_stops_its_owned_task(mocker):
    module = NetworkSurvey(interface="wifi", targets={"router": "192.0.2.1"})
    mocker.patch.object(module, "check_dependencies")
    mocker.patch(
        "dimos.hardware.sensors.wifi.survey.HostCounters.read",
        side_effect=[HostCounters(10, 10, 10, 20, 0, 0), HostCounters(11, 11, 11, 22, 1000000, 0)],
    )
    probe_mock = mocker.patch(
        "dimos.hardware.sensors.wifi.survey.probe",
        return_value=SurveySample(11, "ping/router", {"rtt_ms": 15}),
    )
    received = asyncio.Event()
    published = mocker.patch.object(
        module.survey_metrics, "publish", side_effect=lambda _: received.set()
    )
    try:
        async with aclosing(module.main()) as lifecycle:
            await anext(lifecycle)
            await asyncio.wait_for(received.wait(), timeout=1)
        batch = published.call_args.args[0]
        assert [(s.source, s.values) for s in batch.samples] == [
            ("ping/router", {"rtt_ms": 15}),
            ("host", {"tx_mbps": 8, "rx_mbps": 0, "cpu_pct": 50}),
        ]
        probe_mock.assert_awaited_once_with("wifi", "router", "192.0.2.1")
    finally:
        module.stop()


@pytest.mark.asyncio
async def test_sampler_keeps_publishing_host_and_probes_after_ping_timeout(mocker):
    module = NetworkSurvey(interface="wifi", targets={"router": "192.0.2.1"}, poll_interval_s=0.2)
    mocker.patch.object(module, "check_dependencies")
    mocker.patch(
        "dimos.hardware.sensors.wifi.survey.HostCounters.read",
        side_effect=[HostCounters(i, i, i, i * 2, i * 1000, 0) for i in (10, 11, 12)],
    )
    mocker.patch(
        "dimos.hardware.sensors.wifi.survey.probe",
        side_effect=[asyncio.TimeoutError(), SurveySample(12, "ping/router", {"rtt_ms": 5})],
    )
    received = asyncio.Event()
    batches = []

    def collect(batch):
        batches.append(batch)
        if len(batches) == 2:
            received.set()

    mocker.patch.object(module.survey_metrics, "publish", side_effect=collect)
    try:
        async with aclosing(module.main()) as lifecycle:
            await anext(lifecycle)
            await asyncio.wait_for(received.wait(), timeout=2)
            assert module.get_status()["state"] == "running"
        first, recovered = batches
        assert first.samples[0].values == {}
        assert first.samples[0].labels["error"].startswith("TimeoutError:")
        assert [sample.source for sample in first.samples] == ["ping/router", "host"]
        assert recovered.samples[0].values == {"rtt_ms": 5}
        assert module.get_status()["state"] == "stopped"
    finally:
        module.stop()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "code,stderr,status", [(2, b"", "no_reply"), (1, b"socket failure", "error")]
)
async def test_darwin_default_route_probe_uses_millisecond_timeout(mocker, code, stderr, status):
    mocker.patch("dimos.hardware.sensors.wifi.probes.platform.system", return_value="Darwin")
    process = mocker.Mock(returncode=code)
    process.communicate = AsyncMock(return_value=(b"", stderr))
    spawn = mocker.patch(
        "dimos.hardware.sensors.wifi.probes.asyncio.create_subprocess_exec", return_value=process
    )
    sample = await probe(None, "router", "192.0.2.1")
    assert sample.labels["status"] == status
    assert spawn.call_args.args == ("ping", "-n", "-c", "1", "-W", "1000", "192.0.2.1")
    assert ("probe_loss_pct" in sample.values) == (status == "no_reply")


@pytest.mark.asyncio
async def test_survey_recovers_from_missing_interface_without_fabricating_rates(mocker):
    module = NetworkSurvey(interface="wifi", poll_interval_s=0.2)
    mocker.patch.object(module, "check_dependencies")
    mocker.patch(
        "dimos.hardware.sensors.wifi.survey.HostCounters.read",
        side_effect=[
            KeyError("wifi"),
            KeyError("wifi"),
            HostCounters(11, 11, 11, 22, 1000000, 0),
            HostCounters(12, 12, 12, 24, 2000000, 0),
        ],
    )
    received = asyncio.Event()
    batches = []

    def collect(batch):
        batches.append(batch)
        if len(batches) == 3:
            received.set()

    mocker.patch.object(module.survey_metrics, "publish", side_effect=collect)
    try:
        async with aclosing(module.main()) as lifecycle:
            await anext(lifecycle)
            await asyncio.wait_for(received.wait(), timeout=2)
            assert module.get_status()["state"] == "running"
        missing, baseline, recovered = [batch.samples[0] for batch in batches]
        assert missing.labels["error"] == "interface_unavailable"
        assert missing.values == baseline.values == {}
        assert recovered.values == {"tx_mbps": 8, "rx_mbps": 0, "cpu_pct": 50}
    finally:
        module.stop()
