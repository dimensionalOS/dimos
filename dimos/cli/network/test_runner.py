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

from dataclasses import asdict
import json
import subprocess
import sys
import threading

import pytest

from dimos.cli.network.model import PROTOCOL_VERSION, Settings
from dimos.cli.network.runner import RemotePeer, run_check, ssh_command
from dimos.cli.network.session import Endpoint

PEER = [sys.executable, "-m", "dimos.cli.commands.network", "peer", "--stdio"]


def test_ssh_path_quoting_and_no_shell_interpolation():
    command = ssh_command("robot@host", "/opt/dimos environment/bin/dimos")
    assert command[-1] == "exec '/opt/dimos environment/bin/dimos' network peer --stdio"
    assert command[-2] == "robot@host"
    with pytest.raises(ValueError):
        ssh_command("-oProxyCommand=bad", "/bin/dimos")
    with pytest.raises(ValueError):
        ssh_command("host", "relative/dimos")


def test_bidirectional_loopback_with_actual_zenoh():
    settings = Settings(
        max_mbps=1,
        payload_bytes=1024,
        idle_seconds=0.2,
        step_seconds=0.3,
        warmup_seconds=0,
        drain_seconds=0.05,
        max_seconds=15,
        probe_hz=20,
    )
    report = run_check(
        "127.0.0.1", "/test/dimos", settings, peer_command=PEER, listen_host="127.0.0.1"
    )
    assert report.status == "completed", report.error
    assert report.cleanup == "confirmed"
    assert report.verdict == "not_requested"
    assert len(report.directions["remote_to_local"]) == 4
    assert len(report.directions["local_to_remote"]) == 4
    for steps in report.directions.values():
        assert steps[-1]["receiver"]["unique_received"] > 0
        assert steps[-1]["sender"]["offered_mbps"] <= 1
        assert steps[-1]["rtt"]["replies"] > 0


def test_requested_goodput_stops_ramp_without_capacity_claim():
    settings = Settings(
        max_mbps=1,
        payload_bytes=1024,
        idle_seconds=0.1,
        step_seconds=0.3,
        warmup_seconds=0,
        drain_seconds=0.05,
        max_seconds=15,
        probe_hz=20,
        min_goodput_mbps=0.01,
    )
    report = run_check(
        "127.0.0.1", "/test/dimos", settings, peer_command=PEER, listen_host="127.0.0.1"
    )
    assert report.status == "completed", report.error
    assert report.verdict == "met"
    assert report.stop_reasons == {
        "remote_to_local": "requested_thresholds_met",
        "local_to_remote": "requested_thresholds_met",
    }
    assert all(len(steps) == 1 for steps in report.directions.values())


def test_peer_stdin_disconnect_exits_and_releases_listener():
    peer = subprocess.Popen(
        PEER, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    try:
        assert peer.stdin is not None and peer.stdout is not None
        peer.stdin.write(
            json.dumps(
                {
                    "protocol": PROTOCOL_VERSION,
                    "token": "a" * 32,
                    "settings": asdict(Settings(max_seconds=5)),
                    "listen_host": "127.0.0.1",
                    "port": 0,
                }
            )
            + "\n"
        )
        peer.stdin.flush()
        hello = json.loads(peer.stdout.readline())
        assert hello["op"] == "hello"
        peer.stdin.close()
        assert peer.wait(timeout=3) == 0
        bye = json.loads(peer.stdout.readline())
        assert bye["cleanup"] == "confirmed"
    finally:
        if peer.poll() is None:
            peer.kill()
            peer.wait()
        for stream in (peer.stdout, peer.stderr):
            if stream is not None:
                stream.close()


def test_peer_unavailable_reports_failure_without_leaking_process():
    report = run_check(
        "127.0.0.1",
        "/test/dimos",
        Settings(max_seconds=5),
        peer_command=[sys.executable, "-c", "raise SystemExit(7)"],
    )
    assert report.status == "error"
    assert "disconnected" in report.error
    assert report.cleanup.startswith("unconfirmed")


def test_session_cap_returns_partial_result_and_cleanup():
    settings = Settings(max_seconds=3, idle_seconds=0.1, step_seconds=5)
    report = run_check(
        "127.0.0.1", "/test/dimos", settings, peer_command=PEER, listen_host="127.0.0.1"
    )
    assert report.status == "capped"
    assert report.error is not None
    assert all(not steps for steps in report.directions.values())


def test_remote_peer_close_reaps_subprocess():
    peer = RemotePeer([sys.executable, "-c", "import sys; sys.stdin.read()"])
    assert peer.close().startswith("unconfirmed")


def test_ctrl_c_cleans_both_owned_endpoints(mocker):
    peer = mocker.patch("dimos.cli.network.runner.RemotePeer").return_value
    peer.request.return_value = {
        "op": "hello",
        "protocol": PROTOCOL_VERSION,
        "zenoh": "1.10.1",
        "port": 12345,
    }
    peer.close.return_value = "confirmed"
    endpoint = mocker.patch("dimos.cli.network.runner.Endpoint").return_value
    endpoint.probe.return_value = 0.1
    endpoint.probes.side_effect = KeyboardInterrupt
    report = run_check("localhost", "/test/dimos", Settings())
    assert report.status == "cancelled"
    assert report.cleanup == "confirmed"
    endpoint.close.assert_called_once_with()
    peer.close.assert_called_once_with()


def test_stdin_disconnect_stops_active_remote_sender(mocker):
    peer = subprocess.Popen(
        PEER, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    endpoint = None
    stop = threading.Event()
    arrived = threading.Event()
    try:
        assert peer.stdin is not None and peer.stdout is not None
        settings = Settings(max_seconds=15, payload_bytes=1024)
        peer.stdin.write(
            json.dumps(
                {
                    "protocol": PROTOCOL_VERSION,
                    "token": "b" * 32,
                    "settings": asdict(settings),
                    "listen_host": "127.0.0.1",
                    "port": 0,
                }
            )
            + "\n"
        )
        peer.stdin.flush()
        hello = json.loads(peer.stdout.readline())
        endpoint = Endpoint("b" * 32, "local", f"tcp/127.0.0.1:{hello['port']}", stop)
        endpoint.receiver.reset(1, 5)
        accept = endpoint.receiver.accept

        def collect(data, now):
            accept(data, now)
            arrived.set()

        mocker.patch.object(endpoint.receiver, "accept", side_effect=collect)
        peer.stdin.write(json.dumps({"op": "send", "phase": 1, "rate": 1, "duration": 5}) + "\n")
        peer.stdin.flush()
        assert arrived.wait(3), "remote did not transmit"
        peer.stdin.close()
        assert peer.wait(timeout=3) == 0
        sent = json.loads(peer.stdout.readline())
        assert sent["sender"]["elapsed_seconds"] < 5
        bye = json.loads(peer.stdout.readline())
        assert bye["cleanup"] == "confirmed"
    finally:
        stop.set()
        if endpoint is not None:
            endpoint.close()
        if peer.poll() is None:
            peer.kill()
            peer.wait()
        for stream in (peer.stdout, peer.stderr):
            if stream is not None:
                stream.close()


def test_malformed_peer_json_is_reported_as_protocol_failure():
    peer = RemotePeer([sys.executable, "-c", "print('null', flush=True)"])
    try:
        with pytest.raises(RuntimeError, match="JSON objects"):
            peer.read()
    finally:
        peer.close()


def test_timeout_at_session_deadline_is_capped_even_before_lease_callback(mocker):
    peer = mocker.patch("dimos.cli.network.runner.RemotePeer").return_value
    peer.request.side_effect = TimeoutError("SSH responder did not reply before deadline")
    peer.close.return_value = "confirmed"
    clock = mocker.patch("dimos.cli.network.runner.time.monotonic")
    clock.side_effect = [100, 100, 190]
    mocker.patch("dimos.cli.network.runner.threading.Timer")
    report = run_check("localhost", "/test/dimos", Settings(max_seconds=90))
    assert report.status == "capped"
    assert report.cleanup == "confirmed"
    assert report.error == "SSH responder did not reply before deadline"
