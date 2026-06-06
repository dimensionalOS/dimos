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

from __future__ import annotations

import numpy as np
from typer.testing import CliRunner

from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.cli import doctor as go2_doctor
from dimos.robot.unitree.go2.cli.go2tool import app
from dimos.robot.unitree.go2.cli.landiscovery import Go2Device


def test_check_robot_uses_single_discovered_ip(monkeypatch):
    monkeypatch.setattr(
        go2_doctor,
        "discover",
        lambda timeout: [
            Go2Device(serial="SN123", ip="192.168.0.117", iface="en0", mac="AA:BB")
        ],
    )
    monkeypatch.setattr(go2_doctor, "_signal_endpoint_reachable", lambda host, port, timeout: True)

    check = go2_doctor.check_robot(
        None,
        discover_lan=True,
        discovery_timeout=0.01,
        connect_timeout=0.01,
    )

    assert check.level is go2_doctor.CheckLevel.OK
    assert check.robot_ip == "192.168.0.117"
    assert check.signal_reachable is True
    assert check.discovered[0]["serial"] == "SN123"


def test_check_robot_warns_when_discovered_robot_signal_port_is_unreachable(monkeypatch):
    monkeypatch.setattr(
        go2_doctor,
        "discover",
        lambda timeout: [
            Go2Device(serial="SN123", ip="192.168.0.117", iface="en0", mac=None)
        ],
    )
    monkeypatch.setattr(go2_doctor, "_signal_endpoint_reachable", lambda host, port, timeout: False)

    check = go2_doctor.check_robot(
        None,
        discover_lan=True,
        discovery_timeout=0.01,
        connect_timeout=0.01,
    )

    assert check.level is go2_doctor.CheckLevel.WARN
    assert check.robot_ip == "192.168.0.117"
    assert check.signal_reachable is False


def test_check_robot_reports_discovered_mismatch(monkeypatch):
    monkeypatch.setattr(
        go2_doctor,
        "discover",
        lambda timeout: [
            Go2Device(serial="SN123", ip="192.168.0.117", iface="en0", mac=None)
        ],
    )
    monkeypatch.setattr(go2_doctor, "_signal_endpoint_reachable", lambda host, port, timeout: False)

    check = go2_doctor.check_robot(
        "192.168.0.200",
        discover_lan=True,
        discovery_timeout=0.01,
        connect_timeout=0.01,
    )

    assert check.level is go2_doctor.CheckLevel.FAIL
    assert "192.168.0.117" in check.message


def test_check_ports_classifies_lan_and_localhost(monkeypatch):
    endpoints = [
        go2_doctor.UiEndpoint("Command center", 7779, "http"),
        go2_doctor.UiEndpoint("Phone teleop", 8444, "https"),
        go2_doctor.UiEndpoint("Missing", 12345, "tcp"),
    ]
    monkeypatch.setattr(
        go2_doctor,
        "_tcp_listeners",
        lambda: {
            7779: ["0.0.0.0"],
            8444: ["127.0.0.1"],
        },
    )
    monkeypatch.setattr(
        go2_doctor,
        "_connectable_hosts",
        lambda port, hosts, timeout=0.15: {
            7779: ["192.168.0.105"],
            8444: ["127.0.0.1"],
        }.get(port, []),
    )

    checks = {check.port: check for check in go2_doctor.check_ports(endpoints)}

    assert checks[7779].level is go2_doctor.CheckLevel.OK
    assert checks[7779].lan_reachable is True
    assert checks[8444].level is go2_doctor.CheckLevel.WARN
    assert checks[8444].lan_reachable is False
    assert checks[12345].listening is False


def test_check_ports_trusts_lan_bind_even_when_local_tcp_probe_fails(monkeypatch):
    endpoints = [go2_doctor.UiEndpoint("Command center", 7779, "http")]
    monkeypatch.setattr(go2_doctor, "_tcp_listeners", lambda: {7779: ["0.0.0.0"]})
    monkeypatch.setattr(go2_doctor, "_connectable_hosts", lambda port, hosts, timeout=0.15: [])

    check = go2_doctor.check_ports(endpoints, probe_hosts=["127.0.0.1"])[0]

    assert check.level is go2_doctor.CheckLevel.OK
    assert check.listening is True
    assert check.lan_reachable is True
    assert "LAN devices should be able to connect" in check.message


def test_parse_lsof_tcp_listener_normalizes_wildcard():
    line = "Python  10190 user  28u IPv4 0x0 0t0 TCP *:7779 (LISTEN)"

    assert go2_doctor._parse_lsof_tcp_listener(line) == ("0.0.0.0", 7779)


def test_check_image_receives_frame_and_cleans_up(monkeypatch):
    unsubscribed: list[str] = []
    stopped: list[str] = []

    class FakeTransport:
        def __init__(self, topic: str) -> None:
            self.topic = topic

        def stop(self) -> None:
            stopped.append(self.topic)

    def fake_subscribe_pubsub_uri(topic, callback, *, msg_type=None):  # type: ignore[no-untyped-def]
        callback(Image(data=np.zeros((7, 11, 3), dtype=np.uint8)))
        return FakeTransport(topic), lambda: unsubscribed.append(topic)

    monkeypatch.setattr(
        "dimos.protocol.pubsub.registry.subscribe_pubsub_uri",
        fake_subscribe_pubsub_uri,
    )

    check = go2_doctor.check_image(
        enabled=True,
        topics=["jpeg_lcm:/color_image"],
        timeout=0.01,
    )

    assert check.level is go2_doctor.CheckLevel.OK
    assert check.width == 11
    assert check.height == 7
    assert unsubscribed == ["jpeg_lcm:/color_image"]
    assert stopped == ["jpeg_lcm:/color_image"]


def test_signal_probe_failure_is_warning_when_runtime_image_is_live():
    robot = go2_doctor.RobotCheck(
        level=go2_doctor.CheckLevel.FAIL,
        robot_ip="192.168.0.117",
        signal_port=go2_doctor.GO2_SIGNAL_PORT,
        signal_reachable=False,
        discovered=[],
        message="signal probe failed.",
    )
    image = go2_doctor.ImageCheck(
        level=go2_doctor.CheckLevel.OK,
        checked=True,
        topic="pshm:color_image",
        width=1280,
        height=720,
        message="received frame",
        attempted_topics=["pshm:color_image"],
    )

    check = go2_doctor._adjust_robot_check_with_runtime_evidence(robot, image)

    assert check.level is go2_doctor.CheckLevel.WARN
    assert "WebRTC data plane is active" in check.message


def test_suggested_urls_prefer_robot_subnet_interfaces():
    interfaces = [
        go2_doctor.LocalInterface("tailscale0", "100.64.0.10", None, False),
        go2_doctor.LocalInterface("en0", "192.168.0.105", "255.255.255.0", True),
    ]
    ports = [
        go2_doctor.PortCheck(
            level=go2_doctor.CheckLevel.OK,
            name="Command center",
            port=go2_doctor.COMMAND_CENTER_PORT,
            listening=True,
            bind_hosts=["0.0.0.0"],
            lan_reachable=True,
            message="listening",
        ),
        go2_doctor.PortCheck(
            level=go2_doctor.CheckLevel.OK,
            name="Phone teleop",
            port=go2_doctor.PHONE_TELEOP_PORT,
            listening=True,
            bind_hosts=["0.0.0.0"],
            lan_reachable=True,
            message="listening",
        ),
    ]

    urls = go2_doctor.suggested_urls(interfaces, ports)

    assert urls == [
        "http://192.168.0.105:7779/command-center",
        "https://192.168.0.105:8444/teleop",
    ]


def test_go2tool_doctor_command_is_registered():
    result = CliRunner().invoke(app, ["doctor", "--help"])

    assert result.exit_code == 0
    assert "without sending motion commands" in result.output
