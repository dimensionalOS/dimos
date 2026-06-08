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

from dataclasses import dataclass

import requests

from dimos.robot.unitree.go2.cli.verify import con_notify_url, verify_robot_ip


@dataclass
class _Response:
    status_code: int
    reason: str = "OK"


def test_verify_robot_ip_probes_con_notify_without_movement() -> None:
    calls: list[tuple[str, float]] = []

    def fake_get(url: str, *, timeout: float) -> _Response:
        calls.append((url, timeout))
        return _Response(status_code=200)

    result = verify_robot_ip("192.168.1.50", timeout_s=0.25, http_get=fake_get)

    assert result.ok
    assert result.url == "http://192.168.1.50:9991/con_notify"
    assert calls == [("http://192.168.1.50:9991/con_notify", 0.25)]


def test_verify_robot_ip_returns_structured_failure_on_timeout() -> None:
    def fake_get(url: str, *, timeout: float) -> _Response:
        raise requests.Timeout(f"timeout for {url} after {timeout}")

    result = verify_robot_ip("192.168.1.51", timeout_s=0.1, http_get=fake_get)

    assert not result.ok
    assert result.status_code is None
    assert result.error is not None
    assert "timeout" in result.error
    assert result.to_dict()["url"] == "http://192.168.1.51:9991/con_notify"


def test_verify_robot_ip_returns_http_status_failure() -> None:
    result = verify_robot_ip(
        "192.168.1.52",
        http_get=lambda _url, *, timeout: _Response(status_code=500, reason="Server Error"),
    )

    assert not result.ok
    assert result.status_code == 500
    assert "HTTP 500 Server Error" in result.summary_lines()[0]


def test_con_notify_url_uses_go2_webrtc_port_and_path() -> None:
    assert con_notify_url(" 192.168.1.53 ") == "http://192.168.1.53:9991/con_notify"
