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

from dimos.robot.unitree.go2.cli.setup import setup_go2_wifi
from dimos.robot.unitree.go2.cli.verify import Go2VerifyStatus


@dataclass(frozen=True)
class _BleRobot:
    name: str
    address: str
    serial: str | None = None


@dataclass(frozen=True)
class _LanRobot:
    serial: str
    ip: str
    mac: str | None = None
    iface: str | None = None


def _ok_verify(ip: str) -> Go2VerifyStatus:
    return Go2VerifyStatus(
        robot_ip=ip,
        url=f"http://{ip}:9991/con_notify",
        ok=True,
        status_code=200,
        reason="OK",
        error=None,
        elapsed_s=0.01,
        timeout_s=1.0,
    )


def _failed_verify(ip: str) -> Go2VerifyStatus:
    return Go2VerifyStatus(
        robot_ip=ip,
        url=f"http://{ip}:9991/con_notify",
        ok=False,
        status_code=503,
        reason="Unavailable",
        error=None,
        elapsed_s=0.01,
        timeout_s=1.0,
    )


async def test_setup_success_orchestrates_provision_discovery_verify() -> None:
    calls: list[str] = []

    async def discover_ble() -> list[_BleRobot]:
        calls.append("discover_ble")
        return [_BleRobot(name="Go2_123", address="AA:BB:CC", serial="SN123")]

    async def provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
    ) -> str | None:
        calls.append(f"provision:{address}:{ssid}:{country_code}")
        assert password == "secret-password"
        return "SN123"

    async def discover_lan() -> list[_LanRobot]:
        calls.append("discover_lan")
        return [_LanRobot(serial="SN123", ip="192.168.1.70", mac="00:11", iface="en0")]

    def verify(ip: str) -> Go2VerifyStatus:
        calls.append(f"verify:{ip}")
        return _ok_verify(ip)

    result = await setup_go2_wifi(
        ssid="LabWiFi",
        password="secret-password",
        discover_ble=discover_ble,
        provision_wifi=provision_wifi,
        discover_lan=discover_lan,
        serial="SN123",
        verify_robot=verify,
        rediscovery_attempts=1,
        rediscovery_delay_s=0,
    )

    assert result.ok
    assert result.robot_ip == "192.168.1.70"
    assert result.next_steps == (
        "export ROBOT_IP=192.168.1.70",
        "dimos --robot-ip 192.168.1.70 run unitree-go2",
    )
    assert calls == [
        "discover_ble",
        "provision:AA:BB:CC:LabWiFi:US",
        "discover_lan",
        "verify:192.168.1.70",
    ]


async def test_setup_returns_verification_failure() -> None:
    async def discover_ble() -> list[_BleRobot]:
        return [_BleRobot(name="Go2_123", address="AA:BB:CC", serial="SN123")]

    async def provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
    ) -> str | None:
        return "SN123"

    async def discover_lan() -> list[_LanRobot]:
        return [_LanRobot(serial="SN123", ip="192.168.1.70")]

    result = await setup_go2_wifi(
        ssid="LabWiFi",
        password="secret-password",
        discover_ble=discover_ble,
        provision_wifi=provision_wifi,
        discover_lan=discover_lan,
        verify_robot=_failed_verify,
        rediscovery_attempts=1,
        rediscovery_delay_s=0,
    )

    assert not result.ok
    assert result.robot_ip == "192.168.1.70"
    assert result.verification is not None
    assert not result.verification.ok
    assert result.next_steps == ()
    assert result.error is not None
    assert "HTTP 503" in result.error


async def test_setup_no_robot_found_does_not_provision() -> None:
    provision_called = False

    async def discover_ble() -> list[_BleRobot]:
        return []

    async def provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
    ) -> str | None:
        nonlocal provision_called
        provision_called = True
        return None

    async def discover_lan() -> list[_LanRobot]:
        return []

    result = await setup_go2_wifi(
        ssid="LabWiFi",
        password="secret-password",
        discover_ble=discover_ble,
        provision_wifi=provision_wifi,
        discover_lan=discover_lan,
        rediscovery_attempts=1,
        rediscovery_delay_s=0,
    )

    assert not result.ok
    assert not provision_called
    assert result.error == "No BLE robot matched available robot."


async def test_setup_ambiguous_robot_selection_does_not_provision() -> None:
    provision_called = False

    async def discover_ble() -> list[_BleRobot]:
        return [
            _BleRobot(name="Go2_123", address="AA:BB:01", serial="SN1"),
            _BleRobot(name="Go2_456", address="AA:BB:02", serial="SN2"),
        ]

    async def provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
    ) -> str | None:
        nonlocal provision_called
        provision_called = True
        return None

    async def discover_lan() -> list[_LanRobot]:
        return []

    result = await setup_go2_wifi(
        ssid="LabWiFi",
        password="secret-password",
        discover_ble=discover_ble,
        provision_wifi=provision_wifi,
        discover_lan=discover_lan,
        rediscovery_attempts=1,
        rediscovery_delay_s=0,
    )

    assert not result.ok
    assert not provision_called
    assert result.error is not None
    assert "Ambiguous BLE robot selection" in result.error


async def test_setup_rejects_conflicting_address_and_serial() -> None:
    provision_called = False

    async def discover_ble() -> list[_BleRobot]:
        return [_BleRobot(name="Go2_123", address="AA:BB:CC", serial="SN123")]

    async def provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
    ) -> str | None:
        nonlocal provision_called
        provision_called = True
        return None

    async def discover_lan() -> list[_LanRobot]:
        return []

    result = await setup_go2_wifi(
        ssid="LabWiFi",
        password="secret-password",
        discover_ble=discover_ble,
        provision_wifi=provision_wifi,
        discover_lan=discover_lan,
        address="AA:BB:CC",
        serial="OTHER-SERIAL",
        rediscovery_attempts=1,
        rediscovery_delay_s=0,
    )

    assert not result.ok
    assert not provision_called
    assert result.error is not None
    assert "No BLE robot matched" in result.error


async def test_setup_redacts_password_from_failure_result_and_summary() -> None:
    secret = "super-secret-password"

    async def discover_ble() -> list[_BleRobot]:
        return [_BleRobot(name="Go2_123", address="AA:BB:CC", serial="SN123")]

    async def provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
    ) -> str | None:
        raise RuntimeError(f"provision failed with password {password}")

    async def discover_lan() -> list[_LanRobot]:
        return []

    result = await setup_go2_wifi(
        ssid="LabWiFi",
        password=secret,
        discover_ble=discover_ble,
        provision_wifi=provision_wifi,
        discover_lan=discover_lan,
        rediscovery_attempts=1,
        rediscovery_delay_s=0,
    )

    assert not result.ok
    assert secret not in repr(result)
    assert secret not in "\n".join(result.summary_lines())
    assert result.error is not None
    assert "[REDACTED]" in result.error
