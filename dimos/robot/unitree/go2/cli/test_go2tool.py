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

import asyncio
from collections.abc import AsyncIterator, Callable
from pathlib import Path

import pytest
from typer.testing import CliRunner

from dimos.robot.unitree.go2.cli import (
    ble,
    go2tool,
    landiscovery,
    macos_ble_helper,
    setup,
    verify as verify_module,
)
from dimos.robot.unitree.go2.cli.verify import Go2VerifyStatus

_runner = CliRunner()


def _verify_status(ip: str, *, ok: bool) -> Go2VerifyStatus:
    return Go2VerifyStatus(
        robot_ip=ip,
        url=f"http://{ip}:9991/con_notify",
        ok=ok,
        status_code=200 if ok else 503,
        reason="OK" if ok else "Unavailable",
        error=None,
        elapsed_s=0.01,
        timeout_s=1.0,
    )


def test_auto_backend_uses_helper_on_darwin(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    helper_path = tmp_path / "DimOS BLE Helper.app"
    calls: list[tuple[str, Path | None]] = []

    async def fake_helper_provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
        *,
        timeout: float,
        connect_retries: int,
        helper_app_path: Path | None = None,
        on_progress: Callable[[str], None] | None = None,
    ) -> str | None:
        assert address == "AA:BB:CC:DD:EE:FF"
        assert ssid == "lab-wifi"
        assert password == "secret"
        assert country_code == "US"
        assert timeout == 3.0
        assert connect_retries == go2tool._DEFAULT_BLE_CONNECT_RETRIES
        assert on_progress is not None
        calls.append(("helper", helper_app_path))
        return "SER123"

    async def fake_direct_provision_wifi(*args: object, **kwargs: object) -> str | None:
        pytest.fail("direct BLE backend should not be used on Darwin auto")

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(macos_ble_helper, "provision_wifi", fake_helper_provision_wifi)
    monkeypatch.setattr(ble, "provision_wifi", fake_direct_provision_wifi)

    result = _runner.invoke(
        go2tool.app,
        [
            "connect-wifi",
            "--ssid",
            "lab-wifi",
            "--password",
            "secret",
            "--mac",
            "AA:BB:CC:DD:EE:FF",
            "--timeout",
            "3",
            "--retries",
            "1",
            "--ble-backend",
            "auto",
            "--ble-helper",
            str(helper_path),
        ],
    )

    assert result.exit_code == 0, result.output
    assert calls == [("helper", helper_path)]


def test_auto_backend_uses_direct_on_linux(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[str] = []

    async def fake_direct_provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
        *,
        timeout: float,
        connect_retries: int,
        on_progress: Callable[[str], None] | None = None,
    ) -> str | None:
        assert address == "AA:BB:CC:DD:EE:FF"
        assert ssid == "lab-wifi"
        assert password == "secret"
        assert country_code == "US"
        assert timeout == 3.0
        assert connect_retries == go2tool._DEFAULT_BLE_CONNECT_RETRIES
        assert on_progress is not None
        calls.append("direct")
        return "SER123"

    async def fake_helper_provision_wifi(*args: object, **kwargs: object) -> str | None:
        pytest.fail("helper BLE backend should not be used on Linux auto")

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Linux")
    monkeypatch.setattr(ble, "provision_wifi", fake_direct_provision_wifi)
    monkeypatch.setattr(macos_ble_helper, "provision_wifi", fake_helper_provision_wifi)

    result = _runner.invoke(
        go2tool.app,
        [
            "connect-wifi",
            "--ssid",
            "lab-wifi",
            "--password",
            "secret",
            "--mac",
            "AA:BB:CC:DD:EE:FF",
            "--timeout",
            "3",
            "--retries",
            "1",
            "--ble-backend",
            "auto",
        ],
    )

    assert result.exit_code == 0, result.output
    assert calls == ["direct"]


def test_discover_lan_bypasses_ble_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    async def fake_discover_lan(
        tick: float = 2.0,
        timeout: float = 1.5,
        iface_ip: str | None = None,
    ) -> AsyncIterator[landiscovery.Go2Device]:
        assert tick == 0.0
        assert timeout == 1.5
        assert iface_ip is None
        yield landiscovery.Go2Device(
            serial="SN123",
            ip="192.168.1.70",
            iface="en0",
            mac="00:11:22:33:44:55",
        )
        while True:
            await asyncio.sleep(1)

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Linux")
    monkeypatch.setattr(landiscovery, "discover_lan", fake_discover_lan)

    result = _runner.invoke(
        go2tool.app,
        [
            "discover",
            "--lan",
            "--ble-backend",
            "helper",
            "--timeout",
            "0.01",
            "--lan-tick",
            "0",
        ],
    )

    assert result.exit_code == 0, result.output
    assert "LAN" in result.output
    assert "192.168.1.70" in result.output


def test_helper_backend_rejects_timeout_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(go2tool.platform, "system", lambda: "Darwin")

    result = _runner.invoke(
        go2tool.app,
        ["discover", "--ble", "--ble-backend", "helper", "--timeout", "0"],
    )

    assert result.exit_code == 1
    assert "BLE helper backend cannot run with --timeout 0" in result.output


def test_discover_auto_timeout_zero_uses_direct_ble_on_darwin(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    async def fake_discover_ble() -> AsyncIterator[ble.Go2Device]:
        yield ble.Go2Device(name="Go2_49060", address="AA:BB:CC:DD:EE:FF", serial="SER123")

    async def fake_helper_find_robots(*args: object, **kwargs: object) -> list[ble.Go2Device]:
        pytest.fail("macOS auto discovery with --timeout 0 should use direct BLE")

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(ble, "discover_ble", fake_discover_ble)
    monkeypatch.setattr(macos_ble_helper, "find_robots", fake_helper_find_robots)

    result = _runner.invoke(
        go2tool.app,
        ["discover", "--ble", "--ble-backend", "auto", "--timeout", "0"],
    )

    assert result.exit_code == 0, result.output
    assert "AA:BB:CC:DD:EE:FF" in result.output
    assert "SER123" in result.output


def test_discover_surfaces_ble_backend_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    async def fake_find_robots(*args: object, **kwargs: object) -> list[ble.Go2Device]:
        raise RuntimeError("invalid helper app")

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(macos_ble_helper, "find_robots", fake_find_robots)

    result = _runner.invoke(
        go2tool.app,
        ["discover", "--ble", "--ble-backend", "helper", "--timeout", "0.1"],
    )

    assert result.exit_code == 1
    assert "invalid helper app" in result.output


def test_connect_wifi_mac_skips_scan_and_redacts_prompted_password(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    secret = "super-secret-password"
    scan_called = False

    async def fake_find_robots(*args: object, **kwargs: object) -> list[ble.Go2Device]:
        nonlocal scan_called
        scan_called = True
        return []

    async def fake_provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
        *,
        timeout: float,
        connect_retries: int,
        on_progress: Callable[[str], None] | None = None,
    ) -> str | None:
        assert address == "AA:BB:CC:DD:EE:FF"
        assert ssid == "lab-wifi"
        assert password == secret
        assert country_code == "US"
        assert timeout == 2.0
        assert connect_retries == go2tool._DEFAULT_BLE_CONNECT_RETRIES
        assert on_progress is not None
        on_progress(f"unsafe progress {secret}")
        return "SER123"

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Linux")
    monkeypatch.setattr(ble, "find_robots", fake_find_robots)
    monkeypatch.setattr(ble, "provision_wifi", fake_provision_wifi)

    result = _runner.invoke(
        go2tool.app,
        [
            "connect-wifi",
            "--ssid",
            "lab-wifi",
            "--mac",
            "AA:BB:CC:DD:EE:FF",
            "--timeout",
            "2",
            "--retries",
            "1",
            "--ble-backend",
            "direct",
        ],
        input=f"{secret}\n",
    )

    assert result.exit_code == 0, result.output
    assert not scan_called
    assert secret not in result.output
    assert "[REDACTED]" in result.output
    assert "Provisioned. Serial: SER123" in result.output


def test_connect_wifi_retries_outer_provision_attempts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    connect_retry_counts: list[int] = []

    async def fake_provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
        *,
        timeout: float,
        connect_retries: int,
        on_progress: Callable[[str], None] | None = None,
    ) -> str | None:
        assert address == "AA:BB:CC:DD:EE:FF"
        assert ssid == "lab-wifi"
        assert password == "secret"
        assert country_code == "US"
        assert timeout == 2.0
        connect_retry_counts.append(connect_retries)
        if len(connect_retry_counts) == 1:
            raise RuntimeError("temporary provisioning failure")
        return "SER123"

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Linux")
    monkeypatch.setattr(ble, "provision_wifi", fake_provision_wifi)

    result = _runner.invoke(
        go2tool.app,
        [
            "connect-wifi",
            "--ssid",
            "lab-wifi",
            "--password",
            "secret",
            "--mac",
            "AA:BB:CC:DD:EE:FF",
            "--timeout",
            "2",
            "--retries",
            "2",
            "--ble-backend",
            "direct",
        ],
    )

    assert result.exit_code == 0, result.output
    assert connect_retry_counts == [
        go2tool._DEFAULT_BLE_CONNECT_RETRIES,
        go2tool._DEFAULT_BLE_CONNECT_RETRIES,
    ]
    assert "attempt 1 failed: temporary provisioning failure" in result.output
    assert "Provisioned. Serial: SER123" in result.output


@pytest.mark.parametrize(("ok", "expected_exit_code"), [(True, 0), (False, 1)])
def test_verify_exit_codes(
    monkeypatch: pytest.MonkeyPatch,
    ok: bool,
    expected_exit_code: int,
) -> None:
    def fake_verify_robot_ip(robot_ip: str, *, timeout_s: float) -> Go2VerifyStatus:
        assert robot_ip == "192.168.1.70"
        assert timeout_s == 0.25
        return _verify_status(robot_ip, ok=ok)

    monkeypatch.setattr(verify_module, "verify_robot_ip", fake_verify_robot_ip)

    result = _runner.invoke(
        go2tool.app,
        ["verify", "--robot-ip", "192.168.1.70", "--timeout", "0.25"],
    )

    assert result.exit_code == expected_exit_code
    assert "192.168.1.70" in result.output


def test_setup_success_prints_summary_and_redacts_secret(monkeypatch: pytest.MonkeyPatch) -> None:
    secret = "super-secret-password"

    async def fake_setup_go2_wifi(**kwargs: object) -> setup.Go2WifiSetupResult:
        assert kwargs["ssid"] == "lab-wifi"
        assert kwargs["password"] == secret
        assert kwargs["address"] == "AA:BB:CC:DD:EE:FF"
        return setup.Go2WifiSetupResult(
            ok=True,
            selected_ble=setup.Go2BleSelection(
                name="Go2_123",
                address="AA:BB:CC:DD:EE:FF",
                serial="SN123",
            ),
            lan_robot=setup.Go2LanSelection(
                serial="SN123",
                ip="192.168.1.70",
                mac="00:11:22:33:44:55",
                iface="en0",
            ),
            verification=_verify_status("192.168.1.70", ok=True),
            next_steps=(
                "export ROBOT_IP=192.168.1.70",
                f"redacted next step {secret}",
            ),
            steps=(
                setup.SetupStep(
                    name="provision_wifi",
                    ok=True,
                    message=f"unsafe step {secret}",
                ),
            ),
        )

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Linux")
    monkeypatch.setattr(setup, "setup_go2_wifi", fake_setup_go2_wifi)

    result = _runner.invoke(
        go2tool.app,
        [
            "setup",
            "--ssid",
            "lab-wifi",
            "--password",
            secret,
            "--mac",
            "AA:BB:CC:DD:EE:FF",
            "--ble-backend",
            "direct",
        ],
    )

    assert result.exit_code == 0, result.output
    assert "result=ok robot_ip=192.168.1.70 serial=SN123" in result.output
    assert "export ROBOT_IP=192.168.1.70" in result.output
    assert secret not in result.output
    assert "[REDACTED]" in result.output


def test_setup_retries_outer_provision_attempts(monkeypatch: pytest.MonkeyPatch) -> None:
    connect_retry_counts: list[int] = []

    async def fake_provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
        *,
        timeout: float,
        connect_retries: int,
        on_progress: Callable[[str], None] | None = None,
    ) -> str | None:
        assert address == "AA:BB:CC:DD:EE:FF"
        assert ssid == "lab-wifi"
        assert password == "secret"
        assert country_code == "US"
        assert timeout == 2.0
        connect_retry_counts.append(connect_retries)
        if len(connect_retry_counts) == 1:
            raise RuntimeError("temporary provisioning failure")
        return "SER123"

    def fake_discover_lan(timeout: float = 2.0) -> list[landiscovery.Go2Device]:
        assert timeout == 0.1
        return [
            landiscovery.Go2Device(
                serial="SER123",
                ip="192.168.1.70",
                iface="en0",
                mac="00:11:22:33:44:55",
            )
        ]

    def fake_verify_robot_ip(robot_ip: str, *, timeout_s: float) -> Go2VerifyStatus:
        assert robot_ip == "192.168.1.70"
        assert timeout_s == 0.25
        return _verify_status(robot_ip, ok=True)

    monkeypatch.setattr(go2tool.platform, "system", lambda: "Linux")
    monkeypatch.setattr(ble, "provision_wifi", fake_provision_wifi)
    monkeypatch.setattr(landiscovery, "discover", fake_discover_lan)
    monkeypatch.setattr(verify_module, "verify_robot_ip", fake_verify_robot_ip)

    result = _runner.invoke(
        go2tool.app,
        [
            "setup",
            "--ssid",
            "lab-wifi",
            "--password",
            "secret",
            "--mac",
            "AA:BB:CC:DD:EE:FF",
            "--serial",
            "SER123",
            "--timeout",
            "2",
            "--retries",
            "2",
            "--lan-timeout",
            "0.1",
            "--rediscovery-attempts",
            "1",
            "--rediscovery-delay",
            "0",
            "--verify-timeout",
            "0.25",
            "--ble-backend",
            "direct",
        ],
    )

    assert result.exit_code == 0, result.output
    assert connect_retry_counts == [
        go2tool._DEFAULT_BLE_CONNECT_RETRIES,
        go2tool._DEFAULT_BLE_CONNECT_RETRIES,
    ]
    assert "attempt 1 failed: temporary provisioning failure" in result.output
    assert "result=ok robot_ip=192.168.1.70 serial=SER123" in result.output
