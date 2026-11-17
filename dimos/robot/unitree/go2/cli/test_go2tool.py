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
from collections.abc import AsyncIterator, Callable, Iterator, Sequence
import signal
from unittest.mock import Mock

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.robot.unitree.go2.cli import ble, go2tool, landiscovery


async def _scan(
    devices: Sequence[ble.Go2Device | landiscovery.Go2Device] = (),
    *,
    error: Exception | None = None,
    on_close: Callable[[], None],
) -> AsyncIterator[ble.Go2Device | landiscovery.Go2Device]:
    try:
        for device in devices:
            yield device
        if error is not None:
            raise error
        await asyncio.Event().wait()
    finally:
        on_close()


@pytest.fixture
def scans(mocker: MockerFixture) -> Iterator[dict[str, Mock]]:
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield {
            "BLE": mocker.patch.object(ble, "discover_ble"),
            "LAN": mocker.patch.object(landiscovery, "discover_lan"),
        }
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


@pytest.mark.parametrize("source", ["BLE", "LAN"])
@pytest.mark.parametrize("combined", [False, True])
@pytest.mark.timeout(5)
def test_discover_reports_scan_errors(
    mocker: MockerFixture, scans: dict[str, Mock], source: str, combined: bool
) -> None:
    closed = mocker.Mock()
    for scan in scans.values():
        scan.return_value = _scan(on_close=closed)
    scans[source].return_value = _scan(error=OSError("scan unavailable"), on_close=closed)
    options = [] if combined else [f"--{source.lower()}"]

    result = CliRunner().invoke(go2tool.app, ["discover", *options, "-t", "0"])

    assert result.exit_code == 1
    assert result.stderr == f"{source} scan failed: scan unavailable\n"
    assert "No robots found." not in result.stdout
    assert "Stopped." not in result.stdout
    assert closed.call_count == (2 if combined else 1)


def test_discover_reports_all_scan_errors(mocker: MockerFixture, scans: dict[str, Mock]) -> None:
    for name, scan in scans.items():
        scan.return_value = _scan(error=OSError(f"{name} unavailable"), on_close=mocker.Mock())

    result = CliRunner().invoke(go2tool.app, ["discover", "-t", "0.01"])

    assert result.exit_code == 1
    assert result.stderr == "BLE scan failed: BLE unavailable\nLAN scan failed: LAN unavailable\n"


@pytest.mark.parametrize("failed_source", ["BLE", "LAN"])
def test_discover_finishes_healthy_scan_after_peer_fails(
    mocker: MockerFixture, scans: dict[str, Mock], failed_source: str
) -> None:
    devices: dict[str, ble.Go2Device | landiscovery.Go2Device] = {
        "BLE": ble.Go2Device(name="Go2_test", address="AA:BB:CC:DD:EE:FF", serial="ble-serial"),
        "LAN": landiscovery.Go2Device(serial="lan-serial", ip="192.0.2.1", iface="test"),
    }
    healthy_source = {"BLE": "LAN", "LAN": "BLE"}[failed_source]
    closed = mocker.Mock()

    async def healthy_scan() -> AsyncIterator[ble.Go2Device | landiscovery.Go2Device]:
        try:
            # Deliver a result on a later loop iteration, after the peer fails.
            ready = asyncio.Event()
            asyncio.get_running_loop().call_later(0, ready.set)
            await ready.wait()
            yield devices[healthy_source]
        finally:
            closed()

    scans[healthy_source].return_value = healthy_scan()
    scans[failed_source].return_value = _scan(
        error=OSError("scan unavailable"), on_close=mocker.Mock()
    )

    result = CliRunner().invoke(go2tool.app, ["discover"])

    assert result.exit_code == 1
    assert result.stderr == f"{failed_source} scan failed: scan unavailable\n"
    assert f"{healthy_source.lower()}-serial" in result.stdout
    assert "No robots found." not in result.stdout
    assert "Scan complete." not in result.stdout
    closed.assert_called_once_with()


def test_discover_reports_failure_after_finding_a_robot(
    mocker: MockerFixture, scans: dict[str, Mock]
) -> None:
    device = ble.Go2Device(name="Go2_test", address="AA:BB:CC:DD:EE:FF", serial="test-serial")
    scans["BLE"].return_value = _scan([device], error=TimeoutError(), on_close=mocker.Mock())

    result = CliRunner().invoke(go2tool.app, ["discover", "--ble", "-t", "0.01"])

    assert result.exit_code == 1
    assert result.stderr == "BLE scan failed: TimeoutError\n"
    assert "test-serial" in result.stdout
    assert "Scan complete." not in result.stdout
    assert "No robots found." not in result.stdout


@pytest.mark.parametrize("options", [[], ["--ble"], ["--lan"]])
def test_discover_reports_no_robots_after_timeout(
    mocker: MockerFixture, scans: dict[str, Mock], options: list[str]
) -> None:
    for scan in scans.values():
        scan.return_value = _scan(on_close=mocker.Mock())

    result = CliRunner().invoke(go2tool.app, ["discover", *options, "-t", "0.01"])

    assert result.exit_code == 0
    assert result.stderr == ""
    assert result.stdout.splitlines()[-1] == "No robots found."
    assert "Stopped." not in result.stdout


def test_discover_prints_devices_and_completes(
    mocker: MockerFixture, scans: dict[str, Mock]
) -> None:
    ble_device = ble.Go2Device(name="Go2_test", address="AA:BB:CC:DD:EE:FF", serial="ble-serial")
    lan_device = landiscovery.Go2Device(serial="lan-serial", ip="192.0.2.1", iface="test")
    scans["BLE"].return_value = _scan([ble_device, ble_device], on_close=mocker.Mock())
    scans["LAN"].return_value = _scan([lan_device, lan_device], on_close=mocker.Mock())

    result = CliRunner().invoke(go2tool.app, ["discover", "-t", "0.01"])

    assert result.exit_code == 0
    assert result.stderr == ""
    assert result.stdout.count("ble-serial") == 1
    assert result.stdout.count("lan-serial") == 1
    assert "Go2_test" in result.stdout
    assert "192.0.2.1" in result.stdout
    assert result.stdout.splitlines()[-1] == "Scan complete."
    assert "No robots found." not in result.stdout
    assert "Stopped." not in result.stdout


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_discover_reports_interruption_and_closes_scanners(
    mocker: MockerFixture, scans: dict[str, Mock], sig: signal.Signals
) -> None:
    closed = mocker.Mock()

    async def interrupt() -> AsyncIterator[ble.Go2Device]:
        signal.raise_signal(sig)
        await asyncio.Event().wait()
        yield ble.Go2Device(name="unreachable", address="unreachable")

    scans["BLE"].return_value = interrupt()
    scans["LAN"].return_value = _scan(on_close=closed)

    result = CliRunner().invoke(go2tool.app, ["discover", "-t", "0"])

    assert result.exit_code == 0
    assert result.stderr == ""
    assert result.stdout.splitlines()[-1] == "Stopped."
    assert "No robots found." not in result.stdout
    closed.assert_called_once_with()


def test_discover_lets_scanner_finish_closing_after_second_interrupt(
    mocker: MockerFixture, scans: dict[str, Mock]
) -> None:
    closed = mocker.Mock()

    async def slow_to_close() -> AsyncIterator[ble.Go2Device]:
        signal.raise_signal(signal.SIGINT)
        try:
            await asyncio.Event().wait()
            yield ble.Go2Device(name="unreachable", address="unreachable")
        finally:
            # Closing takes a while, like BlueZ StopDiscovery, and Ctrl-C comes again.
            signal.raise_signal(signal.SIGINT)
            await asyncio.sleep(0.01)
            closed()

    scans["BLE"].return_value = slow_to_close()

    result = CliRunner().invoke(go2tool.app, ["discover", "--ble", "-t", "0"])

    assert result.exit_code == 0
    assert result.stdout.splitlines()[-1] == "Stopped."
    closed.assert_called_once_with()
