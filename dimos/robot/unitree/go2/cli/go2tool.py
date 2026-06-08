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

"""`dimos go2tool` — Go2 setup utilities (BLE wifi provisioning, network discovery)."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Awaitable, Callable, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import platform
from typing import Literal, NoReturn, Protocol, cast

import typer

from dimos.robot.unitree.go2.cli.ble import Go2Device, retry

app = typer.Typer(
    help="Go2 setup utilities (BLE wifi provisioning, network discovery)",
    no_args_is_help=True,
)


_HEADER = f"{'SOURCE':<6} {'NAME':<14} {'IP':<15} {'MAC':<19} SERIAL"
_HELPER_TIMEOUT_ERROR = (
    "BLE helper backend cannot run with --timeout 0. Use --timeout > 0 or --ble-backend direct."
)
_DEFAULT_BLE_CONNECT_RETRIES = 3


class BleBackend(str, Enum):
    auto = "auto"
    helper = "helper"
    direct = "direct"


class _BleFinder(Protocol):
    def __call__(
        self,
        *,
        timeout: float,
        on_device: Callable[[Go2Device], None] | None = None,
    ) -> Awaitable[list[Go2Device]]: ...


class _BleProvisioner(Protocol):
    def __call__(
        self,
        address: str,
        ssid: str,
        password: str,
        country_code: str,
        *,
        timeout: float,
        connect_retries: int,
        on_progress: Callable[[str], None] | None = None,
    ) -> Awaitable[str | None]: ...


@dataclass(frozen=True)
class _BleBackendRuntime:
    kind: Literal["helper", "direct"]
    find_robots: _BleFinder
    provision_wifi: _BleProvisioner
    discover_ble: Callable[[], AsyncIterator[Go2Device]] | None = None


class _Go2ToolError(RuntimeError):
    """User-facing CLI configuration error."""


def _format_row(source: str, name: str, ip: str, mac: str, serial: str) -> str:
    return f"{source:<6} {name:<14} {ip:<15} {mac:<19} {serial}"


def _selected_backend_kind(backend: BleBackend) -> Literal["helper", "direct"]:
    if backend == BleBackend.auto:
        return "helper" if platform.system() == "Darwin" else "direct"
    if backend == BleBackend.helper:
        if platform.system() != "Darwin":
            raise _Go2ToolError(
                "BLE helper backend is only supported on macOS. "
                "Use --ble-backend direct on this platform."
            )
        return "helper"
    return "direct"


def _select_ble_backend(
    backend: BleBackend,
    helper_app_path: Path | None,
) -> _BleBackendRuntime:
    kind = _selected_backend_kind(backend)

    if kind == "helper":
        from dimos.robot.unitree.go2.cli import macos_ble_helper

        async def helper_find_robots(
            *,
            timeout: float,
            on_device: Callable[[Go2Device], None] | None = None,
        ) -> list[Go2Device]:
            return await macos_ble_helper.find_robots(
                timeout=timeout,
                helper_app_path=helper_app_path,
                on_device=on_device,
            )

        async def helper_provision_wifi(
            address: str,
            ssid: str,
            password: str,
            country_code: str,
            *,
            timeout: float,
            connect_retries: int,
            on_progress: Callable[[str], None] | None = None,
        ) -> str | None:
            return await macos_ble_helper.provision_wifi(
                address,
                ssid,
                password,
                country_code,
                timeout=timeout,
                connect_retries=connect_retries,
                helper_app_path=helper_app_path,
                on_progress=on_progress,
            )

        return _BleBackendRuntime(
            kind="helper",
            find_robots=helper_find_robots,
            provision_wifi=helper_provision_wifi,
        )

    from dimos.robot.unitree.go2.cli import ble as direct_ble

    async def direct_find_robots(
        *,
        timeout: float,
        on_device: Callable[[Go2Device], None] | None = None,
    ) -> list[Go2Device]:
        return await direct_ble.find_robots(timeout=timeout, on_device=on_device)

    async def direct_provision_wifi(
        address: str,
        ssid: str,
        password: str,
        country_code: str,
        *,
        timeout: float,
        connect_retries: int,
        on_progress: Callable[[str], None] | None = None,
    ) -> str | None:
        return await direct_ble.provision_wifi(
            address,
            ssid,
            password,
            country_code,
            timeout=timeout,
            connect_retries=connect_retries,
            on_progress=on_progress,
        )

    async def discover_ble() -> AsyncIterator[Go2Device]:
        async for device in direct_ble.discover_ble():
            yield device

    return _BleBackendRuntime(
        kind="direct",
        find_robots=direct_find_robots,
        provision_wifi=direct_provision_wifi,
        discover_ble=discover_ble,
    )


def _exit_error(message: str) -> NoReturn:
    typer.echo(message, err=True)
    raise typer.Exit(1)


def _redact(text: str, secrets: Sequence[str]) -> str:
    safe = text
    for secret in secrets:
        if secret:
            safe = safe.replace(secret, "[REDACTED]")
    return safe


def _echo_safe(message: str, secrets: Sequence[str], *, err: bool = False) -> None:
    typer.echo(_redact(message, secrets), err=err)


def _require_helper_timeout(backend: _BleBackendRuntime, timeout: float) -> None:
    if backend.kind == "helper" and timeout <= 0:
        _exit_error(_HELPER_TIMEOUT_ERROR)


def _discover_backend_for_timeout(backend: BleBackend, timeout: float) -> BleBackend:
    # The LaunchServices helper is one-shot; unbounded discovery needs direct BLE streaming.
    if backend == BleBackend.auto and timeout <= 0 and platform.system() == "Darwin":
        return BleBackend.direct
    return backend


@app.command("discover")
def discover(
    ble: bool = typer.Option(False, "--ble", help="BLE only (default: BLE + LAN)"),
    lan: bool = typer.Option(False, "--lan", help="LAN only (default: BLE + LAN)"),
    lan_tick: float = typer.Option(2.0, "--lan-tick", help="LAN poll interval (s)"),
    timeout: float = typer.Option(
        7.0, "--timeout", "-t", help="Stop after this many seconds (0 = run forever)"
    ),
    ble_backend: BleBackend = typer.Option(
        BleBackend.auto,
        "--ble-backend",
        help="BLE backend to use. On macOS, auto uses direct BLE when --timeout 0.",
    ),
    ble_helper: Path | None = typer.Option(
        None,
        "--ble-helper",
        envvar="DIMOS_GO2_BLE_HELPER",
        help="macOS BLE helper .app path.",
    ),
) -> None:
    """Stream Go2 robot discoveries from BLE and/or LAN."""
    do_ble = ble or not lan
    do_lan = lan or not ble
    backend_runtime: _BleBackendRuntime | None = None
    if do_ble:
        try:
            backend_runtime = _select_ble_backend(
                _discover_backend_for_timeout(ble_backend, timeout), ble_helper
            )
        except _Go2ToolError as e:
            _exit_error(str(e))
        _require_helper_timeout(backend_runtime, timeout)

    typer.echo(_HEADER)

    import signal

    async def run() -> None:
        from dimos.robot.unitree.go2.cli.landiscovery import discover_lan

        seen_ble: set[tuple[str, str | None]] = set()
        seen_lan: set[str] = set()

        def _emit_ble_device(d: Go2Device) -> None:
            key = (d.address, d.serial)
            if key in seen_ble:
                return
            seen_ble.add(key)
            typer.echo(_format_row("BLE", d.name, "-", d.address, d.serial or "?"))

        async def _consume_ble() -> None:
            assert backend_runtime is not None
            if backend_runtime.kind == "helper":
                devices = await backend_runtime.find_robots(timeout=timeout)
                for d in devices:
                    _emit_ble_device(d)
                return

            assert backend_runtime.discover_ble is not None
            async for d in backend_runtime.discover_ble():
                _emit_ble_device(d)

        async def _consume_lan() -> None:
            async for d in discover_lan(tick=lan_tick):
                if d.serial in seen_lan:
                    continue
                seen_lan.add(d.serial)
                typer.echo(_format_row("LAN", "-", d.ip, d.mac or "-", d.serial))

        tasks: list[asyncio.Task[None]] = []
        if do_ble:
            tasks.append(asyncio.create_task(_consume_ble()))
        if do_lan:
            tasks.append(asyncio.create_task(_consume_lan()))

        # Cancel on SIGINT or SIGTERM so the BleakScanner's __aexit__ runs and
        # calls StopDiscovery; otherwise BlueZ retains the scan session.
        loop = asyncio.get_running_loop()

        def _stop() -> None:
            for t in tasks:
                t.cancel()

        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, _stop)

        if timeout > 0:
            grace = 0.25 if backend_runtime is not None and backend_runtime.kind == "helper" else 0
            loop.call_later(timeout + grace, _stop)

        await asyncio.gather(*tasks, return_exceptions=True)

    asyncio.run(run())
    typer.echo("\nStopped.")


@app.command("connect-wifi")
def connect_wifi(
    ssid: str | None = typer.Option(None, "--ssid", help="Wi-Fi SSID"),
    password: str | None = typer.Option(None, "--password", help="Wi-Fi password"),
    country: str = typer.Option("US", "--country", help="Two-letter country code"),
    mac: str | None = typer.Option(None, "--mac", help="BLE MAC (skip scan)"),
    serial: str | None = typer.Option(
        None, "--serial", help="Robot serial — scan and auto-select match"
    ),
    name: str | None = typer.Option(
        None, "--name", help="Robot BLE name (e.g. Go2_49060) — scan and auto-select match"
    ),
    timeout: float = typer.Option(5.0, "--timeout", help="Scan / connect timeout in seconds"),
    retries: int = typer.Option(
        3,
        "--retries",
        help="Number of provisioning attempts; BLE connection retries stay at 3",
    ),
    ble_backend: BleBackend = typer.Option(
        BleBackend.auto,
        "--ble-backend",
        help="BLE backend to use.",
    ),
    ble_helper: Path | None = typer.Option(
        None,
        "--ble-helper",
        envvar="DIMOS_GO2_BLE_HELPER",
        help="macOS BLE helper .app path.",
    ),
) -> None:
    """Provision a Go2 with Wi-Fi credentials over Bluetooth.

    Fully non-interactive when (--mac | --serial | --name) and --ssid/--password
    are all provided.
    """
    try:
        backend_runtime = _select_ble_backend(ble_backend, ble_helper)
    except _Go2ToolError as e:
        _exit_error(str(e))
    _require_helper_timeout(backend_runtime, timeout)

    async def run() -> None:
        target: str
        if mac is not None:
            target = mac
        else:
            typer.echo(f"Scanning BLE for {timeout:.0f}s ...")

            def _on_device(d: Go2Device) -> None:
                typer.echo(_format_row("BLE", d.name, "-", d.address, d.serial or "?"))

            devices = await backend_runtime.find_robots(timeout=timeout, on_device=_on_device)
            if not devices:
                typer.echo("No Unitree robots detected.", err=True)
                raise typer.Exit(1)

            if serial is not None or name is not None:
                matches = [
                    d
                    for d in devices
                    if (serial is None or d.serial == serial) and (name is None or d.name == name)
                ]
                if not matches:
                    crit = ", ".join(
                        f"{k}={v}" for k, v in (("serial", serial), ("name", name)) if v is not None
                    )
                    typer.echo(f"No BLE device matched {crit}.", err=True)
                    raise typer.Exit(1)
                if len(matches) > 1:
                    typer.echo(f"{len(matches)} devices match — refine criteria:", err=True)
                    for d in matches:
                        typer.echo(f"  {d.name} {d.address} serial={d.serial}", err=True)
                    raise typer.Exit(1)
                target = matches[0].address
            else:
                typer.echo("")
                typer.echo("Found:")
                for i, d in enumerate(devices, 1):
                    typer.echo(f"  {i}. {d.name} ({d.address})")
                default = "1" if len(devices) == 1 else None
                idx = typer.prompt("Select device", default=default, type=int)
                if not 1 <= idx <= len(devices):
                    typer.echo("Invalid selection.", err=True)
                    raise typer.Exit(1)
                target = devices[idx - 1].address

        wifi_ssid = ssid if ssid is not None else typer.prompt("Wi-Fi SSID")
        wifi_password = (
            password
            if password is not None
            else typer.prompt("Wi-Fi password", hide_input=True, default="", show_default=False)
        )

        def _on_error(attempt: int, exc: BaseException) -> None:
            _echo_safe(f"  attempt {attempt} failed: {exc}", (wifi_password,), err=True)

        device_serial = await retry(
            lambda: backend_runtime.provision_wifi(
                target,
                wifi_ssid,
                wifi_password,
                country,
                timeout=timeout,
                connect_retries=_DEFAULT_BLE_CONNECT_RETRIES,
                on_progress=lambda m: _echo_safe(f"  {m}", (wifi_password,)),
            ),
            attempts=retries,
            on_error=_on_error,
        )

        if device_serial:
            typer.echo(f"✓ Provisioned. Serial: {device_serial}")
        else:
            typer.echo("✓ Provisioned.")

    asyncio.run(run())


@app.command("verify")
def verify(
    robot_ip: str = typer.Option(..., "--robot-ip", help="Robot IP address to verify"),
    timeout: float = typer.Option(1.0, "--timeout", help="HTTP probe timeout in seconds"),
) -> None:
    """Verify a Go2 LAN IP without sending movement commands."""
    from dimos.robot.unitree.go2.cli.verify import verify_robot_ip

    status = verify_robot_ip(robot_ip, timeout_s=timeout)
    for line in status.summary_lines():
        typer.echo(line)
    raise typer.Exit(0 if status.ok else 1)


@app.command("setup")
def setup(
    ssid: str = typer.Option(..., "--ssid", help="Wi-Fi SSID"),
    password: str | None = typer.Option(None, "--password", help="Wi-Fi password"),
    country: str = typer.Option("US", "--country", help="Two-letter country code"),
    serial: str | None = typer.Option(None, "--serial", help="Robot serial to select"),
    name: str | None = typer.Option(None, "--name", help="Robot BLE name to select"),
    mac: str | None = typer.Option(None, "--mac", help="BLE MAC/address to provision"),
    timeout: float = typer.Option(5.0, "--timeout", help="BLE scan / connect timeout in seconds"),
    retries: int = typer.Option(
        3,
        "--retries",
        help="Number of provisioning attempts; BLE connection retries stay at 3",
    ),
    lan_timeout: float = typer.Option(2.0, "--lan-timeout", help="LAN discovery timeout"),
    rediscovery_attempts: int = typer.Option(
        5,
        "--rediscovery-attempts",
        help="LAN rediscovery attempts after provisioning",
    ),
    rediscovery_delay: float = typer.Option(
        2.0,
        "--rediscovery-delay",
        help="Seconds between LAN rediscovery attempts",
    ),
    verify_timeout: float = typer.Option(
        1.0,
        "--verify-timeout",
        help="HTTP verification timeout in seconds",
    ),
    ble_backend: BleBackend = typer.Option(
        BleBackend.auto,
        "--ble-backend",
        help="BLE backend to use.",
    ),
    ble_helper: Path | None = typer.Option(
        None,
        "--ble-helper",
        envvar="DIMOS_GO2_BLE_HELPER",
        help="macOS BLE helper .app path.",
    ),
) -> None:
    """Provision Wi-Fi, rediscover the robot on LAN, and verify its IP."""
    from dimos.robot.unitree.go2.cli.landiscovery import discover as discover_lan_once
    from dimos.robot.unitree.go2.cli.setup import LanRobot, setup_go2_wifi
    from dimos.robot.unitree.go2.cli.verify import verify_robot_ip

    try:
        backend_runtime = _select_ble_backend(ble_backend, ble_helper)
    except _Go2ToolError as e:
        _exit_error(str(e))
    _require_helper_timeout(backend_runtime, timeout)

    wifi_password = (
        password
        if password is not None
        else typer.prompt("Wi-Fi password", hide_input=True, default="", show_default=False)
    )
    secrets = (wifi_password,)

    async def run() -> None:
        if mac is not None:

            async def discover_ble_for_setup() -> list[Go2Device]:
                return [Go2Device(name=name or mac, address=mac, serial=serial)]

        else:

            async def discover_ble_for_setup() -> list[Go2Device]:
                typer.echo(f"Scanning BLE for {timeout:.0f}s ...")
                return await backend_runtime.find_robots(
                    timeout=timeout,
                    on_device=lambda d: typer.echo(
                        _format_row("BLE", d.name, "-", d.address, d.serial or "?")
                    ),
                )

        async def provision_wifi_for_setup(
            address: str,
            provision_ssid: str,
            provision_password: str,
            country_code: str,
        ) -> str | None:
            return cast(
                "str | None",
                await retry(
                    lambda: backend_runtime.provision_wifi(
                        address,
                        provision_ssid,
                        provision_password,
                        country_code,
                        timeout=timeout,
                        connect_retries=_DEFAULT_BLE_CONNECT_RETRIES,
                        on_progress=lambda m: _echo_safe(f"  {m}", secrets),
                    ),
                    attempts=retries,
                    on_error=lambda attempt, exc: _echo_safe(
                        f"  attempt {attempt} failed: {exc}", secrets, err=True
                    ),
                ),
            )

        async def discover_lan_for_setup() -> Sequence[LanRobot]:
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(
                None,
                lambda: discover_lan_once(timeout=lan_timeout),
            )

        result = await setup_go2_wifi(
            ssid=ssid,
            password=wifi_password,
            discover_ble=discover_ble_for_setup,
            provision_wifi=provision_wifi_for_setup,
            discover_lan=discover_lan_for_setup,
            serial=serial,
            name=name,
            address=mac,
            country_code=country,
            verify_robot=lambda ip: verify_robot_ip(ip, timeout_s=verify_timeout),
            rediscovery_attempts=rediscovery_attempts,
            rediscovery_delay_s=rediscovery_delay,
        )

        robot_ip = result.robot_ip or "-"
        selected_serial = "-"
        if result.lan_robot is not None:
            selected_serial = result.lan_robot.serial
        elif result.selected_ble is not None:
            selected_serial = result.selected_ble.serial or "-"
        typer.echo(
            f"result={'ok' if result.ok else 'fail'} robot_ip={robot_ip} serial={selected_serial}"
        )
        for line in result.summary_lines():
            _echo_safe(line, secrets)

        if not result.ok:
            raise typer.Exit(1)

    asyncio.run(run())


def main() -> None:
    app()


if __name__ == "__main__":
    main()
