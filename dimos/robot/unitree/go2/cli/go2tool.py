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

import typer

app = typer.Typer(
    help="Go2 setup utilities (BLE wifi provisioning, network discovery)",
    no_args_is_help=True,
)


_HEADER = f"{'SOURCE':<6} {'NAME':<14} {'IP':<15} {'MAC':<19} SERIAL"


def _format_row(source: str, name: str, ip: str, mac: str, serial: str) -> str:
    return f"{source:<6} {name:<14} {ip:<15} {mac:<19} {serial}"


@app.command("discover")
def discover(
    ble: bool = typer.Option(False, "--ble", help="BLE only (default: BLE + LAN)"),
    lan: bool = typer.Option(False, "--lan", help="LAN only (default: BLE + LAN)"),
    lan_tick: float = typer.Option(2.0, "--lan-tick", help="LAN poll interval (s)"),
    timeout: float = typer.Option(
        7.0, "--timeout", "-t", help="Stop after this many seconds (0 = run forever)"
    ),
) -> None:
    """Stream Go2 robot discoveries from BLE and/or LAN."""
    do_ble = ble or not lan
    do_lan = lan or not ble

    typer.echo(_HEADER)

    import signal

    async def run() -> None:
        from dimos.robot.unitree.go2.cli.ble import discover_ble
        from dimos.robot.unitree.go2.cli.landiscovery import discover_lan

        seen_ble: set[tuple[str, str | None]] = set()
        seen_lan: set[str] = set()

        async def _consume_ble() -> None:
            async for d in discover_ble():
                key = (d.address, d.serial)
                if key in seen_ble:
                    continue
                seen_ble.add(key)
                typer.echo(_format_row("BLE", d.name, "-", d.address, d.serial or "?"))

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
            loop.call_later(timeout, _stop)

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
    retries: int = typer.Option(3, "--retries", help="Number of provisioning attempts"),
) -> None:
    """Provision a Go2 with Wi-Fi credentials over Bluetooth.

    Fully non-interactive when (--mac | --serial | --name) and --ssid/--password
    are all provided.
    """
    from dimos.robot.unitree.go2.cli.ble import (
        Go2Device,
        find_robots,
        provision_wifi,
        retry,
    )

    async def run() -> None:
        if mac is not None:
            target = mac
        else:
            typer.echo(f"Scanning BLE for {timeout:.0f}s ...")

            def _on_device(d: Go2Device) -> None:
                typer.echo(_format_row("BLE", d.name, "-", d.address, d.serial or "?"))

            devices = await find_robots(timeout=timeout, on_device=_on_device)
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
            typer.echo(f"  attempt {attempt} failed: {exc}", err=True)

        device_serial = await retry(
            lambda: provision_wifi(
                target,  # type: ignore[arg-type]
                wifi_ssid,
                wifi_password,
                country,
                on_progress=lambda m: typer.echo(f"  {m}"),
            ),
            attempts=retries,
            on_error=_on_error,
        )

        if device_serial:
            typer.echo(f"✓ Provisioned. Serial: {device_serial}")
        else:
            typer.echo("✓ Provisioned.")

    asyncio.run(run())


def main() -> None:
    app()


if __name__ == "__main__":
    main()
