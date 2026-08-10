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

"""Diagnostics and host configuration for the Galaxea A1Z."""

from __future__ import annotations

from collections.abc import Callable
import ctypes.util
import importlib
import inspect
from pathlib import Path
import platform
import shutil
import subprocess
import time
from typing import Any

import typer

app = typer.Typer(help="Galaxea A1Z robot commands")

_USB_VENDOR_ID = "a8fa"
_USB_PRODUCT_ID = "8598"
_DEFAULT_CAN_INTERFACE = "a1zcan"
_DEFAULT_CAN_BITRATE = 1_000_000
_CAN_PROBE_FRAME = "1FFFFFFF#"
_CAN_PROBE_ATTEMPTS = 20
_CAN_PROBE_POLL_SECONDS = 0.05
_SYS_USB_DEVICES = Path("/sys/bus/usb/devices")
_SYS_CLASS_NET = Path("/sys/class/net")
_GS_USB_NEW_ID = Path("/sys/bus/usb/drivers/gs_usb/new_id")
_A1Z_GUIDE = (
    "https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/manipulation/a1z.md"
)
_Check = tuple[str, Callable[[], str]]


def _abort(message: str) -> None:
    typer.echo(f"ERROR: {message}", err=True)
    raise typer.Exit(1)


def _verify_sdk() -> str:
    """Return the installed SDK path or raise with actionable instructions."""
    try:
        a1z = importlib.import_module("a1z")
        get_robot_module = importlib.import_module("a1z.robots.get_robot")
        get_a1z_robot = get_robot_module.get_a1z_robot
        parameters = inspect.signature(get_a1z_robot).parameters
    except Exception as exc:
        raise RuntimeError(
            f"A1Z SDK unavailable: {exc}. Install the pinned SDK listed in {_A1Z_GUIDE}."
        ) from exc
    if "with_gripper" not in parameters:
        raise RuntimeError(
            "installed A1Z SDK lacks get_a1z_robot(with_gripper=...). Install the pinned "
            f"gripper-capable SDK listed in {_A1Z_GUIDE}."
        )
    return str(a1z.__file__)


def _verify_adapter_import() -> str:
    try:
        adapter = importlib.import_module("dimos.hardware.manipulators.galaxea_a1z.adapter")
    except Exception as exc:
        raise RuntimeError(
            f"A1Z adapter unavailable: {exc}. Install DimOS manipulation dependencies as "
            f"described in {_A1Z_GUIDE}."
        ) from exc
    return str(adapter.__file__)


def _verify_linux_dependencies() -> str:
    cansend = shutil.which("cansend")
    if cansend is None:
        raise RuntimeError(
            "`cansend` is missing. Install your distribution's can-utils package "
            "(`sudo apt-get install can-utils` on Ubuntu)."
        )
    return cansend


def _macos_usb_modules() -> tuple[Any, Any, Any]:
    try:
        usb_backend = importlib.import_module("usb.backend.libusb1")
        usb_core = importlib.import_module("usb.core")
        importlib.import_module("gs_usb.gs_usb")
        gs_usb_module = importlib.import_module(
            "dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus"
        )
    except Exception as exc:
        raise RuntimeError(
            f"macOS USB-CAN dependencies unavailable: {exc}. Install pyusb and gs-usb as "
            f"described in {_A1Z_GUIDE}."
        ) from exc
    return usb_backend, usb_core, gs_usb_module


def _verify_macos_dependencies() -> str:
    if ctypes.util.find_library("usb-1.0") is None:
        raise RuntimeError("libusb is missing. Install it with `brew install libusb`.")
    usb_backend, _, _ = _macos_usb_modules()
    if usb_backend.get_backend() is None:
        raise RuntimeError("PyUSB could not load libusb. Install it with `brew install libusb`.")
    return "pyusb, gs-usb, and libusb"


def _find_hhs_usb_device() -> Path | None:
    for device in sorted(_SYS_USB_DEVICES.glob("*")):
        try:
            vendor = (device / "idVendor").read_text().strip().lower()
            product = (device / "idProduct").read_text().strip().lower()
        except OSError:
            continue
        if vendor == _USB_VENDOR_ID and product == _USB_PRODUCT_ID:
            return device
    return None


def _find_can_interface(usb_device: Path) -> str | None:
    interfaces = sorted(usb_device.parent.glob(f"{usb_device.name}:*/net/*"))
    return interfaces[0].name if interfaces else None


def _run_privileged(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["sudo", *command], check=True, text=True, **kwargs)


def _read_can_counter(interface: str, counter: str) -> int:
    return int((_SYS_CLASS_NET / interface / "statistics" / counter).read_text())


def _usb_bulk_out_endpoint(usb_device: Path) -> str:
    for endpoint in sorted(usb_device.parent.glob(f"{usb_device.name}:*/ep_*")):
        try:
            if (endpoint / "direction").read_text().strip() == "out" and (
                endpoint / "type"
            ).read_text().strip() == "Bulk":
                return f"0x{(endpoint / 'bEndpointAddress').read_text().strip()}"
        except OSError:
            continue
    return "unknown"


def _verify_can_transmit(interface: str, usb_device: Path) -> None:
    if shutil.which("cansend") is None:
        raise RuntimeError(
            "`cansend` is required to prove that the HHS adapter can transmit. "
            "Install your distribution's can-utils package."
        )

    tx_before = _read_can_counter(interface, "tx_packets")
    dropped_before = _read_can_counter(interface, "tx_dropped")
    result = subprocess.run(
        ["cansend", interface, _CAN_PROBE_FRAME],
        check=False,
        capture_output=True,
        text=True,
    )
    tx_after = tx_before
    dropped_after = dropped_before
    for _ in range(_CAN_PROBE_ATTEMPTS):
        tx_after = _read_can_counter(interface, "tx_packets")
        dropped_after = _read_can_counter(interface, "tx_dropped")
        if tx_after > tx_before or dropped_after > dropped_before:
            break
        time.sleep(_CAN_PROBE_POLL_SECONDS)

    diagnostics = (
        f"interface={interface}, kernel={platform.release()}, "
        f"USB OUT={_usb_bulk_out_endpoint(usb_device)}, "
        f"tx_packets={tx_before}->{tx_after}, "
        f"tx_dropped={dropped_before}->{dropped_after}, "
        f"cansend={result.stderr.strip() or result.stdout.strip() or result.returncode}"
    )
    if dropped_after > dropped_before:
        raise RuntimeError(
            "the Linux gs_usb driver rejected transmission through the HHS adapter "
            f"({diagnostics}). See the kernel and Jetson remediation guide in {_A1Z_GUIDE}."
        )
    if tx_after <= tx_before:
        raise RuntimeError(
            "the CAN interface did not complete a transmission "
            f"({diagnostics}). Check arm power, cabling, termination, and competing processes."
        )


def _configure_linux_can(interface: str, bitrate: int) -> None:
    _run_privileged(["modprobe", "gs_usb"])
    usb_device = _find_hhs_usb_device()
    if usb_device is None:
        raise RuntimeError(
            f"HHS USB-CANFD adapter {_USB_VENDOR_ID}:{_USB_PRODUCT_ID} was not found"
        )

    current_interface = _find_can_interface(usb_device)
    if current_interface is None:
        _run_privileged(
            ["tee", str(_GS_USB_NEW_ID)],
            input=f"{_USB_VENDOR_ID} {_USB_PRODUCT_ID}\n",
            capture_output=True,
        )
        subprocess.run(["udevadm", "settle", "--timeout=3"], check=True)
        for _ in range(30):
            current_interface = _find_can_interface(usb_device)
            if current_interface:
                break
            time.sleep(0.1)
    if current_interface is None:
        raise RuntimeError("the gs_usb driver did not create a CAN interface")

    _run_privileged(["ip", "link", "set", current_interface, "down"])
    if current_interface != interface:
        if (_SYS_CLASS_NET / interface).exists():
            raise RuntimeError(f"target CAN interface {interface!r} already exists")
        _run_privileged(["ip", "link", "set", current_interface, "name", interface])
    _run_privileged(["ip", "link", "set", interface, "type", "can", "bitrate", str(bitrate)])
    _run_privileged(["ip", "link", "set", interface, "up"])
    _verify_can_transmit(interface, usb_device)


def _verify_linux_can(interface: str) -> str:
    usb_device = _find_hhs_usb_device()
    if usb_device is None:
        raise RuntimeError(
            f"HHS USB-CANFD adapter {_USB_VENDOR_ID}:{_USB_PRODUCT_ID} was not found."
        )
    detected_interface = _find_can_interface(usb_device)
    if detected_interface != interface:
        detected = detected_interface or "no SocketCAN interface"
        raise RuntimeError(
            f"expected interface {interface!r}, found {detected!r}. Run "
            "`dimos hardware a1z configure-can`."
        )

    interface_path = _SYS_CLASS_NET / interface
    try:
        flags = int((interface_path / "flags").read_text(), 16)
        driver = (interface_path / "device" / "driver").resolve(strict=True).name
    except OSError as exc:
        raise RuntimeError(f"cannot inspect SocketCAN interface {interface!r}: {exc}") from exc
    if driver != "gs_usb":
        raise RuntimeError(f"interface {interface!r} uses driver {driver!r}, not 'gs_usb'.")
    if not flags & 0x1:
        raise RuntimeError(
            f"interface {interface!r} is DOWN. Run `dimos hardware a1z configure-can`."
        )
    return f"{interface} is UP on gs_usb"


def _verify_macos_can() -> str:
    usb_backend, usb_core, gs_usb_module = _macos_usb_modules()
    backend = usb_backend.get_backend()
    if backend is None:
        raise RuntimeError("PyUSB could not load libusb. Install it with `brew install libusb`.")
    device = usb_core.find(
        idVendor=int(_USB_VENDOR_ID, 16),
        idProduct=int(_USB_PRODUCT_ID, 16),
        backend=backend,
    )
    if device is None:
        raise RuntimeError(
            f"HHS USB-CANFD adapter {_USB_VENDOR_ID}:{_USB_PRODUCT_ID} was not found."
        )
    bus = gs_usb_module.GsUsbMacBus(listen_only=True)
    try:
        return "HHS adapter opened in listen-only mode"
    finally:
        bus.shutdown()


def _doctor_checks(system: str, software_only: bool, interface: str) -> list[_Check]:
    checks: list[_Check] = [
        ("A1Z SDK", _verify_sdk),
        ("DimOS A1Z adapter", _verify_adapter_import),
    ]
    if system == "Linux":
        checks.append(("can-utils", _verify_linux_dependencies))
        if not software_only:
            checks.append(("Linux USB-CAN", lambda: _verify_linux_can(interface)))
    elif system == "Darwin":
        checks.append(("macOS USB-CAN dependencies", _verify_macos_dependencies))
        if not software_only:
            checks.append(("macOS USB-CAN", _verify_macos_can))
    return checks


def _run_doctor_checks(checks: list[_Check]) -> int:
    failures = 0
    for name, check in checks:
        try:
            detail = check()
        except (OSError, RuntimeError, ValueError) as exc:
            failures += 1
            typer.echo(f"FAIL  {name}: {exc}", err=True)
        else:
            typer.echo(f"PASS  {name}: {detail}")
    return failures


@app.command()
def doctor(
    software_only: bool = typer.Option(
        False,
        "--software-only",
        help="Check dependencies without requiring attached hardware",
    ),
    interface: str = typer.Option(
        _DEFAULT_CAN_INTERFACE,
        "--interface",
        help="Expected Linux SocketCAN interface",
    ),
) -> None:
    """Check A1Z dependencies and hardware without changing the host."""
    system = platform.system()
    if system not in {"Linux", "Darwin"}:
        _abort("A1Z diagnostics support Linux and macOS only")

    failures = _run_doctor_checks(_doctor_checks(system, software_only, interface))
    if failures:
        typer.echo(
            f"A1Z doctor found {failures} problem(s). See {_A1Z_GUIDE}.",
            err=True,
        )
        raise typer.Exit(1)
    typer.echo("A1Z doctor passed.")


@app.command("configure-can")
def configure_can(
    interface: str = typer.Option(
        _DEFAULT_CAN_INTERFACE,
        "--interface",
        help="Stable SocketCAN interface name",
    ),
    bitrate: int = typer.Option(
        _DEFAULT_CAN_BITRATE,
        "--bitrate",
        help="CAN bitrate",
    ),
    yes: bool = typer.Option(
        False,
        "--yes",
        "-y",
        help="Skip the confirmation prompt",
    ),
) -> None:
    """Configure and transmission-test the Linux HHS USB-CANFD adapter."""
    if platform.system() != "Linux":
        _abort("`dimos hardware a1z configure-can` is Linux-only; macOS uses userspace USB-CAN")
    if not yes and not typer.confirm(
        "This will request sudo to configure the A1Z CAN interface. Continue?",
        default=False,
    ):
        typer.echo("Aborted.")
        raise typer.Exit(1)
    try:
        _configure_linux_can(interface, bitrate)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        _abort(str(exc))
    typer.echo(f"A1Z CAN configuration passed: {interface!r} transmitted at {bitrate} bit/s.")
