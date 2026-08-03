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

"""Host setup and diagnostics for the Galaxea A1Z."""

from __future__ import annotations

import ctypes.util
import importlib
from importlib import metadata
import inspect
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
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
_A1Z_GUIDE = "docs/capabilities/manipulation/a1z.md"
_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
_A1Z_SDK_REQUIREMENT = (
    "a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@"
    "e931ecd0e25ad35df251097ba42921b3d2fa7224"
)
_MACOS_PYTHON_REQUIREMENTS = ("gs-usb==0.3.1", "pyusb==1.3.1")

_InstallCommand = tuple[list[str], Path | None]


def _abort(message: str) -> None:
    typer.echo(f"ERROR: {message}", err=True)
    raise typer.Exit(1)


def _is_source_checkout() -> bool:
    return (_REPOSITORY_ROOT / "pyproject.toml").is_file() and (
        _REPOSITORY_ROOT / "uv.lock"
    ).is_file()


def _python_dependency_commands(system: str) -> list[_InstallCommand]:
    requirements = [_A1Z_SDK_REQUIREMENT]
    if system == "Darwin":
        requirements.extend(_MACOS_PYTHON_REQUIREMENTS)

    uv = shutil.which("uv")
    if _is_source_checkout():
        if uv is None:
            raise RuntimeError(
                "a DimOS source checkout requires `uv`. Install uv from "
                "https://docs.astral.sh/uv/, then rerun `dimos a1z setup`."
            )
        return [
            (
                [
                    uv,
                    "sync",
                    "--locked",
                    "--extra",
                    "manipulation",
                    "--inexact",
                ],
                _REPOSITORY_ROOT,
            ),
            (
                [uv, "pip", "install", "--python", sys.executable, *requirements],
                _REPOSITORY_ROOT,
            ),
        ]

    try:
        dimos_version = metadata.version("dimos")
    except metadata.PackageNotFoundError as exc:
        raise RuntimeError(
            "could not identify the installed DimOS version. Install DimOS, then rerun "
            "`dimos a1z setup`."
        ) from exc
    requirements.insert(0, f"dimos[manipulation]=={dimos_version}")
    if uv is not None:
        command = [uv, "pip", "install", "--python", sys.executable, *requirements]
    else:
        command = [sys.executable, "-m", "pip", "install", *requirements]
    return [(command, None)]


def _install_python_dependencies(system: str) -> None:
    for command, cwd in _python_dependency_commands(system):
        subprocess.run(command, cwd=cwd, check=True, text=True)


def _system_dependency_plan(system: str) -> tuple[list[str] | None, str | None]:
    if system == "Linux":
        if shutil.which("cansend") is not None:
            return None, None
        try:
            distribution = platform.freedesktop_os_release().get("ID", "")
        except OSError:
            distribution = ""
        if distribution == "ubuntu":
            return ["sudo", "apt-get", "install", "-y", "can-utils"], None
        return None, (
            "Linux CAN prerequisite missing: `cansend`. Install the package that provides "
            "`cansend` (`can-utils` on Ubuntu), then rerun `dimos a1z setup`."
        )

    if ctypes.util.find_library("usb-1.0") is not None:
        return None, None
    brew = shutil.which("brew")
    if brew is not None:
        return [brew, "install", "libusb"], None
    return None, (
        "macOS CAN prerequisite missing: libusb. Install Homebrew and `brew install libusb`, "
        "then rerun `dimos a1z setup`."
    )


def _show_setup_plan(
    python_commands: list[_InstallCommand],
    system_command: list[str] | None,
) -> None:
    typer.echo("A1Z setup will run:")
    for command, _ in python_commands:
        typer.echo(f"  {shlex.join(command)}")
    if system_command is not None:
        typer.echo(f"  {shlex.join(system_command)}")


def _verify_sdk() -> str:
    """Return the installed SDK path or raise with actionable instructions."""
    try:
        a1z = importlib.import_module("a1z")
        get_robot_module = importlib.import_module("a1z.robots.get_robot")
        get_a1z_robot = get_robot_module.get_a1z_robot
        parameters = inspect.signature(get_a1z_robot).parameters
    except Exception as exc:
        raise RuntimeError(
            "the A1Z SDK is unavailable after installation. Rerun `dimos a1z setup`. "
            f"Original error: {exc}"
        ) from exc
    if "with_gripper" not in parameters:
        raise RuntimeError(
            "the installed A1Z SDK lacks get_a1z_robot(with_gripper=...). "
            "Rerun `dimos a1z setup` to install the pinned gripper-capable SDK."
        )
    return str(a1z.__file__)


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


def _verify_macos_can() -> None:
    try:
        usb_backend = importlib.import_module("usb.backend.libusb1")
        usb_core = importlib.import_module("usb.core")
        gs_usb_module = importlib.import_module(
            "dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus"
        )
    except Exception as exc:
        raise RuntimeError(
            "macOS A1Z support requires pyusb, gs-usb, and system libusb. "
            "Rerun `dimos a1z setup`. "
            f"Original error: {exc}"
        ) from exc

    backend = usb_backend.get_backend()
    if backend is None:
        raise RuntimeError("PyUSB could not load libusb. Install libusb with Homebrew, then retry.")
    device = usb_core.find(
        idVendor=int(_USB_VENDOR_ID, 16),
        idProduct=int(_USB_PRODUCT_ID, 16),
        backend=backend,
    )
    if device is None:
        raise RuntimeError(
            f"HHS USB-CANFD adapter {_USB_VENDOR_ID}:{_USB_PRODUCT_ID} was not found"
        )
    bus = gs_usb_module.GsUsbMacBus(listen_only=True)
    try:
        typer.echo("A1Z macOS USB-CAN check passed in listen-only mode.")
    finally:
        bus.shutdown()


@app.command("can-setup")
def can_setup(
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
        _abort("`dimos a1z can-setup` is Linux-only; macOS uses userspace USB-CAN")
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
    typer.echo(f"A1Z CAN setup passed: {interface!r} transmitted at {bitrate} bit/s.")


@app.command("setup")
def setup(
    sdk_only: bool = typer.Option(
        False,
        "--sdk-only",
        help="Install and verify Python dependencies without checking hardware",
    ),
    yes: bool = typer.Option(
        False,
        "--yes",
        "-y",
        help="Run setup without the confirmation prompt",
    ),
) -> None:
    """Install A1Z dependencies, then configure and test the CAN adapter."""
    system = platform.system()
    if system not in {"Linux", "Darwin"}:
        _abort("A1Z host setup supports Linux and macOS only")

    try:
        python_commands = _python_dependency_commands(system)
        system_command, system_error = (None, None) if sdk_only else _system_dependency_plan(system)
    except RuntimeError as exc:
        _abort(str(exc))
    _show_setup_plan(python_commands, system_command)
    if not yes and not typer.confirm("Continue with A1Z setup?", default=False):
        typer.echo("Aborted.")
        raise typer.Exit(1)

    try:
        _install_python_dependencies(system)
        sdk_path = _verify_sdk()
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        _abort(str(exc))
    typer.echo(f"A1Z vendor SDK check passed: {sdk_path}")
    if sdk_only:
        return
    typer.echo("A1Z Python dependencies installed and verified.")

    if system_error is not None:
        _abort(system_error)
    try:
        if system_command is not None:
            subprocess.run(system_command, check=True, text=True)
        if system == "Linux":
            _configure_linux_can(_DEFAULT_CAN_INTERFACE, _DEFAULT_CAN_BITRATE)
            typer.echo(
                f"A1Z CAN setup passed: {_DEFAULT_CAN_INTERFACE!r} transmitted at "
                f"{_DEFAULT_CAN_BITRATE} bit/s."
            )
        else:
            _verify_macos_can()
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        _abort(str(exc))
