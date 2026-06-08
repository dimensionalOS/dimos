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

"""macOS LaunchServices helper for Unitree BLE provisioning.

The BLE protocol lives in :mod:`dimos.robot.unitree.go2.cli.ble`. This module
only gives that protocol a macOS app-bundle wrapper so Bluetooth/TCC attributes
are associated with a LaunchServices-launched app rather than the user's shell.
"""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Callable, Sequence
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import platform
import plistlib
import shlex
import stat
import subprocess
import sys
import tempfile
from typing import Any

from dimos.constants import DIMOS_PROJECT_ROOT, STATE_DIR
from dimos.robot.unitree.go2.cli import ble
from dimos.robot.unitree.go2.cli.ble import Go2Device

REQUIRED_BLUETOOTH_USAGE_KEYS = (
    "NSBluetoothAlwaysUsageDescription",
    "NSBluetoothPeripheralUsageDescription",
    "NSBluetoothUsageDescription",
)

HELPER_APP_ENV = "DIMOS_GO2_BLE_HELPER"
LEGACY_HELPER_APP_ENV = "DIMOS_GO2_BLE_HELPER_APP"
HELPER_APP_NAME = "DimOS BLE Helper.app"
HELPER_BUNDLE_IDENTIFIER = "com.dimensional.dimos.blehelper"
HELPER_EXECUTABLE_NAME = "dimos-ble-helper"
OPEN_COMMAND = "/usr/bin/open"

JsonObject = dict[str, Any]


class MacOSBLEHelperError(RuntimeError):
    """Base error for the macOS BLE helper wrapper."""


class MacOSBLEUnsupportedError(MacOSBLEHelperError):
    """Raised when the LaunchServices BLE helper is used off macOS."""


class HelperAppValidationError(MacOSBLEHelperError):
    """Raised when a helper app bundle is missing required metadata."""


class HelperLaunchError(MacOSBLEHelperError):
    """Raised when LaunchServices fails to run the helper app."""


@dataclass(frozen=True)
class HelperAppInfo:
    app_path: Path
    info_plist_path: Path
    executable_path: Path
    bundle_identifier: str | None


def ensure_macos() -> None:
    """Raise a clear error when the LaunchServices backend is unavailable."""
    if platform.system() != "Darwin":
        raise MacOSBLEUnsupportedError(
            "The macOS LaunchServices BLE helper is only supported on macOS. "
            "Use dimos.robot.unitree.go2.cli.ble directly on Linux."
        )


def helper_cache_dir() -> Path:
    """Return the DimOS state directory used for the generated helper app."""
    return STATE_DIR / "helpers" / "macos-ble"


def validate_helper_app(app_path: str | Path) -> HelperAppInfo:
    """Validate a BLE helper ``.app`` bundle and its Bluetooth usage metadata."""
    app = Path(app_path).expanduser()
    if not app.exists():
        raise HelperAppValidationError(f"helper app does not exist: {app}")
    if not app.is_dir() or app.suffix != ".app":
        raise HelperAppValidationError(f"helper path is not a .app bundle: {app}")

    info_plist = app / "Contents" / "Info.plist"
    if not info_plist.exists():
        raise HelperAppValidationError(f"helper app is missing Info.plist: {info_plist}")

    try:
        with info_plist.open("rb") as f:
            plist = plistlib.load(f)
    except Exception as exc:
        raise HelperAppValidationError(f"helper app Info.plist is invalid: {exc}") from exc

    missing = [
        key
        for key in REQUIRED_BLUETOOTH_USAGE_KEYS
        if not isinstance(plist.get(key), str) or not plist[key].strip()
    ]
    if missing:
        raise HelperAppValidationError(
            "helper app Info.plist is missing Bluetooth usage keys: " + ", ".join(missing)
        )

    executable_name = plist.get("CFBundleExecutable")
    if not isinstance(executable_name, str) or not executable_name:
        raise HelperAppValidationError("helper app Info.plist is missing CFBundleExecutable")

    executable = app / "Contents" / "MacOS" / executable_name
    if not executable.exists():
        raise HelperAppValidationError(f"helper executable does not exist: {executable}")
    if not os.access(executable, os.X_OK):
        raise HelperAppValidationError(f"helper executable is not executable: {executable}")

    bundle_identifier = plist.get("CFBundleIdentifier")
    return HelperAppInfo(
        app_path=app,
        info_plist_path=info_plist,
        executable_path=executable,
        bundle_identifier=bundle_identifier if isinstance(bundle_identifier, str) else None,
    )


def build_cached_helper_app(
    *,
    cache_dir: str | Path | None = None,
    python_executable: str | Path | None = None,
) -> Path:
    """Create or refresh the cached LaunchServices BLE helper app."""
    ensure_macos()

    root = Path(cache_dir) if cache_dir is not None else helper_cache_dir()
    app_path = root / HELPER_APP_NAME
    contents_dir = app_path / "Contents"
    macos_dir = contents_dir / "MacOS"
    macos_dir.mkdir(parents=True, exist_ok=True)

    python = Path(python_executable) if python_executable is not None else Path(sys.executable)
    module = "dimos.robot.unitree.go2.cli.macos_ble_helper"
    script = "\n".join(
        [
            "#!/bin/sh",
            'if [ -n "${PYTHONPATH:-}" ]; then',
            f'  export PYTHONPATH={shlex.quote(str(DIMOS_PROJECT_ROOT))}:"$PYTHONPATH"',
            "else",
            f"  export PYTHONPATH={shlex.quote(str(DIMOS_PROJECT_ROOT))}",
            "fi",
            f'exec {shlex.quote(str(python))} -m {module} --helper-run "$@"',
            "",
        ]
    )

    executable = macos_dir / HELPER_EXECUTABLE_NAME
    executable.write_text(script, encoding="utf-8")
    executable.chmod(0o755)

    bluetooth_description = "DimOS uses Bluetooth to discover and provision Unitree robots."
    plist: JsonObject = {
        "CFBundleDevelopmentRegion": "en",
        "CFBundleExecutable": HELPER_EXECUTABLE_NAME,
        "CFBundleIdentifier": HELPER_BUNDLE_IDENTIFIER,
        "CFBundleInfoDictionaryVersion": "6.0",
        "CFBundleName": "DimOS BLE Helper",
        "CFBundlePackageType": "APPL",
        "CFBundleShortVersionString": "1.0",
        "CFBundleVersion": "1",
        "DimOSPythonExecutable": str(python),
        "LSUIElement": True,
        "NSBluetoothAlwaysUsageDescription": bluetooth_description,
        "NSBluetoothPeripheralUsageDescription": bluetooth_description,
        "NSBluetoothUsageDescription": bluetooth_description,
    }
    with (contents_dir / "Info.plist").open("wb") as f:
        plistlib.dump(plist, f)

    validate_helper_app(app_path)
    return app_path


def locate_helper_app(
    helper_app_path: str | Path | None = None,
    *,
    create: bool = True,
) -> Path:
    """Return a provided, env-configured, or cached BLE helper app path."""
    ensure_macos()

    configured = helper_app_path
    if configured is None:
        env_value = os.environ.get(HELPER_APP_ENV) or os.environ.get(LEGACY_HELPER_APP_ENV)
        configured = env_value if env_value else None

    if configured is not None:
        return validate_helper_app(configured).app_path

    cached = helper_cache_dir() / HELPER_APP_NAME
    if create:
        return build_cached_helper_app()

    return validate_helper_app(cached).app_path


async def _invoke_helper_async(
    action: str,
    params: JsonObject,
    *,
    password: str | None = None,
    helper_app_path: str | Path | None = None,
    on_progress: Callable[[str], None] | None = None,
    secrets: Sequence[str] = (),
) -> JsonObject:
    loop = asyncio.get_running_loop()
    with ThreadPoolExecutor(max_workers=1, thread_name_prefix="dimos-go2-ble-helper") as executor:
        return await loop.run_in_executor(
            executor,
            lambda: _invoke_helper(
                action,
                params,
                password=password,
                helper_app_path=helper_app_path,
                on_progress=on_progress,
                secrets=secrets,
            ),
        )


async def find_robots(
    timeout: float = 15.0,
    prefixes: tuple[str, ...] = ble.UNITREE_NAME_PREFIXES,
    *,
    helper_app_path: str | Path | None = None,
    on_device: Callable[[Go2Device], None] | None = None,
    on_progress: Callable[[str], None] | None = None,
) -> list[Go2Device]:
    """Find Unitree robots through the macOS LaunchServices BLE helper."""
    response = await _invoke_helper_async(
        "find_robots",
        {
            "timeout": timeout,
            "prefixes": list(prefixes),
        },
        helper_app_path=helper_app_path,
        on_progress=on_progress,
    )
    devices = [
        Go2Device(name=str(d["name"]), address=str(d["address"]), serial=d.get("serial"))
        for d in response.get("devices", [])
    ]
    if on_device is not None:
        for device in devices:
            on_device(device)
    return devices


async def provision_wifi(
    address: str,
    ssid: str,
    password: str,
    country_code: str = "US",
    *,
    timeout: float = 30.0,
    connect_retries: int = 3,
    helper_app_path: str | Path | None = None,
    on_progress: Callable[[str], None] | None = None,
) -> str | None:
    """Provision Wi-Fi through the macOS LaunchServices BLE helper."""
    response = await _invoke_helper_async(
        "provision_wifi",
        {
            "address": address,
            "ssid": ssid,
            "country_code": country_code,
            "timeout": timeout,
            "connect_retries": connect_retries,
        },
        password=password,
        helper_app_path=helper_app_path,
        on_progress=on_progress,
        secrets=(password,),
    )
    serial = response.get("serial")
    return str(serial) if serial is not None else None


def _invoke_helper(
    action: str,
    params: JsonObject,
    *,
    password: str | None = None,
    helper_app_path: str | Path | None = None,
    on_progress: Callable[[str], None] | None = None,
    secrets: Sequence[str] = (),
) -> JsonObject:
    helper_app = locate_helper_app(helper_app_path)
    safe_secrets = tuple(secret for secret in secrets if secret)

    with tempfile.TemporaryDirectory(prefix="dimos-go2-ble-") as tmp:
        tmp_path = Path(tmp)
        tmp_path.chmod(0o700)
        request_path = tmp_path / "request.json"
        response_path = tmp_path / "response.json"
        progress_path = tmp_path / "progress.jsonl"
        password_path = tmp_path / "password.txt"

        request: JsonObject = {
            "schema_version": 1,
            "action": action,
            "params": params,
        }
        if password is not None:
            _write_text_0600(password_path, password)
            request["password_file"] = str(password_path)

        _write_json_0600(request_path, request)
        _touch_0600(response_path)
        _touch_0600(progress_path)

        cmd = _open_command(helper_app, request_path, response_path, progress_path)
        completed = subprocess.run(cmd, check=False, capture_output=True, text=True)

        _emit_progress(progress_path, on_progress, secrets=safe_secrets)

        if completed.returncode != 0:
            stderr = _sanitize_text(completed.stderr or completed.stdout or "", safe_secrets)
            raise HelperLaunchError(
                f"BLE helper exited with status {completed.returncode}: {stderr.strip()}"
            )

        response = _read_json_file(response_path)
        if not response:
            raise HelperLaunchError("BLE helper did not write a response")
        if not response.get("ok"):
            error = _sanitize_text(str(response.get("error", "BLE helper failed")), safe_secrets)
            raise MacOSBLEHelperError(error)
        return response


def _open_command(
    helper_app_path: Path,
    request_path: Path,
    response_path: Path,
    progress_path: Path,
) -> list[str]:
    return [
        OPEN_COMMAND,
        "-W",
        "-n",
        str(helper_app_path),
        "--args",
        "--helper-request",
        str(request_path),
        "--helper-response",
        str(response_path),
        "--helper-progress",
        str(progress_path),
    ]


def _write_text_0600(path: Path, text: str) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(text)


def _write_json_0600(path: Path, payload: JsonObject) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(payload, f, separators=(",", ":"))
        f.write("\n")


def _touch_0600(path: Path) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    os.close(fd)


def _read_json_file(path: Path) -> JsonObject:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return {}
    data = json.loads(text)
    if not isinstance(data, dict):
        raise HelperLaunchError(f"expected JSON object in {path}")
    return data


def _append_progress(path: Path, message: str, *, secrets: Sequence[str] = ()) -> None:
    safe_message = _sanitize_text(message, secrets)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, "a", encoding="utf-8") as f:
        json.dump({"message": safe_message}, f, separators=(",", ":"))
        f.write("\n")


def _emit_progress(
    progress_path: Path,
    on_progress: Callable[[str], None] | None,
    *,
    secrets: Sequence[str] = (),
) -> None:
    if on_progress is None or not progress_path.exists():
        return
    for line in progress_path.read_text(encoding="utf-8").splitlines():
        if not line:
            continue
        try:
            payload = json.loads(line)
            message = str(payload.get("message", ""))
        except json.JSONDecodeError:
            message = line
        if message:
            on_progress(_sanitize_text(message, secrets))


def _sanitize_text(text: str, secrets: Sequence[str]) -> str:
    safe = text
    for secret in secrets:
        if secret:
            safe = safe.replace(secret, "<redacted>")
    return safe


def _assert_private_file(path: Path) -> None:
    mode = stat.S_IMODE(path.stat().st_mode)
    if mode & 0o077:
        raise HelperLaunchError(f"helper file is not private: {path} mode={oct(mode)}")


async def _execute_helper_request(request: JsonObject, progress_path: Path) -> JsonObject:
    action = request.get("action")
    params = request.get("params", {})
    if not isinstance(params, dict):
        raise HelperLaunchError("helper request params must be a JSON object")

    if action == "find_robots":
        devices: list[Go2Device] = []

        def _on_device(device: Go2Device) -> None:
            devices.append(device)
            _append_progress(
                progress_path,
                f"found {device.name} {device.address} serial={device.serial or '?'}",
            )

        result = await ble.find_robots(
            timeout=float(params.get("timeout", 15.0)),
            prefixes=tuple(params.get("prefixes", ble.UNITREE_NAME_PREFIXES)),
            on_device=_on_device,
        )
        by_address = {device.address: device for device in devices}
        for device in result:
            by_address[device.address] = device
        return {"ok": True, "devices": [asdict(device) for device in by_address.values()]}

    if action == "provision_wifi":
        password_file = request.get("password_file")
        if not isinstance(password_file, str) or not password_file:
            raise HelperLaunchError("provision_wifi request is missing password_file")
        password_path = Path(password_file)
        _assert_private_file(password_path)
        password = password_path.read_text(encoding="utf-8")

        def _on_progress(message: str) -> None:
            _append_progress(progress_path, message, secrets=(password,))

        serial = await ble.provision_wifi(
            address=str(params["address"]),
            ssid=str(params["ssid"]),
            password=password,
            country_code=str(params.get("country_code", "US")),
            timeout=float(params.get("timeout", 30.0)),
            connect_retries=int(params.get("connect_retries", 3)),
            on_progress=_on_progress,
        )
        return {"ok": True, "serial": serial}

    raise HelperLaunchError(f"unknown helper action: {action}")


def _run_helper_request(request_path: Path, response_path: Path, progress_path: Path) -> int:
    secrets: tuple[str, ...] = ()
    try:
        _assert_private_file(request_path)
        _assert_private_file(response_path)
        _assert_private_file(progress_path)
        request = _read_json_file(request_path)

        password_file = request.get("password_file")
        if isinstance(password_file, str) and password_file:
            password_path = Path(password_file)
            _assert_private_file(password_path)
            secrets = (password_path.read_text(encoding="utf-8"),)

        response = asyncio.run(_execute_helper_request(request, progress_path))
    except Exception as exc:
        response = {
            "ok": False,
            "error_type": type(exc).__name__,
            "error": _sanitize_text(f"{type(exc).__name__}: {exc}", secrets),
        }
    _write_json_0600(response_path, response)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="DimOS macOS BLE helper")
    parser.add_argument("--helper-run", action="store_true")
    parser.add_argument("--helper-request")
    parser.add_argument("--helper-response")
    parser.add_argument("--helper-progress")
    args = parser.parse_args(argv)

    if not args.helper_run:
        parser.error("this module is intended to be launched by the DimOS BLE helper app")
    if not args.helper_request or not args.helper_response or not args.helper_progress:
        parser.error("helper request, response, and progress paths are required")

    return _run_helper_request(
        Path(args.helper_request),
        Path(args.helper_response),
        Path(args.helper_progress),
    )


if __name__ == "__main__":
    raise SystemExit(main())
