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

import json
from pathlib import Path
import plistlib
import stat
import subprocess
import threading

import pytest

from dimos.robot.unitree.go2.cli import macos_ble_helper


def _make_helper_app(tmp_path: Path, *, missing_keys: set[str] | None = None) -> Path:
    app_path = tmp_path / "Test BLE Helper.app"
    macos_dir = app_path / "Contents" / "MacOS"
    macos_dir.mkdir(parents=True)

    executable = macos_dir / "helper"
    executable.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    executable.chmod(0o755)

    plist: dict[str, object] = {
        "CFBundleExecutable": "helper",
        "CFBundleIdentifier": "com.example.dimos.testblehelper",
        "CFBundleName": "Test BLE Helper",
    }
    for key in macos_ble_helper.REQUIRED_BLUETOOTH_USAGE_KEYS:
        plist[key] = "Bluetooth is used to provision Unitree robots."
    for key in missing_keys or set():
        plist.pop(key, None)

    with (app_path / "Contents" / "Info.plist").open("wb") as f:
        plistlib.dump(plist, f)

    return app_path


def _mode(path: Path) -> int:
    return stat.S_IMODE(path.stat().st_mode)


def test_validate_helper_app_accepts_required_bluetooth_metadata(tmp_path: Path) -> None:
    app_path = _make_helper_app(tmp_path)

    info = macos_ble_helper.validate_helper_app(app_path)

    assert info.app_path == app_path
    assert info.bundle_identifier == "com.example.dimos.testblehelper"
    assert info.executable_path.name == "helper"


@pytest.mark.parametrize("missing_key", macos_ble_helper.REQUIRED_BLUETOOTH_USAGE_KEYS)
def test_validate_helper_app_rejects_missing_bluetooth_usage_key(
    tmp_path: Path,
    missing_key: str,
) -> None:
    app_path = _make_helper_app(tmp_path, missing_keys={missing_key})

    with pytest.raises(macos_ble_helper.HelperAppValidationError, match=missing_key):
        macos_ble_helper.validate_helper_app(app_path)


def test_locate_helper_app_builds_cached_app_with_bluetooth_keys(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(macos_ble_helper.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(macos_ble_helper, "STATE_DIR", tmp_path)

    app_path = macos_ble_helper.locate_helper_app()

    assert app_path == tmp_path / "helpers" / "macos-ble" / macos_ble_helper.HELPER_APP_NAME
    info = macos_ble_helper.validate_helper_app(app_path)
    plist = plistlib.loads(info.info_plist_path.read_bytes())
    for key in macos_ble_helper.REQUIRED_BLUETOOTH_USAGE_KEYS:
        assert plist[key]


@pytest.mark.asyncio
async def test_find_robots_rejects_non_macos(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(macos_ble_helper.platform, "system", lambda: "Linux")

    with pytest.raises(macos_ble_helper.MacOSBLEUnsupportedError, match="only supported on macOS"):
        await macos_ble_helper.find_robots(timeout=0.1)


@pytest.mark.asyncio
async def test_helper_async_wrappers_run_off_event_loop(monkeypatch: pytest.MonkeyPatch) -> None:
    loop_thread = threading.current_thread()
    helper_threads: list[threading.Thread] = []
    calls: list[str] = []

    def fake_invoke_helper(
        action: str,
        params: macos_ble_helper.JsonObject,
        **kwargs: object,
    ) -> macos_ble_helper.JsonObject:
        helper_threads.append(threading.current_thread())
        calls.append(action)
        assert threading.current_thread() is not loop_thread
        if action == "find_robots":
            assert params["timeout"] == 2.0
            return {
                "ok": True,
                "devices": [
                    {"name": "Go2_49060", "address": "AA:BB:CC:DD:EE:FF", "serial": "SER123"}
                ],
            }
        if action == "provision_wifi":
            assert params["connect_retries"] == 3
            assert kwargs["password"] == "secret"
            return {"ok": True, "serial": "SER123"}
        raise AssertionError(f"unexpected action {action}")

    monkeypatch.setattr(macos_ble_helper.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(macos_ble_helper, "_invoke_helper", fake_invoke_helper)

    devices = await macos_ble_helper.find_robots(timeout=2.0)
    serial = await macos_ble_helper.provision_wifi(
        "AA:BB:CC:DD:EE:FF",
        "lab-wifi",
        "secret",
        "US",
        connect_retries=3,
    )

    assert calls == ["find_robots", "provision_wifi"]
    assert all(thread is not loop_thread for thread in helper_threads)
    assert devices[0].serial == "SER123"
    assert serial == "SER123"


@pytest.mark.asyncio
async def test_find_robots_uses_launchservices_helper(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    app_path = _make_helper_app(tmp_path)
    calls: list[list[str]] = []
    seen_devices: list[macos_ble_helper.Go2Device] = []

    def fake_run(
        cmd: list[str],
        *,
        check: bool,
        capture_output: bool,
        text: bool,
    ) -> subprocess.CompletedProcess[str]:
        assert check is False
        assert capture_output is True
        assert text is True
        calls.append(cmd)

        request_path = Path(cmd[cmd.index("--helper-request") + 1])
        response_path = Path(cmd[cmd.index("--helper-response") + 1])
        progress_path = Path(cmd[cmd.index("--helper-progress") + 1])
        request = json.loads(request_path.read_text(encoding="utf-8"))

        assert request == {
            "schema_version": 1,
            "action": "find_robots",
            "params": {"timeout": 2.5, "prefixes": ["Go2_"]},
        }
        assert _mode(request_path) == 0o600
        assert _mode(response_path) == 0o600
        assert _mode(progress_path) == 0o600

        response_path.write_text(
            json.dumps(
                {
                    "ok": True,
                    "devices": [
                        {"name": "Go2_49060", "address": "AA:BB:CC:DD:EE:FF", "serial": "SER123"}
                    ],
                }
            ),
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")

    monkeypatch.setattr(macos_ble_helper.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(macos_ble_helper.subprocess, "run", fake_run)

    devices = await macos_ble_helper.find_robots(
        timeout=2.5,
        prefixes=("Go2_",),
        helper_app_path=app_path,
        on_device=seen_devices.append,
    )

    assert calls[0][:3] == [macos_ble_helper.OPEN_COMMAND, "-W", "-n"]
    assert calls[0][3] == str(app_path)
    assert "--args" in calls[0]
    assert devices == seen_devices
    assert devices[0].serial == "SER123"


@pytest.mark.asyncio
async def test_provision_wifi_keeps_password_out_of_argv_request_and_progress(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    app_path = _make_helper_app(tmp_path)
    password = "secret-wifi-password"
    progress: list[str] = []
    captured_cmds: list[list[str]] = []

    def fake_run(
        cmd: list[str],
        *,
        check: bool,
        capture_output: bool,
        text: bool,
    ) -> subprocess.CompletedProcess[str]:
        assert check is False
        assert capture_output is True
        assert text is True
        captured_cmds.append(cmd)
        assert password not in " ".join(cmd)

        request_path = Path(cmd[cmd.index("--helper-request") + 1])
        response_path = Path(cmd[cmd.index("--helper-response") + 1])
        progress_path = Path(cmd[cmd.index("--helper-progress") + 1])
        request_text = request_path.read_text(encoding="utf-8")
        request = json.loads(request_text)

        assert password not in request_text
        assert request["action"] == "provision_wifi"
        assert request["params"] == {
            "address": "AA:BB:CC:DD:EE:FF",
            "ssid": "lab-wifi",
            "country_code": "US",
            "timeout": 7.0,
            "connect_retries": 2,
        }

        password_path = Path(request["password_file"])
        assert _mode(request_path) == 0o600
        assert _mode(response_path) == 0o600
        assert _mode(progress_path) == 0o600
        assert _mode(password_path) == 0o600
        assert password_path.read_text(encoding="utf-8") == password

        progress_path.write_text(
            json.dumps({"message": f"unsafe progress contains {password}"}) + "\n",
            encoding="utf-8",
        )
        response_path.write_text(json.dumps({"ok": True, "serial": "SER123"}), encoding="utf-8")
        return subprocess.CompletedProcess(cmd, 0, stdout=f"stdout {password}", stderr="")

    monkeypatch.setattr(macos_ble_helper.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(macos_ble_helper.subprocess, "run", fake_run)

    serial = await macos_ble_helper.provision_wifi(
        "AA:BB:CC:DD:EE:FF",
        "lab-wifi",
        password,
        "US",
        timeout=7.0,
        connect_retries=2,
        helper_app_path=app_path,
        on_progress=progress.append,
    )

    assert serial == "SER123"
    assert captured_cmds
    assert all(password not in item for cmd in captured_cmds for item in cmd)
    assert progress == ["unsafe progress contains [REDACTED]"]


def test_helper_progress_writer_redacts_password(tmp_path: Path) -> None:
    progress_path = tmp_path / "progress.jsonl"
    password = "secret-wifi-password"

    macos_ble_helper._append_progress(
        progress_path,
        f"set password {password}",
        secrets=(password,),
    )

    text = progress_path.read_text(encoding="utf-8")
    assert password not in text
    assert "[REDACTED]" in text
    assert _mode(progress_path) == 0o600
