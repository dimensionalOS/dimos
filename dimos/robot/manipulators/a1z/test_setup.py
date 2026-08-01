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

import builtins
from pathlib import Path
from typing import Any
from unittest.mock import Mock

from typer.testing import CliRunner

from dimos.robot.manipulators.a1z import cli as a1z_cli

runner = CliRunner()


def test_a1z_help_lists_learning_commands_without_importing_lerobot() -> None:
    result = runner.invoke(a1z_cli.app, ["--help"])

    assert result.exit_code == 0, result.output
    assert "teach" in result.output
    assert "replay" in result.output
    assert "run-policy" in result.output


def test_teach_refuses_to_overwrite_recording(tmp_path: Path) -> None:
    recording = tmp_path / "existing.db"
    recording.touch()

    result = runner.invoke(a1z_cli.app, ["teach", str(recording)])

    assert result.exit_code == 2
    assert "refusing to overwrite existing recording" in result.output


def test_run_policy_reports_missing_optional_runtime(monkeypatch) -> None:
    real_import = builtins.__import__

    def import_without_lerobot(name: str, *args: Any, **kwargs: Any) -> Any:
        if name == "dimos.experimental.robot_policy.lerobot":
            raise ImportError("install the lerobot extra")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", import_without_lerobot)

    result = runner.invoke(a1z_cli.app, ["run-policy", "checkpoint"])

    assert result.exit_code == 1
    assert "install the lerobot extra" in result.output


def test_setup_sdk_only_does_not_check_hardware(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli, "_verify_sdk", Mock(return_value="/sdk/a1z"))
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(a1z_cli.app, ["setup", "--sdk-only"])

    assert result.exit_code == 0, result.output
    assert "A1Z vendor SDK check passed: /sdk/a1z" in result.output
    configure.assert_not_called()


def test_setup_reports_missing_sdk_without_installing(monkeypatch) -> None:
    monkeypatch.setattr(
        a1z_cli,
        "_verify_sdk",
        Mock(side_effect=RuntimeError("install the pinned SDK")),
    )

    result = runner.invoke(a1z_cli.app, ["setup", "--sdk-only"])

    assert result.exit_code == 1
    assert "install the pinned SDK" in result.output


def test_can_setup_rejection_does_not_request_privileges(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli.platform, "system", Mock(return_value="Linux"))
    monkeypatch.setattr(a1z_cli.typer, "confirm", Mock(return_value=False))
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(a1z_cli.app, ["can-setup"])

    assert result.exit_code == 1
    assert "Aborted." in result.output
    configure.assert_not_called()


def test_can_setup_confirms_before_configuring(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli.platform, "system", Mock(return_value="Linux"))
    monkeypatch.setattr(a1z_cli.typer, "confirm", Mock(return_value=True))
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(
        a1z_cli.app,
        ["can-setup", "--interface", "can7", "--bitrate", "500000"],
    )

    assert result.exit_code == 0, result.output
    configure.assert_called_once_with("can7", 500000)


def test_macos_setup_uses_listen_only_transport_check(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli, "_verify_sdk", Mock(return_value="/sdk/a1z"))
    monkeypatch.setattr(a1z_cli.platform, "system", Mock(return_value="Darwin"))
    verify_macos = Mock()
    monkeypatch.setattr(a1z_cli, "_verify_macos_can", verify_macos)

    result = runner.invoke(a1z_cli.app, ["setup"])

    assert result.exit_code == 0, result.output
    verify_macos.assert_called_once_with()


def test_linux_can_setup_limits_privileged_commands(
    monkeypatch,
    tmp_path: Path,
) -> None:
    usb_root = tmp_path / "usb"
    usb_device = usb_root / "1-1"
    usb_interface = usb_root / "1-1:1.0" / "net" / "can0"
    usb_device.mkdir(parents=True)
    usb_interface.mkdir(parents=True)
    (usb_device / "idVendor").write_text("a8fa\n")
    (usb_device / "idProduct").write_text("8598\n")
    sys_class_net = tmp_path / "net"
    sys_class_net.mkdir()
    privileged = Mock()
    verify = Mock()
    monkeypatch.setattr(a1z_cli, "_SYS_USB_DEVICES", usb_root)
    monkeypatch.setattr(a1z_cli, "_SYS_CLASS_NET", sys_class_net)
    monkeypatch.setattr(a1z_cli, "_run_privileged", privileged)
    monkeypatch.setattr(a1z_cli, "_verify_can_transmit", verify)

    a1z_cli._configure_linux_can("a1zcan", 1_000_000)

    assert [call.args[0] for call in privileged.call_args_list] == [
        ["modprobe", "gs_usb"],
        ["ip", "link", "set", "can0", "down"],
        ["ip", "link", "set", "can0", "name", "a1zcan"],
        [
            "ip",
            "link",
            "set",
            "a1zcan",
            "type",
            "can",
            "bitrate",
            "1000000",
        ],
        ["ip", "link", "set", "a1zcan", "up"],
    ]
    verify.assert_called_once_with("a1zcan", usb_device)
