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

from pathlib import Path
from types import SimpleNamespace

from typer.testing import CliRunner

from dimos.cli import hardware_cli
from dimos.cli.hardware import a1z as a1z_cli

runner = CliRunner()


def test_hardware_namespace_exposes_a1z_diagnostics_and_configuration() -> None:
    result = runner.invoke(hardware_cli.app, ["a1z", "--help"])

    assert result.exit_code == 0, result.output
    assert "doctor" in result.output
    assert "configure-can" in result.output
    assert "setup" not in result.output


def test_software_only_doctor_checks_every_linux_dependency_without_hardware(mocker) -> None:
    mocker.patch.object(a1z_cli.platform, "system", return_value="Linux")
    mocker.patch.object(a1z_cli, "_verify_sdk", return_value="/sdk/a1z")
    mocker.patch.object(a1z_cli, "_verify_adapter_import", return_value="/dimos/adapter.py")
    mocker.patch.object(a1z_cli, "_verify_linux_dependencies", return_value="/usr/bin/cansend")
    hardware = mocker.patch.object(a1z_cli, "_verify_linux_can")

    result = runner.invoke(a1z_cli.app, ["doctor", "--software-only"])

    assert result.exit_code == 0, result.output
    assert "PASS  A1Z SDK: /sdk/a1z" in result.output
    assert "PASS  DimOS A1Z adapter: /dimos/adapter.py" in result.output
    assert "PASS  can-utils: /usr/bin/cansend" in result.output
    assert "A1Z doctor passed." in result.output
    hardware.assert_not_called()


def test_doctor_reports_all_failures_in_one_run(mocker) -> None:
    mocker.patch.object(a1z_cli.platform, "system", return_value="Linux")
    mocker.patch.object(a1z_cli, "_verify_sdk", side_effect=RuntimeError("SDK missing"))
    mocker.patch.object(a1z_cli, "_verify_adapter_import", return_value="/dimos/adapter.py")
    mocker.patch.object(
        a1z_cli,
        "_verify_linux_dependencies",
        side_effect=RuntimeError("cansend missing"),
    )
    mocker.patch.object(a1z_cli, "_verify_linux_can", side_effect=RuntimeError("CAN is down"))

    result = runner.invoke(a1z_cli.app, ["doctor"])

    assert result.exit_code == 1
    assert "FAIL  A1Z SDK: SDK missing" in result.output
    assert "PASS  DimOS A1Z adapter: /dimos/adapter.py" in result.output
    assert "FAIL  can-utils: cansend missing" in result.output
    assert "FAIL  Linux USB-CAN: CAN is down" in result.output
    assert "A1Z doctor found 3 problem(s)" in result.output


def test_doctor_checks_selected_linux_interface(mocker) -> None:
    mocker.patch.object(a1z_cli.platform, "system", return_value="Linux")
    mocker.patch.object(a1z_cli, "_verify_sdk", return_value="/sdk/a1z")
    mocker.patch.object(a1z_cli, "_verify_adapter_import", return_value="/dimos/adapter.py")
    mocker.patch.object(a1z_cli, "_verify_linux_dependencies", return_value="/usr/bin/cansend")
    hardware = mocker.patch.object(a1z_cli, "_verify_linux_can", return_value="can7 is ready")

    result = runner.invoke(a1z_cli.app, ["doctor", "--interface", "can7"])

    assert result.exit_code == 0, result.output
    assert "PASS  Linux USB-CAN: can7 is ready" in result.output
    hardware.assert_called_once_with("can7")


def test_configure_can_rejection_does_not_request_privileges(mocker) -> None:
    mocker.patch.object(a1z_cli.platform, "system", return_value="Linux")
    mocker.patch.object(a1z_cli.typer, "confirm", return_value=False)
    configure = mocker.patch.object(a1z_cli, "_configure_linux_can")

    result = runner.invoke(a1z_cli.app, ["configure-can"])

    assert result.exit_code == 1
    assert "Aborted." in result.output
    configure.assert_not_called()


def test_configure_can_yes_skips_confirmation(mocker) -> None:
    mocker.patch.object(a1z_cli.platform, "system", return_value="Linux")
    confirm = mocker.patch.object(a1z_cli.typer, "confirm")
    configure = mocker.patch.object(a1z_cli, "_configure_linux_can")

    result = runner.invoke(
        a1z_cli.app,
        ["configure-can", "--interface", "can7", "--bitrate", "500000", "--yes"],
    )

    assert result.exit_code == 0, result.output
    assert "A1Z CAN configuration passed" in result.output
    confirm.assert_not_called()
    configure.assert_called_once_with("can7", 500000)


def test_macos_hardware_check_opens_adapter_in_listen_only_mode(mocker) -> None:
    backend = mocker.Mock()
    backend.get_backend.return_value = object()
    core = mocker.Mock()
    core.find.return_value = object()
    bus = mocker.Mock()
    gs_usb = SimpleNamespace(GsUsbMacBus=mocker.Mock(return_value=bus))
    mocker.patch.object(a1z_cli, "_macos_usb_modules", return_value=(backend, core, gs_usb))

    detail = a1z_cli._verify_macos_can()

    assert detail == "HHS adapter opened in listen-only mode"
    gs_usb.GsUsbMacBus.assert_called_once_with(listen_only=True)
    bus.shutdown.assert_called_once_with()


def test_linux_can_configuration_limits_privileged_commands(
    mocker,
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
    monkeypatch.setattr(a1z_cli, "_SYS_USB_DEVICES", usb_root)
    monkeypatch.setattr(a1z_cli, "_SYS_CLASS_NET", sys_class_net)
    privileged = mocker.patch.object(a1z_cli, "_run_privileged")
    verify = mocker.patch.object(a1z_cli, "_verify_can_transmit")

    a1z_cli._configure_linux_can("a1zcan", 1_000_000)

    assert [call.args[0] for call in privileged.call_args_list] == [
        ["modprobe", "gs_usb"],
        ["ip", "link", "set", "can0", "down"],
        ["ip", "link", "set", "can0", "name", "a1zcan"],
        ["ip", "link", "set", "a1zcan", "type", "can", "bitrate", "1000000"],
        ["ip", "link", "set", "a1zcan", "up"],
    ]
    verify.assert_called_once_with("a1zcan", usb_device)
