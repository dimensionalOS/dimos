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
import sys
from unittest.mock import Mock

from typer.testing import CliRunner

from dimos.robot.manipulators.a1z import cli as a1z_cli

runner = CliRunner()


def test_source_install_uses_locked_inexact_sync(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(a1z_cli, "_REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(a1z_cli, "_is_source_checkout", Mock(return_value=True))
    monkeypatch.setattr(a1z_cli.shutil, "which", Mock(return_value="/usr/bin/uv"))

    commands = a1z_cli._python_dependency_commands("Linux")

    assert commands == [
        (
            [
                "/usr/bin/uv",
                "sync",
                "--locked",
                "--extra",
                "manipulation",
                "--inexact",
            ],
            tmp_path,
        ),
        (
            [
                "/usr/bin/uv",
                "pip",
                "install",
                "--python",
                sys.executable,
                a1z_cli._A1Z_SDK_REQUIREMENT,
            ],
            tmp_path,
        ),
    ]


def test_package_install_falls_back_to_current_interpreter_pip(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli, "_is_source_checkout", Mock(return_value=False))
    monkeypatch.setattr(a1z_cli.shutil, "which", Mock(return_value=None))
    monkeypatch.setattr(a1z_cli.metadata, "version", Mock(return_value="1.2.3"))

    commands = a1z_cli._python_dependency_commands("Darwin")

    assert commands == [
        (
            [
                sys.executable,
                "-m",
                "pip",
                "install",
                "dimos[manipulation]==1.2.3",
                a1z_cli._A1Z_SDK_REQUIREMENT,
                "gs-usb==0.3.1",
                "pyusb==1.3.1",
            ],
            None,
        )
    ]


def test_ubuntu_installs_can_utils_when_cansend_is_missing(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli.shutil, "which", Mock(return_value=None))
    monkeypatch.setattr(
        a1z_cli.platform,
        "freedesktop_os_release",
        Mock(return_value={"ID": "ubuntu"}),
    )

    command, error = a1z_cli._system_dependency_plan("Linux")

    assert command == ["sudo", "apt-get", "install", "-y", "can-utils"]
    assert error is None


def test_other_linux_distribution_explains_can_utils_requirement(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli.shutil, "which", Mock(return_value=None))
    monkeypatch.setattr(
        a1z_cli.platform,
        "freedesktop_os_release",
        Mock(return_value={"ID": "fedora"}),
    )

    command, error = a1z_cli._system_dependency_plan("Linux")

    assert command is None
    assert error is not None
    assert "package that provides `cansend`" in error


def test_setup_sdk_only_installs_dependencies_without_checking_hardware(monkeypatch) -> None:
    install = Mock()
    monkeypatch.setattr(a1z_cli, "_install_python_dependencies", install)
    monkeypatch.setattr(a1z_cli, "_verify_sdk", Mock(return_value="/sdk/a1z"))
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(a1z_cli.app, ["setup", "--sdk-only", "--yes"])

    assert result.exit_code == 0, result.output
    assert "A1Z vendor SDK check passed: /sdk/a1z" in result.output
    install.assert_called_once_with("Linux")
    configure.assert_not_called()


def test_setup_displays_commands_before_confirmation(monkeypatch, tmp_path: Path) -> None:
    install = Mock()
    monkeypatch.setattr(
        a1z_cli,
        "_python_dependency_commands",
        Mock(
            return_value=[
                (["uv", "pip", "install", "a1z @ git+https://example.test/a1z"], tmp_path)
            ]
        ),
    )
    monkeypatch.setattr(a1z_cli, "_install_python_dependencies", install)
    monkeypatch.setattr(a1z_cli.typer, "confirm", Mock(return_value=False))

    result = runner.invoke(a1z_cli.app, ["setup", "--sdk-only"])

    assert result.exit_code == 1
    assert "uv pip install 'a1z @ git+https://example.test/a1z'" in result.output
    assert "Aborted." in result.output
    install.assert_not_called()


def test_setup_reports_sdk_failure_after_installing(monkeypatch) -> None:
    install = Mock()
    monkeypatch.setattr(a1z_cli, "_install_python_dependencies", install)
    monkeypatch.setattr(
        a1z_cli,
        "_verify_sdk",
        Mock(side_effect=RuntimeError("install the pinned SDK")),
    )

    result = runner.invoke(a1z_cli.app, ["setup", "--sdk-only", "--yes"])

    assert result.exit_code == 1
    assert "install the pinned SDK" in result.output
    install.assert_called_once_with("Linux")


def test_non_ubuntu_installs_python_then_reports_can_utils(monkeypatch) -> None:
    install = Mock()
    monkeypatch.setattr(a1z_cli, "_install_python_dependencies", install)
    monkeypatch.setattr(a1z_cli, "_verify_sdk", Mock(return_value="/sdk/a1z"))
    monkeypatch.setattr(
        a1z_cli,
        "_system_dependency_plan",
        Mock(return_value=(None, "install the package that provides `cansend`")),
    )
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(a1z_cli.app, ["setup", "--yes"])

    assert result.exit_code == 1
    assert "A1Z Python dependencies installed and verified" in result.output
    assert "install the package that provides `cansend`" in result.output
    install.assert_called_once_with("Linux")
    configure.assert_not_called()


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
    monkeypatch.setattr(a1z_cli, "_install_python_dependencies", Mock())
    monkeypatch.setattr(a1z_cli, "_verify_sdk", Mock(return_value="/sdk/a1z"))
    monkeypatch.setattr(a1z_cli.platform, "system", Mock(return_value="Darwin"))
    monkeypatch.setattr(a1z_cli, "_system_dependency_plan", Mock(return_value=(None, None)))
    verify_macos = Mock()
    monkeypatch.setattr(a1z_cli, "_verify_macos_can", verify_macos)

    result = runner.invoke(a1z_cli.app, ["setup", "--yes"])

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
