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

import os
import resource
import subprocess
from unittest.mock import MagicMock, patch

import pytest

from dimos.protocol.service.system_configurator import (
    IDEAL_RMEM_SIZE,
    BufferConfiguratorLinux,
    BufferConfiguratorMacOS,
    MaxFileConfiguratorMacOS,
    MulticastConfiguratorLinux,
    MulticastConfiguratorMacOS,
    SystemConfigurator,
    _is_root_user,
    _read_sysctl_int,
    _write_sysctl_int,
    configure_system,
    sudo_run,
)

# ----------------------------- Helper function tests -----------------------------


class TestIsRootUser:
    def test_is_root_when_euid_is_zero(self) -> None:
        # Clear the cache before testing
        _is_root_user.cache_clear()
        with patch("os.geteuid", return_value=0):
            assert _is_root_user() is True

    def test_is_not_root_when_euid_is_nonzero(self) -> None:
        _is_root_user.cache_clear()
        with patch("os.geteuid", return_value=1000):
            assert _is_root_user() is False

    def test_returns_false_when_geteuid_not_available(self) -> None:
        _is_root_user.cache_clear()
        with patch("os.geteuid", side_effect=AttributeError):
            assert _is_root_user() is False


class TestSudoRun:
    def test_runs_without_sudo_when_root(self) -> None:
        _is_root_user.cache_clear()
        with patch("os.geteuid", return_value=0):
            with patch("subprocess.run") as mock_run:
                mock_run.return_value = MagicMock(returncode=0)
                sudo_run("echo", "hello", check=True)
                mock_run.assert_called_once_with(["echo", "hello"], check=True)

    def test_runs_with_sudo_when_not_root(self) -> None:
        _is_root_user.cache_clear()
        with patch("os.geteuid", return_value=1000):
            with patch("subprocess.run") as mock_run:
                mock_run.return_value = MagicMock(returncode=0)
                sudo_run("echo", "hello", check=True)
                mock_run.assert_called_once_with(["sudo", "echo", "hello"], check=True)


class TestReadSysctlInt:
    def test_reads_value_with_equals_sign(self) -> None:
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=0, stdout="net.core.rmem_max = 67108864")
            result = _read_sysctl_int("net.core.rmem_max")
            assert result == 67108864

    def test_reads_value_with_colon(self) -> None:
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=0, stdout="kern.ipc.maxsockbuf: 8388608")
            result = _read_sysctl_int("kern.ipc.maxsockbuf")
            assert result == 8388608

    def test_returns_none_on_nonzero_returncode(self) -> None:
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=1, stderr="error")
            result = _read_sysctl_int("net.core.rmem_max")
            assert result is None

    def test_returns_none_on_malformed_output(self) -> None:
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=0, stdout="invalid output")
            result = _read_sysctl_int("net.core.rmem_max")
            assert result is None

    def test_returns_none_on_exception(self) -> None:
        with patch("subprocess.run", side_effect=Exception("Command failed")):
            result = _read_sysctl_int("net.core.rmem_max")
            assert result is None


class TestWriteSysctlInt:
    def test_calls_sudo_run_with_correct_args(self) -> None:
        _is_root_user.cache_clear()
        with patch("os.geteuid", return_value=1000):
            with patch("subprocess.run") as mock_run:
                mock_run.return_value = MagicMock(returncode=0)
                _write_sysctl_int("net.core.rmem_max", 67108864)
                mock_run.assert_called_once_with(
                    ["sudo", "sysctl", "-w", "net.core.rmem_max=67108864"],
                    check=True,
                    text=True,
                    capture_output=False,
                )


# ----------------------------- configure_system tests -----------------------------


class MockConfigurator(SystemConfigurator):
    """A mock configurator for testing configure_system."""

    def __init__(self, passes: bool = True, is_critical: bool = False) -> None:
        self._passes = passes
        self.critical = is_critical
        self.fix_called = False

    def check(self) -> bool:
        return self._passes

    def explanation(self) -> str | None:
        if self._passes:
            return None
        return "Mock explanation"

    def fix(self) -> None:
        self.fix_called = True


class TestConfigureSystem:
    def test_skips_in_ci_environment(self) -> None:
        with patch.dict(os.environ, {"CI": "true"}):
            mock_check = MockConfigurator(passes=False)
            configure_system([mock_check])
            assert not mock_check.fix_called

    def test_does_nothing_when_all_checks_pass(self) -> None:
        with patch.dict(os.environ, {"CI": ""}, clear=False):
            mock_check = MockConfigurator(passes=True)
            configure_system([mock_check])
            assert not mock_check.fix_called

    def test_check_only_mode_does_not_fix(self) -> None:
        with patch.dict(os.environ, {"CI": ""}, clear=False):
            mock_check = MockConfigurator(passes=False)
            configure_system([mock_check], check_only=True)
            assert not mock_check.fix_called

    def test_prompts_user_and_fixes_on_yes(self) -> None:
        with patch.dict(os.environ, {"CI": ""}, clear=False):
            mock_check = MockConfigurator(passes=False)
            with patch("builtins.input", return_value="y"):
                configure_system([mock_check])
                assert mock_check.fix_called

    def test_does_not_fix_on_no(self) -> None:
        with patch.dict(os.environ, {"CI": ""}, clear=False):
            mock_check = MockConfigurator(passes=False)
            with patch("builtins.input", return_value="n"):
                configure_system([mock_check])
                assert not mock_check.fix_called

    def test_exits_on_no_with_critical_check(self) -> None:
        with patch.dict(os.environ, {"CI": ""}, clear=False):
            mock_check = MockConfigurator(passes=False, is_critical=True)
            with patch("builtins.input", return_value="n"):
                with pytest.raises(SystemExit) as exc_info:
                    configure_system([mock_check])
                assert exc_info.value.code == 1

    def test_handles_eof_error_on_input(self) -> None:
        with patch.dict(os.environ, {"CI": ""}, clear=False):
            mock_check = MockConfigurator(passes=False)
            with patch("builtins.input", side_effect=EOFError):
                configure_system([mock_check])
                assert not mock_check.fix_called


# ----------------------------- MulticastConfiguratorLinux tests -----------------------------


class TestMulticastConfiguratorLinux:
    def test_check_returns_true_when_fully_configured(self) -> None:
        configurator = MulticastConfiguratorLinux()
        with patch("subprocess.run") as mock_run:
            mock_run.side_effect = [
                MagicMock(returncode=0),  # ip -V
                MagicMock(
                    returncode=0,
                    stdout="1: lo: <LOOPBACK,UP,LOWER_UP,MULTICAST> mtu 65536",
                ),
                MagicMock(returncode=0, stdout="224.0.0.0/4 dev lo scope link"),
            ]
            assert configurator.check() is True
            assert configurator.loopback_ok is True
            assert configurator.route_ok is True

    def test_check_returns_false_when_multicast_flag_missing(self) -> None:
        configurator = MulticastConfiguratorLinux()
        with patch("subprocess.run") as mock_run:
            mock_run.side_effect = [
                MagicMock(returncode=0),  # ip -V
                MagicMock(returncode=0, stdout="1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536"),
                MagicMock(returncode=0, stdout="224.0.0.0/4 dev lo scope link"),
            ]
            assert configurator.check() is False
            assert configurator.loopback_ok is False
            assert configurator.route_ok is True

    def test_check_returns_false_when_route_missing(self) -> None:
        configurator = MulticastConfiguratorLinux()
        with patch("subprocess.run") as mock_run:
            mock_run.side_effect = [
                MagicMock(returncode=0),  # ip -V
                MagicMock(
                    returncode=0,
                    stdout="1: lo: <LOOPBACK,UP,LOWER_UP,MULTICAST> mtu 65536",
                ),
                MagicMock(returncode=0, stdout=""),  # Empty - no route
            ]
            assert configurator.check() is False
            assert configurator.loopback_ok is True
            assert configurator.route_ok is False

    def test_check_returns_false_when_ip_not_found(self) -> None:
        configurator = MulticastConfiguratorLinux()
        with patch("subprocess.run", side_effect=FileNotFoundError):
            assert configurator.check() is False
            assert configurator.loopback_ok is False
            assert configurator.route_ok is False

    def test_explanation_includes_needed_commands(self) -> None:
        configurator = MulticastConfiguratorLinux()
        configurator.loopback_ok = False
        configurator.route_ok = False
        explanation = configurator.explanation()
        assert "ip link set lo multicast on" in explanation
        assert "ip route add 224.0.0.0/4 dev lo" in explanation

    def test_fix_runs_needed_commands(self) -> None:
        _is_root_user.cache_clear()
        configurator = MulticastConfiguratorLinux()
        configurator.loopback_ok = False
        configurator.route_ok = False
        with patch("os.geteuid", return_value=0):
            with patch("subprocess.run") as mock_run:
                mock_run.return_value = MagicMock(returncode=0)
                configurator.fix()
                assert mock_run.call_count == 2


# ----------------------------- MulticastConfiguratorMacOS tests -----------------------------


class TestMulticastConfiguratorMacOS:
    def test_check_returns_true_when_route_exists(self) -> None:
        configurator = MulticastConfiguratorMacOS()
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(
                returncode=0,
                stdout="224.0.0.0/4          link#1             UCS             lo0",
            )
            assert configurator.check() is True

    def test_check_returns_false_when_route_missing(self) -> None:
        configurator = MulticastConfiguratorMacOS()
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(
                returncode=0, stdout="default    192.168.1.1    UGScg    en0"
            )
            assert configurator.check() is False

    def test_check_returns_false_on_netstat_error(self) -> None:
        configurator = MulticastConfiguratorMacOS()
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(returncode=1, stderr="error")
            assert configurator.check() is False

    def test_explanation_includes_route_command(self) -> None:
        configurator = MulticastConfiguratorMacOS()
        explanation = configurator.explanation()
        assert "route add -net 224.0.0.0/4 -interface lo0" in explanation

    def test_fix_runs_route_command(self) -> None:
        _is_root_user.cache_clear()
        configurator = MulticastConfiguratorMacOS()
        with patch("os.geteuid", return_value=0):
            with patch("subprocess.run") as mock_run:
                mock_run.return_value = MagicMock(returncode=0)
                configurator.fix()
                mock_run.assert_called_once()
                args = mock_run.call_args[0][0]
                assert "route" in args
                assert "224.0.0.0/4" in args


# ----------------------------- BufferConfiguratorLinux tests -----------------------------


class TestBufferConfiguratorLinux:
    def test_check_returns_true_when_buffers_sufficient(self) -> None:
        configurator = BufferConfiguratorLinux()
        with patch("dimos.protocol.service.system_configurator._read_sysctl_int") as mock_read:
            mock_read.return_value = IDEAL_RMEM_SIZE
            assert configurator.check() is True
            assert configurator.needs == []

    def test_check_returns_false_when_rmem_max_low(self) -> None:
        configurator = BufferConfiguratorLinux()
        with patch("dimos.protocol.service.system_configurator._read_sysctl_int") as mock_read:
            mock_read.side_effect = [1048576, IDEAL_RMEM_SIZE]  # rmem_max low
            assert configurator.check() is False
            assert len(configurator.needs) == 1
            assert configurator.needs[0][0] == "net.core.rmem_max"

    def test_check_returns_false_when_both_low(self) -> None:
        configurator = BufferConfiguratorLinux()
        with patch("dimos.protocol.service.system_configurator._read_sysctl_int") as mock_read:
            mock_read.return_value = 1048576  # Both low
            assert configurator.check() is False
            assert len(configurator.needs) == 2

    def test_explanation_lists_needed_changes(self) -> None:
        configurator = BufferConfiguratorLinux()
        configurator.needs = [("net.core.rmem_max", IDEAL_RMEM_SIZE)]
        explanation = configurator.explanation()
        assert "net.core.rmem_max" in explanation
        assert str(IDEAL_RMEM_SIZE) in explanation

    def test_fix_writes_needed_values(self) -> None:
        configurator = BufferConfiguratorLinux()
        configurator.needs = [("net.core.rmem_max", IDEAL_RMEM_SIZE)]
        with patch("dimos.protocol.service.system_configurator._write_sysctl_int") as mock_write:
            configurator.fix()
            mock_write.assert_called_once_with("net.core.rmem_max", IDEAL_RMEM_SIZE)


# ----------------------------- BufferConfiguratorMacOS tests -----------------------------


class TestBufferConfiguratorMacOS:
    def test_check_returns_true_when_buffers_sufficient(self) -> None:
        configurator = BufferConfiguratorMacOS()
        with patch("dimos.protocol.service.system_configurator._read_sysctl_int") as mock_read:
            mock_read.side_effect = [
                BufferConfiguratorMacOS.TARGET_BUFFER_SIZE,
                BufferConfiguratorMacOS.TARGET_RECVSPACE,
                BufferConfiguratorMacOS.TARGET_DGRAM_SIZE,
            ]
            assert configurator.check() is True
            assert configurator.needs == []

    def test_check_returns_false_when_values_low(self) -> None:
        configurator = BufferConfiguratorMacOS()
        with patch("dimos.protocol.service.system_configurator._read_sysctl_int") as mock_read:
            mock_read.return_value = 1024  # All low
            assert configurator.check() is False
            assert len(configurator.needs) == 3

    def test_explanation_lists_needed_changes(self) -> None:
        configurator = BufferConfiguratorMacOS()
        configurator.needs = [
            ("kern.ipc.maxsockbuf", BufferConfiguratorMacOS.TARGET_BUFFER_SIZE),
        ]
        explanation = configurator.explanation()
        assert "kern.ipc.maxsockbuf" in explanation

    def test_fix_writes_needed_values(self) -> None:
        configurator = BufferConfiguratorMacOS()
        configurator.needs = [
            ("kern.ipc.maxsockbuf", BufferConfiguratorMacOS.TARGET_BUFFER_SIZE),
        ]
        with patch("dimos.protocol.service.system_configurator._write_sysctl_int") as mock_write:
            configurator.fix()
            mock_write.assert_called_once_with(
                "kern.ipc.maxsockbuf", BufferConfiguratorMacOS.TARGET_BUFFER_SIZE
            )


# ----------------------------- MaxFileConfiguratorMacOS tests -----------------------------


class TestMaxFileConfiguratorMacOS:
    def test_check_returns_true_when_soft_limit_sufficient(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        with patch("resource.getrlimit") as mock_getrlimit:
            mock_getrlimit.return_value = (65536, 1048576)
            assert configurator.check() is True
            assert configurator.current_soft == 65536
            assert configurator.current_hard == 1048576

    def test_check_returns_false_when_soft_limit_low(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        with patch("resource.getrlimit") as mock_getrlimit:
            mock_getrlimit.return_value = (256, 1048576)
            assert configurator.check() is False
            assert configurator.can_fix_without_sudo is True

    def test_check_returns_false_when_both_limits_low(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        with patch("resource.getrlimit") as mock_getrlimit:
            mock_getrlimit.return_value = (256, 10240)
            assert configurator.check() is False
            assert configurator.can_fix_without_sudo is False

    def test_check_returns_false_on_exception(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        with patch("resource.getrlimit", side_effect=Exception("error")):
            assert configurator.check() is False

    def test_explanation_when_sudo_not_needed(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        configurator.current_soft = 256
        configurator.current_hard = 1048576
        configurator.can_fix_without_sudo = True
        explanation = configurator.explanation()
        assert "65536" in explanation
        assert "no sudo" in explanation.lower() or "Raise soft" in explanation

    def test_explanation_when_sudo_needed(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        configurator.current_soft = 256
        configurator.current_hard = 10240
        configurator.can_fix_without_sudo = False
        explanation = configurator.explanation()
        assert "launchctl" in explanation

    def test_fix_raises_soft_limit_without_sudo(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        configurator.current_soft = 256
        configurator.current_hard = 1048576
        configurator.can_fix_without_sudo = True
        with patch("resource.setrlimit") as mock_setrlimit:
            configurator.fix()
            mock_setrlimit.assert_called_once_with(resource.RLIMIT_NOFILE, (65536, 1048576))

    def test_fix_does_nothing_when_already_sufficient(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        configurator.current_soft = 65536
        configurator.current_hard = 1048576
        with patch("resource.setrlimit") as mock_setrlimit:
            configurator.fix()
            mock_setrlimit.assert_not_called()

    def test_fix_uses_launchctl_when_hard_limit_low(self) -> None:
        _is_root_user.cache_clear()
        configurator = MaxFileConfiguratorMacOS(target=65536)
        configurator.current_soft = 256
        configurator.current_hard = 10240
        configurator.can_fix_without_sudo = False
        with patch("os.geteuid", return_value=0):
            with patch("subprocess.run") as mock_run:
                mock_run.return_value = MagicMock(returncode=0)
                with patch("resource.setrlimit"):
                    configurator.fix()
                    # Check launchctl was called
                    args = mock_run.call_args[0][0]
                    assert "launchctl" in args
                    assert "maxfiles" in args

    def test_fix_raises_on_setrlimit_error(self) -> None:
        configurator = MaxFileConfiguratorMacOS(target=65536)
        configurator.current_soft = 256
        configurator.current_hard = 1048576
        configurator.can_fix_without_sudo = True
        with patch("resource.setrlimit", side_effect=ValueError("test error")):
            with pytest.raises(ValueError):
                configurator.fix()
