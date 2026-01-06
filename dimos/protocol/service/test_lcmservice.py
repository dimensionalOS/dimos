# Copyright 2025 Dimensional Inc.
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
import subprocess
from unittest.mock import patch

import pytest

from dimos.protocol.service.lcmservice import (
    TARGET_MAX_DGRAM_SIZE_MACOS,
    TARGET_MAX_SOCKET_BUFFER_SIZE_MACOS,
    TARGET_RMEM_SIZE,
    autoconf,
    check_buffers,
    check_multicast,
    check_root,
)


def get_sudo_prefix() -> str:
    """Return 'sudo ' if not running as root, empty string if running as root."""
    return "" if check_root() else "sudo "


def test_check_multicast_all_configured() -> None:
    """Test check_multicast when system is properly configured."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock successful checks with realistic output format
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": "1: lo: <LOOPBACK,UP,LOWER_UP,MULTICAST> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000\n    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00",
                        "returncode": 0,
                    },
                )(),
                type(
                    "MockResult", (), {"stdout": "224.0.0.0/4 dev lo scope link", "returncode": 0}
                )(),
            ]

            result = check_multicast()
            assert result == []


def test_check_multicast_missing_multicast_flag() -> None:
    """Test check_multicast when loopback interface lacks multicast."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock interface without MULTICAST flag (realistic current system state)
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000\n    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00",
                        "returncode": 0,
                    },
                )(),
                type(
                    "MockResult", (), {"stdout": "224.0.0.0/4 dev lo scope link", "returncode": 0}
                )(),
            ]

            result = check_multicast()
            sudo = get_sudo_prefix()
            assert result == [f"{sudo}ifconfig lo multicast"]


def test_check_multicast_missing_route() -> None:
    """Test check_multicast when multicast route is missing."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock missing route - interface has multicast but no route
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": "1: lo: <LOOPBACK,UP,LOWER_UP,MULTICAST> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000\n    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00",
                        "returncode": 0,
                    },
                )(),
                type(
                    "MockResult", (), {"stdout": "", "returncode": 0}
                )(),  # Empty output - no route
            ]

            result = check_multicast()
            sudo = get_sudo_prefix()
            assert result == [f"{sudo}route add -net 224.0.0.0 netmask 240.0.0.0 dev lo"]


def test_check_multicast_all_missing() -> None:
    """Test check_multicast when both multicast flag and route are missing (current system state)."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock both missing - matches actual current system state
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000\n    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00",
                        "returncode": 0,
                    },
                )(),
                type(
                    "MockResult", (), {"stdout": "", "returncode": 0}
                )(),  # Empty output - no route
            ]

            result = check_multicast()
            sudo = get_sudo_prefix()
            expected = [
                f"{sudo}ifconfig lo multicast",
                f"{sudo}route add -net 224.0.0.0 netmask 240.0.0.0 dev lo",
            ]
            assert result == expected


def test_check_multicast_subprocess_exception() -> None:
    """Test check_multicast when subprocess calls fail."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock subprocess exceptions
            mock_run.side_effect = Exception("Command failed")

            result = check_multicast()
            sudo = get_sudo_prefix()
            expected = [
                f"{sudo}ifconfig lo multicast",
                f"{sudo}route add -net 224.0.0.0 netmask 240.0.0.0 dev lo",
            ]
            assert result == expected


def test_check_multicast_macos() -> None:
    """Test check_multicast on macOS when configuration is needed."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Darwin"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock netstat -nr to not contain the multicast route
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": "default            192.168.1.1        UGScg         en0",
                        "returncode": 0,
                    },
                )(),
            ]

            result = check_multicast()
            sudo = get_sudo_prefix()
            expected = [f"{sudo}route add -net 224.0.0.0/4 -interface lo0"]
            assert result == expected


def test_check_buffers_all_configured() -> None:
    """Test check_buffers when system is properly configured."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock sufficient buffer sizes
            mock_run.side_effect = [
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 67108864", "returncode": 0}
                )(),
                type(
                    "MockResult",
                    (),
                    {"stdout": "net.core.rmem_default = 16777216", "returncode": 0},
                )(),
            ]

            commands, buffer_size = check_buffers()
            assert commands == []
            assert buffer_size >= TARGET_RMEM_SIZE


def test_check_buffers_low_max_buffer() -> None:
    """Test check_buffers when rmem_max is too low."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock low rmem_max
            mock_run.side_effect = [
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 1048576", "returncode": 0}
                )(),
                type(
                    "MockResult",
                    (),
                    {"stdout": f"net.core.rmem_default = {TARGET_RMEM_SIZE}", "returncode": 0},
                )(),
            ]

            commands, buffer_size = check_buffers()
            sudo = get_sudo_prefix()
            assert commands == [f"{sudo}sysctl -w net.core.rmem_max={TARGET_RMEM_SIZE}"]
            assert buffer_size == 1048576


def test_check_buffers_low_default_buffer() -> None:
    """Test check_buffers when rmem_default is too low."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock low rmem_default
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {"stdout": f"net.core.rmem_max = {TARGET_RMEM_SIZE}", "returncode": 0},
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 1048576", "returncode": 0}
                )(),
            ]

            commands, buffer_size = check_buffers()
            sudo = get_sudo_prefix()
            assert commands == [f"{sudo}sysctl -w net.core.rmem_default={TARGET_RMEM_SIZE}"]
            assert buffer_size == TARGET_RMEM_SIZE


def test_check_buffers_both_low() -> None:
    """Test check_buffers when both buffer sizes are too low."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock both low
            mock_run.side_effect = [
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 1048576", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 1048576", "returncode": 0}
                )(),
            ]

            commands, buffer_size = check_buffers()
            sudo = get_sudo_prefix()
            expected = [
                f"{sudo}sysctl -w net.core.rmem_max={TARGET_RMEM_SIZE}",
                f"{sudo}sysctl -w net.core.rmem_default={TARGET_RMEM_SIZE}",
            ]
            assert commands == expected
            assert buffer_size == 1048576


def test_check_buffers_subprocess_exception() -> None:
    """Test check_buffers when subprocess calls fail."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock subprocess exceptions
            mock_run.side_effect = Exception("Command failed")

            commands, buffer_size = check_buffers()
            sudo = get_sudo_prefix()
            expected = [
                f"{sudo}sysctl -w net.core.rmem_max={TARGET_RMEM_SIZE}",
                f"{sudo}sysctl -w net.core.rmem_default={TARGET_RMEM_SIZE}",
            ]
            assert commands == expected
            assert buffer_size is None


def test_check_buffers_parsing_error() -> None:
    """Test check_buffers when output parsing fails."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock malformed output
            mock_run.side_effect = [
                type("MockResult", (), {"stdout": "invalid output", "returncode": 0})(),
                type("MockResult", (), {"stdout": "also invalid", "returncode": 0})(),
            ]

            commands, buffer_size = check_buffers()
            sudo = get_sudo_prefix()
            expected = [
                f"{sudo}sysctl -w net.core.rmem_max={TARGET_RMEM_SIZE}",
                f"{sudo}sysctl -w net.core.rmem_default={TARGET_RMEM_SIZE}",
            ]
            assert commands == expected
            assert buffer_size is None


def test_check_buffers_dev_container() -> None:
    """Test check_buffers in dev container where sysctl fails."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock dev container behavior - sysctl returns non-zero
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": "sysctl: cannot stat /proc/sys/net/core/rmem_max: No such file or directory",
                        "returncode": 255,
                    },
                )(),
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": "sysctl: cannot stat /proc/sys/net/core/rmem_default: No such file or directory",
                        "returncode": 255,
                    },
                )(),
            ]

            commands, buffer_size = check_buffers()
            sudo = get_sudo_prefix()
            expected = [
                f"{sudo}sysctl -w net.core.rmem_max={TARGET_RMEM_SIZE}",
                f"{sudo}sysctl -w net.core.rmem_default={TARGET_RMEM_SIZE}",
            ]
            assert commands == expected
            assert buffer_size is None


def test_check_buffers_macos_all_configured() -> None:
    """Test check_buffers on macOS when system is properly configured."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Darwin"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            # Mock sufficient buffer sizes for macOS
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": f"kern.ipc.maxsockbuf: {TARGET_MAX_SOCKET_BUFFER_SIZE_MACOS}",
                        "returncode": 0,
                    },
                )(),
                type(
                    "MockResult",
                    (),
                    {"stdout": f"net.inet.udp.recvspace: {TARGET_RMEM_SIZE}", "returncode": 0},
                )(),
                type(
                    "MockResult",
                    (),
                    {
                        "stdout": f"net.inet.udp.maxdgram: {TARGET_MAX_DGRAM_SIZE_MACOS}",
                        "returncode": 0,
                    },
                )(),
            ]

            commands, buffer_size = check_buffers()
            assert commands == []
            assert buffer_size == TARGET_MAX_SOCKET_BUFFER_SIZE_MACOS


def test_check_buffers_macos_needs_config() -> None:
    """Test check_buffers on macOS when configuration is needed."""
    with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Darwin"):
        with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
            mock_max_sock_buf_size = 4194304
            # Mock low buffer sizes for macOS
            mock_run.side_effect = [
                type(
                    "MockResult",
                    (),
                    {"stdout": f"kern.ipc.maxsockbuf: {mock_max_sock_buf_size}", "returncode": 0},
                )(),
                type(
                    "MockResult", (), {"stdout": "net.inet.udp.recvspace: 1048576", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.inet.udp.maxdgram: 32768", "returncode": 0}
                )(),
            ]

            commands, buffer_size = check_buffers()
            sudo = get_sudo_prefix()
            expected = [
                f"{sudo}sysctl -w kern.ipc.maxsockbuf={TARGET_MAX_SOCKET_BUFFER_SIZE_MACOS}",
                f"{sudo}sysctl -w net.inet.udp.recvspace={TARGET_RMEM_SIZE}",
                f"{sudo}sysctl -w net.inet.udp.maxdgram={TARGET_MAX_DGRAM_SIZE_MACOS}",
            ]
            assert commands == expected
            assert buffer_size == mock_max_sock_buf_size


def test_autoconf_no_config_needed() -> None:
    """Test autoconf when no configuration is needed."""
    # Clear CI environment variable for this test
    with patch.dict(os.environ, {"CI": ""}, clear=False):
        with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
            with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
                # Mock all checks passing
                mock_run.side_effect = [
                    # check_multicast calls
                    type(
                        "MockResult",
                        (),
                        {
                            "stdout": "1: lo: <LOOPBACK,UP,LOWER_UP,MULTICAST> mtu 65536",
                            "returncode": 0,
                        },
                    )(),
                    type(
                        "MockResult",
                        (),
                        {"stdout": "224.0.0.0/4 dev lo scope link", "returncode": 0},
                    )(),
                    # check_buffers calls
                    type(
                        "MockResult",
                        (),
                        {"stdout": f"net.core.rmem_max = {TARGET_RMEM_SIZE}", "returncode": 0},
                    )(),
                    type(
                        "MockResult",
                        (),
                        {"stdout": f"net.core.rmem_default = {TARGET_RMEM_SIZE}", "returncode": 0},
                    )(),
                ]

                with patch("dimos.protocol.service.lcmservice.logger") as mock_logger:
                    autoconf()
                    # Should not log anything when no config is needed
                    mock_logger.info.assert_not_called()
                mock_logger.error.assert_not_called()
                mock_logger.warning.assert_not_called()


def test_autoconf_with_config_needed_success() -> None:
    """Test autoconf when configuration is needed and commands succeed."""
    # Clear CI environment variable for this test
    with patch.dict(os.environ, {"CI": ""}, clear=False):
        with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
            with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
                # Mock checks failing, then mock the execution succeeding
                mock_run.side_effect = [
                    # check_multicast calls
                    type(
                        "MockResult",
                        (),
                        {"stdout": "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536", "returncode": 0},
                    )(),
                    type("MockResult", (), {"stdout": "", "returncode": 0})(),
                    # check_buffers calls
                    type(
                        "MockResult", (), {"stdout": "net.core.rmem_max = 1048576", "returncode": 0}
                    )(),
                    type(
                        "MockResult",
                        (),
                        {"stdout": "net.core.rmem_default = 1048576", "returncode": 0},
                    )(),
                    # Command execution calls
                    type(
                        "MockResult", (), {"stdout": "success", "returncode": 0}
                    )(),  # ifconfig lo multicast
                    type(
                        "MockResult", (), {"stdout": "success", "returncode": 0}
                    )(),  # route add...
                    type(
                        "MockResult", (), {"stdout": "success", "returncode": 0}
                    )(),  # sysctl rmem_max
                    type(
                        "MockResult", (), {"stdout": "success", "returncode": 0}
                    )(),  # sysctl rmem_default
                ]

                from unittest.mock import call

                with patch("dimos.protocol.service.lcmservice.logger") as mock_logger:
                    autoconf()

                    sudo = get_sudo_prefix()
                    # Verify the expected log calls
                    expected_info_calls = [
                        call("System configuration required. Executing commands..."),
                        call(f"  Running: {sudo}ifconfig lo multicast"),
                        call("  ✓ Success"),
                        call(f"  Running: {sudo}route add -net 224.0.0.0 netmask 240.0.0.0 dev lo"),
                        call("  ✓ Success"),
                        call(f"  Running: {sudo}sysctl -w net.core.rmem_max={TARGET_RMEM_SIZE}"),
                        call("  ✓ Success"),
                        call(
                            f"  Running: {sudo}sysctl -w net.core.rmem_default={TARGET_RMEM_SIZE}"
                        ),
                        call("  ✓ Success"),
                        call("System configuration completed."),
                    ]

                    mock_logger.info.assert_has_calls(expected_info_calls)


def test_autoconf_with_command_failures() -> None:
    """Test autoconf when some commands fail."""
    # Clear CI environment variable for this test
    with patch.dict(os.environ, {"CI": ""}, clear=False):
        with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
            with patch("dimos.protocol.service.lcmservice.subprocess.run") as mock_run:
                # Mock checks failing, then mock some commands failing
                mock_run.side_effect = [
                    # check_multicast calls
                    type(
                        "MockResult",
                        (),
                        {"stdout": "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536", "returncode": 0},
                    )(),
                    type("MockResult", (), {"stdout": "", "returncode": 0})(),
                    # check_buffers calls (no buffer issues for simpler test)
                    type(
                        "MockResult",
                        (),
                        {"stdout": f"net.core.rmem_max = {TARGET_RMEM_SIZE}", "returncode": 0},
                    )(),
                    type(
                        "MockResult",
                        (),
                        {"stdout": f"net.core.rmem_default = {TARGET_RMEM_SIZE}", "returncode": 0},
                    )(),
                    # Command execution calls - first succeeds, second fails
                    type(
                        "MockResult", (), {"stdout": "success", "returncode": 0}
                    )(),  # ifconfig lo multicast
                    subprocess.CalledProcessError(
                        1,
                        [
                            *get_sudo_prefix().split(),
                            "route",
                            "add",
                            "-net",
                            "224.0.0.0",
                            "netmask",
                            "240.0.0.0",
                            "dev",
                            "lo",
                        ],
                        "Permission denied",
                        "Operation not permitted",
                    ),
                ]

                with patch("dimos.protocol.service.lcmservice.logger") as mock_logger:
                    # The function should raise on multicast/route failures
                    with pytest.raises(subprocess.CalledProcessError):
                        autoconf()

                    # Verify it logged the failure before raising
                    info_calls = [call[0][0] for call in mock_logger.info.call_args_list]
                    error_calls = [call[0][0] for call in mock_logger.error.call_args_list]

                    assert "System configuration required. Executing commands..." in info_calls
                assert "  ✓ Success" in info_calls  # First command succeeded
                assert any(
                    "✗ Failed to configure multicast" in call for call in error_calls
                )  # Second command failed
