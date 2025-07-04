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

import subprocess
import time
from unittest.mock import patch

import pytest

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.protocol.pubsub.lcmpubsub import (
    LCM,
    LCMbase,
    Topic,
    autoconf,
    check_buffers,
    check_multicast,
    pickleLCM,
)


class MockLCMMessage:
    """Mock LCM message for testing"""

    name = "geometry_msgs.Mock"

    def __init__(self, data):
        self.data = data

    def lcm_encode(self) -> bytes:
        return str(self.data).encode("utf-8")

    @classmethod
    def lcm_decode(cls, data: bytes) -> "MockLCMMessage":
        return cls(data.decode("utf-8"))

    def __eq__(self, other):
        return isinstance(other, MockLCMMessage) and self.data == other.data


def test_autoconf():
    autoconf()


def test_lcmbase_pubsub():
    lcm = LCMbase()
    lcm.start()

    received_messages = []

    topic = Topic(topic="/test_topic", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("test_data")

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message.lcm_encode())
    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, bytes)
    assert received_data.decode() == "test_data"

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


def test_lcm_autodecoder_pubsub():
    lcm = LCM()
    lcm.start()

    received_messages = []

    topic = Topic(topic="/test_topic", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("test_data")

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message)
    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, MockLCMMessage)
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


test_msgs = [
    (Vector3(1, 2, 3)),
    (Quaternion(1, 2, 3, 4)),
    (Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))),
]


# passes some geometry types through LCM
@pytest.mark.parametrize("test_message", test_msgs)
def test_lcm_geometry_msgs_pubsub(test_message):
    lcm = LCM()
    lcm.start()

    received_messages = []

    topic = Topic(topic="/test_topic", lcm_type=test_message.__class__)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message)

    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, test_message.__class__)
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic

    print(test_message, topic)


# passes some geometry types through pickle LCM
@pytest.mark.parametrize("test_message", test_msgs)
def test_lcm_geometry_msgs_autopickle_pubsub(test_message):
    lcm = pickleLCM()
    lcm.start()

    received_messages = []

    topic = Topic(topic="/test_topic")

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message)

    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, test_message.__class__)
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic

    print(test_message, topic)


class TestSystemChecks:
    """Test suite for system configuration check functions."""

    def test_check_multicast_all_configured(self):
        """Test check_multicast when system is properly configured."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
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

    def test_check_multicast_missing_multicast_flag(self):
        """Test check_multicast when loopback interface lacks multicast."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
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
            assert result == ["sudo ifconfig lo multicast"]

    def test_check_multicast_missing_route(self):
        """Test check_multicast when multicast route is missing."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
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
            assert result == ["sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo"]

    def test_check_multicast_all_missing(self):
        """Test check_multicast when both multicast flag and route are missing (current system state)."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
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
            expected = [
                "sudo ifconfig lo multicast",
                "sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo",
            ]
            assert result == expected

    def test_check_multicast_subprocess_exception(self):
        """Test check_multicast when subprocess calls fail."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
            # Mock subprocess exceptions
            mock_run.side_effect = Exception("Command failed")

            result = check_multicast()
            expected = [
                "sudo ifconfig lo multicast",
                "sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo",
            ]
            assert result == expected

    def test_check_buffers_all_configured(self):
        """Test check_buffers when system is properly configured."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
            # Mock sufficient buffer sizes
            mock_run.side_effect = [
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 2097152", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 2097152", "returncode": 0}
                )(),
            ]

            result = check_buffers()
            assert result == []

    def test_check_buffers_low_max_buffer(self):
        """Test check_buffers when rmem_max is too low."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
            # Mock low rmem_max
            mock_run.side_effect = [
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 1048576", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 2097152", "returncode": 0}
                )(),
            ]

            result = check_buffers()
            assert result == ["sudo sysctl -w net.core.rmem_max=2097152"]

    def test_check_buffers_low_default_buffer(self):
        """Test check_buffers when rmem_default is too low."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
            # Mock low rmem_default
            mock_run.side_effect = [
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 2097152", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 1048576", "returncode": 0}
                )(),
            ]

            result = check_buffers()
            assert result == ["sudo sysctl -w net.core.rmem_default=2097152"]

    def test_check_buffers_both_low(self):
        """Test check_buffers when both buffer sizes are too low."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
            # Mock both low
            mock_run.side_effect = [
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 1048576", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 1048576", "returncode": 0}
                )(),
            ]

            result = check_buffers()
            expected = [
                "sudo sysctl -w net.core.rmem_max=2097152",
                "sudo sysctl -w net.core.rmem_default=2097152",
            ]
            assert result == expected

    def test_check_buffers_subprocess_exception(self):
        """Test check_buffers when subprocess calls fail."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
            # Mock subprocess exceptions
            mock_run.side_effect = Exception("Command failed")

            result = check_buffers()
            expected = [
                "sudo sysctl -w net.core.rmem_max=2097152",
                "sudo sysctl -w net.core.rmem_default=2097152",
            ]
            assert result == expected

    def test_check_buffers_parsing_error(self):
        """Test check_buffers when output parsing fails."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
            # Mock malformed output
            mock_run.side_effect = [
                type("MockResult", (), {"stdout": "invalid output", "returncode": 0})(),
                type("MockResult", (), {"stdout": "also invalid", "returncode": 0})(),
            ]

            result = check_buffers()
            expected = [
                "sudo sysctl -w net.core.rmem_max=2097152",
                "sudo sysctl -w net.core.rmem_default=2097152",
            ]
            assert result == expected

    def test_autoconf_no_config_needed(self):
        """Test autoconf when no configuration is needed."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
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
                    "MockResult", (), {"stdout": "224.0.0.0/4 dev lo scope link", "returncode": 0}
                )(),
                # check_buffers calls
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_max = 2097152", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 2097152", "returncode": 0}
                )(),
            ]

            with patch("builtins.print") as mock_print:
                autoconf()
                # Should not print anything when no config is needed
                mock_print.assert_not_called()

    def test_autoconf_with_config_needed_success(self):
        """Test autoconf when configuration is needed and commands succeed."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
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
                    "MockResult", (), {"stdout": "net.core.rmem_default = 1048576", "returncode": 0}
                )(),
                # Command execution calls
                type(
                    "MockResult", (), {"stdout": "success", "returncode": 0}
                )(),  # sudo ifconfig lo multicast
                type(
                    "MockResult", (), {"stdout": "success", "returncode": 0}
                )(),  # sudo route add...
                type(
                    "MockResult", (), {"stdout": "success", "returncode": 0}
                )(),  # sudo sysctl rmem_max
                type(
                    "MockResult", (), {"stdout": "success", "returncode": 0}
                )(),  # sudo sysctl rmem_default
            ]

            with patch("builtins.print") as mock_print:
                autoconf()

                # Verify the expected print calls
                expected_calls = [
                    ("System configuration required. Executing commands...",),
                    ("  Running: sudo ifconfig lo multicast",),
                    ("  ✓ Success",),
                    ("  Running: sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo",),
                    ("  ✓ Success",),
                    ("  Running: sudo sysctl -w net.core.rmem_max=2097152",),
                    ("  ✓ Success",),
                    ("  Running: sudo sysctl -w net.core.rmem_default=2097152",),
                    ("  ✓ Success",),
                    ("System configuration completed.",),
                ]
                from unittest.mock import call

                mock_print.assert_has_calls([call(*args) for args in expected_calls])

    def test_autoconf_with_command_failures(self):
        """Test autoconf when some commands fail."""
        with patch("dimos.protocol.pubsub.lcmpubsub.subprocess.run") as mock_run:
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
                    "MockResult", (), {"stdout": "net.core.rmem_max = 2097152", "returncode": 0}
                )(),
                type(
                    "MockResult", (), {"stdout": "net.core.rmem_default = 2097152", "returncode": 0}
                )(),
                # Command execution calls - first succeeds, second fails
                type(
                    "MockResult", (), {"stdout": "success", "returncode": 0}
                )(),  # sudo ifconfig lo multicast
                subprocess.CalledProcessError(
                    1,
                    [
                        "sudo",
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

            with patch("builtins.print") as mock_print:
                autoconf()

                # Verify it handles the failure gracefully
                print_calls = [call[0][0] for call in mock_print.call_args_list]
                assert "System configuration required. Executing commands..." in print_calls
                assert "  ✓ Success" in print_calls  # First command succeeded
                assert any("✗ Failed" in call for call in print_calls)  # Second command failed
                assert "System configuration completed." in print_calls
