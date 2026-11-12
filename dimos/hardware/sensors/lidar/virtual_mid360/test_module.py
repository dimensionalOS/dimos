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

import os
import subprocess
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.core.native_module import NativeModule
from dimos.hardware.sensors.lidar.virtual_mid360 import module as virtual_mid360
from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360


def test_build_without_a_way_to_sudo_fails_before_any_privileged_command(
    mocker: MockerFixture,
) -> None:
    """A Desktop launch: sudo needs a password and there is no terminal to ask on."""
    virtual = VirtualMid360(pcap="capture.pcap", host_ip="192.168.1.5")
    refused = subprocess.CompletedProcess([], 1, b"", b"sudo: a password is required")
    sudo = mocker.patch.object(virtual_mid360.subprocess, "run", return_value=refused)
    real_open = os.open

    def no_terminal(path: Any, *args: Any) -> int:
        if path == "/dev/tty":
            raise OSError("no controlling terminal")
        return real_open(path, *args)

    mocker.patch.object(os, "open", side_effect=no_terminal)
    native_build = mocker.patch.object(NativeModule, "build")
    try:
        with pytest.raises(RuntimeError, match="setup_network=False"):
            virtual.build()
        assert [call.args[0] for call in sudo.call_args_list] == [["sudo", "-n", "true"]]
        native_build.assert_not_called()
    finally:
        virtual.config.setup_network = False  # stop() would tear the interface down with sudo
        virtual.stop()
