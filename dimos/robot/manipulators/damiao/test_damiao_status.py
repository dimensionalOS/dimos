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

"""A Damiao connection's status reaches RPC clients that have no motor SDK."""

from __future__ import annotations

import subprocess
import sys

from dimos.protocol.encode.cbor import encode
from dimos.robot.manipulators.calibration.hooks import AppliedCalibration
from dimos.robot.manipulators.damiao import connection
from dimos.robot.manipulators.damiao.status import DamiaoConnectionStatus

# Decode stdin where neither the Damiao SDK nor pinocchio can be imported.
DECODE_WITHOUT_SDK = """
import sys
sys.modules["can_motor_control"] = None
sys.modules["pinocchio"] = None
from dimos.protocol.encode.cbor import decode
status = decode(sys.stdin.buffer.read())
print(type(status).__module__, type(status).__name__)
print(status.calibration["rig/left"]["path"], status.calibration["rig/right"])
"""


def test_a_status_decodes_where_the_motor_sdk_is_missing():
    status = DamiaoConnectionStatus(
        session_id="s",
        connected=True,
        last_command_time=None,
        deadman_fired=False,
        last_rejection=None,
        last_error=None,
        calibration={
            "rig/left": AppliedCalibration(
                path="/arms/rig/left/calibration.json",
                created="2026-11-17T09:30:00+00:00",
                gains=["joint1"],
                links=["link2"],
            ),
            "rig/right": None,
        },
    )
    result = subprocess.run(
        [sys.executable, "-c", DECODE_WITHOUT_SDK],
        input=encode(status),
        capture_output=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr.decode()
    assert result.stdout.decode().splitlines() == [
        "dimos.robot.manipulators.damiao.status DamiaoConnectionStatus",
        "/arms/rig/left/calibration.json None",
    ]


def test_the_connection_module_still_exports_its_status():
    assert connection.DamiaoConnectionStatus is DamiaoConnectionStatus
