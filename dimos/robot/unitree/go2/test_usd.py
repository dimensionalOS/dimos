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

"""The committed Go2 layer says what the Go2 is, so this is what holds it to that."""

import pytest

pytest.importorskip("pxr")


def test_the_layer_carries_the_gains_the_policy_runs_on() -> None:
    """The gains are in the layer, written there once by `scripts/usd_drives.py`, so they can go
    stale if somebody replaces the policy blob. This is what notices; run the script again if it
    fails.
    """
    from dimos.robot.assets.usd import usd_drives
    from dimos.robot.unitree.go2.sim2 import Go2Sim, go2_drives

    assert usd_drives(Go2Sim.usd.usd) == {
        joint: pytest.approx(gains) for joint, gains in go2_drives().items()
    }


def test_every_sensor_mount_the_rig_has_is_a_frame_of_the_layer() -> None:
    """A sensor names one of these as its frame_id, and a manager grafts it onto that frame."""
    from dimos.robot.unitree.go2.sim2 import Go2Sim

    frames = Go2Sim.usd.frame_names()

    assert {Go2Sim.l1_mount_frame, Go2Sim.lidar_mount_frame, Go2Sim.camera_mount_frame} <= frames
