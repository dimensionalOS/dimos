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

"""The committed G1 layer says what the G1 is, so this is what holds it to that."""

import pytest

pytest.importorskip("pxr")


def test_the_layer_carries_the_gains_the_controller_runs_on() -> None:
    """The gains are in the layer, written there once by `scripts/usd_drives.py`.

    Which means they can go stale: the servo table is where they came from, and nothing stops
    somebody changing it. This is what notices. Run the script again if it fails -- the layer is
    the source of truth for whatever runs the robot, and it has to agree with the controller.
    """
    from dimos.robot.assets.usd import usd_drives
    from dimos.robot.unitree.g1.sim2 import G1Sim, g1_drives

    assert usd_drives(G1Sim.usd.usd) == {
        joint: pytest.approx(gains) for joint, gains in g1_drives().items()
    }


def test_the_lidar_mount_is_a_frame_and_the_pelvis_imu_is_a_site() -> None:
    """Two different things, and it matters which is which.

    The Mid-360's mount is a *link* this module adds, because a manager grafts the sensor's own
    description onto it. The pelvis IMU is a *site* the vendor's MJCF already had and the
    conversion kept, so there is nothing to add and `frame_names()` -- which lists links -- will
    never mention it. A robot reading its own IMU resolves it among the world's sites.
    """
    from dimos.robot.unitree.g1.sim2 import IMU_FRAME, G1Sim
    from dimos.sim2.mujoco.world import compile_world
    from dimos.sim2.usd_layer import usd_layer

    assert G1Sim.lidar_mount_frame in G1Sim.usd.frame_names()
    assert IMU_FRAME not in G1Sim.usd.frame_names()
    assert IMU_FRAME in {site.name for site in compile_world(usd_layer(G1Sim.usd)).sites}
