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


"""The G1 and its Mid-360 in a built world: the rig as SimG1World assembles it."""

from __future__ import annotations

from functools import partial

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.simulation.g1_sim.body import LeggedG1
from dimos.simulation.g1_sim.policy import G1Policy
from dimos.simulation.g1_sim.world import G1_RIG, MID360, STAND_HEIGHT
from dimos.simulation.scenes.procedural import Scene, office
from dimos.simulation.sim_world.world import FRAME_DT, LidarFrame, Sim

pytestmark = pytest.mark.self_hosted

STILL = np.zeros(3)
# two seconds, which is where the stance settles and where the cloud used to disappear
SETTLE_TICKS = 100


@pytest.fixture(scope="module")
def policy() -> G1Policy:
    return G1Policy.load(LeggedG1.dt)


@pytest.fixture(scope="module")
def scene() -> Scene:
    return office(1)


@pytest.fixture
def sim(scene: Scene, policy: G1Policy) -> Sim:
    sim = Sim(scene, 1, G1_RIG, partial(LeggedG1, policy=policy))
    sim.reset(*scene.start, 0.0)
    return sim


def _next_frame(sim: Sim, command: NDArray[np.float64]) -> LidarFrame:
    for _ in range(sim.ticks_per_frame):
        if (frame := sim.tick(command)) is not None:
            return frame
    raise AssertionError("no lidar frame within one frame period")


def test_frames_come_out_at_10hz_in_the_sensor_frame(sim: Sim) -> None:
    """The rig's contract, same as the Go2's: one frame per FRAME_DT, in the sensor's own frame."""
    frames = [f for _ in range(10) if (f := sim.tick(STILL)) is not None]
    assert [f.t for f in frames] == pytest.approx([FRAME_DT, 2 * FRAME_DT])
    assert 12_000 < len(frames[-1].points) < 20_000


def test_the_cloud_survives_the_stance_settling(sim: Sim) -> None:
    """The one that was missing. The G1's lens starts just clear of its own head shell and the
    settling stance pulls it 70 mm inside, so every return came back at about 0.02 m -- under the
    Mid-360's 0.16 m blind range. Frames at t=0 were full and every frame after was empty, which
    is why the sim looked like it worked.

    The head mesh is a geom on torso_link, so the rig names that body as the shell its lens looks
    out of and the cast skips it.
    """
    for _ in range(SETTLE_TICKS):
        sim.tick(STILL)

    frame = _next_frame(sim, STILL)
    assert len(frame.points) > 12_000, "the robot is scanning the inside of its own head"
    ranges = np.linalg.norm(frame.points.astype(np.float64), axis=1)
    assert ranges.min() > 0.15, "a return closer than the Mid-360's blind range"
    assert ranges.max() > 2.0, "it saw nothing but itself"


def test_the_mid360_reports_from_its_mount_on_the_torso(sim: Sim) -> None:
    """Odometry reports the sensor, not the base, so the mount has to be in the pose it gives."""
    for _ in range(SETTLE_TICKS):
        sim.tick(STILL)

    base, _ = sim.base_pose()
    sensor, _ = sim.odom_pose()
    assert (
        abs(float(np.linalg.norm(sensor - base)) - float(np.linalg.norm(MID360.mount_xyz))) < 1e-9
    )
    assert sensor[2] > base[2] + 0.4, "the Mid-360 is not above the pelvis"
    assert abs(base[2] - STAND_HEIGHT) < 0.1, "it is not standing"
