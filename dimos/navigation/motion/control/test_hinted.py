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

"""What the hinted law asserts against the REFEREE rather than against itself.

Its behavioural portrait lives in ``rust/tests/hinted.rs`` and reaches the
python through the bit-exact parity gate. What cannot live there is anything
that has to agree with a constant the shared domain owns: rust cannot import
``embodiment/base.py``, so a plant number copied into the law is a number that can
silently drift away from the plant.
"""

from dataclasses import replace

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.laws import hinted
from dimos.navigation.motion.embodiment.go2 import GO2


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
    )


def _straight() -> Path:
    return Path(frame_id="world", poses=[_pose(k * 0.1, 0.0) for k in range(40)])


def test_rate_limit_is_the_plants_own_slew() -> None:
    """The limiter models the plant, so it reads the plant: the embodiment's
    ``command_slew``, not a number of the law's own. From a standing start the
    first tick is exactly one nominal tick of slew, on both halves.
    """
    pytest.importorskip("dimos_motion2_tc")  # the rust half, when it is built
    for slew in [GO2.command_slew, (1.0, 0.5, 2.0)]:
        for factory in (hinted.make, hinted.make_rust):
            law = factory(replace(GO2, command_slew=slew))
            tw = law.update(_pose(0.0, 0.0), _straight(), 0.0, None)
            assert abs(tw.linear.x - slew[0] * hinted.NOMINAL_TICK) < 1e-12, (factory, slew)


def test_envelope_clears_the_gait_dead_zone() -> None:
    """The creep must be a speed this plant actually walks at.

    Gait initiation is a bifurcation, not a ramp: below ~0.28 m/s commanded the
    freewalk policy stands. A floor inside that band is what made the seed's
    "careful" episodes time out having never approached geometry.
    """
    lo, hi = GO2.gait_band
    assert lo >= 0.28, "the governor's creep is inside the gait dead zone"
    # and stops short of the 1.0 expert-switch boundary the sim does not model
    assert hi < 1.0


def test_veto_is_not_rate_limited() -> None:
    """A single-pose plan is the planner saying stop, not "coast to a halt"."""
    law = hinted.make()
    for _ in range(5):  # build up a fast standing command first
        law.update(_pose(0.0, 0.0), _straight(), 0.02, None)
    stub = Path(frame_id="world", poses=[_pose(5.0, 5.0, 1.0)])
    tw = law.update(_pose(0.0, 0.0), stub, 0.12, None)
    assert (tw.linear.x, tw.linear.y, tw.angular.z) == (0.0, 0.0, 0.0)


def test_a_stalled_caller_cannot_bank_slew() -> None:
    """A long gap means the caller stalled, not that a huge step is owed."""
    law, path = hinted.make(), _straight()
    # the limiter bounds the STEP, so compare each tick against the one before
    prev = law.update(_pose(0.0, 0.0), path, 0.0, None).linear.x
    nominal_step = abs(law.update(_pose(0.0, 0.0), path, 0.02, None).linear.x - prev)

    law.reset()
    prev = law.update(_pose(0.0, 0.0), path, 0.0, None).linear.x
    # 10 s later: the limiter must integrate MAX_TICK, not the whole stall
    stalled_step = abs(law.update(_pose(0.0, 0.0), path, 10.0, None).linear.x - prev)
    assert stalled_step <= GO2.command_slew[0] * hinted.MAX_TICK + 1e-12
    assert stalled_step > nominal_step, "the cap swallowed the elapsed time entirely"
