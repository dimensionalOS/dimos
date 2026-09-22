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

"""A verified tilted hold can be maintained, but not tipped farther during planning."""

import numpy as np
import pytest

from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.classical_planning import (
    _ApartmentCollisionWorld,
    preserves_cargo_tilt,
)


@pytest.mark.parametrize(
    ("initial", "current", "expected"),
    [(8, 8, True), (8, 14, False), (14, 16, False), (0, 4, True), (0, 8, False)],
)
def test_carry_gate_preserves_existing_tilt_and_rejects_new_tipping(initial, current, expected):
    assert (
        preserves_cargo_tilt(np.cos(np.deg2rad(initial)), np.cos(np.deg2rad(current))) is expected
    )


def test_rrt_edges_check_intermediate_states_in_the_posture_policy_world(mocker):
    planner = mocker.Mock()
    planner.kinematics.world.get_prepared_model.return_value.joint_space = JointSpace(
        (
            JointCoordinate(
                name="joint",
                mechanism_type="revolute",
                topology=CoordinateTopology.INTERVAL,
                lower=-2,
                upper=2,
                max_velocity=10,
                max_acceleration=10,
            ),
        )
    )
    world = _ApartmentCollisionWorld(planner, 0, "left")
    check = mocker.patch.object(
        world, "check_config_collision_free", side_effect=[True, True, False]
    )

    assert not world.check_edge_collision_free(
        JointState(name=["joint"], position=[0]),
        JointState(name=["joint"], position=[0.2]),
        step_size=0.05,
    )

    assert [call.args[0].position[0] for call in check.call_args_list] == pytest.approx(
        [0, 0.05, 0.1]
    )
    planner.kinematics.world.check_edge_collision_free.assert_not_called()
