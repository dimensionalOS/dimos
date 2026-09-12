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

"""Path shortening retains actual robot/environment collision constraints."""

from itertools import pairwise
import threading

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.grasping_blueprint import R1ProGraspingSim
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport


@pytest.fixture
def checker():
    model = mujoco.MjModel.from_xml_string("""
    <mujoco><worldbody>
      <body name="base_link" pos="0 0 .3">
        <joint name="r1pro/base_x" type="slide" axis="1 0 0"/>
        <joint name="r1pro/base_y" type="slide" axis="0 1 0"/>
        <joint name="r1pro/base_yaw" type="hinge" axis="0 0 1"/>
        <geom type="box" size=".1 .1 .1"/>
      </body>
      <body name="wall" pos=".5 0 .3"><geom type="box" size=".1 .15 .3"/></body>
      <body name="task_bin" pos="0 0 .6">
        <freejoint name="task_tray_free"/>
        <geom type="box" size=".1 .1 .01" contype="0" conaffinity="0"/>
      </body>
    </worldbody><actuator>
      <position name="r1pro/base_x" joint="r1pro/base_x"/>
      <position name="r1pro/base_y" joint="r1pro/base_y"/>
      <position name="r1pro/base_yaw" joint="r1pro/base_yaw"/>
    </actuator></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return PlanarTransport(model, data, cargo_bodies=())


def test_shortening_does_not_cut_through_a_wall(checker):
    path = [[0.0, 0.0, 0.0], [0.0, 0.5, 0.0], [1.0, 0.5, 0.0], [1.0, 0.0, 0.0]]
    shortened = checker.shorten_path(path)
    assert shortened[0] == path[0]
    assert shortened[-1] == path[-1]
    assert len(shortened) > 2
    assert all(checker.clear_pose_segment(np.array(a), np.array(b)) for a, b in pairwise(shortened))


def test_redundant_native_points_can_be_shortened_when_sweep_is_clear(checker):
    assert checker.shorten_path([[0.0, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.6, 0.0]]) == [
        [0.0, 0.0, 0.0],
        [0.0, 0.6, 0.0],
    ]


def test_invalid_destination_is_not_repaired_into_a_partial_route(checker):
    with pytest.raises(RuntimeError, match="cannot clear"):
        checker.shorten_path([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]])


def test_twist_base_can_plan_with_twenty_manipulation_joints(checker, mocker):
    sim = R1ProGraspingSim(dof=20)
    sim._engine = mocker.Mock(model=checker.model, data=checker.probe, _lock=threading.RLock())
    sim.cargo_bodies = ()
    try:
        path = sim.plan_transport(0.0, 0.6, 0.0)
        assert path[-1] == pytest.approx([0.0, 0.6, 0.0])
        assert all(checker.clear_pose_segment(np.array(a), np.array(b)) for a, b in pairwise(path))
    finally:
        sim._engine = None
        sim.stop()


def test_docking_uses_clear_elbow_around_furniture(checker):
    path = checker.plan((1.0, 0.5))
    assert path == [[0.0, 0.0, 0.0], [0.0, 0.5, 0.0], [1.0, 0.5, 0.0]]
    assert all(checker.clear_pose_segment(np.array(a), np.array(b)) for a, b in pairwise(path))


def test_grid_search_has_a_time_limit_when_simple_docking_paths_are_blocked(checker):
    with pytest.raises(RuntimeError, match="planning timed out"):
        checker.plan((1.0, 0.0), timeout=0.0)
