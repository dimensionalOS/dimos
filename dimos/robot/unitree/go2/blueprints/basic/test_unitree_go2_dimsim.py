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

import pytest

from dimos.robot.unitree.go2.blueprints.basic import test_sim2_blueprints as sim2

pytest.importorskip("pxr")

_NAME = "unitree_go2_dimsim"


def test_each_namespace_has_one_unmounted_hardware() -> None:
    sim2.test_each_namespace_has_one_unmounted_hardware(_NAME)


@pytest.mark.self_hosted  # the composed world reads the Go2 meshes and the apartment from Git LFS
def test_the_room_robot_and_sensors_compose_into_one_world() -> None:
    sim2.test_the_room_robot_and_sensors_compose_into_one_world(_NAME)


@pytest.mark.self_hosted  # the Go2 meshes and the apartment are Git LFS data
def test_a_dimsim_go2_publishes_the_tf_a_real_one_does() -> None:
    from dimos.robot.unitree.go2.connection import ConnectionConfig
    from dimos.robot.unitree.go2.dimsim import Go2KinematicBase
    from dimos.sim2.odom_tf import OdomTf

    (odom_atom,) = [atom for atom in sim2._atoms(_NAME) if atom.module is OdomTf]
    odom_tf = OdomTf(**odom_atom.kwargs)
    with sim2._hardware(_NAME) as hardware:
        go2 = next(module for module in hardware if isinstance(module, Go2KinematicBase))
        edges = {(go2.attached_to(), go2.attach_frame), *odom_tf.tf_edge_ids}
        robot, body = go2.namespace, go2.tf_frames[go2.config.usd.root_link]
        announced = go2.simulated_robot().odom_frame
    odom_tf.stop()

    real_odom = ConnectionConfig.model_fields["odom_frame_id"].default
    # The mount holds odom, and odom (the OdomTf beside it) holds the body, as on a real Go2.
    assert edges == {(robot, real_odom), (real_odom, body)}
    assert body == f"{robot}/go2/Geometry/base_link"
    assert announced == real_odom
