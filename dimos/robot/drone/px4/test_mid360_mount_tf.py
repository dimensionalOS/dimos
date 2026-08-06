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

import numpy as np

from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.drone.px4.mid360_mount_tf import BASE_TO_MID360, Mid360MountStaticTf


def test_mount_blueprint_construction_does_not_start_the_publisher() -> None:
    blueprint = Mid360MountStaticTf.blueprint()

    assert [atom.module for atom in blueprint.active_blueprints] == [Mid360MountStaticTf]


def test_mount_transform_connects_body_below_pointlio_sensor_frame() -> None:
    mount = Mid360MountStaticTf()
    try:
        transform = mount.transforms()[0]
        expected = BASE_TO_MID360.inverse()

        assert (transform.frame_id, transform.child_frame_id) == ("mid360_link", "base_link")
        np.testing.assert_allclose(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            (expected.translation.x, expected.translation.y, expected.translation.z),
        )
        np.testing.assert_allclose(transform.to_matrix(), expected.to_matrix())
    finally:
        mount.stop()


def test_mount_is_a_static_tf_publisher() -> None:
    assert issubclass(Mid360MountStaticTf, StaticTfPublisher)
