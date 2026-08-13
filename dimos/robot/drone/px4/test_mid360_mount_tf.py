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

from dimos.robot.drone.px4.mid360_mount_tf import BASE_TO_MID360, Mid360MountStaticTf


def test_mount_transform_attaches_body_below_pointlio_sensor_frame() -> None:
    mount = Mid360MountStaticTf()
    try:
        transform = mount.transforms()[0]

        assert (transform.frame_id, transform.child_frame_id) == ("mid360_link", "base_link")
        np.testing.assert_allclose(transform.to_matrix(), BASE_TO_MID360.inverse().to_matrix())
    finally:
        mount.stop()
