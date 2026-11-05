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

import mujoco
import numpy as np
import pytest


def test_free_fall_motion_contains_gravity(backend):
    backend.place("box", (0, 0, 2, 0, 0, 0, 1))
    motion = next(b for b in backend.read_bodies() if b.body_id == "box")
    assert [
        motion.acceleration.linear.x,
        motion.acceleration.linear.y,
        motion.acceleration.linear.z,
    ] == pytest.approx([0, 0, -9.81], abs=1e-8)
    assert (
        np.linalg.norm(
            [
                motion.acceleration.linear.x,
                motion.acceleration.linear.y,
                motion.acceleration.linear.z,
            ]
        )
        > 9
    )


def test_offset_frame_acceleration_includes_rotation(backend):
    joint = backend.model.joint("door/hinge")
    dof = joint.dofadr[0]
    backend.data.qvel[dof] = 2
    backend.data.qfrc_applied[dof] = 1
    mujoco.mj_forward(backend.model, backend.data)
    angular_acceleration = backend.data.qacc[dof]
    motion = next(b for b in backend.read_bodies() if b.body_id == "door/imu")
    assert [
        motion.twist.angular.x,
        motion.twist.angular.y,
        motion.twist.angular.z,
    ] == pytest.approx([0, 0, 2])
    assert [
        motion.acceleration.linear.x,
        motion.acceleration.linear.y,
        motion.acceleration.linear.z,
    ] == pytest.approx([-4 * 0.225, angular_acceleration * 0.225, 0], abs=1e-8)

    alpha = motion.acceleration.angular
    assert [alpha.x, alpha.y, alpha.z] == pytest.approx([0, 0, angular_acceleration], abs=1e-8)
