# Copyright 2025-2026 Dimensional Inc.
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

"""CPU tests of the native plant, ONNX ABI, and mixed-interface closed loop."""

import os
from pathlib import Path
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.control.connection_source import ConnectionSource
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_GRIPPER_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnectionConfig,
)
from dimos.robot.galaxea.r1pro.mujoco_connection import R1ProMujocoConnection
from dimos.robot.galaxea.r1pro.mujoco_plant import CHECKPOINT, DT, XML
from dimos.robot.galaxea.r1pro.policy_task import R1ProPolicyTask, pack_goal


@pytest.fixture
def bundle_dir():
    root = os.getenv("R1PRO_SIM_BUNDLE")
    if not root:
        pytest.skip("External run45 demo bundle: set R1PRO_SIM_BUNDLE")
    assert (Path(root) / "assets" / XML).is_file()
    pytest.importorskip("mujoco")
    return root


def test_goal_abi():
    root_r = Rotation.from_euler("zyx", [1, 0.4, -0.3]).as_matrix()
    wrist_r = Rotation.from_euler("xyz", [0.1, -0.3, 0.45]).as_matrix()
    root = np.array([0.2, 0.7, 1.0])
    left, right = np.array([0.2, -0.1, 0.4]), np.array([1, 0, 0.5])
    goal = pack_goal(
        ((root + root_r @ left, root_r @ wrist_r), (root + root_r @ right, root_r)), (root, root_r)
    )
    assert goal.shape == (1, 60) and goal.dtype == np.float32
    np.testing.assert_allclose(
        goal[0, 10:20], [*left, *wrist_r[:, 0], *wrist_r[:, 1], 1], atol=1e-7
    )
    np.testing.assert_allclose(goal[0, 20:30], [*right, 1, 0, 0, 0, 1, 0, 1], atol=1e-7)
    assert not np.any(goal[0, :10]) and not np.any(goal[0, 30:])


def roundtrip(msg):
    return type(msg).lcm_decode(msg.lcm_encode())


@pytest.fixture
def connection(bundle_dir):
    conn = R1ProMujocoConnection(bundle_dir=bundle_dir)
    try:
        conn.connect()
        yield conn
    finally:
        conn.stop()


def test_connection_streams_and_stability(connection):
    c = connection
    d = c.plant.data
    assert c.config.joint_commands == R1ProConnectionConfig().joint_commands
    assert c.plant.model.body_mass.sum() == pytest.approx(126)
    for _ in range(100):
        c.plant.step()
        assert d.xmat[c.plant.model.body("base_link").id, 8] > 0.95
    assert d.time == pytest.approx(2.0)
    c.write({f"{R1PRO_GRIPPER_JOINTS[0]}/position": 0.035, f"{R1PRO_BASE_JOINTS[0]}/velocity": 0.1})
    assert d.ctrl[18] == 0.1
    c.halt()
    assert not np.any(d.ctrl[18:])
    stamp = time.time()
    upper, base, jaws, odom, wrists, chassis_imu, torso_imu, pose, tf = [
        roundtrip(m) for _, m in c.feedback(stamp)
    ]
    assert all(
        m.ts == pytest.approx(stamp, abs=1e-6)
        for m in (upper, base, jaws, odom, wrists, chassis_imu, torso_imu, pose)
    )
    assert (
        list(upper.name) == R1PRO_UPPER_BODY_JOINTS
        and len(upper.effort) == len(upper.velocity) == 18
    )
    assert list(base.name) == R1PRO_BASE_JOINTS and not base.position and len(base.velocity) == 3
    assert list(jaws.name) == R1PRO_GRIPPER_JOINTS and list(jaws.position) == pytest.approx(
        [0.035, 0]
    )
    assert odom.frame_id == pose.frame_id == wrists.frame_id == "odom"
    assert odom.child_frame_id == upper.frame_id == "base_link"
    assert chassis_imu.frame_id == "imu_chassis_link" and torso_imu.frame_id == "imu_torso_link"
    assert len(wrists) == 2 and len(tf.transforms) == 1


def test_cpu_policy_and_closed_loop(bundle_dir, connection):
    pytest.importorskip("onnxruntime")
    c = connection
    t = R1ProPolicyTask(
        "policy", os.getenv("R1PRO_POLICY_ONNX", str(Path(bundle_dir) / "models" / CHECKPOINT))
    )
    assert t.actor.session.get_providers() == ["CPUExecutionProvider"]
    source = ConnectionSource(
        c.config, ["position_command", "velocity_command"], ["joint_state", "imu"], 0.2
    )
    targets = [
        Pose(Vector3(w.position.x + 0.06, w.position.y, w.position.z), w.orientation)
        for w in c.plant.wrists()
    ]

    def error():
        return np.linalg.norm(
            [
                [
                    a.position.x - b.position.x,
                    a.position.y - b.position.y,
                    a.position.z - b.position.z,
                ]
                for a, b in zip(targets, c.plant.wrists(), strict=True)
            ],
            axis=1,
        ).mean()

    start = error()
    max_tilt = 0.0
    d = c.plant.data
    for i in range(100):
        stamp = time.time()
        now = time.perf_counter()
        upper, base, jaws, odom, wrists, imu, *_ = [roundtrip(m) for _, m in c.feedback(stamp)]
        for msg in (upper, base, jaws):
            source.on_message("joint_state", msg)
        source.on_message("imu", imu)
        t.on_joints(upper, now)
        t.on_odometry(odom, now)
        t.on_wrists(wrists, now)
        t.on_world_target(PoseArray(Header(stamp, "odom"), targets), now)
        last_raw = t.actor.raw.copy()
        out = t.compute(CoordinatorState(JointStateSnapshot(), t_now=now, dt=DT))
        assert out is not None
        q, twist = out
        assert len(q.positions) == 18 and len(twist.velocities) == 3
        np.testing.assert_array_equal(t.actor.history[0, -1, 39:], last_raw)
        np.testing.assert_array_equal(t.actor.history[0, -1, :18], d.qpos[7:25].astype(np.float32))
        if i == 0:
            np.testing.assert_array_equal(
                t.actor.history[0], np.repeat(t.actor.history[0, :1], 10, axis=0)
            )
        # Use the same named mixed-mode command builder as the coordinator.
        winners = {
            name: (value, group.mode)
            for group in out
            for name, value in zip(group.joint_names, group.get_values(), strict=True)
        }
        c.write(source.command(winners))
        np.testing.assert_array_equal(d.ctrl, [*q.positions, *twist.velocities])
        c.plant.step()
        max_tilt = max(max_tilt, np.degrees(np.arccos(np.clip(d.body("base_link").xmat[8], -1, 1))))
    print(f"wrist error {start * 1000:.2f} -> {error() * 1000:.2f} mm; max tilt {max_tilt:.2f} deg")
    assert np.isfinite(error()) and error() < 0.75 * start
    assert max_tilt < 15
    t.set_estop(True)
    t.on_world_target(PoseArray(Header(time.time(), "odom"), targets), time.perf_counter())
    t.set_estop(False)
    assert not t.is_active()
    assert not np.any(t.actor.raw)

    def xr(frame, poses=None):
        t.on_controllers(PoseArray(Header(time.time(), frame), poses), time.perf_counter())

    xr("webxr", targets)
    assert not t.is_active()
    xr("webxr_released")
    xr("webxr_idle")  # disconnect/timeout is not a physical release
    xr("webxr", targets)
    assert not t.is_active()
    xr("webxr_released")
    xr("webxr", targets)
    assert t.is_active()
    xr("webxr_released")
    assert not t.is_active()
