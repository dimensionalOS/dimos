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

"""DrakeWorld ↔ MujocoWorld parity harness.

Loads the same arm model into both backends and asserts FK, Jacobian,
joint-limit, and collision-verdict agreement over random configurations.
This is the evidence that flipping a robot's planning backend from Drake
to MuJoCo does not change planning behavior.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
if importlib.util.find_spec("pydrake") is None:
    pytest.skip("pydrake not installed", allow_module_level=True)

from dimos.manipulation.planning.world.drake_world import DrakeWorld
from dimos.manipulation.planning.world.mujoco_world import MujocoWorld
from dimos.manipulation.planning.world.test_mujoco_world import (
    _JOINTS,
    _MINI_ARM_XML,
    _arm_config,
    _blocking_obstacle,
    _js,
)
from dimos.msgs.sensor_msgs.JointState import JointState

N_SAMPLES = 50
_RNG = np.random.default_rng(7)


@pytest.fixture(scope="module")
def worlds(tmp_path_factory: pytest.TempPathFactory):
    tmp_path = tmp_path_factory.mktemp("parity")
    (tmp_path / "mini_arm.xml").write_text(_MINI_ARM_XML)

    drake = DrakeWorld()
    mjc = MujocoWorld()
    ids = {}
    for name, world in (("drake", drake), ("mujoco", mjc)):
        robot_id = world.add_robot(_arm_config(tmp_path))
        world.add_obstacle(_blocking_obstacle())
        world.finalize()
        ids[name] = robot_id
    return drake, ids["drake"], mjc, ids["mujoco"]


def _random_configs(lower: np.ndarray, upper: np.ndarray, n: int) -> np.ndarray:
    return _RNG.uniform(lower, upper, size=(n, len(lower)))


def _quat_angle(q1: list[float], q2: list[float]) -> float:
    dot = abs(float(np.dot(q1, q2)))
    return 2.0 * float(np.arccos(np.clip(dot, -1.0, 1.0)))


def test_joint_limits_agree(worlds) -> None:
    drake, drake_id, mjc, mjc_id = worlds
    d_lower, d_upper = drake.get_joint_limits(drake_id)
    m_lower, m_upper = mjc.get_joint_limits(mjc_id)
    assert np.allclose(d_lower, m_lower, atol=1e-9)
    assert np.allclose(d_upper, m_upper, atol=1e-9)


def test_fk_agrees(worlds) -> None:
    drake, drake_id, mjc, mjc_id = worlds
    lower, upper = mjc.get_joint_limits(mjc_id)
    configs = _random_configs(lower, upper, N_SAMPLES)

    worst_pos, worst_ang = 0.0, 0.0
    with drake.scratch_context() as dctx, mjc.scratch_context() as mctx:
        for q in configs:
            js = _js(q.tolist())
            drake.set_joint_state(dctx, drake_id, js)
            mjc.set_joint_state(mctx, mjc_id, js)
            dp = drake.get_ee_pose(dctx, drake_id)
            mp = mjc.get_ee_pose(mctx, mjc_id)
            worst_pos = max(
                worst_pos, float(np.linalg.norm(np.subtract(list(dp.position), list(mp.position))))
            )
            worst_ang = max(worst_ang, _quat_angle(list(dp.orientation), list(mp.orientation)))

    assert worst_pos < 1e-3, f"worst FK position deviation {worst_pos * 1e3:.3f} mm"
    assert worst_ang < np.deg2rad(0.5), (
        f"worst FK orientation deviation {np.rad2deg(worst_ang):.3f} deg"
    )


def test_jacobian_agrees(worlds) -> None:
    drake, drake_id, mjc, mjc_id = worlds
    lower, upper = mjc.get_joint_limits(mjc_id)
    configs = _random_configs(lower, upper, 10)

    with drake.scratch_context() as dctx, mjc.scratch_context() as mctx:
        for q in configs:
            js = _js(q.tolist())
            drake.set_joint_state(dctx, drake_id, js)
            mjc.set_joint_state(mctx, mjc_id, js)
            jd = drake.get_jacobian(dctx, drake_id)
            jm = mjc.get_jacobian(mctx, mjc_id)
            assert jd.shape == jm.shape == (6, 3)
            assert np.allclose(jd, jm, atol=1e-6), f"Jacobian mismatch at q={q}"


def test_collision_verdicts_agree(worlds) -> None:
    """Verdicts must agree away from contact boundaries; configurations within
    ±2 mm of contact may legitimately differ between narrowphases."""
    drake, drake_id, mjc, mjc_id = worlds
    lower, upper = mjc.get_joint_limits(mjc_id)
    configs = _random_configs(lower, upper, N_SAMPLES)

    boundary, disagreements, checked = 0, [], 0
    for q in configs:
        js = _js(q.tolist())
        d_free = drake.check_config_collision_free(drake_id, js)
        m_free = mjc.check_config_collision_free(mjc_id, js)
        with mjc.scratch_context() as mctx:
            mjc.set_joint_state(mctx, mjc_id, js)
            margin = mjc.get_min_distance(mctx, mjc_id)
        if np.isfinite(margin) and abs(margin) < 2e-3:
            boundary += 1
            continue
        checked += 1
        if d_free != m_free:
            disagreements.append((q, d_free, m_free, margin))

    assert checked > N_SAMPLES // 2
    assert not disagreements, (
        f"{len(disagreements)}/{checked} verdict disagreements "
        f"(+{boundary} boundary cases skipped): {disagreements[:3]}"
    )


def test_ik_solution_transfers_across_backends(worlds, tmp_path: Path) -> None:
    """An IK solution computed against one backend must reach the same pose
    when evaluated in the other."""
    from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK

    drake, drake_id, mjc, mjc_id = worlds
    with mjc.scratch_context() as mctx:
        mjc.set_joint_state(mctx, mjc_id, _js([0.4, -0.3, 0.5]))
        target = mjc.get_ee_pose(mctx, mjc_id)

    result = JacobianIK().solve(
        mjc, mjc_id, target, seed=_js([0.0, 0.0, 0.0]), check_collision=False
    )
    assert result.is_success(), result.message

    with drake.scratch_context() as dctx:
        drake.set_joint_state(
            dctx, drake_id, JointState(name=_JOINTS, position=result.joint_state.position)
        )
        reached = drake.get_ee_pose(dctx, drake_id)
    assert np.allclose(list(reached.position), list(target.position), atol=2e-3)
