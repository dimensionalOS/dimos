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

"""The MuJoCo backend behind the seam. Needs a menagerie checkout."""

from __future__ import annotations

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")


def _menagerie_available() -> bool:
    """A missing checkout is an ENVIRONMENT gap, not a failure of this package."""
    from dimos.robot.unitree.go2.sim.model import scene_path

    try:
        scene_path()
    except FileNotFoundError:
        return False
    return True


# NOT `pytest.mark.mujoco`: the repo's addopts deselects that marker wholesale,
# which is how a run once reported 70 passed while the acceptance test was
# failing. These skip on a missing menagerie with the reason printed — a test
# that vanishes silently is worse than one that fails.
pytestmark = [
    pytest.mark.go2sim,
    pytest.mark.skipif(
        not _menagerie_available(),
        reason="no mujoco_menagerie checkout: set MUJOCO_MENAGERIE",
    ),
]

from dimos.robot.unitree.go2.sim import anchors, model as go2_model
from dimos.robot.unitree.go2.sim.backend import BaseCondition, Commands, RolloutPlan, State
from dimos.robot.unitree.go2.sim.model import MujocoBackend

STAND_Q = np.tile([0.0, 0.9, -1.8], 4)


def _plan(
    duration: float = 0.2,
    base: BaseCondition = BaseCondition.FREE,
    target: np.ndarray | None = None,
) -> RolloutPlan:
    """Drive toward ``target`` (default: hold the standing pose) from rest."""
    q_des = STAND_Q if target is None else target
    return RolloutPlan(
        t0=0.0,
        duration=duration,
        commands=Commands(
            t=np.array([0.0]),
            q=q_des[None],
            dq=np.zeros((1, 12)),
            kp=np.full((1, 12), 40.0),
            kd=np.full((1, 12), 2.0),
            tau_ff=np.zeros((1, 12)),
        ),
        reinit=[State(t=0.0, q=STAND_Q.copy(), dq=np.zeros(12), rot=np.eye(3), gyro=np.zeros(3))],
        base=base,
    )


def test_the_model_carries_the_imu_site_the_virtual_imu_reads_at():
    """49 mm of lever arm costs 1.8x on the landing residual: an off-axis
    accelerometer reads alpha x r + omega x (omega x r) on top of the frame's
    acceleration — 5-25 m/s2 during a landing, the same order as the impact."""
    m, _d = go2_model.load()
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, go2_model.IMU_SITE)
    assert sid >= 0, f"the model must carry the {go2_model.IMU_SITE!r} site"
    assert np.linalg.norm(m.site_pos[sid]) > 0.01, "a zero offset would make this moot"
    # identity orientation is what lets the readout ignore site_quat
    assert np.allclose(m.site_quat[sid], [1.0, 0.0, 0.0, 0.0])


def test_the_virtual_imu_reads_specific_force_like_the_real_one():
    """Free fall must read ~0, or the sim's accel is not comparable to
    rt/lowstate (which is exactly how flight spans are detected)."""
    model, data = go2_model.load()
    data.qpos[2] = 3.0  # well clear of the floor: pure ballistic
    mujoco.mj_forward(model, data)
    for _ in range(20):
        mujoco.mj_step(model, data)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, go2_model.IMU_SITE)
    acc = np.zeros(6)
    mujoco.mj_rnePostConstraint(model, data)
    mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_SITE, sid, acc, 1)
    assert np.linalg.norm(acc[3:]) < 1.0  # 9.8 would mean gravity was not folded in


def test_the_site_and_the_body_frame_disagree_once_the_trunk_rotates():
    """The whole point of reading at the site: under angular acceleration the
    lever arm contributes, so the two readouts must differ measurably."""
    model, data = go2_model.load()
    data.qpos[2] = 3.0
    data.qvel[3] = 6.0  # spinning in free fall: omega x (omega x r) is pure lever arm
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, go2_model.IMU_SITE)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    at_site, at_body = np.zeros(6), np.zeros(6)
    mujoco.mj_rnePostConstraint(model, data)
    mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_SITE, sid, at_site, 1)
    mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, at_body, 1)
    assert np.linalg.norm(at_site[3:] - at_body[3:]) > 0.5


def test_the_anchor_constants_match_the_menagerie_model():
    """anchors.py must describe THIS model, or every derivation drifts."""
    m, _d = go2_model.load()
    trunk = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    assert m.body_mass[trunk] == pytest.approx(anchors.STOCK_TRUNK_MASS, abs=1e-6)
    assert np.allclose(m.body_ipos[trunk], anchors.STOCK_TRUNK_IPOS, atol=1e-6)
    assert np.allclose(
        sorted(m.body_inertia[trunk]), sorted(anchors.STOCK_TRUNK_INERTIA), atol=1e-6
    )
    assert m.body_mass.sum() == pytest.approx(anchors.STOCK_MODEL_TOTAL_MASS, abs=1e-5)


def test_apply_physics_writes_what_it_names_and_nothing_else():
    fresh, _ = go2_model.load()
    model, _ = go2_model.load()
    go2_model.apply_physics(model, {"armature": 0.03, "trunk_mass_scale": 1.1})
    assert np.allclose(model.dof_armature[go2_model.LEG_DOFS], 0.03)
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    assert model.body_mass[trunk] == pytest.approx(fresh.body_mass[trunk] * 1.1)
    # an ABSENT key is never written: damping keeps the menagerie default
    assert np.array_equal(model.dof_damping, fresh.dof_damping)


def test_apply_physics_rejects_an_unknown_key():
    model, _ = go2_model.load()
    with pytest.raises(ValueError, match="unknown physics"):
        go2_model.apply_physics(model, {"solar_flux": 1.0})


def test_the_backend_rejects_an_unknown_knob():
    with pytest.raises(ValueError, match="unknown knob"):
        MujocoBackend().apply({"warp_drive": 1.0})


def test_the_backend_timestep_is_the_scene_timestep():
    assert MujocoBackend().timestep == pytest.approx(0.002)


def test_a_plan_must_start_with_the_state_at_t0():
    plan = _plan()
    late = RolloutPlan(
        t0=0.0,
        duration=0.1,
        commands=plan.commands,
        reinit=[State(t=0.05, q=STAND_Q, dq=np.zeros(12), rot=np.eye(3), gyro=np.zeros(3))],
    )
    with pytest.raises(ValueError, match="t0"):
        MujocoBackend().rollout(late)


def test_the_same_plan_yields_the_same_prediction_bit_for_bit():
    """Every rollout compared against another must share one clip schedule —
    and the backend itself must be deterministic for that to mean anything."""
    be = MujocoBackend()
    be.apply({"armature": 0.02, "actuator_tau": 0.005})
    a = be.rollout(_plan())
    b = be.rollout(_plan())
    assert np.array_equal(a.q, b.q)
    assert np.array_equal(a.imu_accel, b.imu_accel)
    assert np.array_equal(a.body_pos, b.body_pos)


def test_two_plants_disagree_or_the_knobs_do_nothing():
    stiff = MujocoBackend()
    stiff.apply({"armature": 0.05})
    loose = MujocoBackend()
    loose.apply({"armature": 0.001})
    assert not np.array_equal(stiff.rollout(_plan()).q, loose.rollout(_plan()).q)


def test_a_pinned_base_holds_the_trunk_and_frees_the_legs():
    """The rope fixes the trunk; the feet never meet a floor the real robot
    never met. And the trunk is pinned to the MEASURED pose, well off the
    ground, so gravity still points the measured way through every leg."""
    be = MujocoBackend()
    swung = STAND_Q + np.tile([0.0, 0.4, 0.3], 4)
    pred = be.rollout(_plan(duration=0.3, base=BaseCondition.PINNED, target=swung))
    assert np.all(pred.body_pos[:, 2] >= 2.0)  # clear of the floor entirely
    assert np.allclose(pred.body_pos, pred.body_pos[0], atol=1e-12)  # trunk held
    assert np.abs(pred.q - pred.q[0]).max() > 0.1  # commands drive the legs


def test_a_suspended_clip_repins_to_the_measured_orientation_not_t0s():
    """The robot hung 70-85 deg off level and MOVED while hanging: each clip
    must pin the trunk to the pose measured at ITS start. Holding t0's pose
    points gravity the wrong way through every leg for the rest of the file."""
    from dimos.robot.unitree.go2.sim.rotations import quat_to_mat

    tilted = quat_to_mat(np.array([np.cos(0.6), np.sin(0.6), 0.0, 0.0]))  # 68.7 deg roll
    plan = RolloutPlan(
        t0=0.0,
        duration=0.2,
        commands=_plan().commands,
        reinit=[
            State(t=0.0, q=STAND_Q.copy(), dq=np.zeros(12), rot=np.eye(3), gyro=np.zeros(3)),
            State(t=0.1, q=STAND_Q.copy(), dq=np.zeros(12), rot=tilted, gyro=np.zeros(3)),
        ],
        base=BaseCondition.PINNED,
    )
    pred = MujocoBackend().rollout(plan)
    first, second = pred.body_rot[pred.t < 0.1], pred.body_rot[pred.t >= 0.1]
    assert np.allclose(first, np.eye(3), atol=1e-9)
    assert np.allclose(second, tilted, atol=1e-9)


def test_the_prediction_samples_the_imu_at_the_physics_rate():
    """An impact is 30-50 ms wide with a ~30 ms rise; at 100 Hz its rise is
    two points, which is not a profile."""
    pred = MujocoBackend().rollout(_plan(duration=0.1))
    assert len(pred.at) == 5 * len(pred.t)  # 500 Hz against the 100 Hz pose log
    assert pred.imu_accel.shape == (len(pred.at), 3)
    assert pred.tau.shape == (len(pred.at), 12)
