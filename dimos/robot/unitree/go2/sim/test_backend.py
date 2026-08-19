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


# NOT `pytest.mark.mujoco`: the repo's addopts deselects that marker wholesale,
# which is how a run once reported 70 passed while the acceptance test was
# failing. And no menagerie skip either: the go2 assets are vendored
# (data/go2_menagerie), so a missing scene is a real failure — the same
# silent-vanish shape, closed the same way.
pytestmark = [pytest.mark.go2sim]

from dimos.robot.unitree.go2.sim import anchors
from dimos.robot.unitree.go2.sim.backend import BaseCondition, Commands, RolloutPlan, State
from dimos.robot.unitree.go2.sim.engines import model as go2_model
from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

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


def test_the_derived_anchors_reproduce_the_weighed_mass_on_the_compiled_model():
    """The physics-facing check the anchors' reproduction test cannot be:
    trunk_mass_scale exists to make the MODEL weigh what the kitchen scale
    read, 16.500 kg. This goes through the real compiled model and
    apply_physics, so it fails if menagerie masses drift, the stock
    constants go stale, or the mass bookkeeping breaks — physics reasons,
    not the solved-payload algebra."""
    spec = anchors.RobotSpec(mass_kg=16.500)
    got = anchors.derive(spec)
    model, _ = go2_model.load()
    go2_model.apply_physics(model, {"trunk_mass_scale": got["trunk_mass_scale"]})
    assert model.body_mass.sum() == pytest.approx(16.500, abs=1e-4)


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


def test_recording_the_solver_on_measured_wrote_down_what_was_already_there():
    """The 2026-08-19 schema change: `measured` now carries its contact
    solver explicitly instead of inheriting it silently from the scene. The
    hard invariant is that WRITING IT DOWN CHANGED NOTHING — the recorded
    values must be exactly what the scene compiles to, and applying the full
    preset must leave the options where they were."""
    from dimos.robot.unitree.go2.sim.ranges import MEASURED, SOLVER_DEFAULTS

    fresh, _ = go2_model.load()
    assert fresh.opt.iterations == SOLVER_DEFAULTS["solver_iterations"]
    assert fresh.opt.ls_iterations == SOLVER_DEFAULTS["solver_ls_iterations"]
    assert fresh.opt.cone == SOLVER_DEFAULTS["solver_cone"] == int(mujoco.mjtCone.mjCONE_ELLIPTIC)
    model, _ = go2_model.load()
    go2_model.apply_physics(model, MEASURED.physics)
    assert model.opt.iterations == fresh.opt.iterations
    assert model.opt.ls_iterations == fresh.opt.ls_iterations
    assert model.opt.cone == fresh.opt.cone


def test_the_measured_plant_moves_identically_with_its_solver_recorded():
    """The same fact from the physics side: a contact-rich collapse is
    bit-identical whether the preset's solver keys are applied or omitted.
    If this ever fails, the schema change moved `measured` — the one thing
    it must never do."""
    from dimos.robot.unitree.go2.sim.engines.mjx import qpos_after
    from dimos.robot.unitree.go2.sim.ranges import MEASURED, SOLVER_KEYS

    def collapse(physics: dict[str, float]) -> np.ndarray:
        model, data = go2_model.load()
        go2_model.apply_physics(model, physics)
        data.qpos[2] = 0.30
        return qpos_after(model, data, 500)

    with_solver = collapse(dict(MEASURED.physics))
    without = collapse({k: v for k, v in MEASURED.physics.items() if k not in SOLVER_KEYS})
    assert np.array_equal(with_solver, without)


def test_apply_physics_sets_a_cheap_solver_and_only_when_asked():
    fresh, _ = go2_model.load()
    model, _ = go2_model.load()
    go2_model.apply_physics(
        model, {"solver_iterations": 1.0, "solver_ls_iterations": 5.0, "solver_cone": 0.0}
    )
    assert model.opt.iterations == 1
    assert model.opt.ls_iterations == 5
    assert model.opt.cone == int(mujoco.mjtCone.mjCONE_PYRAMIDAL)
    # an ABSENT key is never written
    only_geom, _ = go2_model.load()
    go2_model.apply_physics(only_geom, {"armature": 0.03})
    assert only_geom.opt.iterations == fresh.opt.iterations
    assert only_geom.opt.cone == fresh.opt.cone


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


def _motors_off(duration: float = 3.0) -> RolloutPlan:
    """A pinned plan with every gain zero: the legs are purely passive."""
    return RolloutPlan(
        t0=0.0,
        duration=duration,
        commands=Commands(
            t=np.array([0.0]),
            q=np.zeros((1, 12)),
            dq=np.zeros((1, 12)),
            kp=np.zeros((1, 12)),
            kd=np.zeros((1, 12)),
            tau_ff=np.zeros((1, 12)),
        ),
        reinit=[State(t=0.0, q=STAND_Q.copy(), dq=np.zeros(12), rot=np.eye(3), gyro=np.zeros(3))],
        base=BaseCondition.PINNED,
    )


def test_gravity_reaches_the_legs_while_the_trunk_is_held(monkeypatch):
    """THE point of holding the trunk DURING the step, not after it.

    The old mechanism let the base fall freely through ``mj_step`` and snapped
    it back afterwards, so the whole robot free-fell and gravity cancelled out
    of the relative leg dynamics: with motors off, the passive leg pose after
    3 s was IDENTICAL with gravity on and gravity zeroed, to 0.000000 rad —
    a weightless plant, and every ``damping`` ever fitted on the hanging
    recording absorbed the gravity torque the legs never felt. With the trunk
    held by a weld during the step, gravity must load the legs: the two poses
    must differ measurably."""

    def passive_pose(zero_g: bool) -> np.ndarray:
        with monkeypatch.context() as m:
            if zero_g:
                orig = go2_model.load

                def no_gravity(*a: object, **k: object):
                    model, data = orig(*a, **k)
                    model.opt.gravity[:] = 0.0
                    return model, data

                m.setattr(go2_model, "load", no_gravity)
            return MujocoBackend().rollout(_motors_off()).q[-1]

    diff = np.abs(passive_pose(zero_g=False) - passive_pose(zero_g=True))
    assert diff.max() > 0.1, (
        f"passive leg pose is gravity-independent (max diff {diff.max():.6f} rad): "
        "the trunk hold has put the legs back in a weightless plant"
    )


def test_a_pinned_base_holds_the_trunk_and_frees_the_legs():
    """The rope fixes the trunk; the feet never meet a floor the real robot
    never met. The hold is a WELD acting during the step — a constraint, not
    a qpos overwrite — so the tolerances are the weld's measured hold error
    (~6 um / ~0.003 deg), not machine epsilon."""
    be = MujocoBackend()
    swung = STAND_Q + np.tile([0.0, 0.4, 0.3], 4)
    pred = be.rollout(_plan(duration=0.3, base=BaseCondition.PINNED, target=swung))
    assert np.all(pred.body_pos[:, 2] >= 1.999)  # clear of the floor entirely
    assert np.allclose(pred.body_pos, pred.body_pos[0], atol=1e-4)  # trunk held
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
    assert np.allclose(first, np.eye(3), atol=1e-3)  # weld hold error, not epsilon
    assert np.allclose(second, tilted, atol=1e-3)


def test_a_pinned_base_follows_the_measured_attitude_track():
    """The robot SWINGS on the rope — median 5.4 deg of attitude change within
    a 0.4 s clip, p90 26 deg on the hanging recording — so the weld target
    follows the measured attitude every step instead of freezing the pose the
    clip started from."""
    from dimos.robot.unitree.go2.sim.backend import BaseTrack
    from dimos.robot.unitree.go2.sim.rotations import quat_to_mat

    tt = np.arange(0.0, 0.32, 0.002)
    roll = np.minimum(tt / 0.3, 1.0) * 0.5  # ramp to 0.5 rad over the clip
    rot = np.stack([quat_to_mat(np.array([np.cos(a / 2), np.sin(a / 2), 0.0, 0.0])) for a in roll])
    plan = RolloutPlan(
        t0=0.0,
        duration=0.3,
        commands=_plan().commands,
        reinit=[State(t=0.0, q=STAND_Q.copy(), dq=np.zeros(12), rot=rot[0], gyro=np.zeros(3))],
        base=BaseCondition.PINNED,
        base_track=BaseTrack(t=tt, rot=rot),
    )
    pred = MujocoBackend().rollout(plan)
    # held rigid at the start pose the trunk would end 0.5 rad wrong; tracking
    # the measurement it ends within ~1 deg of the ramp's end (the weld lags
    # a target moving at 95 deg/s by a fraction of a degree)
    assert np.abs(pred.body_rot[0] - rot[0]).max() < 1e-3
    assert np.abs(pred.body_rot[-1] - rot[np.searchsorted(tt, pred.t[-1]) - 1]).max() < 2e-2
    assert np.abs(pred.body_rot[-1] - rot[0]).max() > 0.4


def test_the_prediction_samples_the_imu_at_the_physics_rate():
    """An impact is 30-50 ms wide with a ~30 ms rise; at 100 Hz its rise is
    two points, which is not a profile."""
    pred = MujocoBackend().rollout(_plan(duration=0.1))
    assert len(pred.at) == 5 * len(pred.t)  # 500 Hz against the 100 Hz pose log
    assert pred.imu_accel.shape == (len(pred.at), 3)
    assert pred.tau.shape == (len(pred.at), 12)
