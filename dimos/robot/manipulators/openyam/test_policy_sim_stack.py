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

"""Focused tests for the OpenYAM Quest -> reference -> policy -> MuJoCo stack.

These pin the three contract points the blueprint exists to guarantee, without
deploying a blueprint or opening a simulation:

* the stack runs on its own zenoh bus, so no other DimOS instance can feed it
  controller commands, and it cannot build a hardware/CAN adapter at all;
* the actor cadence is an exact decimation of the coordinator tick rate;
* the Quest reference is clipped into the *trained* joint envelope before the
  velocity, the goal FK and the diagnostics derive from it, and a reference
  interval that is not one policy period is refused.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.blueprints import policy as blueprint
from dimos.robot.manipulators.openyam.policy.contract import JOINT_ORDER, PolicyContractError
from dimos.robot.manipulators.openyam.policy.demo_clip_compare import compare
from dimos.robot.manipulators.openyam.policy.demo_compare_actors import (
    anchor_rows,
    channel_report,
    correct_anchor_achievable,
    session_anchor_ticks,
)
from dimos.robot.manipulators.openyam.policy.quest_reference import (
    QuestReferenceGenerator,
    ReferenceRefusedError,
)
from dimos.robot.manipulators.openyam.policy.reference import ClipReference, ReferenceClip
from dimos.robot.manipulators.openyam.policy.tcp import pose_error_summary, quat_angle_deg
from dimos.utils.transform_utils import matrix_to_pose

PERIOD = 1.0 / 30.0
SIM_LOWER = [-2.61799, 0.0, 0.0, -1.65806, -1.5708, -2.35619]
SIM_UPPER = [3.14159, 3.66519, 4.01426, 1.65806, 1.5708, 1.8326]


def _manifest() -> SimpleNamespace:
    return SimpleNamespace(
        tick_period_s=PERIOD,
        robot=SimpleNamespace(sim_lower=SIM_LOWER, sim_upper=SIM_UPPER),
    )


def _pose(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Pose:
    matrix = np.eye(4)
    matrix[:3, 3] = (x, y, z)
    return matrix_to_pose(matrix)


class _StubSolver:
    """Streaming IK stand-in: moves the joints by ``delta_per_s * dt``."""

    def __init__(self, delta_per_s: np.ndarray) -> None:
        self._delta = np.asarray(delta_per_s, dtype=np.float64)
        self.step_calls: list[tuple[float, np.ndarray]] = []
        self.reset_calls = 0

    def reset(self) -> None:
        self.reset_calls += 1

    def step(self, targets: object, state: JointState, dt: float) -> SimpleNamespace:
        del targets
        q = np.asarray(state.position, dtype=np.float64) + self._delta * dt
        self.step_calls.append((float(dt), q.copy()))
        return SimpleNamespace(position=q.tolist())

    def frame_poses(self, state: JointState, frames: tuple[str, ...]) -> dict[str, Pose]:
        q = np.asarray(state.position, dtype=np.float64)
        return {name: _pose(q[0], q[1], q[2]) for name in frames}


class _StubFk:
    """Deterministic plant FK: TCP position is the first three joints."""

    def __init__(self) -> None:
        self.calls: list[np.ndarray] = []

    def __call__(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.calls.append(np.asarray(q, dtype=np.float64).copy())
        position = np.asarray(q[:3], dtype=np.float64) + np.array([0.3, -0.2, 0.5])
        return position, np.array([1.0, 0.0, 0.0, 0.0])


def _generator(
    delta_per_s: np.ndarray, **kwargs: object
) -> tuple[QuestReferenceGenerator, _StubSolver, _StubFk]:
    solver = _StubSolver(delta_per_s)
    fk = _StubFk()
    generator = QuestReferenceGenerator(
        _manifest(),  # type: ignore[arg-type]
        solver=solver,  # type: ignore[arg-type]
        goal_fk=fk,
        joint_names=JOINT_ORDER,
        **kwargs,  # type: ignore[arg-type]
    )
    return generator, solver, fk


# --------------------------------------------------------------------- wiring
def test_transport_isolation_pins_a_private_bus() -> None:
    assert blueprint.transport_isolation() == {
        "zenoh_domain": "openyam-policy-sim",
        "zenoh_scout_addr": "224.0.0.231:7446",
    }
    replay = blueprint.openyam_policy_sim_replay_blueprint
    assert replay.global_config_overrides["zenoh_domain"] == "openyam-policy-sim"
    assert replay.global_config_overrides["zenoh_scout_addr"] == "224.0.0.231:7446"


def test_blueprint_can_only_build_a_simulated_adapter() -> None:
    component = blueprint.policy_hardware(Path("/tmp/openyam_plant.xml"))
    assert component.adapter_type == "sim_mujoco"
    assert component.hardware_type.value == "manipulator"
    assert tuple(component.joints) == tuple(blueprint.policy_joint_names())

    names = {
        atom.module.__name__ for atom in blueprint.openyam_policy_sim_replay_blueprint.blueprints
    }
    assert "MujocoSimModule" in names
    assert "WebXRReplayModule" in names
    assert not {name for name in names if "damiao" in name.lower() or "can" == name.lower()}


def test_actor_cadence_is_an_exact_decimation_of_the_coordinator() -> None:
    assert blueprint.COORDINATOR_TICK_RATE / blueprint.ACTOR_EVERY_N_TICKS == 30.0
    config = blueprint.policy_task_config(
        model_dir=Path("/tmp/model"), manifest_path=Path("/tmp/manifest.json")
    )
    assert config.params["every_n_ticks"] == blueprint.ACTOR_EVERY_N_TICKS
    assert config.params["reference_source"] == "teleop"


def test_clip_variant_threads_the_frozen_clip_and_drops_webxr() -> None:
    config = blueprint.policy_task_config(
        model_dir=Path("/tmp/model"),
        manifest_path=Path("/tmp/manifest.json"),
        reference_source="clip",
        clip_path=Path("/tmp/reference_clip.npz"),
        clip_anchor_index=100,
    )
    assert config.params["reference_source"] == "clip"
    assert config.params["clip_path"] == "/tmp/reference_clip.npz"
    assert config.params["clip_anchor_index"] == 100
    with pytest.raises(ValueError):
        blueprint.policy_task_config(
            model_dir=Path("/tmp/model"),
            manifest_path=Path("/tmp/manifest.json"),
            reference_source="clip",
        )


# ------------------------------------------------------------------- reference
def test_reference_is_clipped_into_the_trained_envelope_before_fk_and_dq() -> None:
    # Ask the stub IK for +60 rad/s on joint 6: one 33 ms step would reach
    # 3.0 rad, far outside the trained envelope (upper 1.8326, margin 0.015).
    generator, solver, fk = _generator(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 60.0]))
    q_measured = np.array([0.0, 1.0, 1.0, 0.0, 0.0, 1.0])
    generator.anchor(q_measured, _pose(), 10.0)
    row = generator.update(_pose(), 10.0 + PERIOD, tick_index=1)

    assert row.q[5] == pytest.approx(SIM_UPPER[5] - 0.015)
    # dq is the finite difference of the *final* reference over the actual dt.
    np.testing.assert_allclose(row.dq, (row.q - q_measured) / PERIOD, atol=1e-6)
    # The goal FK and the diagnostics are computed from the final q_ref, not the
    # unclipped IK solution.
    np.testing.assert_allclose(fk.calls[-1], row.q, atol=1e-9)
    diagnostics = generator.diagnostics
    assert diagnostics is not None
    assert diagnostics.dt_s == pytest.approx(PERIOD)
    assert diagnostics.envelope_clip_rad == pytest.approx(1.0 + 60.0 * PERIOD - row.q[5])
    assert row.time_s == pytest.approx(PERIOD)
    assert row.goal_position[2] == pytest.approx(row.q[2] + 0.5)
    # Requested and achievable poses are reported separately and finite.
    assert diagnostics.requested_position.shape == (3,)
    assert diagnostics.achievable_position.shape == (3,)
    assert diagnostics.projection_error_m >= 0.0
    assert np.isfinite(diagnostics.requested_quaternion_wxyz).all()


def test_anchor_reports_requested_and_achievable_in_one_urdf_frame() -> None:
    """Regression (run 229): the anchor diagnostic took ``achievable`` from the plant
    goal FK while ``requested`` was the deployed URDF tool frame, so the frame offset
    (~80 mm / ~100 deg) showed up as IK projection error at every session anchor."""
    generator, _, fk = _generator(np.zeros(6))
    q_measured = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    generator.anchor(q_measured, _pose(), 0.0)
    diagnostics = generator.diagnostics
    assert diagnostics is not None
    # `_StubSolver.frame_poses` is the URDF tool frame: position == first three joints.
    np.testing.assert_allclose(diagnostics.requested_position, q_measured[:3], atol=1e-12)
    np.testing.assert_allclose(
        diagnostics.achievable_position, diagnostics.requested_position, atol=1e-12
    )
    np.testing.assert_allclose(
        diagnostics.achievable_quaternion_wxyz,
        diagnostics.requested_quaternion_wxyz,
        atol=1e-12,
    )
    assert diagnostics.projection_error_m == 0.0
    # The plant-frame policy goal is a *different* pose and must not leak in here.
    goal_position, _ = fk(q_measured)
    assert np.linalg.norm(diagnostics.achievable_position - goal_position) > 0.1
    # The next tick keeps the same convention: the stub IK does not move, so the
    # operator's request is achievable and the projection stays at zero.
    generator.update(_pose(), PERIOD, tick_index=1)
    next_diagnostics = generator.diagnostics
    assert next_diagnostics is not None
    assert next_diagnostics.projection_error_m == pytest.approx(0.0, abs=1e-12)


def _trace_arrays(ticks: list[int], epochs: list[float] | None = None) -> dict[str, np.ndarray]:
    """Minimal closed-loop trace with the pose columns the comparison report reads."""
    size = len(ticks)
    arrays: dict[str, np.ndarray] = {
        "t": np.arange(size, dtype=np.float64),
        "epoch": np.asarray(epochs if epochs is not None else [1.0] * size, dtype=np.float64),
        "active": np.ones(size),
        "policy_ticks": np.asarray(ticks, dtype=np.float64),
    }
    for prefix in ("goal", "exec", "req", "ach"):
        arrays[f"{prefix}_x"] = np.zeros(size)
        arrays[f"{prefix}_y"] = np.zeros(size)
        arrays[f"{prefix}_z"] = np.zeros(size)
        arrays[f"{prefix}_qw"] = np.ones(size)
        arrays[f"{prefix}_qx"] = np.zeros(size)
        arrays[f"{prefix}_qy"] = np.zeros(size)
        arrays[f"{prefix}_qz"] = np.zeros(size)
    return arrays


def test_anchor_rows_resolve_from_the_full_session_not_the_selection() -> None:
    """Regression (round-3 review): ``anchor_rows`` took ``min`` of the ticks in the
    supplied selection, so a post-anchor selection (IDs 2..150) re-labelled tick 2 as
    an anchor and replaced a genuine 10 mm projection there with 0 mm."""
    arrays = _trace_arrays([1, 1, 2, 2, 3, 3])
    full = np.arange(6)
    np.testing.assert_array_equal(anchor_rows(arrays, full), [0, 1])

    post_anchor = full[arrays["policy_ticks"] > 1]
    assert anchor_rows(arrays, post_anchor).size == 0
    assert session_anchor_ticks(arrays) == {1.0: 1}

    # The reviewer's counterexample: 10 mm of real projection on tick 2, which the
    # helper must preserve once tick 1 is excluded from the selection.
    tick2 = post_anchor[0]
    arrays["ach_x"][tick2] = arrays["req_x"][tick2] + 0.01
    corrected = correct_anchor_achievable(arrays, post_anchor)
    assert corrected["ach_x"][tick2] - arrays["req_x"][tick2] == pytest.approx(0.01)
    report = channel_report(arrays, post_anchor, urdf_fk=None)
    assert report["requested_vs_achievable"]["position_mm"]["max"] == pytest.approx(10.0)
    # Four post-anchor samples, exactly one of them carrying the 10 mm projection:
    # the tick-2 rows must not be treated as anchors (that would give 0 mm / 10 mm).
    assert report["requested_vs_achievable"]["position_mm"]["mean"] == pytest.approx(
        10.0 / post_anchor.size
    )
    assert report["anchor_rows_in_selection"] == []


def test_anchor_identity_is_resolved_per_session_epoch() -> None:
    """A re-engaged session (new epoch) starts a new anchor; a mid-epoch selection
    still contributes none."""
    arrays = _trace_arrays([1, 2, 3, 1, 2], epochs=[1.0, 1.0, 1.0, 2.0, 2.0])
    assert session_anchor_ticks(arrays) == {1.0: 1, 2.0: 1}
    np.testing.assert_array_equal(anchor_rows(arrays, np.arange(5)), [0, 3])
    np.testing.assert_array_equal(anchor_rows(arrays, np.array([1, 2, 4])), [])


def test_anchor_achievable_is_identity_reconstruction_not_saved_q_fk() -> None:
    """The anchor correction is ``achievable := requested``: it reads no joint column
    and runs no FK, so it must not be described as an independent saved-q FK."""
    arrays = _trace_arrays([1, 1, 2, 2])
    for index in range(6):
        arrays[f"q{index}"] = np.full(4, np.nan)  # no usable saved joints at all
    arrays["ach_x"][:2] = 0.5  # as recorded: plant-frame value at the anchor
    corrected = correct_anchor_achievable(arrays, np.arange(4))
    np.testing.assert_array_equal(corrected["ach_x"][:2], arrays["req_x"][:2])
    # Rows after the anchor keep their recorded value untouched.
    np.testing.assert_array_equal(corrected["ach_x"][2:], arrays["ach_x"][2:])


@pytest.mark.parametrize("dt", [0.05, 0.0167, 0.0, -0.01])
def test_reference_refuses_a_tick_that_is_not_one_policy_period(dt: float) -> None:
    generator, _, _ = _generator(np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    generator.anchor(np.zeros(6), _pose(), 5.0)
    with pytest.raises(ReferenceRefusedError) as excinfo:
        generator.update(_pose(), 5.0 + dt, tick_index=1)
    assert excinfo.value.reason == "dt"


def test_reference_accepts_measured_jitter_and_refuses_a_jump() -> None:
    generator, _, _ = _generator(np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
    generator.anchor(np.zeros(6), _pose(), 1.0)
    # 2% fast tick: still one period of motion, differenced over the actual dt.
    row = generator.update(_pose(), 1.0 + PERIOD * 0.98, tick_index=1)
    assert row.q[0] == pytest.approx(0.2 * PERIOD * 0.98)
    assert row.dq[0] == pytest.approx(0.2)

    jumping, _, _ = _generator(
        np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]), max_reference_velocity_rad_s=2.0
    )
    jumping.anchor(np.zeros(6), _pose(), 1.0)
    with pytest.raises(ReferenceRefusedError) as excinfo:
        jumping.update(_pose(), 1.0 + PERIOD, tick_index=1)
    assert excinfo.value.reason == "jump"


# ---------------------------------------------------------------- clip source
def _write_clip(path: Path, ticks: int = 400, hz: float = 30.0) -> None:
    time_s = np.arange(ticks, dtype=np.float64) / hz
    q = np.tile(np.linspace(-0.5, 0.5, ticks, dtype=np.float32)[:, None], (1, 6))
    np.savez(
        path,
        time_s=time_s,
        q=q,
        dq=np.zeros_like(q),
        tcp_pos=np.zeros((ticks, 3), dtype=np.float32),
        tcp_quat=np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (ticks, 1)),
    )


def test_reference_clip_decimates_a_faster_recorded_clip(tmp_path: Path) -> None:
    """The recorded bank is 60 Hz; the 30 Hz actor takes every second row."""
    path = tmp_path / "clip60.npz"
    _write_clip(path, ticks=400, hz=60.0)
    clip = ReferenceClip(path, policy_hz=30.0, episode_ticks=180)
    assert clip.file_hz == pytest.approx(60.0)
    assert clip.stride == 2
    assert clip.rows == 400
    assert clip.length == 200
    assert clip.row(50).time_s == pytest.approx(50.0 / 30.0)
    np.testing.assert_allclose(clip.row(50).q, clip._q[100])

    with pytest.raises(PolicyContractError):
        ReferenceClip(path, policy_hz=25.0, episode_ticks=180)


def test_reference_clip_anchors_on_the_measured_pose_and_refuses_far_poses(
    tmp_path: Path,
) -> None:
    path = tmp_path / "clip.npz"
    _write_clip(path)
    clip = ReferenceClip(path, policy_hz=30.0, episode_ticks=180)
    assert clip.length == 400
    q = clip.row(0).q
    assert clip.anchor(q, tolerance=0.01) == 0
    with pytest.raises(PolicyContractError):
        clip.anchor(np.full(6, 3.0), tolerance=0.01)

    reference = ClipReference(clip, anchor_index=100, hz=30.0)
    anchor_row = reference.anchor(q + 10.0, None, 0.0)
    assert anchor_row.index == 0
    assert anchor_row.time_s == 0.0
    np.testing.assert_allclose(anchor_row.q, clip.row(100).q)
    next_row = reference.update(None, PERIOD, tick_index=1)
    np.testing.assert_allclose(next_row.q, clip.row(101).q)
    with pytest.raises(PolicyContractError):
        reference.update(None, PERIOD, tick_index=clip.length)


def test_quat_angle_is_reported_in_degrees_exactly_once() -> None:
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    quarter = np.pi / 4.0
    rotate_90 = np.array([np.cos(quarter), 0.0, 0.0, np.sin(quarter)])
    assert quat_angle_deg(identity, identity) == pytest.approx(0.0, abs=1e-9)
    assert quat_angle_deg(identity, rotate_90) == pytest.approx(90.0, abs=1e-6)
    # q and -q are the same rotation (double cover).
    assert quat_angle_deg(identity, -rotate_90) == pytest.approx(90.0, abs=1e-6)

    summary = pose_error_summary([0.001, 0.002, 0.003], [90.0, 0.0, 0.0])
    assert summary["position_mm"]["mean"] == pytest.approx(2.0)
    assert summary["orientation_deg"]["mean"] == pytest.approx(30.0)
    assert summary["orientation_deg"]["max"] == pytest.approx(90.0)


def test_clip_comparison_summarises_orientation_without_a_second_conversion() -> None:
    """Regression: the old summary applied ``np.degrees`` to degrees (90 -> 5156.62)."""
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    quarter = np.pi / 4.0
    rotated = np.array([np.cos(quarter), 0.0, 0.0, np.sin(quarter)])
    samples = 3
    arrays = {
        **{f"q{i}": np.zeros(samples) for i in range(6)},
        "exec_x": np.zeros(samples),
        "exec_y": np.zeros(samples),
        "exec_z": np.zeros(samples),
        **{f"exec_q{axis}": np.full(samples, rotated[index]) for index, axis in enumerate("wxyz")},
        "goal_x": np.zeros(samples),
        "goal_y": np.zeros(samples),
        "goal_z": np.zeros(samples),
        "active": np.ones(samples),
        "tick": np.array([1.0, 2.0, 3.0]),
    }
    training = {
        "q": np.zeros((samples, 1, 6)),
        "full_q": np.zeros((samples, 1, 6)),
        "goal": np.zeros((samples, 1, 7)),
    }

    def kinematics(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return np.zeros(3), identity

    result = compare(arrays, training, kinematics, env=0)
    assert result["aligned_tick"]["tcp_orientation_mean_deg"] == pytest.approx(90.0, abs=1e-6)
    assert result["aligned_tick"]["tcp_orientation_max_deg"] == pytest.approx(90.0, abs=1e-6)
    assert result["aligned_tick"]["tcp_orientation_p95_deg"] == pytest.approx(90.0, abs=1e-6)
