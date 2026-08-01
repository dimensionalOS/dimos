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

from dimos.benchmark.dimsim.models import NavigateContract, Pose2
from dimos.simulation.dimsim.evaluation import (
    AuthoritativeBodySample,
    DimSimNativeEvaluator,
    DimSimResetCoordinator,
    DimSimResetRequest,
    NavigationRubric,
)

_ATTEMPT_ID = "attempt_" + "1" * 32
_OPERATION_ID = "operation_" + "2" * 32
_TASK_ID = "task_" + "3" * 64
_START_POSE = Pose2(x_m=3.0, z_m=0.0, yaw_rad=0.0)
_TARGET = ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5))


class _FakeControl:
    def __init__(self, sample: AuthoritativeBodySample | BaseException) -> None:
        self.sample = sample
        self.calls: list[object] = []

    def clear_motion(self) -> None:
        self.calls.append("clear")

    def settle_motion(self) -> None:
        self.calls.append("settle")

    def teleport(self, pose: Pose2) -> None:
        self.calls.append(("teleport", pose))

    def wait_body_sample(self, timeout_s: float) -> AuthoritativeBodySample:
        self.calls.append(("wait", timeout_s))
        if isinstance(self.sample, BaseException):
            raise self.sample
        return self.sample


def _sample(
    x: float,
    *,
    time_s: float = 0.0,
    linear: float = 0.0,
    angular: float = 0.0,
    yaw: float = 0.0,
    z: float = 0.0,
) -> AuthoritativeBodySample:
    return AuthoritativeBodySample(
        pose=Pose2(x_m=x, z_m=z, yaw_rad=yaw),
        linear_speed_m_s=linear,
        angular_speed_rad_s=angular,
        simulated_time_s=time_s,
        pose_timestamp_s=time_s + 100.0,
    )


def _reset_request(**updates: object) -> DimSimResetRequest:
    values = {
        "attempt_id": _ATTEMPT_ID,
        "operation_id": _OPERATION_ID,
        "task_id": _TASK_ID,
        "episode_id": "episode-1",
        "scene_id": "dimsim-apartment",
        "profile_revision": "profile-v1",
        "reset_revision": "reset-v1",
        "requested_start_pose": _START_POSE,
        "expected_previous_generation": 0,
    }
    values.update(updates)
    return DimSimResetRequest(**values)


def _coordinator(
    sample: AuthoritativeBodySample | BaseException,
) -> tuple[DimSimResetCoordinator, _FakeControl]:
    control = _FakeControl(sample)
    return (
        DimSimResetCoordinator(
            control,
            scene_id="dimsim-apartment",
            profile_revision="profile-v1",
            reset_revision="reset-v1",
        ),
        control,
    )


def _contract() -> NavigateContract:
    return NavigateContract(
        target_entity_id="bathtub",
        threshold_m=1.0,
        clearance_policy_version="clearance-v1",
        linear_speed_tolerance_m_s=0.05,
        angular_speed_tolerance_rad_s=0.1,
        stationary_dwell_s=1.0,
    )


def _rubric() -> NavigationRubric:
    return NavigationRubric(_contract(), _TARGET, robot_footprint_radius_m=0.2)


def _start_evaluator(
    initial: AuthoritativeBodySample | None = None,
    *,
    deadline_s: float = 5.0,
) -> tuple[DimSimNativeEvaluator, str]:
    evaluator = DimSimNativeEvaluator()
    handle = evaluator.start(
        attempt_id=_ATTEMPT_ID,
        operation_id=_OPERATION_ID,
        task_id=_TASK_ID,
        episode_id="episode-1",
        reset_generation=1,
        deadline_s=deadline_s,
        rubric=_rubric(),
        initial_sample=initial or _sample(3.0),
    )
    return evaluator, handle.evaluation_id


def test_reset_clears_motion_teleports_waits_and_acknowledges() -> None:
    coordinator, control = _coordinator(_sample(3.0))

    acknowledgement = coordinator.reset(_reset_request(), timeout_s=2.0)

    assert acknowledgement.attempt_id == _ATTEMPT_ID
    assert acknowledgement.operation_id == _OPERATION_ID
    assert acknowledgement.applied_start_pose == _START_POSE
    assert acknowledgement.reset_generation == 1
    assert control.calls == [
        "clear",
        "settle",
        ("teleport", _START_POSE),
        ("wait", 2.0),
    ]


@pytest.mark.parametrize(
    ("update", "message"),
    [
        ({"scene_id": "other-scene"}, "scene identity"),
        ({"profile_revision": "other-profile"}, "profile revision"),
        ({"reset_revision": "other-reset"}, "reset policy"),
        ({"expected_previous_generation": 1}, "stale reset generation"),
        (
            {"requested_start_pose": Pose2(x_m=3.0, z_m=0.0, yaw_rad=0.5)},
            "zero-yaw",
        ),
    ],
)
def test_reset_rejects_incompatible_or_stale_request(
    update: dict[str, object], message: str
) -> None:
    coordinator, _ = _coordinator(_sample(3.0))

    with pytest.raises(ValueError, match=message):
        coordinator.reset(_reset_request(**update), timeout_s=2.0)


@pytest.mark.parametrize(
    ("sample", "message"),
    [
        (_sample(3.2), "position mismatch"),
        (_sample(3.0, yaw=0.2), "yaw mismatch"),
        (_sample(3.0, linear=0.2), "residual linear"),
        (_sample(3.0, angular=0.2), "residual angular"),
    ],
)
def test_reset_rejects_bad_acknowledged_body_state(
    sample: AuthoritativeBodySample, message: str
) -> None:
    coordinator, _ = _coordinator(sample)

    with pytest.raises(ValueError, match=message):
        coordinator.reset(_reset_request(), timeout_s=2.0)


def test_reset_propagates_body_sample_timeout() -> None:
    coordinator, _ = _coordinator(TimeoutError("odometry timeout"))

    with pytest.raises(TimeoutError, match="odometry timeout"):
        coordinator.reset(_reset_request(), timeout_s=0.1)


def test_rubric_uses_robot_footprint_surface_distance_and_boundary() -> None:
    rubric = _rubric()

    assert rubric.distance(Pose2(x_m=1.7, z_m=0.0, yaw_rad=0.0)) == pytest.approx(1.0)
    assert rubric.instantaneous_condition(_sample(1.7))
    assert not rubric.instantaneous_condition(_sample(1.71))


def test_rubric_requires_continuous_stationary_dwell() -> None:
    rubric = _rubric()

    assert not rubric.observe(_sample(1.6, time_s=1.0))
    assert not rubric.observe(_sample(1.6, time_s=1.5))
    assert not rubric.observe(_sample(1.6, time_s=1.6, linear=0.1))
    assert not rubric.observe(_sample(1.6, time_s=2.0))
    assert rubric.observe(_sample(1.6, time_s=3.0))
    assert rubric.stationary_dwell_s == pytest.approx(1.0)


def test_pass_through_goal_region_does_not_pass() -> None:
    rubric = _rubric()

    assert not rubric.observe(_sample(1.6, time_s=1.0, linear=0.5))
    assert not rubric.observe(_sample(0.0, time_s=2.0, linear=0.5))
    assert not rubric.observe(_sample(-3.0, time_s=3.0))


def test_evaluator_rejects_initially_satisfied_episode() -> None:
    with pytest.raises(ValueError, match="initially satisfied"):
        _start_evaluator(_sample(1.6))


def test_evaluator_emits_exactly_one_correlated_success() -> None:
    evaluator, evaluation_id = _start_evaluator()

    assert evaluator.observe(evaluation_id, _sample(1.6, time_s=1.0)) is None
    result = evaluator.observe(evaluation_id, _sample(1.6, time_s=2.0))

    assert result is not None
    assert result.passed
    assert result.evaluation_id == evaluation_id
    assert result.attempt_id == _ATTEMPT_ID
    assert evaluator.result(evaluation_id) == result
    with pytest.raises(RuntimeError, match="terminal result"):
        evaluator.observe(evaluation_id, _sample(1.6, time_s=3.0))


def test_evaluator_timeout_retains_metric_evidence() -> None:
    evaluator, evaluation_id = _start_evaluator(deadline_s=2.0)

    result = evaluator.observe(evaluation_id, _sample(2.0, time_s=2.0))

    assert result is not None
    assert not result.passed
    assert result.terminal_stage == "episode-timeout"
    assert result.final_distance_m is not None


def test_evaluator_rejects_stale_identity() -> None:
    evaluator, _ = _start_evaluator()

    with pytest.raises(ValueError, match="stale evaluation identity"):
        evaluator.observe("dimsim_evaluation_" + "0" * 32, _sample(3.0))


def test_evaluator_turns_malformed_time_into_terminal_failure() -> None:
    evaluator, evaluation_id = _start_evaluator()
    evaluator.observe(evaluation_id, _sample(3.0, time_s=2.0))

    result = evaluator.observe(evaluation_id, _sample(3.0, time_s=1.0))

    assert result is not None
    assert result.terminal_stage == "malformed-sample"


def test_evaluator_cancel_is_idempotent() -> None:
    evaluator, evaluation_id = _start_evaluator()

    first = evaluator.cancel(evaluation_id)
    second = evaluator.cancel(evaluation_id)

    assert first == second
    assert first is not None
    assert first.terminal_stage == "cancelled"


def test_evaluator_rejects_overlapping_active_evaluation() -> None:
    evaluator, _ = _start_evaluator()

    with pytest.raises(RuntimeError, match="already active"):
        evaluator.start(
            attempt_id=_ATTEMPT_ID,
            operation_id=_OPERATION_ID,
            task_id=_TASK_ID,
            episode_id="episode-2",
            reset_generation=2,
            deadline_s=5.0,
            rubric=_rubric(),
            initial_sample=_sample(3.0),
        )
