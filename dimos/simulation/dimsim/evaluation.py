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

"""Private correlated reset and navigation rubric for attached DimSim evaluation."""

from __future__ import annotations

from typing import Annotated, Literal, Protocol
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field
from shapely.geometry import Point, Polygon

from dimos.benchmark.agent_eval.models import AttemptId, OperationId
from dimos.benchmark.dimsim.models import NavigateContract, OpaqueId, Point2, Pose2

NonEmpty = Annotated[str, Field(min_length=1)]
EvaluationId = Annotated[str, Field(pattern=r"^dimsim_evaluation_[0-9a-f]{32}$")]


class DimSimEvaluationModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    schema_version: Literal["1.0"] = "1.0"


class AuthoritativeBodySample(DimSimEvaluationModel):
    record_type: Literal["dimsim-body-sample"] = "dimsim-body-sample"
    pose: Pose2
    linear_speed_m_s: Annotated[float, Field(ge=0)]
    angular_speed_rad_s: Annotated[float, Field(ge=0)]
    simulated_time_s: Annotated[float, Field(ge=0)]
    pose_timestamp_s: Annotated[float, Field(ge=0)]


class DimSimResetRequest(DimSimEvaluationModel):
    record_type: Literal["dimsim-reset-request"] = "dimsim-reset-request"
    attempt_id: AttemptId
    operation_id: OperationId
    task_id: OpaqueId
    episode_id: NonEmpty
    scene_id: NonEmpty
    profile_revision: NonEmpty
    reset_revision: NonEmpty
    requested_start_pose: Pose2
    expected_previous_generation: Annotated[int, Field(ge=0)]


class DimSimResetAcknowledgement(DimSimEvaluationModel):
    record_type: Literal["dimsim-reset-acknowledgement"] = "dimsim-reset-acknowledgement"
    attempt_id: AttemptId
    operation_id: OperationId
    task_id: OpaqueId
    episode_id: NonEmpty
    scene_id: NonEmpty
    profile_revision: NonEmpty
    reset_revision: NonEmpty
    requested_start_pose: Pose2
    applied_start_pose: Pose2
    reset_generation: Annotated[int, Field(ge=1)]
    linear_speed_m_s: Annotated[float, Field(ge=0)]
    angular_speed_rad_s: Annotated[float, Field(ge=0)]
    pose_timestamp_s: Annotated[float, Field(ge=0)]


class DimSimEvaluationStart(DimSimEvaluationModel):
    record_type: Literal["dimsim-evaluation-start"] = "dimsim-evaluation-start"
    attempt_id: AttemptId
    operation_id: OperationId
    evaluation_id: EvaluationId
    task_id: OpaqueId
    episode_id: NonEmpty
    reset_generation: Annotated[int, Field(ge=1)]
    deadline_s: Annotated[float, Field(gt=0)]


class DimSimNativeResult(DimSimEvaluationModel):
    record_type: Literal["dimsim-native-result"] = "dimsim-native-result"
    attempt_id: AttemptId
    operation_id: OperationId
    evaluation_id: EvaluationId
    task_id: OpaqueId
    episode_id: NonEmpty
    passed: bool
    terminal_stage: Literal[
        "predicate-satisfied",
        "episode-timeout",
        "cancelled",
        "malformed-sample",
    ]
    reason: NonEmpty
    duration_s: Annotated[float, Field(ge=0)]
    final_distance_m: Annotated[float, Field(ge=0)] | None
    stationary_dwell_s: Annotated[float, Field(ge=0)]


class DimSimRuntimeControl(Protocol):
    """Authoritative reset operations supplied by the attached simulator adapter."""

    def clear_motion(self) -> None: ...

    def settle_motion(self) -> None: ...

    def teleport(self, pose: Pose2) -> None: ...

    def wait_body_sample(self, timeout_s: float) -> AuthoritativeBodySample: ...


class DimSimResetCoordinator:
    """Apply and validate attempt-correlated resets for one compatible scene."""

    def __init__(
        self,
        control: DimSimRuntimeControl,
        *,
        scene_id: str,
        profile_revision: str,
        reset_revision: str,
        position_tolerance_m: float = 0.03,
        yaw_tolerance_rad: float = 0.03,
        stopped_linear_tolerance_m_s: float = 0.05,
        stopped_angular_tolerance_rad_s: float = 0.1,
    ) -> None:
        self._control = control
        self._scene_id = scene_id
        self._profile_revision = profile_revision
        self._reset_revision = reset_revision
        self._position_tolerance_m = position_tolerance_m
        self._yaw_tolerance_rad = yaw_tolerance_rad
        self._linear_tolerance = stopped_linear_tolerance_m_s
        self._angular_tolerance = stopped_angular_tolerance_rad_s
        self._generation = 0

    @property
    def generation(self) -> int:
        return self._generation

    def reset(self, request: DimSimResetRequest, timeout_s: float) -> DimSimResetAcknowledgement:
        if request.scene_id != self._scene_id:
            raise ValueError("reset scene identity mismatch")
        if request.profile_revision != self._profile_revision:
            raise ValueError("reset profile revision mismatch")
        if request.reset_revision != self._reset_revision:
            raise ValueError("reset policy revision mismatch")
        if request.expected_previous_generation != self._generation:
            raise ValueError("stale reset generation")
        if request.requested_start_pose.yaw_rad != 0.0:
            raise ValueError("pinned DimSim reset supports only the profile's zero-yaw spawn")

        self._control.clear_motion()
        self._control.settle_motion()
        self._control.teleport(request.requested_start_pose)
        sample = self._control.wait_body_sample(timeout_s)
        _validate_applied_pose(
            request.requested_start_pose,
            sample.pose,
            self._position_tolerance_m,
            self._yaw_tolerance_rad,
        )
        if sample.linear_speed_m_s > self._linear_tolerance:
            raise ValueError("reset acknowledgement has residual linear motion")
        if sample.angular_speed_rad_s > self._angular_tolerance:
            raise ValueError("reset acknowledgement has residual angular motion")
        self._generation += 1
        return DimSimResetAcknowledgement(
            attempt_id=request.attempt_id,
            operation_id=request.operation_id,
            task_id=request.task_id,
            episode_id=request.episode_id,
            scene_id=request.scene_id,
            profile_revision=request.profile_revision,
            reset_revision=request.reset_revision,
            requested_start_pose=request.requested_start_pose,
            applied_start_pose=sample.pose,
            reset_generation=self._generation,
            linear_speed_m_s=sample.linear_speed_m_s,
            angular_speed_rad_s=sample.angular_speed_rad_s,
            pose_timestamp_s=sample.pose_timestamp_s,
        )


class NavigationRubric:
    """Contract-driven robot-footprint distance and stationary-dwell state."""

    def __init__(
        self,
        contract: NavigateContract,
        target_footprint: tuple[Point2, ...],
        robot_footprint_radius_m: float,
    ) -> None:
        if robot_footprint_radius_m <= 0:
            raise ValueError("robot footprint radius must be positive")
        self.contract = contract
        self._target = Polygon(target_footprint)
        if not self._target.is_valid or self._target.is_empty:
            raise ValueError("target footprint must be a valid polygon")
        self._robot_radius = robot_footprint_radius_m
        self._stationary_since_s: float | None = None
        self._last_time_s: float | None = None
        self._last_distance_m: float | None = None
        self._stationary_dwell_s = 0.0

    @property
    def last_distance_m(self) -> float | None:
        return self._last_distance_m

    @property
    def stationary_dwell_s(self) -> float:
        return self._stationary_dwell_s

    def instantaneous_condition(self, sample: AuthoritativeBodySample) -> bool:
        distance = self.distance(sample.pose)
        return (
            distance <= self.contract.threshold_m
            and sample.linear_speed_m_s <= self.contract.linear_speed_tolerance_m_s
            and sample.angular_speed_rad_s <= self.contract.angular_speed_tolerance_rad_s
        )

    def observe(self, sample: AuthoritativeBodySample) -> bool:
        if self._last_time_s is not None and sample.simulated_time_s < self._last_time_s:
            raise ValueError("simulated time moved backwards")
        self._last_time_s = sample.simulated_time_s
        self._last_distance_m = self.distance(sample.pose)
        if not self.instantaneous_condition(sample):
            self._stationary_since_s = None
            self._stationary_dwell_s = 0.0
            return False
        if self._stationary_since_s is None:
            self._stationary_since_s = sample.simulated_time_s
        self._stationary_dwell_s = sample.simulated_time_s - self._stationary_since_s
        return self._stationary_dwell_s >= self.contract.stationary_dwell_s

    def distance(self, pose: Pose2) -> float:
        robot = Point(pose.x_m, pose.z_m).buffer(self._robot_radius)
        return float(robot.distance(self._target))


class DimSimNativeEvaluator:
    """Exactly-once active lifecycle around the pure navigation rubric."""

    def __init__(self) -> None:
        self._start: DimSimEvaluationStart | None = None
        self._rubric: NavigationRubric | None = None
        self._started_simulated_time_s: float | None = None
        self._result: DimSimNativeResult | None = None
        self._cancelled_ids: set[str] = set()

    def start(
        self,
        *,
        attempt_id: str,
        operation_id: str,
        task_id: str,
        episode_id: str,
        reset_generation: int,
        deadline_s: float,
        rubric: NavigationRubric,
        initial_sample: AuthoritativeBodySample,
    ) -> DimSimEvaluationStart:
        if self._start is not None and self._result is None:
            raise RuntimeError("a DimSim evaluation is already active")
        if rubric.instantaneous_condition(initial_sample):
            raise ValueError("navigation predicate is initially satisfied")
        self._start = DimSimEvaluationStart(
            attempt_id=attempt_id,
            operation_id=operation_id,
            evaluation_id=f"dimsim_evaluation_{uuid4().hex}",
            task_id=task_id,
            episode_id=episode_id,
            reset_generation=reset_generation,
            deadline_s=deadline_s,
        )
        self._rubric = rubric
        self._started_simulated_time_s = initial_sample.simulated_time_s
        self._result = None
        return self._start

    def observe(
        self,
        evaluation_id: str,
        sample: AuthoritativeBodySample,
    ) -> DimSimNativeResult | None:
        start, rubric, started_at = self._require_active(evaluation_id)
        elapsed = max(0.0, sample.simulated_time_s - started_at)
        try:
            passed = rubric.observe(sample)
        except ValueError:
            return self._finish(
                start,
                passed=False,
                stage="malformed-sample",
                reason="authoritative sample was malformed",
                duration_s=elapsed,
                rubric=rubric,
            )
        if passed:
            return self._finish(
                start,
                passed=True,
                stage="predicate-satisfied",
                reason="distance and stationary-dwell predicate satisfied",
                duration_s=elapsed,
                rubric=rubric,
            )
        if elapsed >= start.deadline_s:
            return self._finish(
                start,
                passed=False,
                stage="episode-timeout",
                reason="episode deadline elapsed",
                duration_s=elapsed,
                rubric=rubric,
            )
        return None

    def result(self, evaluation_id: str) -> DimSimNativeResult | None:
        if self._start is None or self._start.evaluation_id != evaluation_id:
            raise ValueError("stale evaluation identity")
        return self._result

    def cancel(self, evaluation_id: str) -> DimSimNativeResult | None:
        if self._start is None or self._start.evaluation_id != evaluation_id:
            raise ValueError("stale evaluation identity")
        if evaluation_id in self._cancelled_ids or self._result is not None:
            return self._result
        start, rubric, started_at = self._require_active(evaluation_id)
        duration = 0.0
        if rubric._last_time_s is not None:
            duration = max(0.0, rubric._last_time_s - started_at)
        self._cancelled_ids.add(evaluation_id)
        return self._finish(
            start,
            passed=False,
            stage="cancelled",
            reason="evaluation cancelled by runner",
            duration_s=duration,
            rubric=rubric,
        )

    def _require_active(
        self, evaluation_id: str
    ) -> tuple[DimSimEvaluationStart, NavigationRubric, float]:
        if self._start is None or self._start.evaluation_id != evaluation_id:
            raise ValueError("stale evaluation identity")
        if self._result is not None:
            raise RuntimeError("evaluation already has a terminal result")
        assert self._rubric is not None
        assert self._started_simulated_time_s is not None
        return self._start, self._rubric, self._started_simulated_time_s

    def _finish(
        self,
        start: DimSimEvaluationStart,
        *,
        passed: bool,
        stage: Literal[
            "predicate-satisfied",
            "episode-timeout",
            "cancelled",
            "malformed-sample",
        ],
        reason: str,
        duration_s: float,
        rubric: NavigationRubric,
    ) -> DimSimNativeResult:
        if self._result is not None:
            raise RuntimeError("duplicate terminal result")
        self._result = DimSimNativeResult(
            attempt_id=start.attempt_id,
            operation_id=start.operation_id,
            evaluation_id=start.evaluation_id,
            task_id=start.task_id,
            episode_id=start.episode_id,
            passed=passed,
            terminal_stage=stage,
            reason=reason,
            duration_s=duration_s,
            final_distance_m=rubric.last_distance_m,
            stationary_dwell_s=rubric.stationary_dwell_s,
        )
        return self._result


def _validate_applied_pose(
    requested: Pose2,
    applied: Pose2,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> None:
    error = ((requested.x_m - applied.x_m) ** 2 + (requested.z_m - applied.z_m) ** 2) ** 0.5
    if error > position_tolerance_m:
        raise ValueError("reset applied pose position mismatch")
    yaw_error = abs(
        (requested.yaw_rad - applied.yaw_rad + 3.141592653589793) % (2 * 3.141592653589793)
        - 3.141592653589793
    )
    if yaw_error > yaw_tolerance_rad:
        raise ValueError("reset applied pose yaw mismatch")
