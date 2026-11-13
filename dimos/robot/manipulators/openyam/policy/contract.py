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

"""Deployment contract for the OpenYAM single-arm history policy.

This is *not* the dual-Metal PPO contract: the trained controller here is a
six-joint OpenYAM arm driven by a 16-frame sensor history plus a 19-value
reference goal, at 30 Hz with an unbounded actor output and no decoded-position
clamp. Everything the effective training plant pins down - frame layout, goal
layout, decode law, PD gains, peak torque and the reference format - is
serialised in a hash-pinned manifest so a deployment cannot drift from it
silently.

The simulator contract comes from ``openyam.py`` of the training entry point,
not from the model file text: the training code overrides the compiled model's
actuator range in memory to the motor peak torques below.
"""

from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
from pathlib import Path
from typing import Annotated, Any

import numpy as np
from numpy.typing import NDArray
from pydantic import AfterValidator, ConfigDict, Field, ValidationError, model_validator

from dimos.protocol.service.spec import BaseConfig

SCHEMA = "dimos.openyam_policy.manifest/1"

#: Trained joint order: shoulder to wrist. The coordinator addresses the same
#: joints as ``<robot>/<joint>``; the suffix must match this order.
JOINT_ORDER: tuple[str, ...] = (
    "yam_joint1",
    "yam_joint2",
    "yam_joint3",
    "yam_joint4",
    "yam_joint5",
    "yam_joint6",
)

#: Observation frame layout: ``(name, width)`` in concatenation order.
#:
#: ``issued`` is the command the previous tick sent, ``valid`` marks frames the
#: robot actually observed (reset padding carries 0) and ``time_s`` is the
#: policy tick time. The trained sensor vector is therefore
#: ``[q, dq, issued, valid, time_s]``.
FRAME_FIELDS: tuple[tuple[str, int], ...] = (
    ("q", 6),
    ("dq", 6),
    ("issued", 6),
    ("valid", 1),
    ("time_s", 1),
)

#: Reference-goal layout: ``(name, width)`` in concatenation order. The goal is
#: the recorded TCP pose (position then ``wxyz`` quaternion) followed by the
#: reference joint positions and velocities at the same tick.
GOAL_FIELDS: tuple[tuple[str, int], ...] = (
    ("goal_position", 3),
    ("goal_quaternion_wxyz", 4),
    ("q_reference", 6),
    ("dq_reference", 6),
)

HISTORY_FRAMES = 16
FRAME_DIM = sum(width for _, width in FRAME_FIELDS)
GOAL_DIM = sum(width for _, width in GOAL_FIELDS)
OBS_DIM = HISTORY_FRAMES * FRAME_DIM + GOAL_DIM

#: ``q_des = q_ref + ACTION_SCALE * raw``; the actor output is never clamped.
ACTION_SCALE = 0.1
#: Trained episode length: 180 policy ticks = six seconds of reference motion.
EPISODE_TICKS = 180
POLICY_HZ = 30.0
PHYSICS_HZ = 480.0
CONTROL_SUBSTEPS = 16
#: rsl_rl ``EmpiricalNormalization`` forward is ``(x - mean) / (std + eps)``.
NORMALIZER_EPS = 1e-2

#: Trained PD gains, in ``JOINT_ORDER``.
KP: tuple[float, ...] = (50.0, 60.0, 50.0, 25.0, 15.0, 10.0)
KD: tuple[float, ...] = (3.0, 4.0, 3.0, 2.0, 1.0, 0.8)
#: Declared motor mapping of the training plant and this repository's OpenYAM
#: data: J1-J3 DM4340, J4-J6 DM4310. The exact DM4310 hardware revision is not
#: confirmed on the deployed arm.
MOTOR_TYPES: tuple[str, ...] = ("DM4340", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310")
#: Continuous/rated torque used only by the training objective's soft cost.
RATED: tuple[float, ...] = (9.0, 9.0, 9.0, 3.0, 3.0, 3.0)
#: Peak torque the training plant clamps the PD output to. The runtime never
#: clips commands: the clamp is a plant property (motor peak), not a controller.
PEAK: tuple[float, ...] = (27.0, 27.0, 27.0, 7.0, 7.0, 7.0)
#: Gravity acts on the trained plant and the controller adds no feedforward.
GRAVITY_COMPENSATION = False

#: Joint limits of the trained simulation model (``openyam_single``).
SIM_LOWER: tuple[float, ...] = (-2.61799, 0.0, 0.0, -1.65806, -1.5708, -2.35619)
SIM_UPPER: tuple[float, ...] = (3.14159, 3.66519, 4.01426, 1.65806, 1.5708, 1.8326)
#: Joint limits of the deployed OpenYAM description (``yam_description/i2rt``).
#: They are *not* a subset of the simulated model: the description has a
#: different hand and slightly different joint geometry, so J3 and J6 are
#: narrower while J4 reaches further below the simulated lower limit. A
#: deployment guard therefore refuses anything outside the intersection of the
#: two envelopes instead of assuming one contains the other.
HARDWARE_LOWER: tuple[float, ...] = (-2.61799, 0.0, 0.0, -1.69297, -1.5708, -2.0944)
HARDWARE_UPPER: tuple[float, ...] = (3.14159, 3.66519, 3.14159, 1.5708, 1.5708, 2.0944)
#: Largest per-joint distance between the measured pose and the reference row a
#: session anchors to. The trained reset starts *on* the reference, so a
#: deployment that starts far away would run outside the trained distribution.
ANCHOR_TOLERANCE = 0.35


class PolicyContractError(ValueError):
    """Raised when a manifest, artifact, reference clip or observation is invalid."""


class _ContractModel(BaseConfig):
    model_config = ConfigDict(
        extra="forbid", frozen=True, arbitrary_types_allowed=True, allow_inf_nan=False
    )


Vec6 = Annotated[tuple[float, ...], Field(min_length=6, max_length=6)]


def _current_schema(value: str) -> str:
    if value != SCHEMA:
        raise ValueError(f"unsupported manifest schema {value!r}, expected {SCHEMA!r}")
    return value


SchemaName = Annotated[str, AfterValidator(_current_schema)]


def _message(exc: ValidationError) -> str:
    errors = exc.errors()
    loc = ".".join(str(part) for part in errors[0].get("loc", ()))
    return f"{loc}: {errors[0]['msg']}" if loc else errors[0]["msg"]


class FrameField(_ContractModel):
    name: str
    width: int


def _trained_joint_order(values: tuple[str, ...]) -> tuple[str, ...]:
    if values != JOINT_ORDER:
        raise ValueError("manifest joint order differs from the trained order")
    return values


def _trained_layout(
    expected: tuple[tuple[str, int], ...],
) -> Any:
    def check(values: tuple[FrameField, ...]) -> tuple[FrameField, ...]:
        if tuple((field.name, field.width) for field in values) != expected:
            raise ValueError("manifest layout differs from the trained layout")
        return values

    return check


TrainedJointOrder = Annotated[tuple[str, ...], AfterValidator(_trained_joint_order)]
TrainedFrameLayout = Annotated[
    tuple[FrameField, ...], AfterValidator(_trained_layout(FRAME_FIELDS))
]
TrainedGoalLayout = Annotated[tuple[FrameField, ...], AfterValidator(_trained_layout(GOAL_FIELDS))]


class CheckpointSpec(_ContractModel):
    """Identity of the frozen actor checkpoint the artifact was built from."""

    checkpoint_sha256: str
    actor_updates: int = Field(ge=0)
    identity: str
    extraction_sha256: str
    source_sha256: str


class ModelSpec(_ContractModel):
    onnx_sha256: str
    normalizer_sha256: str
    input: str = "obs"
    output: str = "action"
    obs_dim: int = OBS_DIM
    action_dim: int = len(JOINT_ORDER)
    architecture: str = "multi-encoder-ae:goal19-256-256-32-tanh|history320-decoder-352-256-256-6"


class ObservationSpec(_ContractModel):
    history_frames: int = Field(default=HISTORY_FRAMES, ge=1)
    frame_dim: int = Field(default=FRAME_DIM, ge=1)
    goal_dim: int = Field(default=GOAL_DIM, ge=1)
    joint_order: TrainedJointOrder = JOINT_ORDER
    frame_fields: TrainedFrameLayout = tuple(FrameField(name=n, width=w) for n, w in FRAME_FIELDS)
    goal_fields: TrainedGoalLayout = tuple(FrameField(name=n, width=w) for n, w in GOAL_FIELDS)


class ActionSpec(_ContractModel):
    """The trained decode law. Nothing here clips or filters the command."""

    action_scale: float = Field(default=ACTION_SCALE, gt=0.0)
    anchor: str = "reference_q"
    clamp: str = "none"


class TimingSpec(_ContractModel):
    policy_hz: float = Field(default=POLICY_HZ, gt=0.0)
    physics_hz: float = Field(default=PHYSICS_HZ, gt=0.0)
    control_substeps: int = Field(default=CONTROL_SUBSTEPS, ge=1)
    episode_ticks: int = Field(default=EPISODE_TICKS, ge=1)


class NormalizerSpec(_ContractModel):
    eps: float = NORMALIZER_EPS


class MotorSpec(_ContractModel):
    """Motors, gains and the peak clamp that live in the plant, not the actor."""

    motor_types: tuple[str, ...] = MOTOR_TYPES
    kp: Vec6 = KP
    kd: Vec6 = KD
    rated: Vec6 = RATED
    peak: Vec6 = PEAK
    gravity_compensation: bool = GRAVITY_COMPENSATION


class RobotSpec(_ContractModel):
    """Robot identity and the two joint envelopes that matter at deployment."""

    name: str = "openyam"
    description: str = "yam_description/i2rt/yam.urdf"
    tool_frame: str = "gripper"
    sim_lower: Vec6 = SIM_LOWER
    sim_upper: Vec6 = SIM_UPPER
    hardware_lower: Vec6 = HARDWARE_LOWER
    hardware_upper: Vec6 = HARDWARE_UPPER
    anchor_tolerance: float = Field(default=ANCHOR_TOLERANCE, gt=0.0)

    @property
    def effective_lower(self) -> tuple[float, ...]:
        """Intersection of the simulated and deployed envelopes, per joint."""
        return tuple(
            max(lo, hi) for lo, hi in zip(self.sim_lower, self.hardware_lower, strict=True)
        )

    @property
    def effective_upper(self) -> tuple[float, ...]:
        return tuple(
            min(lo, hi) for lo, hi in zip(self.sim_upper, self.hardware_upper, strict=True)
        )


class ReferenceSpec(_ContractModel):
    """Frozen reference-motion clip the actor is driven from.

    The clip carries exactly the recorded quantities the training bank stored:
    joint positions and velocities plus the TCP pose, sampled at the policy
    rate. It is the *same* data type the training sampler reads, so the
    deployment never invents a reference from IK or from a live privileged
    source.
    """

    clip_sha256: str
    clip_file: str = "reference_clip.npz"
    hz: float = Field(default=POLICY_HZ, gt=0.0)
    fields: tuple[str, ...] = ("time_s", "q", "dq", "tcp_pos", "tcp_quat")
    quaternion_order: str = "wxyz"


class PolicyManifest(_ContractModel):
    """Validated manifest document for one exported OpenYAM policy artifact."""

    schema_: SchemaName = Field(alias="schema")
    name: str
    checkpoint: CheckpointSpec
    model: ModelSpec
    observation: ObservationSpec = ObservationSpec()
    action: ActionSpec = ActionSpec()
    timing: TimingSpec = TimingSpec()
    normalizer: NormalizerSpec = NormalizerSpec()
    motors: MotorSpec = MotorSpec()
    robot: RobotSpec = RobotSpec()
    reference: ReferenceSpec | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _validate_dimensions(self) -> PolicyManifest:
        observation, model, motors, robot = (
            self.observation,
            self.model,
            self.motors,
            self.robot,
        )
        joints = len(observation.joint_order)
        if model.action_dim != joints:
            raise ValueError(f"action_dim {model.action_dim} != {joints} joint names")
        if sum(f.width for f in observation.frame_fields) != observation.frame_dim:
            raise ValueError("frame fields do not sum to frame_dim")
        if sum(f.width for f in observation.goal_fields) != observation.goal_dim:
            raise ValueError("goal fields do not sum to goal_dim")
        if (
            model.obs_dim
            != observation.history_frames * observation.frame_dim + observation.goal_dim
        ):
            raise ValueError("obs_dim must equal history_frames * frame_dim + goal_dim")
        for name in ("motor_types", "kp", "kd", "rated", "peak"):
            if len(getattr(motors, name)) != joints:
                raise ValueError(f"motors.{name} must have one entry per joint")
        if any(kp <= 0.0 for kp in motors.kp) or any(kd <= 0.0 for kd in motors.kd):
            raise ValueError("PD gains must be positive")
        if any(peak <= rated for peak, rated in zip(motors.peak, motors.rated, strict=True)):
            raise ValueError("peak torque must exceed the rated torque")
        for lower, upper in (
            (robot.sim_lower, robot.sim_upper),
            (robot.hardware_lower, robot.hardware_upper),
        ):
            if any(lo >= hi for lo, hi in zip(lower, upper, strict=True)):
                raise ValueError("joint lower limit must be below the upper limit")
        if self.reference is not None and abs(self.reference.hz - self.timing.policy_hz) > 1e-6:
            raise ValueError("reference clip rate must equal the policy rate")
        return self

    @property
    def tick_period_s(self) -> float:
        return 1.0 / self.timing.policy_hz

    def verify_artifacts(self, model_dir: str | Path) -> tuple[Path, Path]:
        """Check the ONNX and normalizer hashes and return their paths."""
        root = Path(model_dir)
        onnx_path = root / self.metadata.get("onnx_file", "openyam_policy.onnx")
        normalizer_path = root / self.metadata.get(
            "normalizer_file", "openyam_policy.normalizer.npz"
        )
        for path, expected, label in (
            (onnx_path, self.model.onnx_sha256, "ONNX actor"),
            (normalizer_path, self.model.normalizer_sha256, "observation normalizer"),
        ):
            if not path.is_file():
                raise PolicyContractError(f"{label} not found: {path}")
            actual = sha256_file(path)
            if actual != expected:
                raise PolicyContractError(
                    f"{label} hash mismatch at {path}: {actual} != manifest {expected}"
                )
        return onnx_path, normalizer_path

    def verify_reference(self, model_dir: str | Path) -> Path:
        """Check the frozen reference clip hash and return its path."""
        if self.reference is None:
            raise PolicyContractError("manifest does not pin a reference clip")
        path = Path(model_dir) / self.reference.clip_file
        if not path.is_file():
            raise PolicyContractError(f"reference clip not found: {path}")
        actual = sha256_file(path)
        if actual != self.reference.clip_sha256:
            raise PolicyContractError(
                f"reference clip hash mismatch at {path}: {actual} != manifest "
                f"{self.reference.clip_sha256}"
            )
        return path


def validate_manifest(payload: Mapping[str, Any]) -> PolicyManifest:
    try:
        return PolicyManifest.model_validate(payload)
    except ValidationError as exc:
        raise PolicyContractError(_message(exc)) from exc


def load_manifest(path: str | Path) -> PolicyManifest:
    payload = json.loads(Path(path).read_text())
    if not isinstance(payload, dict):
        raise PolicyContractError("manifest root must be a JSON object")
    return validate_manifest(payload)


def write_manifest(manifest: PolicyManifest, path: str | Path) -> Path:
    out = Path(path)
    out.write_text(json.dumps(manifest.model_dump(by_alias=True), indent=2) + "\n")
    return out


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_normalizer(path: str | Path, manifest: PolicyManifest) -> tuple[NDArray[np.float32], ...]:
    """Load the learned statistics, rejecting non-finite or wrongly shaped arrays."""
    required = {"obs_mean", "obs_std", "goal_mean", "goal_std"}
    with np.load(path, allow_pickle=False) as data:
        missing = required - set(data.files)
        if missing:
            raise PolicyContractError(f"normalizer is missing arrays: {sorted(missing)}")
        arrays = {key: np.asarray(data[key], dtype=np.float32).reshape(-1) for key in required}
    observation = manifest.observation
    for key, width in (
        ("obs_mean", observation.frame_dim),
        ("obs_std", observation.frame_dim),
        ("goal_mean", observation.goal_dim),
        ("goal_std", observation.goal_dim),
    ):
        if arrays[key].shape != (width,):
            raise PolicyContractError(f"normalizer {key} width {arrays[key].shape} != ({width},)")
        if not np.isfinite(arrays[key]).all():
            raise PolicyContractError(f"normalizer {key} contains non-finite values")
    for key in ("obs_std", "goal_std"):
        # A constant column (the validity flag, and time at reset) has exactly
        # zero variance; the trained forward divides by ``std + eps``, so zero is
        # legitimate and a negative value is not.
        if (arrays[key] < 0.0).any():
            raise PolicyContractError(f"normalizer {key} std must not be negative")
    return arrays["obs_mean"], arrays["obs_std"], arrays["goal_mean"], arrays["goal_std"]


def check_observation(obs: NDArray[np.float32], manifest: PolicyManifest) -> None:
    if obs.shape != (1, manifest.model.obs_dim):
        raise PolicyContractError(f"observation shape {obs.shape} != (1, {manifest.model.obs_dim})")
    if not np.isfinite(obs).all():
        raise PolicyContractError("observation contains non-finite values")


def check_goal_motion(goal: NDArray[np.float32], manifest: PolicyManifest) -> None:
    """Fail closed on a malformed or non-finite reference goal."""
    width = manifest.observation.goal_dim
    values = np.asarray(goal, dtype=np.float32).reshape(-1)
    if values.shape != (width,):
        raise PolicyContractError(f"goal width {values.shape} != ({width},)")
    if not np.isfinite(values).all():
        raise PolicyContractError("goal contains non-finite values")
    quaternion = values[3:7]
    norm = float(np.linalg.norm(quaternion))
    if not 0.5 <= norm <= 1.5:
        raise PolicyContractError(f"goal quaternion is not unit length (norm {norm:.3f})")


def check_feedback(
    q: NDArray[np.float32], dq: NDArray[np.float32], manifest: PolicyManifest
) -> None:
    """Reject incomplete or out-of-envelope joint feedback before the actor sees it."""
    width = manifest.model.action_dim
    for values, name in ((q, "joint positions"), (dq, "joint velocities")):
        if values.shape != (width,):
            raise PolicyContractError(f"{name} shape {values.shape} != ({width},)")
        if not np.isfinite(values).all():
            raise PolicyContractError(f"{name} contain non-finite values")
    outside = [
        name
        for name, value, lower, upper in zip(
            manifest.observation.joint_order,
            q,
            manifest.robot.sim_lower,
            manifest.robot.sim_upper,
            strict=True,
        )
        if not lower - 1e-3 <= value <= upper + 1e-3
    ]
    if outside:
        raise PolicyContractError(f"measured joints outside the simulated limits: {outside}")


def hardware_violations(command: NDArray[np.float32], manifest: PolicyManifest) -> list[str]:
    """Joints whose command leaves the deployable envelope.

    The envelope is the intersection of the simulated and deployed descriptions,
    because neither contains the other. This is a deployment guard, not part of
    the trained decode: the decoder is never clamped, and a violating command
    ends the session instead of being silently rewritten into something the
    actor never produced.
    """
    robot = manifest.robot
    values = np.asarray(command, dtype=np.float32).reshape(-1)
    return [
        name
        for name, value, lower, upper in zip(
            manifest.observation.joint_order,
            values,
            robot.effective_lower,
            robot.effective_upper,
            strict=True,
        )
        if not lower - 1e-6 <= value <= upper + 1e-6
    ]
