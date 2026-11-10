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

"""Deployment contract for the trained dual Metal bimanual PPO policy.

Everything the runtime is allowed to assume about the trained controller lives
here and in a versioned JSON manifest that pins the exact checkpoint lineage,
observation layout, action decode order and station geometry. The manifest is a
validated configuration model: unknown keys, wrong widths, non-finite values and
mismatched trained layouts fail closed before the runtime starts.
"""

from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
from pathlib import Path
from typing import Annotated, Any

import numpy as np
from numpy.typing import NDArray
from pydantic import AfterValidator, Field, ValidationError, model_validator

from dimos.protocol.service.spec import BaseConfig

SCHEMA = "dimos.metal_arm_policy.manifest/1"

#: Canonical joint order of the trained controller: left arm then right arm,
#: each shoulder-to-wrist. The coordinator addresses the same joints as
#: ``<robot>/<joint>``; the suffix must match this order.
JOINT_ORDER: tuple[str, ...] = (
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
)

#: Per-side TCP site order used by the trained observation and goal vectors.
SIDES: tuple[str, ...] = ("left", "right")

#: Training-plant joint limits (station.xml), in ``JOINT_ORDER``. The repo URDF
#: widens joint3's lower stop to -0.06 rad so a resting arm can be held; the
#: policy was trained against the narrower vendor range, so the action decoder
#: keeps the training values and the runtime rejects limbs outside them.
TRAIN_LOWER: tuple[float, ...] = (-2.79, -3.14, 0.0, -2.145, -1.482, -2.529) * 2
TRAIN_UPPER: tuple[float, ...] = (2.79, 0.0, 3.14, 1.413, 1.482, 2.529) * 2

#: Rest pose of the training episodes (both arms folded).
HOME: tuple[float, ...] = (0.0, -1.9, 1.6, -1.25, 0.0, 0.0) * 2

ACTION_SCALE = 0.6
#: Multiplicative low-pass on the issued target, applied before the slew clamp.
LOW_PASS_ALPHA = 0.05
#: Largest issued-target change per policy tick, in radians.
SLEW_LIMIT = 0.06
#: Margin inside the training joint limits used when clipping the desired pose.
CLAMP_MARGIN = 0.03
HISTORY_FRAMES = 6
FRAME_DIM = 96
POLICY_HZ = 20.0
PHYSICS_DT = 0.005
CONTROL_SUBSTEPS = 10
#: Velocity scale of the ``dq`` slot in the observation frame.
DQ_SCALE = 5.0
#: Normalizer epsilon of rsl_rl's ``EmpiricalNormalization`` (``(x-mean)/(std+eps)``).
NORMALIZER_EPS = 1e-2

#: Observation frame layout: ``(name, width)`` in concatenation order.
FRAME_FIELDS: tuple[tuple[str, int], ...] = (
    ("q_minus_home", 12),
    ("dq_scaled", 12),
    ("issued_minus_home", 12),
    ("previous_action", 12),
    ("tcp_position", 6),
    ("tcp_rotation", 18),
    ("goal_minus_tcp", 24),
)

#: Decode order is part of the trained contract; it is serialized verbatim.
DECODE_ORDER: tuple[str, ...] = ("action_clip", "desired_clip", "low_pass", "slew_clip")


class PolicyContractError(ValueError):
    """Raised when a manifest, artifact or observation violates the contract."""


class _ContractModel(BaseConfig):
    model_config = {"extra": "forbid", "frozen": True, "arbitrary_types_allowed": True}


def _finite(values: tuple[float, ...]) -> tuple[float, ...]:
    if not all(np.isfinite(value) for value in values):
        raise ValueError("contains a non-finite value")
    return values


Vec3 = Annotated[tuple[float, float, float], AfterValidator(_finite)]
Vec12 = Annotated[tuple[float, ...], Field(min_length=12, max_length=12), AfterValidator(_finite)]


def _message(exc: ValidationError) -> str:
    """Flatten a pydantic error into the one-line contract error style."""
    errors = exc.errors()
    loc = ".".join(str(part) for part in errors[0].get("loc", ()))
    return f"{loc}: {errors[0]['msg']}" if loc else errors[0]["msg"]


class FrameField(_ContractModel):
    """One named, fixed-width slot of the 96-value observation frame."""

    name: str
    width: int


class CheckpointSpec(_ContractModel):
    """Lineage of the trained checkpoint the exported graph came from."""

    sha256: str
    iteration: int
    source_sha256: str
    recipe_variant: str


class ModelSpec(_ContractModel):
    """Exported ONNX graph and its learned normalizer."""

    onnx_sha256: str
    normalizer_sha256: str
    input: str = "obs"
    output: str = "action"
    obs_dim: int = HISTORY_FRAMES * FRAME_DIM
    action_dim: int = 12


class ObservationSpec(_ContractModel):
    """Observation history and the exact trained frame layout."""

    history_frames: int = HISTORY_FRAMES
    frame_dim: int = FRAME_DIM
    dq_scale: float = DQ_SCALE
    joint_order: tuple[str, ...] = JOINT_ORDER
    frame_fields: tuple[FrameField, ...] = tuple(
        FrameField(name=n, width=w) for n, w in FRAME_FIELDS
    )


class ActionSpec(_ContractModel):
    """Action-to-command decode contract, in execution order."""

    home: Vec12 = HOME
    lower: Vec12 = TRAIN_LOWER
    upper: Vec12 = TRAIN_UPPER
    action_scale: float = ACTION_SCALE
    clamp_margin: float = CLAMP_MARGIN
    low_pass_alpha: float = LOW_PASS_ALPHA
    slew_limit: float = SLEW_LIMIT
    decode_order: tuple[str, ...] = DECODE_ORDER


class TimingSpec(_ContractModel):
    """Policy rate and the training plant's stepping."""

    policy_hz: float = POLICY_HZ
    physics_dt: float = PHYSICS_DT
    control_substeps: int = CONTROL_SUBSTEPS


class NormalizerSpec(_ContractModel):
    """rsl_rl empirical normalizer epsilon."""

    eps: float = NORMALIZER_EPS


class StationGeometry(_ContractModel):
    """Mount and TCP geometry of the trained bimanual station.

    The training plant mounts each arm on the station base at
    ``(0, -0.345, 0.008)`` (left) and ``(0, 0.345, 0.008)`` (right) with no
    rotation, and measures the TCP as a site 60 mm out along the gripper-base
    frame's local +Z. Both differ from this repo's example bench layout
    (``±0.3`` with mirrored sides), so they are carried explicitly and
    verified against the recorded station poses.
    """

    frame: str = "station"
    left_mount_xyz: Vec3 = (0.0, -0.345, 0.008)
    right_mount_xyz: Vec3 = (0.0, 0.345, 0.008)
    left_mount_rpy: Vec3 = (0.0, 0.0, 0.0)
    right_mount_rpy: Vec3 = (0.0, 0.0, 0.0)
    tcp_offset_xyz: Vec3 = (0.0, 0.0, 0.06)

    def mount(self, side: str) -> tuple[float, float, float]:
        return self.left_mount_xyz if side == "left" else self.right_mount_xyz


class PolicyManifest(_ContractModel):
    """Validated manifest document for one exported policy artifact.

    The nested shape is the versioned on-disk schema itself, so serialization
    and validation need no hand-written bidirectional mapping.
    """

    schema_: str = Field(default=SCHEMA, alias="schema")
    name: str
    checkpoint: CheckpointSpec
    model: ModelSpec
    observation: ObservationSpec = ObservationSpec()
    action: ActionSpec = ActionSpec()
    timing: TimingSpec = TimingSpec()
    normalizer: NormalizerSpec = NormalizerSpec()
    station: StationGeometry = StationGeometry()
    metadata: dict[str, Any] = Field(default_factory=dict)

    def __init__(self, **data: Any) -> None:
        try:
            super().__init__(**data)
        except ValidationError as exc:
            raise PolicyContractError(_message(exc)) from exc

    @model_validator(mode="after")
    def _validate_trained_layout(self) -> PolicyManifest:
        observation, action, model = self.observation, self.action, self.model
        if observation.joint_order != JOINT_ORDER:
            raise ValueError("manifest joint order differs from the trained order")
        fields = tuple((field.name, field.width) for field in observation.frame_fields)
        if fields != FRAME_FIELDS:
            raise ValueError("manifest observation layout differs from the trained frame layout")
        if model.action_dim != len(observation.joint_order):
            raise ValueError(
                f"action_dim {model.action_dim} != {len(observation.joint_order)} joint names"
            )
        if any(lo >= hi for lo, hi in zip(action.lower, action.upper, strict=True)):
            raise ValueError("joint lower limit must be below the upper limit")
        if any(
            not lo <= home <= hi
            for home, lo, hi in zip(action.home, action.lower, action.upper, strict=True)
        ):
            raise ValueError("home pose must lie inside the joint limits")
        if observation.history_frames < 1 or observation.frame_dim < 1:
            raise ValueError("history_frames and frame_dim must be positive")
        if sum(width for _, width in fields) != observation.frame_dim:
            raise ValueError("observation frame fields do not sum to frame_dim")
        if model.obs_dim != observation.history_frames * observation.frame_dim:
            raise ValueError("obs_dim must equal history_frames * frame_dim")
        if not 0.0 < action.low_pass_alpha <= 1.0:
            raise ValueError("low_pass_alpha must be in (0, 1]")
        if action.slew_limit <= 0.0 or action.action_scale <= 0.0:
            raise ValueError("slew_limit and action_scale must be positive")
        if action.clamp_margin < 0.0:
            raise ValueError("clamp_margin must not be negative")
        if self.timing.policy_hz <= 0.0:
            raise ValueError("policy_hz must be positive")
        return self

    @property
    def tick_period_s(self) -> float:
        return 1.0 / self.timing.policy_hz

    def to_json(self) -> dict[str, Any]:
        """The versioned manifest document."""
        return self.model_dump(by_alias=True)

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> PolicyManifest:
        if payload.get("schema") != SCHEMA:
            raise PolicyContractError(
                f"unsupported manifest schema {payload.get('schema')!r}, expected {SCHEMA!r}"
            )
        allowed = {field.alias or name for name, field in cls.model_fields.items()}
        unknown = set(payload) - allowed
        if unknown:
            raise PolicyContractError(f"manifest has unknown keys: {sorted(unknown)}")
        try:
            return cls.model_validate(payload)
        except ValidationError as exc:
            raise PolicyContractError(_message(exc)) from exc

    def dump(self, path: str | Path) -> Path:
        out = Path(path)
        out.write_text(json.dumps(self.to_json(), indent=2, sort_keys=False) + "\n")
        return out

    @classmethod
    def load(cls, path: str | Path) -> PolicyManifest:
        payload = json.loads(Path(path).read_text())
        if not isinstance(payload, dict):
            raise PolicyContractError("manifest root must be a JSON object")
        return cls.from_json(payload)

    def verify_artifacts(self, model_dir: str | Path) -> tuple[Path, Path]:
        """Check the artifact hashes and return ``(onnx_path, normalizer_path)``."""
        root = Path(model_dir)
        onnx_path = root / self.metadata.get("onnx_file", "metal_arm_policy.onnx")
        normalizer_path = root / self.metadata.get(
            "normalizer_file", "metal_arm_policy.normalizer.npz"
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


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_normalizer(path: str | Path) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    """Load the learned observation mean/std, rejecting non-finite or wrong shapes."""
    with np.load(path, allow_pickle=False) as data:
        missing = {"mean", "std"} - set(data.files)
        if missing:
            raise PolicyContractError(f"normalizer is missing arrays: {sorted(missing)}")
        mean = np.asarray(data["mean"], dtype=np.float32).reshape(1, -1)
        std = np.asarray(data["std"], dtype=np.float32).reshape(1, -1)
    if mean.shape != std.shape:
        raise PolicyContractError("normalizer mean and std shapes differ")
    if mean.shape[1] != HISTORY_FRAMES * FRAME_DIM:
        raise PolicyContractError(
            f"normalizer width {mean.shape[1]} != {HISTORY_FRAMES * FRAME_DIM}"
        )
    if not (np.isfinite(mean).all() and np.isfinite(std).all()):
        raise PolicyContractError("normalizer contains non-finite values")
    if (std <= 0.0).any():
        raise PolicyContractError("normalizer std must be positive")
    return mean, std


def check_observation(obs: NDArray[np.float32], manifest: PolicyManifest) -> None:
    """Fail closed on malformed, wrongly shaped or non-finite observations."""
    if obs.shape != (1, manifest.model.obs_dim):
        raise PolicyContractError(f"observation shape {obs.shape} != (1, {manifest.model.obs_dim})")
    if not np.isfinite(obs).all():
        raise PolicyContractError("observation contains non-finite values")


def check_feedback(
    q: NDArray[np.float32], dq: NDArray[np.float32], manifest: PolicyManifest
) -> None:
    """Reject incomplete or out-of-range joint feedback before it reaches the actor."""
    action, observation = manifest.action, manifest.observation
    action_dim = manifest.model.action_dim
    for values, name in ((q, "joint positions"), (dq, "joint velocities")):
        if values.shape != (action_dim,):
            raise PolicyContractError(f"{name} shape {values.shape} != ({action_dim},)")
        if not np.isfinite(values).all():
            raise PolicyContractError(f"{name} contain non-finite values")
    slack = 1e-3
    outside = [
        observation.joint_order[i]
        for i in range(action_dim)
        if not action.lower[i] - slack <= q[i] <= action.upper[i] + slack
    ]
    if outside:
        raise PolicyContractError(f"measured joints outside the trained limits: {outside}")
