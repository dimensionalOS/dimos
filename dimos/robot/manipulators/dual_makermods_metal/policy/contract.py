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
observation layout, action decode order and station geometry. The runtime
refuses to start when the manifest does not match the artifact hashes or when a
field it does not understand appears, instead of silently falling back.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

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


class PolicyContractError(ValueError):
    """Raised when a manifest, artifact or observation violates the contract."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _as_float_tuple(values: Sequence[float], width: int, name: str) -> tuple[float, ...]:
    out = tuple(float(v) for v in values)
    if len(out) != width:
        raise PolicyContractError(f"{name} must have {width} entries, got {len(out)}")
    if not all(np.isfinite(out)):
        raise PolicyContractError(f"{name} contains a non-finite value")
    return out


@dataclass(frozen=True)
class StationGeometry:
    """Mount and TCP geometry of the trained bimanual station.

    The training plant mounts each arm on the station base at
    ``(0, -0.345, 0.008)`` (left) and ``(0, 0.345, 0.008)`` (right) with no
    rotation, and measures the TCP as a site 60 mm out along the gripper-base
    frame's local +Z. Both differ from this repo's example bench layout
    (``±0.3`` with mirrored sides), so they are carried explicitly and
    verified against the recorded station poses.
    """

    left_mount_xyz: tuple[float, float, float] = (0.0, -0.345, 0.008)
    right_mount_xyz: tuple[float, float, float] = (0.0, 0.345, 0.008)
    left_mount_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    right_mount_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tcp_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.06)
    frame: str = "station"

    def mount(self, side: str) -> tuple[float, float, float]:
        return self.left_mount_xyz if side == "left" else self.right_mount_xyz

    def to_json(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "left_mount_xyz": list(self.left_mount_xyz),
            "right_mount_xyz": list(self.right_mount_xyz),
            "left_mount_rpy": list(self.left_mount_rpy),
            "right_mount_rpy": list(self.right_mount_rpy),
            "tcp_offset_xyz": list(self.tcp_offset_xyz),
        }

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> StationGeometry:
        return cls(
            left_mount_xyz=_as_float_tuple(payload["left_mount_xyz"], 3, "left_mount_xyz"),
            right_mount_xyz=_as_float_tuple(payload["right_mount_xyz"], 3, "right_mount_xyz"),
            left_mount_rpy=_as_float_tuple(payload["left_mount_rpy"], 3, "left_mount_rpy"),
            right_mount_rpy=_as_float_tuple(payload["right_mount_rpy"], 3, "right_mount_rpy"),
            tcp_offset_xyz=_as_float_tuple(payload["tcp_offset_xyz"], 3, "tcp_offset_xyz"),
            frame=str(payload.get("frame", "station")),
        )


@dataclass(frozen=True)
class PolicyManifest:
    """Validated description of one exported policy artifact."""

    name: str
    checkpoint_sha256: str
    checkpoint_iteration: int
    checkpoint_source_sha256: str
    recipe_variant: str
    onnx_sha256: str
    normalizer_sha256: str
    obs_dim: int = HISTORY_FRAMES * FRAME_DIM
    action_dim: int = 12
    home: tuple[float, ...] = HOME
    lower: tuple[float, ...] = TRAIN_LOWER
    upper: tuple[float, ...] = TRAIN_UPPER
    action_scale: float = ACTION_SCALE
    clamp_margin: float = CLAMP_MARGIN
    low_pass_alpha: float = LOW_PASS_ALPHA
    slew_limit: float = SLEW_LIMIT
    history_frames: int = HISTORY_FRAMES
    frame_dim: int = FRAME_DIM
    policy_hz: float = POLICY_HZ
    physics_dt: float = PHYSICS_DT
    control_substeps: int = CONTROL_SUBSTEPS
    dq_scale: float = DQ_SCALE
    normalizer_eps: float = NORMALIZER_EPS
    joint_order: tuple[str, ...] = JOINT_ORDER
    frame_fields: tuple[tuple[str, int], ...] = FRAME_FIELDS
    station: StationGeometry = field(default_factory=StationGeometry)
    onnx_input: str = "obs"
    onnx_output: str = "action"
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.action_dim != len(self.joint_order):
            raise PolicyContractError(
                f"action_dim {self.action_dim} != {len(self.joint_order)} joint names"
            )
        for values, name in ((self.home, "home"), (self.lower, "lower"), (self.upper, "upper")):
            _as_float_tuple(values, self.action_dim, name)
        if any(lo >= hi for lo, hi in zip(self.lower, self.upper, strict=True)):
            raise PolicyContractError("joint lower limit must be below the upper limit")
        if any(
            not lo <= h <= hi for h, lo, hi in zip(self.home, self.lower, self.upper, strict=True)
        ):
            raise PolicyContractError("home pose must lie inside the joint limits")
        if self.history_frames < 1 or self.frame_dim < 1:
            raise PolicyContractError("history_frames and frame_dim must be positive")
        if sum(width for _, width in self.frame_fields) != self.frame_dim:
            raise PolicyContractError("observation frame fields do not sum to frame_dim")
        if self.obs_dim != self.history_frames * self.frame_dim:
            raise PolicyContractError("obs_dim must equal history_frames * frame_dim")
        if not 0.0 < self.low_pass_alpha <= 1.0:
            raise PolicyContractError("low_pass_alpha must be in (0, 1]")
        if self.slew_limit <= 0.0 or self.action_scale <= 0.0:
            raise PolicyContractError("slew_limit and action_scale must be positive")
        if self.clamp_margin < 0.0:
            raise PolicyContractError("clamp_margin must not be negative")
        if self.policy_hz <= 0.0:
            raise PolicyContractError("policy_hz must be positive")

    @property
    def tick_period_s(self) -> float:
        return 1.0 / self.policy_hz

    def to_json(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "name": self.name,
            "checkpoint": {
                "sha256": self.checkpoint_sha256,
                "iteration": self.checkpoint_iteration,
                "source_sha256": self.checkpoint_source_sha256,
                "recipe_variant": self.recipe_variant,
            },
            "model": {
                "onnx_sha256": self.onnx_sha256,
                "normalizer_sha256": self.normalizer_sha256,
                "input": self.onnx_input,
                "output": self.onnx_output,
                "obs_dim": self.obs_dim,
                "action_dim": self.action_dim,
            },
            "observation": {
                "history_frames": self.history_frames,
                "frame_dim": self.frame_dim,
                "dq_scale": self.dq_scale,
                "joint_order": list(self.joint_order),
                "frame_fields": [{"name": n, "width": w} for n, w in self.frame_fields],
            },
            "action": {
                "home": list(self.home),
                "lower": list(self.lower),
                "upper": list(self.upper),
                "action_scale": self.action_scale,
                "clamp_margin": self.clamp_margin,
                "low_pass_alpha": self.low_pass_alpha,
                "slew_limit": self.slew_limit,
                "decode_order": ["action_clip", "desired_clip", "low_pass", "slew_clip"],
            },
            "timing": {
                "policy_hz": self.policy_hz,
                "physics_dt": self.physics_dt,
                "control_substeps": self.control_substeps,
            },
            "normalizer": {"eps": self.normalizer_eps},
            "station": self.station.to_json(),
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> PolicyManifest:
        if payload.get("schema") != SCHEMA:
            raise PolicyContractError(
                f"unsupported manifest schema {payload.get('schema')!r}, expected {SCHEMA!r}"
            )
        known = {
            "schema",
            "name",
            "checkpoint",
            "model",
            "observation",
            "action",
            "timing",
            "normalizer",
            "station",
            "metadata",
        }
        unknown = set(payload) - known
        if unknown:
            raise PolicyContractError(f"manifest has unknown keys: {sorted(unknown)}")

        checkpoint = payload["checkpoint"]
        model = payload["model"]
        observation = payload["observation"]
        action = payload["action"]
        timing = payload["timing"]
        fields_payload = observation["frame_fields"]
        fields = tuple((str(f["name"]), int(f["width"])) for f in fields_payload)
        if fields != FRAME_FIELDS:
            raise PolicyContractError(
                "manifest observation layout differs from the trained frame layout"
            )
        joint_order = tuple(str(j) for j in observation["joint_order"])
        if joint_order != JOINT_ORDER:
            raise PolicyContractError("manifest joint order differs from the trained order")
        return cls(
            name=str(payload["name"]),
            checkpoint_sha256=str(checkpoint["sha256"]),
            checkpoint_iteration=int(checkpoint["iteration"]),
            checkpoint_source_sha256=str(checkpoint["source_sha256"]),
            recipe_variant=str(checkpoint["recipe_variant"]),
            onnx_sha256=str(model["onnx_sha256"]),
            normalizer_sha256=str(model["normalizer_sha256"]),
            obs_dim=int(model.get("obs_dim", cls.obs_dim)),
            action_dim=int(model.get("action_dim", cls.action_dim)),
            onnx_input=str(model.get("input", "obs")),
            onnx_output=str(model.get("output", "action")),
            home=_as_float_tuple(action["home"], 12, "home"),
            lower=_as_float_tuple(action["lower"], 12, "lower"),
            upper=_as_float_tuple(action["upper"], 12, "upper"),
            action_scale=float(action["action_scale"]),
            clamp_margin=float(action["clamp_margin"]),
            low_pass_alpha=float(action["low_pass_alpha"]),
            slew_limit=float(action["slew_limit"]),
            history_frames=int(observation["history_frames"]),
            frame_dim=int(observation["frame_dim"]),
            dq_scale=float(observation["dq_scale"]),
            policy_hz=float(timing["policy_hz"]),
            physics_dt=float(timing["physics_dt"]),
            control_substeps=int(timing["control_substeps"]),
            normalizer_eps=float(payload["normalizer"]["eps"]),
            joint_order=joint_order,
            frame_fields=fields,
            station=StationGeometry.from_json(payload["station"]),
            metadata=dict(payload.get("metadata") or {}),
        )

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
            (onnx_path, self.onnx_sha256, "ONNX actor"),
            (normalizer_path, self.normalizer_sha256, "observation normalizer"),
        ):
            if not path.is_file():
                raise PolicyContractError(f"{label} not found: {path}")
            actual = sha256_file(path)
            if actual != expected:
                raise PolicyContractError(
                    f"{label} hash mismatch at {path}: {actual} != manifest {expected}"
                )
        return onnx_path, normalizer_path


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
    if obs.shape != (1, manifest.obs_dim):
        raise PolicyContractError(f"observation shape {obs.shape} != (1, {manifest.obs_dim})")
    if not np.isfinite(obs).all():
        raise PolicyContractError("observation contains non-finite values")


def check_feedback(
    q: NDArray[np.float32], dq: NDArray[np.float32], manifest: PolicyManifest
) -> None:
    """Reject incomplete or out-of-range joint feedback before it reaches the actor."""
    for values, name in ((q, "joint positions"), (dq, "joint velocities")):
        if values.shape != (manifest.action_dim,):
            raise PolicyContractError(f"{name} shape {values.shape} != ({manifest.action_dim},)")
        if not np.isfinite(values).all():
            raise PolicyContractError(f"{name} contain non-finite values")
    slack = 1e-3
    outside = [
        manifest.joint_order[i]
        for i in range(manifest.action_dim)
        if not manifest.lower[i] - slack <= q[i] <= manifest.upper[i] + slack
    ]
    if outside:
        raise PolicyContractError(f"measured joints outside the trained limits: {outside}")
