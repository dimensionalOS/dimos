# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""In-process, deployment-fixed GraspGenX proposal adapter."""

from __future__ import annotations

from collections.abc import Callable
import importlib
import math
import os
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import Field, field_validator, model_validator

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.protocol.service.spec import BaseConfig


class SweepVolumeGripperConfig(BaseConfig):
    """Finite, axis-aligned open and half-open sweep-volume description."""

    extents_open: tuple[float, float, float]
    offset_open: tuple[float, float, float]
    extents_half_open: tuple[float, float, float]
    offset_half_open: tuple[float, float, float]
    fingertip_depth: float
    family: str = "parallel_2f"

    @field_validator("extents_open", "offset_open", "extents_half_open", "offset_half_open")
    @classmethod
    def finite_vector(cls, value: tuple[float, ...]) -> tuple[float, float, float]:
        if len(value) != 3 or not all(math.isfinite(x) for x in value):
            raise ValueError("sweep-volume fields must be finite 3-vectors")
        return value  # type: ignore[return-value]

    @field_validator("extents_open", "extents_half_open")
    @classmethod
    def positive_extents(cls, value: tuple[float, float, float]) -> tuple[float, float, float]:
        if not all(x > 0 for x in value):
            raise ValueError("sweep-volume extents must be strictly positive")
        return value

    @field_validator("fingertip_depth")
    @classmethod
    def positive_depth(cls, value: float) -> float:
        if not math.isfinite(value) or value <= 0:
            raise ValueError("fingertip_depth must be positive and finite")
        return value

    @field_validator("extents_open", "extents_half_open")
    @classmethod
    def conservative_extent_bounds(
        cls, value: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        if any(x > 0.5 for x in value):
            raise ValueError("sweep-volume extents must be <= 0.5 m")
        return value

    @field_validator("offset_open", "offset_half_open")
    @classmethod
    def conservative_offset_bounds(
        cls, value: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        if any(abs(x) > 0.5 for x in value):
            raise ValueError("sweep-volume offsets must be within +/-0.5 m")
        return value

    @field_validator("fingertip_depth")
    @classmethod
    def conservative_depth_bounds(cls, value: float) -> float:
        if value > 0.5:
            raise ValueError("fingertip_depth must be <= 0.5 m")
        return value

    @field_validator("family")
    @classmethod
    def valid_family(cls, value: str) -> str:
        if value not in {"parallel_2f", "revolute_2f", "revolute_3f"}:
            raise ValueError(f"unsupported sweep-volume gripper family: {value}")
        return value


class GraspGenXConfig(ModuleConfig):
    """GraspGenX deployment settings, serializable by DimOS blueprints."""

    checkpoint_path: str
    gripper: SweepVolumeGripperConfig
    grasp_frame_to_tcp: tuple[tuple[float, float, float, float], ...] = Field(
        default=(
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
    )
    max_candidates: int = 100

    @field_validator("grasp_frame_to_tcp")
    @classmethod
    def finite_transform(
        cls, value: tuple[tuple[float, ...], ...]
    ) -> tuple[tuple[float, ...], ...]:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError("grasp_frame_to_tcp must be a finite 4x4 matrix")
        if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
            raise ValueError("grasp_frame_to_tcp must be homogeneous")
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-6
        ):
            raise ValueError("grasp_frame_to_tcp rotation must be orthonormal with determinant +1")
        return value

    @field_validator("max_candidates")
    @classmethod
    def positive_limit(cls, value: int) -> int:
        if isinstance(value, bool) or value <= 0:
            raise ValueError("max_candidates must be a positive integer")
        return value

    @model_validator(mode="after")
    def checkpoint_root(self) -> GraspGenXConfig:
        if not self.checkpoint_path:
            raise ValueError("checkpoint_path must not be empty")
        return self


class GraspGenXError(RuntimeError):
    """Base error for model loading and inference failures."""


BackendFactory = Callable[[GraspGenXConfig], Any]
Inference = Callable[[Any, np.ndarray], tuple[Any, Any]]


def _prepare_offline_environment(config: GraspGenXConfig) -> Path:
    """Point GraspGenX's import-time setup hook at already-local assets.

    Upstream uses ``GRASPGENX_CHECKPOINT_DIR`` and
    ``GRASPGENX_GRIPPER_CFG_DIR`` (not DimOS's convenience variable).  The
    sweep-volume path does not read gripper description files, but upstream
    still requires the directory to exist, so the checkpoint root is used as
    that deployment directory.  This is an intentional invariant: only
    sweep-volume backbones are accepted below and no named gripper asset is
    consumed.
    """
    root = Path(config.checkpoint_path).expanduser().resolve()
    missing = [name for name in ("gen", "dis") if not (root / name).is_dir()]
    if missing:
        raise FileNotFoundError(
            "GraspGenX offline deployment requires an existing checkpoint root "
            f"with {', '.join(missing)}/: {root}. Populate it before starting DimOS; "
            "runtime download/clone is disabled."
        )
    gripper_root = os.environ.get("GRASPGENX_GRIPPER_CFG_DIR", str(root))
    gripper_path = Path(gripper_root).expanduser().resolve()
    if not gripper_path.is_dir():
        raise FileNotFoundError(
            "GraspGenX offline deployment requires an existing directory for "
            f"GRASPGENX_GRIPPER_CFG_DIR: {gripper_path}. The sweep-volume adapter "
            "does not consume named gripper assets, but upstream requires this "
            "directory to exist; create/populate it before starting DimOS."
        )
    os.environ["GRASPGENX_CHECKPOINT_DIR"] = str(root)
    os.environ["GRASPGENX_GRIPPER_CFG_DIR"] = str(gripper_path)
    return root


def _default_factory(config: GraspGenXConfig) -> Any:
    """Load the released checkpoint layout (root/{gen,dis})."""
    root = _prepare_offline_environment(config)
    gen_dir, dis_dir = root / "gen", root / "dis"
    if not root.is_dir() or not gen_dir.is_dir() or not dis_dir.is_dir():
        raise FileNotFoundError(f"GraspGenX checkpoint root must contain gen/ and dis/: {root}")
    try:
        checkpoint_io = importlib.import_module("graspgenx.utils.checkpoint_io")
        server = importlib.import_module("graspgenx.grasp_server")
        x_grippers = importlib.import_module("graspgenx.x_grippers")
        cfg = checkpoint_io.load_model_cfg(gen_dir, dis_dir, gen_pth=None, dis_pth=None)
        allowed_backbones = server.SWEEP_VOLUME_ONLY_BACKBONES
        for component in ("diffusion", "discriminator"):
            backbone = getattr(cfg, component).gripper_backbone
            if backbone not in allowed_backbones:
                raise ValueError(
                    f"GraspGenX {component}.gripper_backbone={backbone!r} "
                    f"requires an asset-backed gripper; only sweep-volume backbones "
                    f"are supported by this adapter: {sorted(allowed_backbones)}"
                )
        gripper_info = x_grippers.make_sweep_volume_gripper_info(
            extents_open=config.gripper.extents_open,
            offset_open=config.gripper.offset_open,
            extents_mid=config.gripper.extents_half_open,
            offset_mid=config.gripper.offset_half_open,
            gripper_type={"parallel_2f": 0, "revolute_2f": 1, "revolute_3f": 2}[
                config.gripper.family
            ],
            fingertip_depth=config.gripper.fingertip_depth,
        )
        return server.GraspGenXSampler(cfg, gripper_info=gripper_info)
    except FileNotFoundError:
        raise
    except ValueError:
        raise
    except Exception as exc:
        raise GraspGenXError("failed to import or initialize GraspGenX") from exc


def _default_inference(sampler: Any, points: np.ndarray) -> tuple[Any, Any]:
    try:
        server = importlib.import_module("graspgenx.grasp_server")
        return server.GraspGenXSampler.run_inference(points, sampler)
    except Exception as exc:
        raise GraspGenXError("GraspGenX inference failed") from exc


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


class GraspGenXModule(Module, GraspGenSpec):
    """Direct adapter whose sampler is constructed once in its worker."""

    dedicated_worker = True
    config: GraspGenXConfig  # type: ignore[assignment]

    def __init__(
        self,
        config: GraspGenXConfig | None = None,
        factory: BackendFactory = _default_factory,
        inference: Inference = _default_inference,
        **kwargs: Any,
    ) -> None:
        config_args = (
            config.model_dump(exclude={"rpc_transport", "tf_transport", "g"}) if config else kwargs
        )
        super().__init__(**config_args)
        self._factory, self._inference = factory, inference
        self._sampler: Any = None

    @rpc
    def start(self) -> None:
        super().start()
        if self._sampler is not None:
            return
        try:
            self._sampler = self._factory(self.config)
        except (ValueError, FileNotFoundError, GraspGenXError):
            raise
        except Exception as exc:
            raise GraspGenXError("GraspGenX model construction failed") from exc

    @rpc
    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        if self._sampler is None:
            raise GraspGenXError("GraspGenX module has not been started")
        if object_pointcloud.ts is None:
            raise ValueError("object pointcloud must have a timestamp")
        if not object_pointcloud.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")
        timestamp = float(object_pointcloud.ts)
        frame_id = str(object_pointcloud.frame_id)
        points = _as_numpy(object_pointcloud.pointcloud_tensor.point["positions"])
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("object pointcloud must contain at least one XYZ point")
        if not np.issubdtype(points.dtype, np.floating) or not np.all(np.isfinite(points)):
            raise ValueError("object pointcloud XYZ values must be finite floats in metres")
        try:
            native_poses, native_scores = self._inference(self._sampler, points.astype(np.float32))
            poses, scores = _as_numpy(native_poses), _as_numpy(native_scores).reshape(-1)
        except (ValueError, TypeError, GraspGenXError):
            raise
        except Exception as exc:
            raise GraspGenXError("GraspGenX inference failed") from exc
        if poses.size == 0 and scores.size == 0:
            return GraspCandidateArray(Header(timestamp, frame_id), [])
        if poses.shape != (len(scores), 4, 4):
            raise ValueError("backend poses must have shape (N, 4, 4)")
        if not np.all(np.isfinite(poses)) or not np.all(np.isfinite(scores)):
            raise ValueError("backend returned non-finite poses or scores")
        if not np.allclose(poses[:, 3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-7):
            raise ValueError("backend poses must be homogeneous")
        rotations = poses[:, :3, :3]
        if not np.allclose(np.einsum("nij,nkj->nik", rotations, rotations), np.eye(3), atol=1e-5):
            raise ValueError("backend poses must have orthonormal rotations")
        if not np.allclose(np.linalg.det(rotations), 1.0, atol=1e-5):
            raise ValueError("backend poses must have proper rotations")
        tcp_poses = poses @ np.asarray(self.config.grasp_frame_to_tcp)
        if not np.all(np.isfinite(tcp_poses)):
            raise ValueError("backend returned non-finite poses")
        order = np.argsort(-scores, kind="stable")[: self.config.max_candidates]
        candidates = [
            GraspCandidate(self._pose_from_matrix(tcp_poses[i]), float(scores[i])) for i in order
        ]
        return GraspCandidateArray(Header(timestamp, frame_id), candidates)

    @staticmethod
    def _pose_from_matrix(matrix: np.ndarray) -> Pose:
        return Pose(
            {
                "position": Vector3(matrix[:3, 3]),
                "orientation": Quaternion.from_rotation_matrix(matrix[:3, :3]),
            }
        )


SweepVolumeGripperModel = SweepVolumeGripperConfig
