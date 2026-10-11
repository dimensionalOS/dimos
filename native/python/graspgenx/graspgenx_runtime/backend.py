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

"""First-use GraspGenX runtime with top-level optional dependency imports."""

from __future__ import annotations

import os
from pathlib import Path

from huggingface_hub import snapshot_download
import numpy as np

from dimos.manipulation.grasping.grasp_gen_x.module import (
    GRASPGENX_MODEL_REPO,
    GRASPGENX_MODEL_REVISION,
    GRASPGENX_MODEL_VERSION,
    GraspGenXConfig,
    gripper_body_points,
)

_snapshot_root = Path(
    snapshot_download(
        repo_id=GRASPGENX_MODEL_REPO,
        revision=GRASPGENX_MODEL_REVISION,
        allow_patterns=[
            f"{GRASPGENX_MODEL_VERSION}/gen/*",
            f"{GRASPGENX_MODEL_VERSION}/dis/*",
        ],
    )
).resolve()
_checkpoint_root = _snapshot_root / GRASPGENX_MODEL_VERSION
_gen_dir = _checkpoint_root / "gen"
_dis_dir = _checkpoint_root / "dis"
if not _gen_dir.is_dir() or not _dis_dir.is_dir():
    raise FileNotFoundError(
        f"GraspGenX checkpoint must contain release/gen and release/dis: {_snapshot_root}"
    )

# Upstream's package initializer otherwise performs Git clones while importing. The
# sweep-volume runtime does not consume named gripper assets, so the existing snapshot
# directory also suppresses that unused asset clone.
os.environ["GRASPGENX_CHECKPOINT_DIR"] = str(_snapshot_root)
os.environ["GRASPGENX_GRIPPER_CFG_DIR"] = str(_snapshot_root)

from graspgenx.grasp_server import (
    SWEEP_VOLUME_ONLY_BACKBONES,
    GraspGenXSampler,
)
from graspgenx.utils.checkpoint_io import load_model_cfg
from graspgenx.x_grippers import make_sweep_volume_gripper_info

from .scene_filter import collision_free

_GRIPPER_TYPES = {
    "parallel_2f": 0,
    "revolute_2f": 1,
    "revolute_3f": 2,
}


class GraspGenXRuntime:
    """Loaded GraspGenX sampler and exact tensor conversion boundary."""

    def __init__(self, config: GraspGenXConfig) -> None:
        model_config = load_model_cfg(str(_gen_dir), str(_dis_dir), gen_pth=None, dis_pth=None)
        for component in ("diffusion", "discriminator"):
            backbone = getattr(model_config, component).gripper_backbone
            if backbone not in SWEEP_VOLUME_ONLY_BACKBONES:
                raise ValueError(
                    f"GraspGenX {component}.gripper_backbone={backbone!r} "
                    "requires an asset-backed gripper"
                )
        gripper_info = make_sweep_volume_gripper_info(
            extents_open=config.gripper.extents_open,
            offset_open=config.gripper.offset_open,
            extents_mid=config.gripper.extents_half_open,
            offset_mid=config.gripper.offset_half_open,
            gripper_type=_GRIPPER_TYPES[config.gripper.family],
            fingertip_depth=config.gripper.fingertip_depth,
        )
        self._sampler = GraspGenXSampler(model_config, gripper_info=gripper_info)
        self._gripper_points = gripper_body_points(config.gripper_body)

    def infer(self, points: np.ndarray, num_samples: int) -> tuple[np.ndarray, np.ndarray]:
        """Sample grasps for an object and return every one, as CPU NumPy arrays.

        Args:
            points: (M, 3) object points, metres.
            num_samples: How many grasps to sample. GraspGenX samples again while
                it has fewer than 40, so a small value can return more.

        Returns (N, 4, 4) grasp poses in GraspGenX's grasp frame and (N,) scores.
        """
        # GraspGenX keeps only its best 100 unless told to keep a set number.
        poses, scores = GraspGenXSampler.run_inference(
            points, self._sampler, num_grasps=num_samples, topk_num_grasps=num_samples
        )
        return (
            poses.detach().cpu().numpy(),
            scores.detach().cpu().numpy(),
        )

    def collision_free(
        self, scene: np.ndarray, poses: np.ndarray, collision_threshold: float
    ) -> np.ndarray:
        """Which grasps keep the open gripper clear of the scene; see ``scene_filter``."""
        return collision_free(scene, poses, self._gripper_points, collision_threshold)
