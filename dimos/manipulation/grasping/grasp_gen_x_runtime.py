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

from dimos.manipulation.grasping.grasp_gen_x import (
    GRASPGENX_MODEL_REPO,
    GRASPGENX_MODEL_REVISION,
    GRASPGENX_MODEL_VERSION,
    GraspGenXConfig,
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

_GRIPPER_TYPES = {
    "parallel_2f": 0,
    "revolute_2f": 1,
    "revolute_3f": 2,
}


class GraspGenXRuntime:
    """Loaded GraspGenX sampler and exact tensor conversion boundary."""

    def __init__(self, config: GraspGenXConfig) -> None:
        model_config = load_model_cfg(_gen_dir, _dis_dir, gen_pth=None, dis_pth=None)
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

    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Run inference and copy the known torch tensors to CPU NumPy arrays."""
        poses, scores = GraspGenXSampler.run_inference(points, self._sampler)
        return (
            poses.detach().cpu().numpy(),
            scores.detach().cpu().numpy(),
        )
