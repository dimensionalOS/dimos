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

"""First-use in-process GraspGenX runtime."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from dimos.manipulation.grasping.grasp_gen_x import (
    GRASPGENX_MODEL_REPO,
    GRASPGENX_MODEL_REVISION,
    GRASPGENX_MODEL_VERSION,
    GraspGenXConfig,
)

_GRIPPER_TYPES = {
    "parallel_2f": 0,
    "revolute_2f": 1,
    "revolute_3f": 2,
}


class GraspGenXRuntime:
    """Loaded GraspGenX sampler and exact tensor conversion boundary."""

    def __init__(self, config: GraspGenXConfig) -> None:
        from huggingface_hub import snapshot_download

        snapshot_root = Path(
            snapshot_download(
                repo_id=GRASPGENX_MODEL_REPO,
                revision=GRASPGENX_MODEL_REVISION,
                allow_patterns=[
                    f"{GRASPGENX_MODEL_VERSION}/gen/*",
                    f"{GRASPGENX_MODEL_VERSION}/dis/*",
                ],
            )
        ).resolve()
        checkpoint_root = snapshot_root / GRASPGENX_MODEL_VERSION
        gen_dir = checkpoint_root / "gen"
        dis_dir = checkpoint_root / "dis"
        if not gen_dir.is_dir() or not dis_dir.is_dir():
            raise FileNotFoundError(
                f"GraspGenX checkpoint must contain release/gen and release/dis: {snapshot_root}"
            )

        # Sweep-volume grippers do not use named gripper assets, so avoid the
        # upstream package's on-import Git clone.
        os.environ["GRASPGENX_CHECKPOINT_DIR"] = str(snapshot_root)
        os.environ["GRASPGENX_GRIPPER_CFG_DIR"] = str(snapshot_root)
        from graspgenx.grasp_server import SWEEP_VOLUME_ONLY_BACKBONES, GraspGenXSampler
        from graspgenx.utils.checkpoint_io import load_model_cfg
        from graspgenx.x_grippers import make_sweep_volume_gripper_info

        model_config = load_model_cfg(gen_dir, dis_dir, gen_pth=None, dis_pth=None)
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
        self._sampler_type = GraspGenXSampler
        self._sampler = GraspGenXSampler(model_config, gripper_info=gripper_info)

    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Run inference and copy the known torch tensors to CPU NumPy arrays."""
        poses, scores = self._sampler_type.run_inference(points, self._sampler)
        return poses.detach().cpu().numpy(), scores.detach().cpu().numpy()

    def stop(self) -> None:
        """Release the sampler when the module stops."""
        del self._sampler
