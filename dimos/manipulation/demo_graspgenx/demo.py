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

"""One-shot GraspGenX inference and static image generation."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import (
    GRASPGENX_MODEL_REPO,
    GRASPGENX_MODEL_REVISION,
    GraspGenXConfig,
    GraspGenXModule,
    SweepVolumeGripperConfig,
)
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

from .fixture import load_demo_clouds
from .render import SweepVolumeLike, render_grasp_image


@dataclass(frozen=True)
class DemoResult:
    image_path: Path
    scene_points: int
    object_points: int
    candidate_count: int
    best_score: float
    frame: str


def deployment_config() -> GraspGenXConfig:
    """Build the fixed sweep-volume deployment without downloading the checkpoint."""
    return GraspGenXConfig(
        gripper=SweepVolumeGripperConfig(
            extents_open=(0.08, 0.045, 0.04),
            offset_open=(0.0, 0.0, 0.135),
            extents_half_open=(0.04, 0.045, 0.035),
            offset_half_open=(0.0, 0.0, 0.118),
            fingertip_depth=0.15,
            family="revolute_3f",
        ),
        max_candidates=100,
    )


def _cuda_context() -> dict[str, object]:
    available = bool(torch.cuda.is_available())
    return {"available": available, "device": torch.cuda.get_device_name(0) if available else "cpu"}


def _validate(result: GraspCandidateArray, object_cloud: PointCloud2) -> None:
    if not result.candidates:
        raise ValueError("proposer returned no grasp candidates")
    if result.header.frame_id != "world":
        raise ValueError("proposer returned a result outside world frame")
    if (
        result.header.frame_id != object_cloud.frame_id
        or result.header.timestamp != object_cloud.ts
    ):
        raise ValueError("proposer changed the object point-cloud frame or timestamp")

    scores = np.asarray([candidate.score for candidate in result.candidates], dtype=float)
    if not np.all(np.isfinite(scores)) or np.any(scores[:-1] < scores[1:]):
        raise ValueError("grasp scores must be finite and descending")
    for candidate in result.candidates:
        p, q = candidate.pose.position, candidate.pose.orientation
        values = np.asarray([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("proposer returned a non-finite TCP pose")


def run_demo(
    proposer: GraspGenSpec,
    output_path: Path,
    *,
    gripper: SweepVolumeLike,
    renderer: Callable[
        [Path, PointCloud2, PointCloud2, GraspCandidateArray, SweepVolumeLike], Path
    ] = render_grasp_image,
) -> DemoResult:
    """Load one scene, run inference once, render one PNG, and return."""
    if not callable(getattr(proposer, "propose_grasps", None)):
        raise TypeError("proposer must implement GraspGenSpec.propose_grasps")

    scene, object_cloud = load_demo_clouds()
    result = proposer.propose_grasps(object_cloud)
    _validate(result, object_cloud)
    image_path = renderer(output_path, scene, object_cloud, result, gripper)
    best_score = float(result.candidates[0].score)
    print(
        "graspgenx-ycb-demo "
        f"scene_points={len(scene)} object_points={len(object_cloud)} "
        f"candidates={len(result)} best_score={best_score:.6f} "
        f"image={image_path}",
        flush=True,
    )
    return DemoResult(
        image_path=image_path,
        scene_points=len(scene),
        object_points=len(object_cloud),
        candidate_count=len(result),
        best_score=best_score,
        frame=result.header.frame_id,
    )


def run_contributor_demo(
    *,
    output_path: Path,
    config: GraspGenXConfig | None = None,
) -> DemoResult:
    """Run the real adapter once and save its annotated point-cloud image."""
    active_config = config if config is not None else deployment_config()
    cuda = _cuda_context()
    print(
        "graspgenx-ycb-demo "
        f"checkpoint=hf://{GRASPGENX_MODEL_REPO}@{GRASPGENX_MODEL_REVISION} "
        f"cuda={cuda['available']} device={cuda['device']}",
        flush=True,
    )

    module_args = active_config.model_dump(
        exclude={"rpc_transport", "tf_transport", "g"},
    )
    adapter = GraspGenXModule(**module_args)
    try:
        adapter.start()
        return run_demo(adapter, output_path, gripper=active_config.gripper)
    finally:
        adapter.stop()
