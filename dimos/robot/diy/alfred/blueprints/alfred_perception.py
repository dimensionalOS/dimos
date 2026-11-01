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

"""alfred-base plus object perception: ask where something is, get it back in 3D.

    dimos run alfred-perception
    dimos mcp call state
    dimos mcp call localize -a 'objects=can,tape'

``PerceptionLocalize`` (SigLIP + OWLv2 + EdgeTAM, the perception stack's localize) embeds the D455's
colour feed once a second and answers ``localize`` and ``localize_boxes`` with verified 3D
instances, lifted through the aligned depth. They are kept in ``odom``, so a found object stays
where it is when the base drives. McpServer serves that and the base layer's skills on :9990.

It needs CUDA: EdgeTAM refuses a CPU, and on the Orin's CPU SigLIP embeds a frame in ~30 s and OWLv2
detects in ~90 s. A stack on a CPU torch refuses to start instead of coming up with its models
missing.
"""

from __future__ import annotations

from typing import Any, Literal

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.localize.module import PerceptionLocalize
from dimos.perception.localize.types import LocalizeParams
from dimos.robot.diy.alfred.blueprints.alfred_base import ODOM_FRAME, alfred_base
from dimos.utils.gpu_utils import is_cuda_available, is_mps_available

# The bench rig: one D455 on the mast looking down at a table. The defaults are desk scale (fit on
# the xArm recordings). A robot standing still gives one viewpoint, so an instance is confirmed
# from one view rather than two; OWLv2's scores on a cluttered bench peak around 0.3-0.6 (measured:
# can 0.50, box 0.57, tape 0.30), which the default accept_score of 0.40 would cut through. A pick
# hands the instance's cloud to the grasp generator: at the default 1 cm a tennis ball came out as
# 43 points, too few for GraspGenX's outlier removal, and the D455 resolves ~1.5 mm at this range.
ALFRED_BENCH_PARAMS = LocalizeParams(min_views=1, accept_score=0.3, fuse_voxel_m=0.002)
# Raw colour and depth frames kept for the 1 Hz embed tail: 3 s at 15 Hz. The default (300)
# would hold 600 MB of frames.
LOCALIZE_FEED_FRAMES = 45
LOCALIZE_HORIZON_S = 120.0
# What the Manipulation app lists without being asked, one pass over the newest frame after another.
# OWLv2 takes words, not "whatever is there": a pass costs the same for one label or eight.
ALFRED_WATCH_LABELS = ["cup", "bottle", "box", "can", "tool", "ball"]

PerceptionDevice = Literal["cuda", "mps", "cpu"]
"""Where this machine runs the perception models."""


def perception_device() -> PerceptionDevice:
    """CUDA, then Apple's MPS, then the CPU (also where torch is missing)."""
    if is_cuda_available():  # type: ignore[no-untyped-call]
        return "cuda"
    if is_mps_available():
        return "mps"
    return "cpu"


def cuda_torch() -> str | None:
    """A blueprint requirement: why this machine cannot run the models, or None."""
    if is_cuda_available():  # type: ignore[no-untyped-call]
        return None
    return (
        "Alfred's perception needs a CUDA torch and this venv has none: sync it on the "
        "integration lock (see the README), or run alfred-base."
    )


def alfred_localize(
    *,
    detector: str = "owlv2",
    segmenter: str = "edgetam",
    params: LocalizeParams = ALFRED_BENCH_PARAMS,
) -> Any:
    """Dan's localize on the mast D455: text queries to verified 3D instances, from the colour feed
    and the aligned depth, posed by tf in ``odom``. ``detector``, ``segmenter`` and ``params`` are
    for a machine without CUDA and for rendered pictures."""
    return PerceptionLocalize.blueprint(
        world_frame=ODOM_FRAME,
        geometry="depth",
        mobile=False,
        params=params,
        detector=detector,
        segmenter=segmenter,
        feed_frames=LOCALIZE_FEED_FRAMES,
        horizon_s=LOCALIZE_HORIZON_S,
        watch=ALFRED_WATCH_LABELS,
    )


alfred_perception = (
    autoconnect(alfred_base, alfred_localize(), McpServer.blueprint())
    .requirements(cuda_torch)
    .global_config(n_workers=5, robot_model="alfred")
)
