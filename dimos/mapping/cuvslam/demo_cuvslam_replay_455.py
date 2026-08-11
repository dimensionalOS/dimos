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

"""cuVSLAM fed from a D455 recording instead of a live camera.

    dimos run demo-cuvslam-replay-455 --viewer rerun --rerun-host 0.0.0.0 \
        --recordingreplay.dataset sf_office1

The recording's IR pair, camera infos and mount tf come out of a
:class:`RecordingReplay` on the same topics a live RealSense would drive, so the
tracker under test is byte-for-byte the production module.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.mapping.cuvslam.demo_cuvslam_realsense import (
    cuvslam_rerun_blueprint,
    path_at_true_height,
)
from dimos.mapping.odometry_path import OdometryPath
from dimos.memory2.replay_module import RecordingReplay
from dimos.visualization.vis_module import vis_module

demo_cuvslam_replay_455 = autoconnect(
    RecordingReplay.blueprint(),
    CuvslamOdometry.blueprint(),
    OdometryPath.blueprint(),
    vis_module(
        global_config.viewer,
        rerun_config={
            "blueprint": cuvslam_rerun_blueprint,
            "visual_override": {"world/path": path_at_true_height},
        },
    ),
).global_config(n_workers=4)
