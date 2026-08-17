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

"""Integration smoke test: multicam cuVSLAM on a short Spot snippet.

spot_multicam_short.db is 4 s of steady walking cut from spot_small_loop.db (all five
grayscale + depth cameras, infos, odometry, tf). The test replays it through the
Multisensor rig and checks that poses flow and the motion magnitude agrees with the
robot's own odometry — cuVSLAM on the GPU is not bit-deterministic, so only sanity
bounds are asserted, not exact poses.
"""

import numpy as np
import pytest

pytest.importorskip("cuvslam", reason="pycuvslam is only installed on self-hosted boxes")

from dimos.mapping.cuvslam.demo_cuvslam_spot import (
    CAMERAS,
    recorded_odometry,
    replay_spot,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.utils.data import get_data

pytestmark = pytest.mark.self_hosted

DEPTH_CAMERAS = ["left", "right", "back"]
MAX_FRAME_JUMP_M = 1.0


def test_spot_multicam_snippet_tracks() -> None:
    store = SqliteStore(path=str(get_data("spot_multicam_short.db")), must_exist=True)
    store.start()
    # GPU, not by choice: Multisensor's depth path runs CUDA kernels even in an
    # ENFORCE_GPU=OFF build (use_gpu=False aborts with CUDA error 700), so the
    # reproducible CPU path is not available and the assertions are bounds, not poses.
    result = replay_spot(store, list(CAMERAS), DEPTH_CAMERAS, warmup=2)

    trajectory = np.asarray(result["trajectory"], dtype=float)
    assert result["frames"] >= 15, result
    assert result["lost"] <= 2, result
    assert len(trajectory) >= 15
    assert np.isfinite(trajectory).all()

    jumps = np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1)
    assert jumps.max() < MAX_FRAME_JUMP_M, f"teleport in snippet replay: {jumps.max():.2f} m"

    odometry = np.asarray(recorded_odometry(store), dtype=float)
    window = odometry[(odometry[:, 0] >= trajectory[0, 0]) & (odometry[:, 0] <= trajectory[-1, 0])]
    odometry_path = float(np.linalg.norm(np.diff(window[:, 1:4], axis=0), axis=1).sum())
    assert odometry_path > 0.5, "snippet window should contain real motion"
    assert 0.3 * odometry_path < result["path_m"] < 3.0 * odometry_path, (
        f"cuVSLAM path {result['path_m']:.2f} m vs odometry {odometry_path:.2f} m"
    )
