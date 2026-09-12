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

"""Explicit CUDA acceptance check; launches only the grasp proposal module."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess

import numpy as np

from dimos.experimental.isolated_python.module import isolated_python_run_command
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def run(fixture: Path) -> None:
    points = np.load(fixture, allow_pickle=False)
    cloud = PointCloud2.from_numpy(points, frame_id="camera", timestamp=12.5)
    # The production xArm sweep volume, in the model's +Z approach convention.
    module = GraspGenXModule(
        gripper={
            "extents_open": (0.0889, 0.030, 0.0370),
            "offset_open": (0.0, 0.0, 0.1421),
            "extents_half_open": (0.0479, 0.030, 0.0370),
            "offset_half_open": (0.0, 0.0, 0.1530),
            "fingertip_depth": 0.1606,
        },
        grasp_frame_to_tcp=(
            (0.0, 1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.172),
            (0.0, 0.0, 0.0, 1.0),
        ),
    )
    try:
        module.build()
        process = module._process
        assert process is not None
        subprocess.run(
            isolated_python_run_command(
                module.runtime_project,
                "python",
                "-c",
                "import sys, torch, torchvision, numpy; "
                "assert sys.version_info[:2] == (3, 12); "
                "assert torch.__version__ == '2.7.1+cu128', torch.__version__; "
                "assert torchvision.__version__ == '0.22.1+cu128', torchvision.__version__; "
                "assert torch.version.cuda == '12.8', torch.version.cuda; "
                "assert torch.cuda.is_available(), 'CUDA unavailable'; "
                "print('Runtime:', sys.executable, 'Torch:', torch.__version__, "
                "'NumPy:', numpy.__version__, 'GPU:', torch.cuda.get_device_name())",
            ),
            cwd=module.runtime_project,
            env=module._runtime_env(),
            check=True,
            timeout=120,
        )
        module.start()
        result = module.propose_grasps(cloud)
        assert 0 < len(result) <= module.config.max_candidates
        assert result.header.frame_id == "camera"
        assert result.header.timestamp == 12.5
        scores = np.asarray([candidate.score for candidate in result])
        assert np.isfinite(scores).all()
        assert np.all(scores[:-1] >= scores[1:])
        for candidate in result:
            matrix = Transform.from_pose("camera", candidate.pose).to_matrix()
            assert np.isfinite(matrix).all()
            rotation = matrix[:3, :3]
            assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)
        # Validation errors must also survive the real RPC boundary.
        cloud.frame_id = ""
        try:
            module.propose_grasps(cloud)
        except ValueError as error:
            assert "frame_id" in str(error)
        else:
            raise AssertionError("Invalid cloud was accepted")
        print(f"Validated {len(result)} ranked grasps from {len(points)} recorded points")
    finally:
        module.stop()
    assert process.poll() is not None, "Isolated process survived module.stop()"
    print("Isolated process stopped")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fixture", type=Path, default=Path(__file__).parent / "fixtures/object_cloud.npy"
    )
    run(parser.parse_args().fixture)
