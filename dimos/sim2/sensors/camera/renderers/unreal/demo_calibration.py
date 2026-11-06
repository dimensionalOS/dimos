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

"""Exercise the real native renderer with an off-center, nonsquare pinhole camera."""

import argparse
from contextlib import ExitStack
from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sim_msgs import WorldState
import numpy as np
from PIL import Image

from dimos.robot.assets.model import RobotModel
from dimos.sim2.description import Instance, SceneConfig
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.unreal.renderer import UnrealRenderer


def validate(executable: Path, project: Path, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    with ExitStack() as cleanup:
        root = Path(cleanup.enter_context(TemporaryDirectory()))
        urdf = root / "box.urdf"
        urdf.write_text(
            '<robot name="calibration"><link name="box"><visual><origin xyz="3 0 1"/><geometry><box size="1 1 1"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual></link></robot>'
        )
        model = RobotModel.from_file(urdf).with_fixed_frame(
            "camera", "box", xyz=(0, 0, 1), rpy=(-np.pi / 2, 0, -np.pi / 2)
        )
        scene = SceneConfig((Instance("box", model),), ground_plane=False)
        backend = MujocoBackend()
        cleanup.callback(backend.close)
        backend.load(scene, 2_000_000)
        renderer = UnrealRenderer(
            executable, project=project, log_path=output / "unreal.log", startup_timeout=300
        )
        cleanup.callback(renderer.close)
        renderer.load(
            scene, CameraConfig("camera", width=200, height=150, fx=90, fy=110, cx=77, cy=63)
        )
        bodies = backend.read_bodies()
        state = WorldState(
            header=SampleHeader(
                clock_id="calibration",
                epoch=3,
                time_ns=1793928412345678901,
                sequence=9007199254740993,
            ),
            model_revision=scene.revision,
            bodies_length=len(bodies),
            bodies=bodies,
        )
        renderer.set_poses(state)
        rgb, depth = renderer.render(poses_by_id(state)["camera"])
        assert abs(float(depth[63, 77]) - 2.5) < 0.0001
        yy, xx = np.where(np.isfinite(depth))
        assert abs(float(xx.mean()) - 77) <= 1
        assert abs(float(yy.mean()) - 63) <= 1
        assert rgb[63, 77, 0] > rgb[63, 77, 1] + 50
        Image.fromarray(rgb).save(output / "color.png")
        np.save(output / "depth.npy", depth)
        # Exercise consecutive asynchronous captures, target resizing, epoch
        # changes, and dimensions that don't fill a compute workgroup. Moving
        # the camera also detects a stale depth buffer paired with a new header.
        captures = []
        for width, height, camera_x in [(201, 153, 0.5), (317, 119, -0.25), (200, 150, 0.0)]:
            renderer.camera = replace(renderer.camera, width=width, height=height)
            state.header.sequence += 1
            state.header.time_ns += 1
            state.header.epoch += 1
            camera_pose = poses_by_id(state)["camera"]
            camera_pose.position.x = camera_x
            renderer.set_poses(state)
            next_rgb, next_depth = renderer.render(camera_pose)
            assert next_rgb.shape == (height, width, 3)
            assert next_depth.shape == (height, width)
            expected_depth = 2.5 - camera_x
            assert abs(float(next_depth[63, 77]) - expected_depth) < 0.0001
            assert next_rgb[63, 77, 0] > next_rgb[63, 77, 1] + 50
            assert np.isinf(next_depth[0, 0])
            captures.append(
                {"width": width, "height": height, "center_depth_m": float(next_depth[63, 77])}
            )
        report = {
            "center_depth_m": float(depth[63, 77]),
            "projected_center_px": [float(xx.mean()), float(yy.mean())],
            "time_ns": state.header.time_ns,
            "sequence": state.header.sequence,
            "resized_captures": captures,
            "passed": True,
        }
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(report)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    validate(args.executable, args.project, args.output)
