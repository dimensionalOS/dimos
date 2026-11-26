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

"""Render a recorded run from a labelled observer camera; never drives physics.

The source WorldStates remain on disk unchanged. Only a copy of the camera body
pose is replaced for this review movie, outside Sim2's sensor/publication ports.
"""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import struct
import subprocess

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from dimos.experimental.gta_sim2.renderer import SixNGRenderer
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2_scene import PELVIS, TORSO
from dimos.sim2.sensors.camera.config import CameraConfig


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--recording", type=Path, required=True)
    p.add_argument("--scene", type=Path, required=True)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--executable", type=Path, required=True)
    p.add_argument("--game", type=Path, required=True)
    p.add_argument("--work", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--gpu", help="Optional SixNG adapter-name filter")
    args = p.parse_args()
    origin = tuple(json.loads(args.manifest.read_text())["origin_rage_m"])
    name = TORSO + "/camera_mount/camera/camera"
    camera = CameraConfig(name, width=960, height=540, fx=570, fy=570, cx=480, cy=270)
    renderer = SixNGRenderer(
        args.executable,
        args.game,
        args.work,
        args.output.parent / "replay-runtime",
        origin,
        gpu=args.gpu,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as cleanup:
        cleanup.callback(renderer.close)
        renderer.load(UsdModel.from_file(args.scene), (camera,))
        encoder = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                "960x540",
                "-r",
                "10",
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(args.output),
            ],
            stdin=subprocess.PIPE,
        )

        def close_encoder() -> None:
            if encoder.stdin:
                encoder.stdin.close()
            try:
                encoder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                encoder.kill()
                encoder.wait()

        cleanup.callback(close_encoder)
        assert encoder.stdin is not None
        recording = cleanup.enter_context(args.recording.open("rb"))
        frames = 0
        next_time: float | None = None
        while prefix := recording.read(4):
            count = struct.unpack("<I", prefix)[0]
            encoded = recording.read(count)
            if len(encoded) != count:
                raise ValueError("Truncated WorldState recording")
            sample = WorldState.lcm_decode(encoded)
            stamp = sample.header.time_ns / 1e9
            if next_time is not None and stamp < next_time:
                continue
            next_time = stamp + 0.09
            bodies = {b.body_id: b for b in sample.bodies}
            pelvis = bodies[PELVIS].pose.position
            target = np.array([pelvis.x, pelvis.y, pelvis.z + 0.15])
            eye = target + np.array([-2.6, -3.3, 1.6])
            forward = target - eye
            forward /= np.linalg.norm(forward)
            right = np.cross(forward, [0, 0, 1])
            right /= np.linalg.norm(right)
            down = np.cross(forward, right)
            q = Rotation.from_matrix(np.column_stack([right, down, forward])).as_quat()
            pose = bodies[name].pose
            pose.position.x, pose.position.y, pose.position.z = eye.tolist()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
                q.tolist()
            )
            rgb, _ = renderer.render(sample, (name,))[name]
            encoder.stdin.write(rgb.tobytes())
            if frames == 0 or frames % 100 == 0:
                Image.fromarray(rgb).save(args.output.with_suffix(".png"))
                print(f"{frames} frames; sim time {stamp:.2f}s", flush=True)
            frames += 1
        if frames == 0:
            raise ValueError("Empty WorldState recording")
        Image.fromarray(rgb).save(args.output.with_suffix(".png"))
        print(f"Rendered {frames} observer frames", flush=True)
    if encoder.returncode != 0:
        raise RuntimeError(f"Video encoder failed: {encoder.returncode}")


if __name__ == "__main__":
    main()
