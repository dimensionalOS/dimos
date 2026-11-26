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

"""Render recorded Sim2 body poses in the synthetic M20 evaluation fixture."""

import argparse
from contextlib import ExitStack
from pathlib import Path
import struct
import subprocess

from dimos_lcm.sim_msgs import WorldState
import mujoco
from PIL import Image, ImageDraw

from dimos.robot.assets.model import UsdModel
from dimos.sim2.description import scene_revision
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.geometry.query import MujocoGeometry


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", default="M20 / Sim2 — synthetic stair evaluation")
    args = parser.parse_args()
    scene = UsdModel.from_file(args.scene.resolve())
    spec = build_scene(scene, kinematic=True)
    spec.worldbody.add_light(
        name="observer_light",
        pos=[-1, -3, 5],
        dir=[0.2, 0.2, -1],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.55, 0.55, 0.55],
        ambient=[0.08, 0.08, 0.08],
    )
    geometry = MujocoGeometry(spec.compile(), scene_revision(scene))
    geometry.select_surfaces()
    model = geometry.model
    model.vis.global_.offwidth, model.vis.global_.offheight = 960, 540
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = [0.08, 0.08, 0.08]
    model.vis.headlight.diffuse[:] = [0.2, 0.2, 0.2]
    for index in range(model.ngeom):
        name = model.geom(index).name
        if "/evaluation/step_" in name:
            model.geom_rgba[index] = [0.26, 0.4, 0.5, 1]
        elif "/evaluation/ground" in name:
            model.geom_rgba[index] = [0.72, 0.74, 0.76, 1]
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [2.2, 0, 0.55]
    camera.distance, camera.azimuth, camera.elevation = 5.2, 225, -23
    with ExitStack() as cleanup:
        renderer = mujoco.Renderer(model, height=540, width=960)
        cleanup.callback(renderer.close)
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
                "20",
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "2",
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
        first_time: float | None = None
        next_frame = 0.0
        frames = 0
        while prefix := recording.read(4):
            size = struct.unpack("<I", prefix)[0]
            encoded = recording.read(size)
            if len(encoded) != size:
                raise ValueError("Truncated body-pose recording")
            sample = WorldState.lcm_decode(encoded)
            stamp = sample.header.time_ns / 1e9
            if first_time is None:
                first_time = stamp
            elapsed = stamp - first_time
            if elapsed + 0.005 < next_frame:
                continue
            geometry.set_poses(sample)
            renderer.update_scene(geometry.data, camera=camera)
            image = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 960, 58), fill=(15, 22, 30))
            draw.text((18, 10), args.label, fill="white", font_size=19)
            draw.text(
                (18, 34),
                f"Measured articulated poses / observer replay / sim time {elapsed:.2f} s",
                fill="#b7c9d7",
                font_size=14,
            )
            encoder.stdin.write(image.tobytes())
            if frames == 60:
                image.save(args.output.with_suffix(".png"))
            frames += 1
            next_frame += 0.05
        if not frames:
            raise ValueError("Empty body-pose recording")
    if encoder.returncode != 0:
        raise RuntimeError(f"Video encoder exited with {encoder.returncode}")
    print(f"Rendered {frames} measured-pose frames to {args.output}")


if __name__ == "__main__":
    main()
