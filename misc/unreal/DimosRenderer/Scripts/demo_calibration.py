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

"""Exercise actual Unreal RGB-D pixels, calibration, pose updates, reset and EOF.

Run from an installed DimOS environment, passing the native executable:
    python misc/unreal/DimosRenderer/Scripts/demo_calibration.py --executable ...
Add --project .../DimosRenderer.uproject to test UnrealEditor -game instead.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
from typing import Any

import numpy as np
from PIL import Image

from dimos.sim2.unreal.client import RendererProcess


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--project", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or Path(tempfile.mkdtemp(prefix="dimos-unreal-calibration-"))
    output.mkdir(parents=True, exist_ok=True)
    manifest = output / "scene.json"
    manifest.write_text(
        json.dumps(
            {
                "version": 1,
                "meshes": [
                    {
                        "id": "plane",
                        "vertices": [
                            [0, -100, -100],
                            [0, 100, -100],
                            [0, 100, 100],
                            [0, -100, 100],
                        ],
                        "normals": [[-1, 0, 0]] * 4,
                        "triangles": [0, 1, 2, 0, 2, 3],
                    }
                ],
                "geoms": [
                    {
                        "id": 0,
                        "mesh_id": "plane",
                        "rgba": [1, 0.1, 0.05, 1],
                        "roughness": 0.7,
                        "position": [500, 0, 0],
                        "quaternion": [0, 0, 0, 1],
                    }
                ],
            }
        )
    )
    command: dict[str, Any] = {
        "episode": 1,
        "sequence": 0,
        "physics_tick": 42,
        "sim_time": 0.084,
        "timestamp": 12345.678,
        "poses": [{"id": 0, "position": [500, 0, 0], "quaternion": [0, 0, 0, 1]}],
        "camera": {
            "position": [0, 0, 0],
            "quaternion": [0, 0, 0, 1],
            "width": 240,
            "height": 160,
            "fx": 100.0,
            "fy": 100.0,
            "cx": 100.0,
            "cy": 70.0,
            "near_m": 0.01,
            "far_m": 100.0,
        },
    }
    process = RendererProcess(
        args.executable,
        output / "renderer.log",
        project=args.project,
        startup_timeout=240,
        capture_timeout=30,
    )
    measurements = []
    with process as client:
        client.load(manifest)
        for sequence in range(3):
            command["sequence"] = sequence
            frame = client.capture(command)
            finite = np.isfinite(frame.depth)
            yy, xx = np.nonzero(finite)
            assert finite.any(), "No geometry in depth image"
            assert np.allclose(frame.depth[finite], 5.0, atol=0.003), "Incorrect axial metric depth"
            assert abs(xx.mean() - 100) <= 0.5 and abs(yy.mean() - 70) <= 0.5, (
                "Incorrect principal point"
            )
            rgb = frame.rgba[:, :, :3].astype(np.float32)
            colored = (rgb[:, :, 0] > rgb[:, :, 1] + 30) & (rgb[:, :, 0] > rgb[:, :, 2] + 30)
            assert colored[finite].mean() > 0.95, (
                "RGB material color or RGB/depth alignment is incorrect"
            )
            assert colored[~finite].mean() < 0.01, "RGB geometry extends beyond the depth mask"
            Image.fromarray(frame.rgba).save(output / f"frame-{sequence}.png")
            np.save(output / f"depth-{sequence}.npy", frame.depth)
            measurements.append(
                {
                    **frame.header,
                    "roundtrip_ms": frame.roundtrip_seconds * 1000,
                    "depth_min_m": float(frame.depth[finite].min()),
                    "depth_max_m": float(frame.depth[finite].max()),
                    "bounds_xyxy": [int(xx.min()), int(yy.min()), int(xx.max()), int(yy.max())],
                    "center_rgba": frame.rgba[70, 100].tolist(),
                }
            )
        client.reset(2)
        command.update(episode=2, sequence=0, physics_tick=0, sim_time=0.0)
        command["poses"][0]["position"] = [700, 0, 0]
        frame = client.capture(command)
        assert abs(float(frame.depth[70, 100]) - 7.0) < 0.003, (
            "Reset/geometry pose returned stale pixels"
        )
        command["sequence"] = 1
        command["camera"]["position"] = [200, 0, 0]
        frame = client.capture(command)
        assert abs(float(frame.depth[70, 100]) - 5.0) < 0.003, "Camera translation is incorrect"
        command["sequence"] = 2
        yaw_90 = [0, 0, float(np.sqrt(0.5)), float(np.sqrt(0.5))]
        command["camera"].update(position=[0, 0, 0], quaternion=yaw_90)
        command["poses"][0].update(position=[0, 500, 0], quaternion=yaw_90)
        frame = client.capture(command)
        assert abs(float(frame.depth[70, 100]) - 5.0) < 0.003, (
            "Camera/geometry rotation is incorrect"
        )
        command["sequence"] = 3
        command["camera"]["far_m"] = 4.0
        frame = client.capture(command)
        assert not np.isfinite(frame.depth).any(), "Far clipping did not reject the 5 m plane"
        assert frame.rgba[:, :, :3].max() < 20, "RGB and depth use different far clipping"
        command["sequence"] = 4
        command["camera"].update(near_m=6.0, far_m=100.0)
        frame = client.capture(command)
        assert not np.isfinite(frame.depth).any(), "Near clipping did not reject the 5 m plane"
        assert frame.rgba[:, :, :3].max() < 20, "RGB and depth use different near clipping"
        # A disappearing Python owner must not leave an idle native renderer.
        client.close()
        assert process.process is not None
        assert process.process.wait(timeout=10) == 0, "Renderer did not exit on peer EOF"
    result = {
        "checks": [
            "colored RGB",
            "axial metric depth",
            "RGB/depth alignment",
            "off-center intrinsics",
            "geometry translation",
            "camera translation/rotation",
            "shared RGB/depth near/far clipping",
            "reset",
            "snapshot identity",
            "peer EOF",
        ],
        "frames": measurements,
    }
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"passed": result["checks"], "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
