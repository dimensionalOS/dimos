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

"""Camera device parameters in optical coordinates: +X right, +Y down, +Z forward."""

from dataclasses import dataclass
import math
from typing import Protocol

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from numpy.typing import NDArray

from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.assets.usd_utils.frames import link_prim_paths
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr


@dataclass(frozen=True)
class CameraConfig:
    frame_id: str
    hz: float = 15
    width: int = 320
    height: int = 240
    fx: float = 188.36
    fy: float = 188.36
    cx: float = 160
    cy: float = 120

    def __post_init__(self) -> None:
        if min(self.width, self.height, self.fx, self.fy, self.hz) <= 0 or not all(
            map(math.isfinite, (self.fx, self.fy, self.cx, self.cy, self.hz))
        ):
            raise ValueError("invalid camera intrinsics/rate")


class CameraRenderer(Protocol):
    """One scene, many camera views, called only on the rendering owner thread.

    Load scene resources once; share native contexts between compatible views.
    A render call applies one complete acquisition to every requested camera.
    Results are keyed by camera frame ID: RGB uint8 (H, W, 3), optical-axis
    depth float32 (H, W) in metres. Arrays must survive subsequent render calls.
    Unsupported projections must fail at load, never silently change intrinsics.
    """

    def load(self, scene: UsdModel, cameras: tuple[CameraConfig, ...]) -> None: ...
    def render(
        self, sample: WorldState, camera_ids: tuple[str, ...]
    ) -> dict[str, tuple[NDArray[np.uint8], NDArray[np.float32]]]: ...
    def close(self) -> None: ...


def camera_usd(camera: CameraConfig) -> str:
    """Camera description in the same optical frame as Image and CameraInfo."""
    pxr = load_pxr()
    layer = layer_from(
        pxr, RobotModel.from_root_link(camera.frame_id, name="camera").to_usd().usd, "camera", None
    )
    stage = pxr.Usd.Stage.Open(layer)
    path = link_prim_paths(stage)[camera.frame_id]
    optics = pxr.UsdGeom.Camera.Define(stage, path + "/optics")
    optics.GetPrim().SetDisplayName(camera.frame_id + "/optics")
    # USD cameras look down -Z with +Y up; DimOS optical coordinates are +Z, -Y.
    optics.AddRotateXOp().Set(180.0)
    focal = 20.0
    optics.CreateFocalLengthAttr(focal)
    optics.CreateHorizontalApertureAttr(focal * camera.width / camera.fx)
    optics.CreateVerticalApertureAttr(focal * camera.height / camera.fy)
    optics.CreateHorizontalApertureOffsetAttr(focal * (camera.cx - camera.width / 2) / camera.fx)
    optics.CreateVerticalApertureOffsetAttr(focal * (camera.height / 2 - camera.cy) / camera.fy)
    return str(layer.ExportToString())
