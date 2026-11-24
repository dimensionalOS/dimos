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

"""A stage's UsdLux sun and dome as MuJoCo lights, in Blender's units (MuJoCo's USD reader drops lights)."""

import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def apply_lights(spec: mujoco.MjSpec, scene: Path) -> int:
    """Light `spec` with the lights its stage authors; the number of MuJoCo lights added."""
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    from pxr import UsdLux

    stage = pxr.Usd.Stage.Open(str(scene))
    if stage is None:
        return 0
    lights = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdLux.LightAPI)
        and pxr.UsdGeom.Imageable(prim).ComputeVisibility() != pxr.UsdGeom.Tokens.invisible
    ]
    if not lights:
        return 0
    meters = pxr.UsdGeom.GetStageMetersPerUnit(stage)
    bounds = (
        pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), [pxr.UsdGeom.Tokens.default_])
        .ComputeWorldBound(stage.GetPseudoRoot())
        .ComputeAlignedRange()
    )
    center = np.array(bounds.GetMidpoint()) * meters
    reach = float(np.linalg.norm(bounds.GetSize())) * meters / 2
    scale = _camera_exposure_scale(pxr, stage)
    added = 0
    for prim in lights:
        light = UsdLux.LightAPI(prim)
        radiance = _radiance(UsdLux, light) * scale
        diffuse = _value(light.GetDiffuseAttr(), 1.0)
        specular = _value(light.GetSpecularAttr(), 1.0)
        name = str(prim.GetPath())
        if prim.IsA(UsdLux.DistantLight):
            to_world = np.array(
                pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
            )
            # USD matrices are row-vector, so a direction maps through the upper 3x3 from the left.
            direction = np.array([0.0, 0.0, -1.0]) @ to_world[:3, :3]
            direction /= np.linalg.norm(direction)
            # Blender's sun strength is irradiance E, and a Lambert surface of albedo a sends back a·E/π.
            irradiance = radiance / math.pi
            spec.worldbody.add_light(
                name=name,
                type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                # A directional light's shadow is cast from its position, so it stands outside the stage.
                pos=center - direction * reach,
                dir=direction,
                diffuse=irradiance * diffuse,
                specular=irradiance * specular,
                ambient=np.zeros(3),
                castshadow=bool(_value(UsdLux.ShadowAPI(prim).GetShadowEnableAttr(), True)),
            )
        elif prim.IsA(UsdLux.DomeLight):
            if _value(prim.GetAttribute("inputs:texture:file"), None):
                logger.warning(f"sim2: {name} lights with a texture; MuJoCo takes its color only")
            # Blender's World strength is sky radiance L, and a Lambert surface sends back a·L.
            spec.worldbody.add_light(
                name=name,
                type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                pos=center,
                dir=(0.0, 0.0, -1.0),
                diffuse=np.zeros(3),
                specular=np.zeros(3),
                ambient=radiance * diffuse,
                castshadow=False,
            )
        else:
            logger.warning(
                f"sim2: {name} is a {prim.GetTypeName()}, which MuJoCo's render does not get; "
                f"only distant and dome lights are carried across"
            )
            continue
        added += 1
    if added:
        # Blender has no headlight, so a stage that brings its own lights turns MuJoCo's off.
        spec.visual.headlight.active = 0
    return added


def _radiance(UsdLux: Any, light: Any) -> np.ndarray:
    color = np.array(_value(light.GetColorAttr(), (1.0, 1.0, 1.0)), dtype=float)
    if _value(light.GetEnableColorTemperatureAttr(), False):
        color *= np.array(
            UsdLux.BlackbodyTemperatureAsRgb(_value(light.GetColorTemperatureAttr(), 6500.0))
        )
    intensity = _value(light.GetIntensityAttr(), 1.0)
    return color * intensity * 2.0 ** _value(light.GetExposureAttr(), 0.0)


def _camera_exposure_scale(pxr: Any, stage: Any) -> float:
    """The linear exposure of the camera the stage says it is rendered through, else 1."""
    from pxr import UsdRender

    settings = UsdRender.Settings.GetStageRenderSettings(stage)
    if not settings:
        everything = [p for p in stage.Traverse() if p.IsA(UsdRender.Settings)]
        settings = UsdRender.Settings(everything[0]) if len(everything) == 1 else None
    targets = settings.GetCameraRel().GetForwardedTargets() if settings else []
    camera = pxr.UsdGeom.Camera(stage.GetPrimAtPath(targets[0])) if targets else None
    return float(camera.ComputeLinearExposureScale()) if camera else 1.0


def _value(attribute: Any, default: Any) -> Any:
    value = attribute.Get() if attribute else None
    return default if value is None else value
