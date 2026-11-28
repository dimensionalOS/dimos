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
from numpy.typing import NDArray

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
    added, skied, left_out = 0, False, []
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
            irradiance = _sun_irradiance(UsdLux, prim, radiance) / math.pi
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
        elif _is_dome(UsdLux, prim):
            if _value(prim.GetAttribute("inputs:texture:file"), None):
                logger.warning(
                    f"sim2: {name}'s texture shows as the sky; its light takes the color only"
                )
            if not skied:
                add_sky(spec, _sky(pxr, stage, prim, radiance))
                skied = True
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
            left_out.append(name)
            continue
        added += 1
    if left_out:
        # MuJoCo lights a mesh at its vertices and draws 8 lights, so a lamp's pool of light is left to renderers that can draw it.
        logger.info(
            f"sim2: {len(left_out)} lamp(s) not drawn, only distant and dome lights are: {left_out[:3]}"
        )
    if added:
        # Blender has no headlight, so a stage that brings its own lights turns MuJoCo's off.
        spec.visual.headlight.active = 0
    return added


def dome_sky(scene: Path) -> NDArray[np.float32] | None:
    """The sky the stage's dome shows, as `add_sky` gives it the model but unclipped; None with no dome."""
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    from pxr import UsdLux

    stage = pxr.Usd.Stage.Open(str(scene))
    for prim in stage.Traverse() if stage else []:
        if (
            _is_dome(UsdLux, prim)
            and pxr.UsdGeom.Imageable(prim).ComputeVisibility() != pxr.UsdGeom.Tokens.invisible
        ):
            radiance = _radiance(UsdLux, UsdLux.LightAPI(prim)) * _camera_exposure_scale(pxr, stage)
            return _sky(pxr, stage, prim, radiance)
    return None


def add_sky(spec: mujoco.MjSpec, sky: NDArray[np.float32]) -> None:
    """Give `spec` the skybox `sky` (MuJoCo's six faces stacked, 6N x N x 3, linear light), clipped at 1."""
    width = sky.shape[1]
    spec.add_texture(
        name="dome_sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        width=width,
        height=6 * width,
        nchannel=3,
        colorspace=mujoco.mjtColorSpace.mjCOLORSPACE_LINEAR,
    ).data = sky_bytes(sky).tobytes()


def sky_bytes(sky: NDArray[np.float32]) -> NDArray[np.uint8]:
    """Linear light as the 8 bits MuJoCo draws a skybox with, unlit and unencoded, as it writes linear light."""
    return np.rint(np.clip(sky, 0.0, 1.0) * 255).astype(np.uint8)


# Each skybox face's outward axis, then the directions its columns and its rows run, as MuJoCo draws them.
_FACES = np.array(
    [
        [[1, 0, 0], [0, 1, 0], [0, 0, -1]],
        [[-1, 0, 0], [0, -1, 0], [0, 0, -1]],
        [[0, 0, 1], [1, 0, 0], [0, -1, 0]],
        [[0, 0, -1], [1, 0, 0], [0, 1, 0]],
        [[0, -1, 0], [1, 0, 0], [0, 0, -1]],
        [[0, 1, 0], [-1, 0, 0], [0, 0, -1]],
    ],
    dtype=float,
)


def _is_dome(UsdLux: Any, prim: Any) -> bool:
    return bool(prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.DomeLight_1))


def _sky(pxr: Any, stage: Any, prim: Any, radiance: np.ndarray) -> NDArray[np.float32]:
    """The dome's radiance in every direction as MuJoCo skybox faces: its texture times its color, else its color."""
    uniform = np.broadcast_to(radiance.astype(np.float32), (6, 1, 3)).copy()
    asset = _value(prim.GetAttribute("inputs:texture:file"), None)
    if asset is None:
        return uniform
    name, layout = (
        str(prim.GetPath()),
        str(_value(prim.GetAttribute("inputs:texture:format"), "automatic")),
    )
    image = _linear_image(Path(asset.resolvedPath)) if asset.resolvedPath else None
    if image is None:
        logger.warning(f"sim2: {name}'s texture {asset.path} does not load; its sky is its color")
        return uniform
    if layout not in ("automatic", "latlong"):
        logger.warning(f"sim2: {name}'s {layout} texture is not implemented; its sky is its color")
        return uniform
    size = max(1, image.shape[0] // 2)
    # A face spans 90 degrees, as half the height of a latlong spans, so each keeps the image's resolution.
    across = (np.arange(size) + 0.5) / size * 2 - 1
    rows, columns = np.meshgrid(across, across, indexing="ij")
    directions = (
        _FACES[:, None, None, 0]
        + columns[None, ..., None] * _FACES[:, None, None, 1]
        + rows[None, ..., None] * _FACES[:, None, None, 2]
    )
    to_world = np.array(
        pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
    )[:3, :3]
    # USD matrices are row-vector, so a world direction maps back to the dome's through the inverse.
    local = directions.reshape(-1, 3) @ np.linalg.inv(to_world)
    local /= np.linalg.norm(local, axis=1, keepdims=True)
    pole = str(_value(prim.GetAttribute("poleAxis"), "Y"))
    if pole == "Z" or (pole == "scene" and pxr.UsdGeom.GetStageUpAxis(stage) == "Z"):
        # A Z pole is the Y pole turned down onto the horizon, as Hydra turns it.
        local = np.stack([local[:, 0], local[:, 2], -local[:, 1]], axis=1)
    # Hydra Storm's latlong: u = 0.5 looks down +Z, v = 0 is the +Y pole at the image's top row.
    u = (np.arctan2(local[:, 2], local[:, 0]) + 0.5 * np.pi) / (2 * np.pi) % 1.0
    v = np.arccos(np.clip(local[:, 1], -1.0, 1.0)) / np.pi
    height, width = image.shape[:2]
    # One column wrapped onto each side, so a sample across the seam blends both edges.
    wrapped = np.concatenate([image[:, -1:], image, image[:, :1]], axis=1)
    import cv2

    sampled = cv2.remap(
        wrapped,
        (u * width + 0.5).astype(np.float32).reshape(6 * size, size),
        (v * height - 0.5).astype(np.float32).reshape(6 * size, size),
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
    return (sampled * radiance).astype(np.float32)


def srgb_to_linear(encoded: NDArray[Any]) -> NDArray[Any]:
    """sRGB code values in [0, 1] as linear light: the transfer curve IEC 61966-2-1 defines, inverted."""
    return np.where(encoded <= 0.04045, encoded / 12.92, ((encoded + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(light: NDArray[Any]) -> NDArray[Any]:
    """Linear light in [0, 1] as sRGB code values: the transfer curve IEC 61966-2-1 defines."""
    return np.where(light <= 0.0031308, 12.92 * light, 1.055 * light ** (1 / 2.4) - 0.055)


def _linear_image(path: Path) -> NDArray[np.float32] | None:
    """An image file as linear RGB: float files as they are, integer ones decoded from sRGB."""
    import cv2

    raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        return None
    color = raw[..., None].repeat(3, axis=2) if raw.ndim == 2 else raw[..., 2::-1]
    if np.issubdtype(color.dtype, np.floating):
        return np.ascontiguousarray(color, dtype=np.float32)
    return srgb_to_linear(color.astype(np.float32) / np.iinfo(color.dtype).max).astype(np.float32)


def _sun_irradiance(UsdLux: Any, prim: Any, radiance: np.ndarray) -> np.ndarray:
    """A DistantLight's irradiance as Blender 5.1 imports and renders it, so a Lambert surface sends back a·E/π."""
    light = UsdLux.DistantLight(prim)
    # Measured in Blender 5.1: strength 4·I; unnormalized, that is the radiance of a disc of half-angle inputs:angle, so times π·sin²(angle).
    if _value(light.GetNormalizeAttr(), False):
        return 4.0 * radiance
    half_angle = math.radians(_value(light.GetAngleAttr(), 0.53))
    return 4.0 * radiance * math.pi * math.sin(half_angle) ** 2


def _radiance(UsdLux: Any, light: Any) -> np.ndarray:
    color = np.array(_value(light.GetColorAttr(), (1.0, 1.0, 1.0)), dtype=float)
    if _value(light.GetEnableColorTemperatureAttr(), False):
        color *= np.array(
            UsdLux.BlackbodyTemperatureAsRgb(_value(light.GetColorTemperatureAttr(), 6500.0))
        )
    intensity = _value(light.GetIntensityAttr(), 1.0)
    return color * intensity * 2.0 ** _value(light.GetExposureAttr(), 0.0)


def render_settings(stage: Any) -> Any:
    """The RenderSettings the stage names, else its only one, else None."""
    from pxr import UsdRender

    settings = UsdRender.Settings.GetStageRenderSettings(stage)
    if settings:
        return settings
    everything = [p for p in stage.Traverse() if p.IsA(UsdRender.Settings)]
    return UsdRender.Settings(everything[0]) if len(everything) == 1 else None


def _camera_exposure_scale(pxr: Any, stage: Any) -> float:
    """The linear exposure of the camera the stage says it is rendered through, else 1."""
    settings = render_settings(stage)
    targets = settings.GetCameraRel().GetForwardedTargets() if settings else []
    camera = pxr.UsdGeom.Camera(stage.GetPrimAtPath(targets[0])) if targets else None
    return float(camera.ComputeLinearExposureScale()) if camera else 1.0


def _value(attribute: Any, default: Any) -> Any:
    value = attribute.Get() if attribute else None
    return default if value is None else value
