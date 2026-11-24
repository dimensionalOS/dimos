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

"""The view transform a stage's RenderSettings ask for, applied to a MuJoCo render: linear light in, display sRGB out."""

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.mujoco.lights import render_settings
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Omniverse's /rtx/post/tonemap/op numbers and the names a layer may use, as dimos-react reads them.
_RTX_TONE_MAPS = {0: "none", 1: "linear", 4: "filmic", 5: "filmic"}
_NAMED_TONE_MAPS = {
    "agx": "agx",
    "neutral": "neutral",
    "filmic": "filmic",
    "aces": "filmic",
    "acesfilmic": "filmic",
    "linear": "linear",
    "none": "none",
}
_CURVES = {"neutral", "linear", "none"}

# Khronos PBR Neutral's own definition (KhronosGroup/ToneMapping), not a tuning.
_START_COMPRESSION = 0.8 - 0.04
_DESATURATION = 0.15


def stage_tone_map(scene: Path) -> str | None:
    """The tone curve the stage's RenderSettings name, or None when it names none."""
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(str(scene))
    settings = render_settings(stage) if stage else None
    if not settings:
        return None
    prim = settings.GetPrim()
    op = prim.GetAttribute("omni:rtx:post:tonemap:op").Get()
    named = prim.GetAttribute("dimos:look:toneMap").Get()
    curve = _RTX_TONE_MAPS.get(int(op)) if op is not None else None
    curve = curve or (_NAMED_TONE_MAPS.get(str(named).lower()) if named is not None else None)
    if curve is not None and curve not in _CURVES:
        logger.warning(f"sim2: {curve} tone mapping is not implemented; showing it without a curve")
    return curve


def linear_textures(spec: mujoco.MjSpec, scene: Path) -> int:
    """Have MuJoCo decode each texture by the color space its UsdUVTexture declares; the number changed."""
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(str(scene))
    declared: dict[str, str] = {}
    for prim in stage.Traverse() if stage else []:
        if prim.GetAttribute("info:id").Get() != "UsdUVTexture":
            continue
        asset = prim.GetAttribute("inputs:file").Get()
        if asset is not None and asset.resolvedPath:
            space = prim.GetAttribute("inputs:sourceColorSpace").Get()
            declared[str(Path(asset.resolvedPath).resolve())] = str(space or "auto")
    changed = 0
    for texture in spec.textures:
        space = declared.get(str(Path(texture.file).resolve())) if texture.file else None
        if space is None:
            continue
        # MuJoCo loads every texture at 8 bits a channel, which USD's "auto" reads as sRGB.
        texture.colorspace = (
            mujoco.mjtColorSpace.mjCOLORSPACE_LINEAR
            if space == "raw"
            else mujoco.mjtColorSpace.mjCOLORSPACE_SRGB
        )
        changed += 1
    return changed


def light_headroom(model: mujoco.MjModel) -> float:
    """The most linear light any lit surface can send back, so a render scaled down by it never clips."""
    active = model.light_active.astype(bool)
    total = (model.light_ambient + model.light_diffuse + model.light_specular)[active].sum(axis=0)
    headlight = model.vis.headlight
    if headlight.active:
        total = total + headlight.ambient + headlight.diffuse + headlight.specular
    return max(
        1.0, float(np.max(total, initial=0.0)) + float(np.max(model.mat_emission, initial=0.0))
    )


def scale_light(model: mujoco.MjModel, factor: float) -> None:
    """Multiply every source of light in the model by `factor`."""
    model.light_ambient *= factor
    model.light_diffuse *= factor
    model.light_specular *= factor
    model.mat_emission *= factor
    headlight = model.vis.headlight
    headlight.ambient, headlight.diffuse, headlight.specular = (
        headlight.ambient * factor,
        headlight.diffuse * factor,
        headlight.specular * factor,
    )


def pbr_neutral(color: NDArray[np.float32]) -> NDArray[np.float32]:
    """Khronos PBR Neutral on linear RGB (..., 3), as Blender's "Khronos PBR Neutral" view transform."""
    red, green, blue = (color[..., channel] for channel in range(3))
    low = np.minimum(np.minimum(red, green), blue)[..., None]
    toned = color - np.where(low < 0.08, low - np.float32(6.25) * low * low, np.float32(0.04))
    peak = np.maximum(np.maximum(toned[..., 0], toned[..., 1]), toned[..., 2])
    span = np.float32(1.0 - _START_COMPRESSION)
    with np.errstate(divide="ignore", invalid="ignore"):
        new_peak = 1.0 - span * span / (peak + span - np.float32(_START_COMPRESSION))
        toward_white = 1.0 - 1.0 / (np.float32(_DESATURATION) * (peak - new_peak) + 1.0)
        bright = peak >= _START_COMPRESSION
        # Below the knee the curve is the offset alone; past it, compressed and blended toward white.
        scale = np.where(bright, new_peak / peak * (1.0 - toward_white), np.float32(1.0))
    lift = np.where(bright, new_peak * toward_white, np.float32(0.0))
    toned *= scale[..., None]
    toned += lift[..., None]
    return toned


def srgb_bytes(linear: NDArray[np.float32]) -> NDArray[np.uint8]:
    """Linear light in [0, 1] encoded with the sRGB transfer curve, as 8-bit."""
    # The sRGB transfer curve as IEC 61966-2-1 defines it.
    light = np.clip(linear, 0.0, 1.0)
    encoded = np.where(light <= 0.0031308, 12.92 * light, 1.055 * light ** (1 / 2.4) - 0.055)
    return np.rint(encoded * 255).astype(np.uint8)


def display(render: NDArray[np.uint8], headroom: float, curve: str) -> NDArray[np.uint8]:
    """A render of light scaled down by `headroom`, back to linear light, tone mapped and sRGB encoded."""
    linear = render.astype(np.float32) * np.float32(headroom / 255.0)
    return srgb_bytes(pbr_neutral(linear) if curve == "neutral" else linear)
