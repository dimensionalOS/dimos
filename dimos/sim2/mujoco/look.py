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

import functools
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.mujoco.lights import render_settings, sky_bytes
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
_CURVES = {"agx", "neutral", "linear", "none"}

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


def blank_textures(spec: mujoco.MjSpec) -> None:
    """Make every texture in `spec` one blank pixel, for a view that never renders; deleting them takes ~14 ms each."""
    for texture in spec.textures:
        texture.file, texture.data, texture.builtin = "", b"", mujoco.mjtBuiltin.mjBUILTIN_FLAT
        texture.width = texture.height = 1


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


def show_sky(model: mujoco.MjModel, sky: NDArray[np.float32]) -> None:
    """Redraw the model's skybox as `sky`, the linear light `add_sky` takes, so it scales as its lights did."""
    texture = int(np.flatnonzero(model.tex_type == mujoco.mjtTexture.mjTEXTURE_SKYBOX)[0])
    start = int(model.tex_adr[texture])
    model.tex_data[start : start + sky.size] = sky_bytes(sky).ravel()


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


_AGX = Path(__file__).parent / "assets" / "agx_base_srgb.npz"


@functools.cache
def _agx_base() -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.float32]]:
    """Blender's "AgX Base sRGB" LUT (57^3, [r, g, b] -> Rec.1886 code value), its Linear Rec.709 -> E-Gamut matrix and lg2 range."""
    stored = np.load(_AGX)
    return (
        stored["lut"].astype(np.float32) / np.float32(65535),
        stored["rec709_to_egamut"].astype(np.float32),
        stored["log2_range"].astype(np.float32),
    )


def _tetrahedral(lut: NDArray[np.float32], point: NDArray[np.float32]) -> NDArray[np.float32]:
    """`lut` (N, N, N, 3) at `point` (..., 3) in [0, 1], interpolated tetrahedrally as OCIO does."""
    size = lut.shape[0]
    flat = lut.reshape(-1, 3)
    scaled = np.clip(point, 0.0, 1.0) * (size - 1)
    base = np.minimum(scaled.astype(np.int32), size - 2)
    red, green, blue = (scaled - base).transpose(-1, *range(point.ndim - 1))
    corner = (base[..., 0] * size + base[..., 1]) * size + base[..., 2]
    step_red, step_green, step_blue = size * size, size, 1
    # The cell's low corner to its high one, stepping first along the axis with the largest fraction, last the smallest.
    first = np.where(
        (red >= green) & (red >= blue), step_red, np.where(green >= blue, step_green, step_blue)
    )
    last = np.where(
        (blue <= red) & (blue <= green), step_blue, np.where(green <= red, step_green, step_red)
    )
    middle = step_red + step_green + step_blue - first - last
    high = np.maximum(np.maximum(red, green), blue)
    low = np.minimum(np.minimum(red, green), blue)
    mid = red + green + blue - high - low
    return (
        flat[corner] * (1.0 - high)[..., None]
        + flat[corner + first] * (high - mid)[..., None]
        + flat[corner + first + middle] * (mid - low)[..., None]
        + flat[corner + step_red + step_green + step_blue] * low[..., None]
    )


def agx(color: NDArray[np.float32]) -> NDArray[np.float32]:
    """Blender's AgX view on linear Rec.709 (..., 3) as display sRGB in [0, 1]: its config.ocio "AgX Base Rec.1886" chain for an sRGB display."""
    # Linear Rec.709 -> E-Gamut, the lg2 allocation, AgX_Base_sRGB.cube (Blender 5.1's, at 16 bits), then Rec.1886's 2.4 power to sRGB.
    lut, to_egamut, (low, high) = _agx_base()
    egamut = color @ to_egamut.T
    stops = np.log2(np.maximum(egamut, np.float32(2.0) ** low))
    encoded = _tetrahedral(lut, (stops - low) / (high - low))
    light = np.clip(encoded, 0.0, 1.0) ** np.float32(2.4)
    # The sRGB transfer curve as IEC 61966-2-1 defines it.
    return np.where(light <= 0.0031308, 12.92 * light, 1.055 * light ** (1 / 2.4) - 0.055)


def display(render: NDArray[np.uint8], headroom: float, curve: str) -> NDArray[np.uint8]:
    """A render of light scaled down by `headroom`, back to linear light, tone mapped and sRGB encoded."""
    linear = render.astype(np.float32) * np.float32(headroom / 255.0)
    if curve == "agx":
        return np.rint(np.clip(agx(linear), 0.0, 1.0) * 255).astype(np.uint8)
    return srgb_bytes(pbr_neutral(linear) if curve == "neutral" else linear)
