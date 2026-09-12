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

"""Dense metric depth from a sparse stereo one, using the colour frame.

Stereo depth needs texture, so blank walls, dark corners, shiny floors and thin
legs come back empty -- on an indoor D455 clip about 60% of the pixels have no
reading. Depth Anything V2 has an opinion about every pixel and gets the shape
right, but its metric scale is off by up to 2x and drifts as the camera turns.

So each frame the prediction is fitted to the pixels the sensor is trusted on
(``raw ~ a * pred + b``, refit once without the residual outliers), the fit is
smoothed across frames, and the sensor's own reading is kept wherever it agrees
with the aligned prediction. Real geometry survives untouched; the holes get a
prediction that had to agree with the sensor everywhere it could be checked.

A port of the Rust crate (github.com/jeff-hykin/depth2depth), same defaults.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

# A METRIC checkpoint: the fit assumes the prediction is depth. The relative
# checkpoints predict inverse depth, so fitting them affinely to meters is
# wrong. The crate uses the Hypersim indoor vit-small, which has no transformers
# port; this is the same training set one size up.
DEPTH_MODEL_NAME = "depth-anything/Depth-Anything-V2-Metric-Indoor-Base-hf"


@dataclass
class FuseConfig:
    """Thresholds of the fusion. The defaults are the crate's."""

    # Model input size. Smaller is faster; both are snapped to the patch grid.
    model_height: int = 280
    model_width: int = 504
    # Raw depth outside this range is a hole to fill, not evidence.
    near_m: float = 0.3
    far_m: float = 6.0
    # Weight of the newest fit in the EMA across frames (1.0 = no smoothing).
    ema_new_weight: float = 0.3
    # A raw pixel is kept when |aligned - raw| < max(abs_tol, rel_tol * aligned).
    abs_tol: float = 0.3
    rel_tol: float = 0.1
    # Fewer trusted pixels than this and the frame keeps the previous fit.
    min_fit_pixels: int = 500


@dataclass
class Fusion:
    """One fused frame, in meters."""

    fused: NDArray[np.float32]
    aligned: NDArray[np.float32]
    kept_raw: NDArray[np.bool_]
    a: float
    b: float

    @property
    def kept_fraction(self) -> float:
        return float(self.kept_raw.mean())


def fit_affine(
    prediction: NDArray[np.float32],
    raw: NDArray[np.float32],
    valid: NDArray[np.bool_],
    config: FuseConfig,
) -> tuple[float, float] | None:
    """Least squares ``raw ~ a * prediction + b`` over the trusted pixels, refit
    once without the residual outliers. None when there is too little to fit."""
    a, b = 1.0, 0.0
    inlier = valid
    fitted = False
    for _ in range(2):
        if int(inlier.sum()) < config.min_fit_pixels:
            break
        p, r = prediction[inlier].astype(np.float64), raw[inlier].astype(np.float64)
        n = float(p.size)
        sum_p, sum_pp, sum_r, sum_pr = p.sum(), (p * p).sum(), r.sum(), (p * r).sum()
        det = sum_pp * n - sum_p * sum_p
        if abs(det) < 1e-9:
            break
        a = (sum_pr * n - sum_p * sum_r) / det
        b = (sum_pp * sum_r - sum_p * sum_pr) / det
        fitted = True
        residual = np.abs(a * prediction + b - raw)
        inlier = valid & (residual < np.maximum(config.abs_tol, config.rel_tol * raw))
    return (float(a), float(b)) if fitted else None


def fuse(
    prediction: NDArray[np.float32],
    raw_depth_m: NDArray[np.float32],
    config: FuseConfig,
    previous: tuple[float, float] | None = None,
) -> Fusion:
    """Align one prediction to one raw depth frame and fill the holes with it."""
    valid = (raw_depth_m >= config.near_m) & (raw_depth_m <= config.far_m)
    fit = fit_affine(prediction, raw_depth_m, valid, config)
    if fit is None:
        # Nothing to fit against (a wall up close, an empty frame): the last
        # frame's scale is a better guess than a fresh 1:1.
        fit = previous if previous is not None else (1.0, 0.0)
    if previous is not None:
        weight = config.ema_new_weight
        fit = (
            (1.0 - weight) * previous[0] + weight * fit[0],
            (1.0 - weight) * previous[1] + weight * fit[1],
        )
    aligned = (fit[0] * prediction + fit[1]).astype(np.float32)
    agrees = np.abs(aligned - raw_depth_m) < np.maximum(config.abs_tol, config.rel_tol * aligned)
    kept_raw = valid & agrees
    return Fusion(
        fused=np.where(kept_raw, raw_depth_m, aligned).astype(np.float32),
        aligned=aligned,
        kept_raw=kept_raw,
        a=fit[0],
        b=fit[1],
    )


@dataclass
class Depth2Depth:
    """The model and the running fit. One instance per camera: the EMA is the
    camera's scale over time, and ``reset()`` drops it at a scene cut."""

    config: FuseConfig = field(default_factory=FuseConfig)
    model_name: str = DEPTH_MODEL_NAME
    device: str = "auto"
    _fit: tuple[float, float] | None = field(default=None, init=False)
    _model: Any = field(default=None, init=False)
    _processor: Any = field(default=None, init=False)

    def start(self) -> None:
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        if self.device == "auto":
            if torch.cuda.is_available():
                self.device = "cuda"
            elif torch.backends.mps.is_available():
                self.device = "mps"
            else:
                self.device = "cpu"
        self._processor = AutoImageProcessor.from_pretrained(self.model_name)
        self._model = AutoModelForDepthEstimation.from_pretrained(self.model_name).to(self.device)
        self._model.eval()

    def stop(self) -> None:
        self._model, self._processor = None, None

    def reset(self) -> None:
        self._fit = None

    def predict(self, rgb: NDArray[np.uint8]) -> NDArray[np.float32]:
        """Metric depth for every pixel of an HxWx3 frame, at its own size."""
        import torch

        if self._model is None:
            raise RuntimeError("Depth2Depth.start() first")
        height, width = rgb.shape[:2]
        inputs = self._processor(
            images=rgb,
            return_tensors="pt",
            size={"height": self.config.model_height, "width": self.config.model_width},
        )
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            predicted = self._model(**inputs).predicted_depth
        # The processor's own post-processing resizes back to the frame.
        resized = torch.nn.functional.interpolate(
            predicted.unsqueeze(1).float(),
            size=(height, width),
            mode="bilinear",
            align_corners=False,
        )
        return resized[0, 0].cpu().numpy().astype(np.float32)

    def fuse(self, rgb: NDArray[np.uint8], raw_depth_m: NDArray[np.float32]) -> Fusion:
        """Dense depth for one paired colour/depth frame, in meters. Both must
        be the same moment: a stale colour frame fills the holes with geometry
        from wherever the camera used to point, and it looks plausible."""
        fusion = fuse(self.predict(rgb), raw_depth_m, self.config, self._fit)
        self._fit = (fusion.a, fusion.b)
        return fusion
