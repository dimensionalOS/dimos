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

"""Semantic segments with a flat-plane gate: the noun channel of hyperspace.

A segmenter (SegFormer on ADE20K, 150 classes) labels every pixel. The
segmenter over-calls the structural classes (wall, floor, ceiling, door,
stairs) on things that merely look like them, so those labels only survive
where the depth says the surface really is flat: a plane fitted over two
window sizes must have an RMS residual under ``rms_max`` and no point further
than ``max_deviation``. Object classes are never gated.

Everything here is numpy except the model wrapper, and nothing does I/O.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
from scipy import ndimage

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.mapping.hyperspace.patches import Intrinsics
    from dimos.msgs.sensor_msgs.Image import Image

SEGFORMER_MODEL_NAME = "nvidia/segformer-b2-finetuned-ade-512-512"

# ADE20K classes whose label is only trusted on a flat surface.
STRUCTURAL_CLASSES = ("wall", "floor", "ceiling", "door", "stairs", "stairway", "road", "sidewalk")
# Label of a structural pixel the flatness gate rejected.
UNSURE = -1


@dataclass
class FlatnessConfig:
    # Box sizes, in pixels, of the two plane fits. Both must pass.
    windows: tuple[int, int] = (9, 21)
    # Plane residual RMS a flat surface may have, metres.
    rms_max: float = 0.015
    # Largest single deviation from the plane a flat surface may have, metres.
    max_deviation: float = 0.04
    # Stereo depth noise grows with depth squared (RealSense: ~2 mm at 1 m,
    # ~15 mm at 2.5 m, ~25 mm at 4 m over a 21 px window), so the limits grow
    # with it: rms_max becomes max(rms_max, noise_per_m2 * z^2) and
    # max_deviation the same with 2.5x the slope. 0 = fixed limits.
    noise_per_m2: float = 0.003
    # Fraction of a window that must have depth for the fit to count.
    min_valid: float = 0.6
    # Demote structural labels where there is no depth at all. Off: no depth
    # means no evidence, and the segmenter's word stands.
    demote_without_depth: bool = False


def points_from_depth(depth_m: NDArray[np.floating], camera: Intrinsics) -> NDArray[np.float64]:
    """``[H, W, 3]`` camera-frame points; NaN where depth is missing."""
    height, width = depth_m.shape
    us, vs = np.meshgrid(np.arange(width), np.arange(height))
    z = depth_m.astype(np.float64)
    invalid = ~(z > 0) | ~np.isfinite(z)
    z = np.where(invalid, np.nan, z)
    x = (us - camera.cx) / camera.fx * z
    y = (vs - camera.cy) / camera.fy * z
    return np.stack([x, y, z], axis=-1)


def _box_mean(values: NDArray[np.float64], size: int) -> NDArray[np.float64]:
    return ndimage.uniform_filter(values, size=size, mode="nearest")


def plane_residuals(
    points: NDArray[np.float64], window: int, min_valid: float
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.bool_]]:
    """Per pixel: RMS residual and largest deviation of the window's points from
    the best-fit plane through them, plus whether enough of the window had depth.

    The plane is the local covariance's smallest eigenvector (box-filtered
    moments, so the whole image costs a handful of convolutions). The largest
    deviation is the maximum filter of every point's distance to *its own*
    window's plane, a close stand-in for the exact per-window maximum.
    """
    valid = np.isfinite(points[..., 2])
    filled = np.where(valid[..., None], points, 0.0)
    count = _box_mean(valid.astype(np.float64), window)
    enough = count >= min_valid
    safe_count = np.maximum(count, 1e-9)
    mean = (
        np.stack([_box_mean(filled[..., i], window) for i in range(3)], axis=-1)
        / safe_count[..., None]
    )
    # Second moments E[p p^T] over valid points, then covariance.
    cov = np.empty((*points.shape[:2], 3, 3))
    for i in range(3):
        for j in range(i, 3):
            moment = _box_mean(filled[..., i] * filled[..., j], window) / safe_count
            cov[..., i, j] = cov[..., j, i] = moment - mean[..., i] * mean[..., j]
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    rms = np.sqrt(np.clip(eigenvalues[..., 0], 0.0, None))
    normal = eigenvectors[..., :, 0]
    deviation = np.abs(np.einsum("hwi,hwi->hw", points - mean, normal))
    deviation = np.where(valid, deviation, 0.0)
    # A 3x3 median first, so a single flying pixel does not condemn its whole window.
    deviation = ndimage.median_filter(deviation, size=3, mode="nearest")
    max_deviation = ndimage.maximum_filter(deviation, size=window, mode="nearest")
    return rms, max_deviation, enough & valid


def flatness_mask(
    depth_m: NDArray[np.floating], camera: Intrinsics, config: FlatnessConfig
) -> tuple[NDArray[np.bool_], NDArray[np.bool_]]:
    """(flat, judged): flat where the surface under a pixel is flat at every
    window size; judged where there was enough depth to say either way."""
    points = points_from_depth(depth_m, camera)
    z = np.nan_to_num(points[..., 2])
    rms_max = np.maximum(config.rms_max, config.noise_per_m2 * z**2)
    max_deviation = np.maximum(config.max_deviation, 2.5 * config.noise_per_m2 * z**2)
    flat = np.ones(depth_m.shape, dtype=bool)
    judged = np.ones(depth_m.shape, dtype=bool)
    for window in config.windows:
        rms, deviation, ok = plane_residuals(points, window, config.min_valid)
        judged &= ok
        flat &= ok & (rms < rms_max) & (deviation < max_deviation)
    return flat, judged


def apply_flatness_gate(
    labels: NDArray[np.integer],
    flat: NDArray[np.bool_],
    structural_ids: set[int],
    judged: NDArray[np.bool_] | None = None,
) -> NDArray[np.int16]:
    """Structural labels on non-flat pixels become UNSURE; everything else stays.
    With ``judged``, pixels the depth could not judge keep their label."""
    gated = labels.astype(np.int16)
    structural = np.isin(gated, list(structural_ids))
    demote = structural & ~flat
    if judged is not None:
        demote &= judged
    gated[demote] = UNSURE
    return gated


@dataclass
class Segment:
    label: int
    name: str
    confidence: float
    area: int
    bbox: tuple[int, int, int, int]  # x0, y0, x1, y1 (exclusive)
    # Of the segment's pixels: how many the depth judged, and how many of those were flat.
    depth_fraction: float
    flat_fraction: float
    # Row-major run-length encoding of the mask: alternating background/foreground run lengths.
    rle: list[int]


def rle_encode(mask: NDArray[np.bool_]) -> list[int]:
    flat = mask.ravel().astype(np.int8)
    changes = np.flatnonzero(np.diff(flat)) + 1
    starts = np.concatenate([[0], changes])
    ends = np.concatenate([changes, [flat.size]])
    runs = (ends - starts).tolist()
    # Convention: first run is background, so a mask starting with foreground gets a 0 first.
    return runs if flat[0] == 0 else [0, *runs]


def rle_decode(rle: list[int], shape: tuple[int, int]) -> NDArray[np.bool_]:
    out = np.zeros(shape[0] * shape[1], dtype=bool)
    position, foreground = 0, False
    for run in rle:
        if foreground:
            out[position : position + run] = True
        position += run
        foreground = not foreground
    return out.reshape(shape)


def segments_from_labels(
    labels: NDArray[np.integer],
    confidence: NDArray[np.floating],
    flat: NDArray[np.bool_],
    judged: NDArray[np.bool_],
    names: dict[int, str],
    min_area: int = 200,
) -> list[Segment]:
    """Connected components per class, largest first."""
    segments: list[Segment] = []
    for label in np.unique(labels):
        label = int(label)
        if label == UNSURE:
            continue
        components, count = ndimage.label(labels == label)
        for index in range(1, count + 1):
            mask = components == index
            area = int(mask.sum())
            if area < min_area:
                continue
            ys, xs = np.nonzero(mask)
            judged_pixels = int(judged[mask].sum())
            segments.append(
                Segment(
                    label=label,
                    name=names.get(label, str(label)),
                    confidence=float(confidence[mask].mean()),
                    area=area,
                    bbox=(int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1),
                    depth_fraction=judged_pixels / area,
                    flat_fraction=float((flat & judged)[mask].sum() / judged_pixels)
                    if judged_pixels
                    else 0.0,
                    rle=rle_encode(mask),
                )
            )
    segments.sort(key=lambda s: -s.area)
    return segments


def palette(count: int) -> NDArray[np.uint8]:
    """A fixed, well-spread colour per class id (golden-angle hues)."""
    hues = (np.arange(count) * 0.618033988749895) % 1.0
    colors = np.zeros((count, 3), dtype=np.uint8)
    for i, h in enumerate(hues):
        # HSV -> RGB with s=0.75, v=0.95
        s, v = 0.75, 0.95
        k = h * 6.0
        f = k - np.floor(k)
        p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
        r, g, b = [(v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q)][int(k) % 6]
        colors[i] = (int(r * 255), int(g * 255), int(b * 255))
    return colors


def overlay(
    rgb: NDArray[np.uint8],
    labels: NDArray[np.integer],
    colors: NDArray[np.uint8],
    alpha: float = 0.45,
) -> NDArray[np.uint8]:
    """Blend class colours over the image; UNSURE pixels get a dark grey."""
    paint = np.zeros_like(rgb)
    known = labels >= 0
    paint[known] = colors[np.clip(labels[known], 0, len(colors) - 1)]
    paint[~known] = (60, 60, 60)
    blended = rgb.astype(np.float32) * (1 - alpha) + paint.astype(np.float32) * alpha
    return blended.astype(np.uint8)


@dataclass
class SegmenterConfig:
    model_name: str = SEGFORMER_MODEL_NAME
    device: str = "cpu"
    flatness: FlatnessConfig = field(default_factory=FlatnessConfig)
    min_area: int = 200


class SegFormerSegmenter:
    """SegFormer (ADE20K) behind one method: pixels in, labels + confidence out."""

    def __init__(self, config: SegmenterConfig) -> None:
        import torch
        from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor

        self.config = config
        self.torch = torch
        self.model = (
            SegformerForSemanticSegmentation.from_pretrained(config.model_name)
            .eval()
            .to(config.device)
        )
        self.processor = SegformerImageProcessor.from_pretrained(config.model_name)
        self.names: dict[int, str] = {
            int(i): n.strip() for i, n in self.model.config.id2label.items()
        }
        self.structural_ids = {i for i, n in self.names.items() if n in STRUCTURAL_CLASSES}
        self.colors = palette(len(self.names))

    def segment(self, rgb: NDArray[np.uint8]) -> tuple[NDArray[np.int16], NDArray[np.float32]]:
        """Per-pixel class id and softmax confidence, at the image's resolution."""
        torch = self.torch
        with torch.inference_mode():
            inputs = self.processor(images=rgb, return_tensors="pt").to(self.config.device)
            logits = self.model(**inputs).logits
            logits = torch.nn.functional.interpolate(
                logits, size=rgb.shape[:2], mode="bilinear", align_corners=False
            )
            probabilities = logits.softmax(dim=1)
            confidence, labels = probabilities.max(dim=1)
        return labels[0].to(torch.int16).cpu().numpy(), confidence[0].float().cpu().numpy()


@dataclass
class FrameSegments:
    labels: NDArray[np.int16]  # after the flatness gate; UNSURE where demoted
    confidence: NDArray[np.float32]
    flat: NDArray[np.bool_]
    judged: NDArray[np.bool_]  # where the depth was good enough to judge flatness
    segments: list[Segment]
    demoted_fraction: float


def segment_frame(
    segmenter: SegFormerSegmenter,
    rgb: NDArray[np.uint8],
    depth_m: NDArray[np.floating] | None,
    camera: Intrinsics | None,
) -> FrameSegments:
    """Label a frame and gate the structural classes by flatness (when depth exists)."""
    labels, confidence = segmenter.segment(rgb)
    flatness = segmenter.config.flatness
    if depth_m is not None and camera is not None:
        flat, judged = flatness_mask(depth_m, camera, flatness)
        gated = apply_flatness_gate(
            labels,
            flat,
            segmenter.structural_ids,
            None if flatness.demote_without_depth else judged,
        )
    else:
        flat = np.zeros(labels.shape, dtype=bool)
        judged = np.zeros(labels.shape, dtype=bool)
        gated = labels.astype(np.int16)
    demoted = float((gated == UNSURE).mean())
    segments = segments_from_labels(
        gated, confidence, flat, judged, segmenter.names, segmenter.config.min_area
    )
    return FrameSegments(
        labels=gated,
        confidence=confidence,
        flat=flat,
        judged=judged,
        segments=segments,
        demoted_fraction=demoted,
    )


def segment_record(
    segment: Segment, *, camera_frame: str, ts: float, width: int, height: int
) -> dict[str, Any]:
    """The memory-store payload for one segment."""
    return {
        "camera_frame": camera_frame,
        "ts": ts,
        "width": width,
        "height": height,
        "label": segment.label,
        "name": segment.name,
        "confidence": segment.confidence,
        "area": segment.area,
        "bbox": list(segment.bbox),
        "depth_fraction": segment.depth_fraction,
        "flat_fraction": segment.flat_fraction,
        "rle": segment.rle,
    }


def image_rgb(image: Image) -> NDArray[np.uint8]:
    return np.ascontiguousarray(np.asarray(image.to_rgb().data))
