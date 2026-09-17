"""Shared state/action normalization and image preprocessing."""

import json
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F


IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)


def parse_norm_stats(raw):
    stats = raw.get("norm_stats", raw)
    if "state" not in stats and "actions" not in stats:
        key = "xdof" if "xdof" in stats else next(iter(stats))
        stats = stats[key]
    return {
        key: {k: np.asarray(v, dtype=np.float32) for k, v in stats[key].items()}
        for key in ("state", "actions")
    }


def load_norm_stats(path):
    return parse_norm_stats(json.loads(Path(path).read_text()))


def normalize(x, stats):
    return (x - stats["mean"]) / (stats["std"] + 1e-6)


def unnormalize(x, stats):
    return x * (stats["std"] + 1e-6) + stats["mean"]


def resize_with_pad(img_hwc, target_h=224, target_w=224):
    h, w, _ = img_hwc.shape
    if (h, w) == (target_h, target_w):
        return img_hwc
    ratio = max(w / target_w, h / target_h)
    new_h = max(1, int(round(h / ratio)))
    new_w = max(1, int(round(w / ratio)))
    resized = F.interpolate(
        img_hwc.permute(2, 0, 1).unsqueeze(0),
        size=(new_h, new_w),
        mode="bilinear",
        align_corners=False,
        antialias=True,
    ).squeeze(0)
    pad_h0 = (target_h - new_h) // 2
    pad_h1 = target_h - new_h - pad_h0
    pad_w0 = (target_w - new_w) // 2
    pad_w1 = target_w - new_w - pad_w0
    padded = F.pad(resized, (pad_w0, pad_w1, pad_h0, pad_h1), value=0)
    return padded.permute(1, 2, 0)


def imagenet_normalize(img_chw):
    mean = IMAGENET_MEAN.to(device=img_chw.device, dtype=img_chw.dtype)
    std = IMAGENET_STD.to(device=img_chw.device, dtype=img_chw.dtype)
    return (img_chw - mean) / (std + 1e-6)


def resize_pad_normalize(img_chw, target_h=224, target_w=224):
    x = torch.as_tensor(img_chw).float()
    if x.max() > 1.0:
        x = x / 255.0
    x = resize_with_pad(x.permute(1, 2, 0), target_h, target_w).permute(2, 0, 1)
    return imagenet_normalize(x)
