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

"""Visualize which AprilTag glimpses pass/fail each gate in detect_apriltags.

Runs the exact gates from `apriltags.py` over a recording's color stream, crops
each detected tag, labels it with its metrics + verdict (kept, or the first gate
that rejected it), and writes a self-contained HTML gallery grouped by verdict.

    uv run python dimos/mapping/recording/scripts/demo_apriltag_gallery.py REC_DIR_OR_DB \
        --out /tmp/apriltag_gallery.html --stride 3 --per-category 16
"""

from __future__ import annotations

import argparse
import base64
from pathlib import Path

import cv2
import numpy as np

from dimos.mapping.recording.multi_map_anchor import _load_camera
from dimos.mapping.recording.utils import apriltags
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image

DB_NAME = "mem2.db"
CROP_MARGIN_FRAC = 0.6  # context around the tag, as a fraction of its bbox size
CROP_DISPLAY_PX = 220  # rendered crop width in the gallery
# verdict -> (label, blurb) in display order; "kept" first, then each reject reason.
VERDICTS = {
    "kept": ("KEPT", "passed every gate"),
    "blur": ("blur", f"tag-ROI sharpness < {apriltags.DEFAULT_MIN_SHARPNESS:g}"),
    "reproj": ("reproj", f"solvePnP RMS > {apriltags.DEFAULT_MAX_REPROJ_PX:g}px"),
    "small": ("small", f"tag side < {apriltags.DEFAULT_MIN_TAG_PX:g}px"),
    "far": ("far", f"distance > {apriltags.DEFAULT_MAX_DISTANCE_M:g}m"),
    "oblique": ("oblique", f"view angle > {apriltags.DEFAULT_MAX_VIEW_ANGLE_DEG:g}deg"),
    "motion": (
        "motion",
        f"speed > {apriltags.DEFAULT_MAX_LINEAR_SPEED_MPS:g}m/s "
        f"or {apriltags.DEFAULT_MAX_ANGULAR_SPEED_DPS:g}deg/s",
    ),
}


def _verdict(metrics: dict) -> str:
    """First gate (in detect_apriltags order) that rejects this glimpse, else 'kept'."""
    if metrics["sharpness"] < apriltags.DEFAULT_MIN_SHARPNESS:
        return "blur"
    if metrics["reproj_px"] > apriltags.DEFAULT_MAX_REPROJ_PX:
        return "reproj"
    if metrics["tag_px"] < apriltags.DEFAULT_MIN_TAG_PX:
        return "small"
    if metrics["distance"] > apriltags.DEFAULT_MAX_DISTANCE_M:
        return "far"
    if metrics["view_angle"] > apriltags.DEFAULT_MAX_VIEW_ANGLE_DEG:
        return "oblique"
    speed = metrics["speed"]
    if speed is not None and (
        speed[0] > apriltags.DEFAULT_MAX_LINEAR_SPEED_MPS
        or speed[1] > apriltags.DEFAULT_MAX_ANGULAR_SPEED_DPS
    ):
        return "motion"
    return "kept"


def _crop_png_b64(bgr: np.ndarray, corners: np.ndarray) -> str:
    quad = corners.reshape(4, 2)
    center = quad.mean(0)
    size = max(float((quad.max(0) - quad.min(0)).max()), 1.0)
    half = size * (0.5 + CROP_MARGIN_FRAC)
    x_min, y_min = int(center[0] - half), int(center[1] - half)
    x_max, y_max = int(center[0] + half), int(center[1] + half)
    height, width = bgr.shape[:2]
    x_min, y_min = max(x_min, 0), max(y_min, 0)
    x_max, y_max = min(x_max, width), min(y_max, height)
    crop = bgr[y_min:y_max, x_min:x_max].copy()
    if crop.size == 0:
        crop = bgr.copy()
        x_min, y_min = 0, 0
    shifted = (quad - [x_min, y_min]).astype(np.int32)
    cv2.polylines(crop, [shifted], isClosed=True, color=(0, 255, 0), thickness=2)
    if crop.shape[1] > CROP_DISPLAY_PX:
        scale = CROP_DISPLAY_PX / crop.shape[1]
        crop = cv2.resize(crop, (CROP_DISPLAY_PX, int(crop.shape[0] * scale)))
    ok, buffer = cv2.imencode(".png", crop)
    return base64.b64encode(buffer).decode() if ok else ""


def _collect(db: Path, stride: int, max_images: int) -> tuple[list[dict], int]:
    intrinsics, distortion, _optical_in_base, _resolution = _load_camera(db)
    detector = apriltags.make_detector("DICT_APRILTAG_36h11")
    examples: list[dict] = []
    with SqliteStore(path=str(db)) as store:
        images = store.stream("color_image", Image).to_list()
        speed_by_ts, _available = apriltags._camera_speeds(images)
        sampled = images[::stride][:max_images] if max_images else images[::stride]
        for image_obs in sampled:
            image = image_obs.data
            bgr = image.numpy() if hasattr(image, "numpy") else np.asarray(image.data)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
            all_corners, marker_ids, _ = detector.detectMarkers(bgr)
            if marker_ids is None:
                continue
            for corners, marker_id in zip(all_corners, marker_ids.flatten(), strict=False):
                pose = apriltags.estimate_marker_pose(corners, 0.10, intrinsics, distortion)
                if pose is None:
                    continue
                rotation_vector, translation_vector = pose
                quaternion = (
                    apriltags.Rotation.from_rotvec(rotation_vector.reshape(3)).as_quat().tolist()
                )
                tag_in_camera = [*translation_vector.reshape(3).tolist(), *quaternion]
                distance, view_angle = apriltags.view_quality(tag_in_camera)
                metrics = {
                    "marker_id": int(marker_id),
                    "ts": float(image_obs.ts),
                    "sharpness": apriltags.tag_sharpness(gray, corners),
                    "reproj_px": apriltags.reprojection_error_px(
                        corners, rotation_vector, translation_vector, 0.10, intrinsics, distortion
                    ),
                    "tag_px": apriltags.tag_pixel_size(corners),
                    "distance": distance,
                    "view_angle": view_angle,
                    "speed": speed_by_ts.get(float(image_obs.ts)),
                }
                metrics["verdict"] = _verdict(metrics)
                metrics["img"] = _crop_png_b64(bgr, corners)
                examples.append(metrics)
    return examples, len(images)


def _card_html(example: dict) -> str:
    speed = example["speed"]
    speed_text = f"{speed[0]:.2f}m/s {speed[1]:.0f}deg/s" if speed else "n/a"
    return (
        '<div class="card">'
        f'<img src="data:image/png;base64,{example["img"]}"/>'
        f'<div class="meta">id {example["marker_id"]}<br>'
        f"sharp {example['sharpness']:.0f} &middot; reproj {example['reproj_px']:.2f}px<br>"
        f"side {example['tag_px']:.0f}px &middot; {example['distance']:.2f}m "
        f"&middot; {example['view_angle']:.0f}deg<br>"
        f"speed {speed_text}</div></div>"
    )


def _write_html(examples: list[dict], per_category: int, out: Path, total_images: int) -> None:
    counts: dict[str, int] = {}
    for example in examples:
        counts[example["verdict"]] = counts.get(example["verdict"], 0) + 1
    sections = []
    for verdict, (label, blurb) in VERDICTS.items():
        bucket = [e for e in examples if e["verdict"] == verdict]
        if not bucket:
            continue
        shown = bucket[:: max(1, len(bucket) // per_category)][:per_category]
        cards = "".join(_card_html(example) for example in shown)
        sections.append(
            f'<h2 class="{verdict}">{label} '
            f'<span class="n">{counts[verdict]} total &mdash; {blurb}</span></h2>'
            f'<div class="grid">{cards}</div>'
        )
    style = (
        "body{background:#111;color:#ddd;font:13px system-ui;margin:24px}"
        "h1{font-size:18px}h2{margin-top:32px;border-bottom:1px solid #333;padding-bottom:6px}"
        "h2.kept{color:#5f5}h2 .n{font-weight:400;color:#888;font-size:12px}"
        ".grid{display:flex;flex-wrap:wrap;gap:12px}"
        f".card{{background:#1b1b1b;border:1px solid #2a2a2a;border-radius:6px;"
        f"padding:6px;width:{CROP_DISPLAY_PX}px}}"
        ".card img{width:100%;border-radius:4px;display:block}"
        ".meta{margin-top:5px;color:#aaa;line-height:1.45}"
    )
    summary = " &middot; ".join(f"{VERDICTS[v][0]} {counts[v]}" for v in VERDICTS if v in counts)
    out.write_text(
        f"<!doctype html><meta charset=utf8><title>AprilTag gates</title><style>{style}</style>"
        f"<h1>AprilTag gate gallery &mdash; {len(examples)} detections "
        f"over {total_images} images</h1><p>{summary}</p>" + "".join(sections)
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("recording", help="recording dir or a .db")
    parser.add_argument("--out", default="/tmp/apriltag_gallery.html", help="output HTML path")
    parser.add_argument("--stride", type=int, default=3, help="process every Nth image")
    parser.add_argument(
        "--max-images", type=int, default=0, help="cap images processed (0 = all sampled)"
    )
    parser.add_argument("--per-category", type=int, default=16, help="crops shown per verdict")
    args = parser.parse_args()

    target = Path(args.recording)
    db = target if target.suffix == ".db" else target / DB_NAME
    if not db.exists():
        raise SystemExit(f"no db at {target}")

    print(f">> scanning {db} (stride {args.stride}) ...")
    examples, total_images = _collect(db, args.stride, args.max_images)
    out = Path(args.out)
    _write_html(examples, args.per_category, out, total_images)
    print(f"   {len(examples)} detections -> {out}")


if __name__ == "__main__":
    main()
