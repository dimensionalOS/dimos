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

"""DINOv3 patch heatmaps on recording frames, queried by an exemplar region.

DINOv3 has no text tower, so instead of a text prompt the query is an image
region: patches inside ``--exemplar`` (given as ``t_offset:x0,y0,x1,y1`` in
source-image pixels) are averaged into a query vector, and every frame's
patches are scored by cosine against it. Also writes a PCA sheet (first three
feature components as RGB) to visualize the raw dense-feature structure.

Example:
    python -m dimos.perception.dinov3.tool_frames \
        --db ~/datasets/d455/sf_office1_2/sf_office1_2.db \
        --t-offsets 27.6 40.7 75.0 91.5 156.9 190.2 \
        --exemplar "75.0:493,108,540,208" --label fridge --out /tmp/dinov3_frames
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.memory.store.sqlite import SqliteStore
from dimos.models.embedding.dinov3 import DINOv3Model
from dimos.perception.hyperspace.render import normalize_scores
from dimos.perception.hyperspace.tool_debug_frames import (
    contact_sheet,
    fetch_images,
    overlay_heatmap,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def exemplar_vector(
    grid: NDArray[np.float32], source_wh: tuple[int, int], box: tuple[int, int, int, int]
) -> NDArray[np.float32]:
    """Mean of the normalized patch vectors under a source-pixel box."""
    grid_h, grid_w = grid.shape[:2]
    width, height = source_wh
    x0, y0, x1, y1 = box
    c0 = int(np.clip(x0 * grid_w // width, 0, grid_w - 1))
    c1 = int(np.clip((x1 * grid_w + width - 1) // width, c0 + 1, grid_w))
    r0 = int(np.clip(y0 * grid_h // height, 0, grid_h - 1))
    r1 = int(np.clip((y1 * grid_h + height - 1) // height, r0 + 1, grid_h))
    vector = grid[r0:r1, c0:c1].reshape(-1, grid.shape[2]).mean(axis=0)
    return np.asarray(vector / max(float(np.linalg.norm(vector)), 1e-8), dtype=np.float32)


def pca_rgb(grids: list[NDArray[np.float32]]) -> list[NDArray[np.float32]]:
    """Project each patch grid onto the shared top-3 PCA components as [0,1] RGB."""
    stacked = np.concatenate([g.reshape(-1, g.shape[2]) for g in grids])
    centered = stacked - stacked.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    out: list[NDArray[np.float32]] = []
    lo, hi = None, None
    projected = [(g.reshape(-1, g.shape[2]) - stacked.mean(axis=0)) @ vt[:3].T for g in grids]
    all_proj = np.concatenate(projected)
    lo, hi = np.percentile(all_proj, [2.0, 98.0], axis=0)
    for proj, g in zip(projected, grids, strict=True):
        rgb = np.clip((proj - lo) / np.maximum(hi - lo, 1e-8), 0.0, 1.0)
        out.append(np.asarray(rgb.reshape(g.shape[0], g.shape[1], 3), dtype=np.float32))
    return out


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", required=True, help="Recording .db path")
    parser.add_argument(
        "--t-offsets", nargs="+", type=float, required=True, help="Seconds from recording start"
    )
    parser.add_argument(
        "--exemplar",
        required=True,
        help="Query region as t_offset:x0,y0,x1,y1 in source pixels",
    )
    parser.add_argument("--label", default="exemplar", help="Caption label for the query")
    parser.add_argument("--model", default=None, help="timm DINOv3 model name")
    parser.add_argument(
        "--input-size", default=None, help="Inference resolution as WxH, e.g. 848x480"
    )
    parser.add_argument("--out", default="/tmp/dinov3_frames", help="Artifact output directory")
    args = parser.parse_args(argv)

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    db_path = str(Path(args.db).expanduser())

    exemplar_t, box_str = args.exemplar.split(":")
    box = tuple(int(v) for v in box_str.split(","))
    assert len(box) == 4
    offsets = list(args.t_offsets)
    if float(exemplar_t) not in offsets:
        offsets.append(float(exemplar_t))

    store = SqliteStore(path=db_path)
    t0 = store.stream("realsense_color_image").first().ts
    store.stop()
    targets = {offset: t0 + offset for offset in offsets}
    images = fetch_images(db_path, list(targets.values()))
    missing = [o for o, ts in targets.items() if ts not in images]
    if missing:
        raise RuntimeError(f"no color image near t offsets {missing}")

    model_kwargs = {}
    if args.model:
        model_kwargs["model_name"] = args.model
    if args.input_size:
        w, h = (int(v) for v in args.input_size.split("x"))
        model_kwargs["input_size"] = (w, h)
    model = DINOv3Model(**model_kwargs)

    grids: dict[float, NDArray[np.float32]] = {}
    for offset in offsets:
        image = images[targets[offset]]
        patches = model.embed_patches(image)
        grids[offset] = np.asarray(patches.to_numpy(), dtype=np.float32)
    logger.info("embedded %d frames, grid %s", len(grids), grids[offsets[0]].shape)

    ref_image = images[targets[float(exemplar_t)]]
    query = exemplar_vector(grids[float(exemplar_t)], (ref_image.width, ref_image.height), box)

    scored = {o: grids[o] @ query for o in args.t_offsets}
    stacked = np.stack([scored[o] for o in args.t_offsets])
    norm = normalize_scores(stacked.reshape(-1), p_lo=50.0, p_hi=99.5).reshape(stacked.shape)

    tiles = []
    for i, offset in enumerate(args.t_offsets):
        rgb = images[targets[offset]].to_rgb().data
        caption = f"dinov3 {args.label!r} #{i + 1}  t=+{offset:.1f}s"
        tiles.append(overlay_heatmap(rgb, norm[i], caption))
    sheet = contact_sheet(tiles)
    sheet_path = out_dir / f"{args.label}_frames.png"
    cv2.imwrite(str(sheet_path), cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))
    logger.info("wrote %s", sheet_path)

    rgb_grids = pca_rgb([grids[o] for o in args.t_offsets])
    tiles = []
    for i, offset in enumerate(args.t_offsets):
        rgb = images[targets[offset]].to_rgb().data
        h, w = rgb.shape[:2]
        pca_img = cv2.resize(rgb_grids[i], (w, h), interpolation=cv2.INTER_NEAREST)
        blend = (rgb.astype(np.float32) * 0.35 + pca_img * 255.0 * 0.65).astype(np.uint8)
        bar = np.zeros((28, w, 3), dtype=np.uint8)
        cv2.putText(
            bar,
            f"dinov3 PCA #{i + 1}  t=+{offset:.1f}s",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        tiles.append(np.vstack([bar, blend]))
    pca_path = out_dir / "pca_frames.png"
    cv2.imwrite(str(pca_path), cv2.cvtColor(contact_sheet(tiles), cv2.COLOR_RGB2BGR))
    logger.info("wrote %s", pca_path)

    (out_dir / "run.json").write_text(
        json.dumps(
            {
                "model": model.config.model_name,
                "input_size": list(model.config.input_size),
                "exemplar": args.exemplar,
                "t_offsets": list(args.t_offsets),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
