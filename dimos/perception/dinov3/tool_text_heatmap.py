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

"""Text-query heatmaps at DINOv3 resolution: SigLIP grounds, DINOv3 sharpens.

SigLIP 2 scores a text query on its coarse patch grid (9x16 at the module
default); DINOv3 features then redistribute those scores onto its much finer
grid (30x53 at 848x480): each fine patch takes a similarity-weighted average
of the coarse scores, where the weights compare the patch's DINOv3 feature
with DINOv3 prototypes of the coarse cells. Same-object patches share score
regardless of distance, so edges come from DINOv3 while the text grounding
stays SigLIP's.

Example:
    python -m dimos.perception.dinov3.tool_text_heatmap \
        --db ~/datasets/d455/sf_office1_2/sf_office1_2.db \
        --t-offsets 27.6 40.7 73.8 91.5 156.9 190.2 \
        --queries fridge --out /tmp/dinov3_text
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
from dimos.models.embedding.siglip2 import SigLIP2Model
from dimos.perception.hyperspace.query import DEFAULT_BACKGROUND_PROMPTS
from dimos.perception.hyperspace.render import normalize_scores
from dimos.perception.hyperspace.tool_debug_frames import (
    contact_sheet,
    fetch_images,
    overlay_heatmap,
    patch_scores,
)
from dimos.perception.siglip2.module import SigLIP2ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def refine_scores(
    coarse: NDArray[np.float32],
    fine_features: NDArray[np.float32],
    temperature: float = 0.07,
    top_k: int = 12,
) -> NDArray[np.float32]:
    """Lift a coarse score grid onto the fine DINOv3 grid.

    Prototypes are the mean DINOv3 feature of each coarse cell; every fine
    patch softmax-attends (over the ``top_k`` most similar prototypes) and
    averages their coarse scores. Non-local on purpose: patches of the same
    object share score even when far apart in the image.
    """
    fine_h, fine_w, dim = fine_features.shape
    coarse_h, coarse_w = coarse.shape

    rows = (np.arange(fine_h) * coarse_h) // fine_h
    cols = (np.arange(fine_w) * coarse_w) // fine_w
    cell = (rows[:, None] * coarse_w + cols[None, :]).reshape(-1)

    flat = fine_features.reshape(-1, dim)
    prototypes = np.zeros((coarse_h * coarse_w, dim), dtype=np.float64)
    np.add.at(prototypes, cell, flat)
    norms = np.linalg.norm(prototypes, axis=1, keepdims=True)
    prototypes = (prototypes / np.maximum(norms, 1e-8)).astype(np.float32)

    similarity = flat @ prototypes.T  # (fine, coarse)
    if top_k < similarity.shape[1]:
        cutoff = np.partition(similarity, -top_k, axis=1)[:, -top_k, None]
        similarity = np.where(similarity >= cutoff, similarity, -np.inf)
    weights = np.exp((similarity - similarity.max(axis=1, keepdims=True)) / temperature)
    weights /= weights.sum(axis=1, keepdims=True)
    refined = weights @ coarse.reshape(-1)
    return np.asarray(refined.reshape(fine_h, fine_w), dtype=np.float32)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", required=True, help="Recording .db path")
    parser.add_argument(
        "--t-offsets", nargs="+", type=float, required=True, help="Seconds from recording start"
    )
    parser.add_argument("--queries", nargs="+", default=["fridge"], help="Text queries")
    parser.add_argument("--siglip-model", default=None, help="SigLIP 2 checkpoint override")
    parser.add_argument("--dinov3-model", default=None, help="timm DINOv3 model name override")
    parser.add_argument("--temperature", type=float, default=0.07, help="Refinement softmax temp")
    parser.add_argument("--top-k", type=int, default=12, help="Coarse cells each fine patch sees")
    parser.add_argument(
        "--side-by-side", action="store_true", help="Also tile the raw SigLIP heat next to hybrid"
    )
    parser.add_argument("--out", default="/tmp/dinov3_text", help="Artifact output directory")
    args = parser.parse_args(argv)

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    db_path = str(Path(args.db).expanduser())

    store = SqliteStore(path=db_path)
    t0 = store.stream("realsense_color_image").first().ts
    store.stop()
    targets = {offset: t0 + offset for offset in args.t_offsets}
    images = fetch_images(db_path, list(targets.values()))
    missing = [o for o, ts in targets.items() if ts not in images]
    if missing:
        raise RuntimeError(f"no color image near t offsets {missing}")

    # Match the live module defaults so results transfer to pipeline runs.
    module_defaults = SigLIP2ModuleConfig()
    siglip = SigLIP2Model(
        model_name=args.siglip_model or module_defaults.model_name,
        max_num_patches=module_defaults.max_num_patches,
    )
    dinov3_kwargs = {"model_name": args.dinov3_model} if args.dinov3_model else {}
    dinov3 = DINOv3Model(**dinov3_kwargs)

    fine_grids = {
        offset: np.asarray(
            dinov3.embed_patches(images[targets[offset]]).to_numpy(), dtype=np.float32
        )
        for offset in args.t_offsets
    }
    logger.info("dinov3 grids: %s", fine_grids[args.t_offsets[0]].shape)

    for query in args.queries:
        slug = query.replace(" ", "_")
        coarse_grids = {}
        refined = {}
        for offset in args.t_offsets:
            coarse = patch_scores(
                siglip, images[targets[offset]], query, DEFAULT_BACKGROUND_PROMPTS
            )
            coarse_grids[offset] = coarse
            refined[offset] = refine_scores(
                coarse, fine_grids[offset], args.temperature, args.top_k
            )

        stacked = np.stack([refined[o] for o in args.t_offsets])
        norm = normalize_scores(stacked.reshape(-1)).reshape(stacked.shape)
        coarse_stacked = np.stack([coarse_grids[o] for o in args.t_offsets])
        coarse_norm = normalize_scores(coarse_stacked.reshape(-1)).reshape(coarse_stacked.shape)

        tiles = []
        for i, offset in enumerate(args.t_offsets):
            rgb = images[targets[offset]].to_rgb().data
            caption = f"hybrid {query!r} #{i + 1}  t=+{offset:.1f}s"
            tile = overlay_heatmap(rgb, norm[i], caption)
            if args.side_by_side:
                raw = overlay_heatmap(rgb, coarse_norm[i], f"siglip {query!r}  t=+{offset:.1f}s")
                tile = np.hstack([raw, tile])
            tiles.append(tile)
        sheet_path = out_dir / f"{slug}_frames.png"
        cv2.imwrite(
            str(sheet_path),
            cv2.cvtColor(
                contact_sheet(tiles, columns=1 if args.side_by_side else 2), cv2.COLOR_RGB2BGR
            ),
        )
        logger.info("wrote %s", sheet_path)

    (out_dir / "run.json").write_text(
        json.dumps(
            {
                "siglip_model": siglip.config.model_name,
                "dinov3_model": dinov3.config.model_name,
                "queries": list(args.queries),
                "t_offsets": list(args.t_offsets),
                "temperature": args.temperature,
                "top_k": args.top_k,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
