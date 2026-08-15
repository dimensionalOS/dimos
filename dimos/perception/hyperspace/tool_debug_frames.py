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

"""Show the recording frames behind a query's hottest voxels.

Ranks frames by how much they contributed to the high-scoring voxels of a
text query (using the provenance saved in map.npz), pulls their color images
from the recording, and overlays the SigLIP 2 patch-level heatmap — what the
model actually responded to in each image.

Example:
    python -m dimos.perception.hyperspace.tool_debug_frames \
        --map /tmp/hyperspace/map.npz \
        --db ~/datasets/d455/sf_office1_2/sf_office1_2.db \
        --query fridge --out /tmp/hyperspace/debug
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import TYPE_CHECKING

import cv2
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.models.embedding.siglip2 import SigLIP2Model
from dimos.perception.hyperspace.query import DEFAULT_BACKGROUND_PROMPTS, score_query
from dimos.perception.hyperspace.render import normalize_scores, scores_to_colors
from dimos.perception.hyperspace.voxel_map import EmbeddingVoxelMap, keys_near_mask
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.sensor_msgs.Image import Image

logger = setup_logger()

#: A frame counts as matching a voxel-score at or above this normalized level.
HOT_THRESHOLD = 0.75
#: Widest tolerated gap between a ranked frame ts and a recorded color image.
IMAGE_MATCH_TOLERANCE_S = 0.2


def rank_frames(
    voxel_map: EmbeddingVoxelMap,
    scores: NDArray[np.float32],
    keep_mask: NDArray[np.bool_] | None,
    top_k: int,
    min_gap_s: float,
) -> list[tuple[float, float]]:
    """Pick the frames that contributed most to the hot voxels of a query.

    Returns up to ``top_k`` (frame_ts, contribution) tuples, best first,
    greedily enforcing ``min_gap_s`` between picks so near-duplicate frames
    of the same viewpoint don't crowd out other sightings.
    """
    extras = voxel_map.extras
    if "frame_ts" not in extras:
        raise RuntimeError("map.npz has no frame provenance — re-run tool_hyperspace")
    keys = voxel_map.voxel_keys()
    if keep_mask is not None:
        keys, scores = keys[keep_mask], scores[keep_mask]
    norm = normalize_scores(scores)
    hot = norm >= HOT_THRESHOLD
    hot_keys = keys[hot]
    hot_norm = norm[hot]
    logger.info("%d hot voxels above %.2f", int(hot.sum()), HOT_THRESHOLD)

    frame_ts = extras["frame_ts"]
    frame_keys = extras["frame_keys"]
    offsets = extras["frame_key_offsets"]
    contributions = np.zeros(frame_ts.size, dtype=np.float64)
    for i in range(frame_ts.size):
        keys_i = frame_keys[offsets[i] : offsets[i + 1]]
        idx = np.searchsorted(hot_keys, keys_i)
        idx = np.minimum(idx, max(hot_keys.size - 1, 0))
        matched = hot_keys.size > 0
        if matched:
            hit = hot_keys[idx] == keys_i
            contributions[i] = float(hot_norm[idx[hit]].sum())

    picked: list[tuple[float, float]] = []
    for i in np.argsort(-contributions):
        if contributions[i] <= 0:
            break
        ts = float(frame_ts[i])
        if any(abs(ts - prev) < min_gap_s for prev, _ in picked):
            continue
        picked.append((ts, float(contributions[i])))
        if len(picked) >= top_k:
            break
    return picked


def fetch_images(db_path: str, targets: list[float]) -> dict[float, Image]:
    """Nearest color image per target timestamp, in one pass over the db."""
    best: dict[float, tuple[float, Image]] = {}
    store = SqliteStore(path=db_path)
    try:
        for obs in store.stream("realsense_color_image"):
            for target in targets:
                gap = abs(obs.ts - target)
                if gap < IMAGE_MATCH_TOLERANCE_S and (target not in best or gap < best[target][0]):
                    best[target] = (gap, obs.data)
    finally:
        store.stop()
    return {ts: img for ts, (_, img) in best.items()}


def patch_scores(
    model: SigLIP2Model, image: Image, query: str, background_prompts: tuple[str, ...] | list[str]
) -> NDArray[np.float32]:
    """(grid_h, grid_w) query score per patch, background-contrasted like score_query()."""
    patches = model.embed_patches(image)
    grid = np.asarray(patches.to_numpy(), dtype=np.float32)  # (h, w, dim), normalized
    embeddings = model.embed_text(query, *background_prompts)
    if not isinstance(embeddings, list):
        embeddings = [embeddings]
    vectors = np.stack([e.to_numpy() for e in embeddings]).astype(np.float32)
    cosines = grid @ vectors.T  # (h, w, prompts)
    scores = cosines[:, :, 0]
    distinct = [i for i in range(1, vectors.shape[0]) if float(vectors[0] @ vectors[i]) < 0.85]
    if distinct:
        scores = scores - cosines[:, :, distinct].max(axis=2)
    return np.asarray(scores, dtype=np.float32)


def overlay_heatmap(
    rgb: NDArray[np.uint8], norm_grid: NDArray[np.float32], caption: str
) -> NDArray[np.uint8]:
    """Blend a normalized patch-score grid over the image, hot patches opaque."""
    h, w = rgb.shape[:2]
    norm = np.clip(cv2.resize(norm_grid, (w, h), interpolation=cv2.INTER_CUBIC), 0.0, 1.0).astype(
        np.float32
    )
    colors = scores_to_colors(norm.reshape(-1)).reshape(h, w, 3).astype(np.float32)
    alpha = (0.25 + 0.5 * norm)[:, :, None]
    blended = (rgb.astype(np.float32) * (1 - alpha) + colors * alpha).astype(np.uint8)
    bar_h = 28
    out = np.zeros((h + bar_h, w, 3), dtype=np.uint8)
    out[bar_h:] = blended
    cv2.putText(
        out, caption, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA
    )
    return out


def contact_sheet(tiles: list[NDArray[np.uint8]], columns: int = 2) -> NDArray[np.uint8]:
    """Pad tiles to a common size and lay them out in a grid."""
    h = max(t.shape[0] for t in tiles)
    w = max(t.shape[1] for t in tiles)
    padded = []
    for t in tiles:
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        canvas[: t.shape[0], : t.shape[1]] = t
        padded.append(canvas)
    while len(padded) % columns:
        padded.append(np.zeros((h, w, 3), dtype=np.uint8))
    rows = [np.hstack(padded[i : i + columns]) for i in range(0, len(padded), columns)]
    return np.vstack(rows)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, help="map.npz written by tool_hyperspace")
    parser.add_argument("--db", required=True, help="Recording .db the map was built from")
    parser.add_argument("--query", required=True, help="Text query, e.g. fridge")
    parser.add_argument("--top-frames", type=int, default=6)
    parser.add_argument("--min-gap-s", type=float, default=3.0, help="Min spacing between frames")
    parser.add_argument("--smooth", type=int, default=2, help="Voxel score smoothing iterations")
    parser.add_argument("--out", default=None, help="Output dir (default: map's directory)")
    args = parser.parse_args(argv)

    map_path = Path(args.map).expanduser()
    out_dir = Path(args.out).expanduser() if args.out else map_path.parent / "debug"
    out_dir.mkdir(parents=True, exist_ok=True)

    voxel_map = EmbeddingVoxelMap.load(str(map_path))
    logger.info(
        "loaded %s: %d voxels, aggregate=%s, extras=%s",
        map_path,
        voxel_map.voxel_count,
        voxel_map.aggregate,
        sorted(voxel_map.extras),
    )
    model_kwargs = {}
    if "model_name" in voxel_map.extras:
        model_kwargs["model_name"] = str(voxel_map.extras["model_name"])
        logger.info("using the map's model: %s", model_kwargs["model_name"])
    model = SigLIP2Model(**model_kwargs)
    _, scores = score_query(voxel_map, model, args.query, smooth_iterations=args.smooth)

    keep_mask = None
    if "lidar_keys" in voxel_map.extras:
        keep_mask = keys_near_mask(
            voxel_map.voxel_keys(),
            voxel_map.extras["lidar_keys"],
            radius=int(voxel_map.extras.get("lidar_dilation", 1)),
        )
        logger.info("lidar clean map keeps %d/%d voxels", int(keep_mask.sum()), keep_mask.size)

    picked = rank_frames(voxel_map, scores, keep_mask, args.top_frames, args.min_gap_s)
    if not picked:
        raise RuntimeError(f"no frames contributed to hot voxels for {args.query!r}")
    t0 = float(voxel_map.extras["frame_ts"].min())
    logger.info(
        "top frames for %r: %s",
        args.query,
        [(round(ts - t0, 1), round(c, 1)) for ts, c in picked],
    )

    images = fetch_images(str(Path(args.db).expanduser()), [ts for ts, _ in picked])

    # Score all frames first so the color scale is shared across them.
    grids: list[NDArray[np.float32]] = []
    frames: list[tuple[float, float, Image]] = []
    for ts, contribution in picked:
        if ts not in images:
            logger.warning("no color image within %.2fs of ts %.3f", IMAGE_MATCH_TOLERANCE_S, ts)
            continue
        grids.append(patch_scores(model, images[ts], args.query, DEFAULT_BACKGROUND_PROMPTS))
        frames.append((ts, contribution, images[ts]))
    stacked = np.stack(grids)
    norm = normalize_scores(stacked.reshape(-1)).reshape(stacked.shape)

    slug = args.query.replace(" ", "_")
    tiles = []
    report = []
    for rank, ((ts, contribution, image), norm_grid) in enumerate(
        zip(frames, norm, strict=True), start=1
    ):
        caption = f"{args.query!r} #{rank}  t=+{ts - t0:.1f}s  contribution={contribution:.1f}"
        tile = overlay_heatmap(image.to_rgb().data, norm_grid, caption)
        tiles.append(tile)
        path = out_dir / f"{slug}_frame{rank}.png"
        cv2.imwrite(str(path), cv2.cvtColor(tile, cv2.COLOR_RGB2BGR))
        report.append({"rank": rank, "ts": ts, "t_offset_s": ts - t0, "contribution": contribution})
        logger.info("wrote %s", path)

    sheet_path = out_dir / f"{slug}_frames.png"
    cv2.imwrite(str(sheet_path), cv2.cvtColor(contact_sheet(tiles), cv2.COLOR_RGB2BGR))
    (out_dir / f"{slug}_frames.json").write_text(json.dumps(report, indent=2))
    logger.info("wrote %s", sheet_path)


if __name__ == "__main__":
    main()
