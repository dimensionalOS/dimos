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

"""Re-run text queries against a saved voxel map — no replay needed.

Example:
    python -m dimos.perception.hyperspace.tool_requery \
        --map /tmp/hyperspace/map.npz --queries fridge --views top --rrd
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from dimos.models.embedding.siglip2 import SigLIP2Model
from dimos.perception.hyperspace.cluster import top_clusters
from dimos.perception.hyperspace.query import score_query
from dimos.perception.hyperspace.render import (
    VIEWS,
    normalize_scores,
    render_view,
    scores_to_colors,
)
from dimos.perception.hyperspace.voxel_map import EmbeddingVoxelMap, keys_near_mask
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, help="map.npz written by tool_hyperspace")
    parser.add_argument("--queries", nargs="+", default=["fridge"])
    parser.add_argument("--views", nargs="+", default=list(VIEWS))
    parser.add_argument("--out", default=None, help="Output dir (default: map's directory)")
    parser.add_argument("--rrd", action="store_true", help="Also save a .rrd per query")
    parser.add_argument("--smooth", type=int, default=2, help="Score smoothing iterations")
    args = parser.parse_args(argv)

    map_path = Path(args.map).expanduser()
    out_dir = Path(args.out).expanduser() if args.out else map_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    voxel_map = EmbeddingVoxelMap.load(str(map_path))
    logger.info("loaded %s: %d voxels, dim %d", map_path, voxel_map.voxel_count, voxel_map.dim)
    model_kwargs = {}
    if "model_name" in voxel_map.extras:
        model_kwargs["model_name"] = str(voxel_map.extras["model_name"])
        logger.info("using the map's text tower: %s", model_kwargs["model_name"])
    model = SigLIP2Model(**model_kwargs)

    # Same lidar clean-map filtering the live module applies, if it was saved.
    keep_mask = None
    if "lidar_keys" in voxel_map.extras:
        keep_mask = keys_near_mask(
            voxel_map.voxel_keys(),
            voxel_map.extras["lidar_keys"],
            radius=int(voxel_map.extras.get("lidar_dilation", 1)),
        )
        logger.info("lidar clean map keeps %d/%d voxels", int(keep_mask.sum()), keep_mask.size)

    for query in args.queries:
        centers, scores = score_query(voxel_map, model, query, smooth_iterations=args.smooth)
        if keep_mask is not None:
            centers, scores = centers[keep_mask], scores[keep_mask]
        slug = query.replace(" ", "_")
        for view in args.views:
            img = render_view(centers, scores, view=view, voxel_size=voxel_map.voxel_size)
            path = out_dir / f"{slug}_{view}.png"
            cv2.imwrite(str(path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            logger.info("wrote %s", path)
        clusters = top_clusters(centers, scores, voxel_size=voxel_map.voxel_size)
        (out_dir / f"{slug}_clusters.json").write_text(json.dumps(clusters, indent=2))
        logger.info("%s clusters: %s", query, clusters)
        if args.rrd:
            import rerun as rr

            rrd_path = out_dir / f"{slug}.rrd"
            recording = rr.RecordingStream(application_id="hyperspace")
            recording.save(str(rrd_path))
            recording.log(
                f"world/hyperspace/{slug}",
                rr.Points3D(
                    positions=centers,
                    colors=scores_to_colors(normalize_scores(scores)),
                    radii=voxel_map.voxel_size / 2,
                ),
                static=True,
            )
            recording.flush()
            logger.info("wrote %s", rrd_path)


if __name__ == "__main__":
    main()
