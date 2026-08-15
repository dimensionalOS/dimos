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

"""Run the hyperspace replay blueprint over a recording and export artifacts.

Example:
    python -m dimos.perception.hyperspace.tool_hyperspace \
        --db ~/datasets/d455/sf_office1_2/sf_office1_2.db \
        --speed 2 --queries fridge robot floor --out /tmp/hyperspace
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.perception.hyperspace.module import HyperspaceModule
from dimos.perception.hyperspace.render import VIEWS
from dimos.perception.hyperspace.replay import D455ReplayModule
from dimos.perception.siglip2.module import SigLIP2Module
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", required=True, help="Recording .db path")
    parser.add_argument("--speed", type=float, default=2.0, help="Replay speed multiplier")
    parser.add_argument(
        "--run-seconds",
        type=float,
        default=0.0,
        help="Wall seconds to ingest; 0 = recording duration / speed + margin",
    )
    parser.add_argument("--queries", nargs="+", default=["fridge"], help="Text queries")
    parser.add_argument("--views", nargs="+", default=list(VIEWS), help="Heatmap views")
    parser.add_argument("--out", default="/tmp/hyperspace", help="Artifact output directory")
    parser.add_argument("--rrd", action="store_true", help="Also save a .rrd per query")
    parser.add_argument("--nearby", nargs="*", default=[], help="Queries for the nearby skill")
    parser.add_argument("--max-freq", type=float, default=2.0, help="SigLIP embed rate (Hz)")
    args = parser.parse_args(argv)

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    db_path = str(Path(args.db).expanduser())

    run_seconds = args.run_seconds
    if run_seconds <= 0:
        from dimos.memory.store.sqlite import SqliteStore

        store = SqliteStore(path=db_path)
        obs = store.stream("realsense_color_image")
        first = obs.first()
        last = obs.order_by("ts", desc=True).first()
        store.stop()
        run_seconds = (last.ts - first.ts) / args.speed + 20.0
        logger.info("recording spans %.0fs -> running %.0fs", last.ts - first.ts, run_seconds)

    blueprint = autoconnect(
        D455ReplayModule.blueprint(db_path=db_path, speed=args.speed),
        SigLIP2Module.blueprint(max_freq=args.max_freq),
        HyperspaceModule.blueprint(),
    )
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        hyperspace = coordinator.get_instance(HyperspaceModule)
        assert hyperspace is not None

        deadline = time.time() + run_seconds
        while time.time() < deadline:
            time.sleep(10)
            logger.info("stats: %s", hyperspace.stats())

        stats = hyperspace.stats()
        logger.info("final stats: %s", stats)
        if not stats["voxels"]:
            raise RuntimeError(f"no voxels ingested: {stats}")
        (out_dir / "stats.json").write_text(json.dumps(stats, indent=2))

        for query in args.queries:
            for view in args.views:
                image = hyperspace.query_heatmap(query, view)
                path = out_dir / f"{query.replace(' ', '_')}_{view}.png"
                cv2.imwrite(str(path), cv2.cvtColor(image.data, cv2.COLOR_RGB2BGR))
                logger.info("wrote %s", path)
            clusters = hyperspace.query_clusters(query)
            cluster_path = out_dir / f"{query.replace(' ', '_')}_clusters.json"
            cluster_path.write_text(json.dumps(clusters, indent=2))
            logger.info("%s clusters: %s", query, clusters)
            if args.rrd:
                rrd_path = out_dir / f"{query.replace(' ', '_')}.rrd"
                hyperspace.log_query_rerun(query, str(rrd_path))
                logger.info("wrote %s", rrd_path)

        for query in args.nearby:
            heading = hyperspace.nearby(query)
            (out_dir / f"nearby_{query.replace(' ', '_')}.json").write_text(
                json.dumps(heading, indent=2)
            )
            logger.info("nearby %r: %s", query, heading)
            image = hyperspace.nearby_heatmap(query)
            path = out_dir / f"nearby_{query.replace(' ', '_')}.png"
            cv2.imwrite(str(path), cv2.cvtColor(image.data, cv2.COLOR_RGB2BGR))
            logger.info("wrote %s", path)
    finally:
        coordinator.stop()


if __name__ == "__main__":
    main()
