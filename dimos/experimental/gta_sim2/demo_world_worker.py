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

"""Owned world compiler with bounded decoded geometry and compiled-model caches."""

import argparse
from collections import OrderedDict
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Any

import mujoco
import numpy as np

from dimos.experimental.gta_sim2.streaming import collision_base
from dimos.experimental.gta_sim2.world_compile import compile_world_window
from dimos.experimental.gta_sim2.world_geometry import WorldCatalog
from dimos.experimental.gta_sim2.world_objects import ObjectStateStore
from dimos.robot.assets.model import UsdModel


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("scene", "catalog", "directory", "state"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--digest", required=True)
    parser.add_argument("--dt-ns", type=int, required=True)
    parser.add_argument("--gravity", nargs=3, type=float, required=True)
    parser.add_argument("--delay-s", type=float, default=0)
    args = parser.parse_args()
    catalog = WorldCatalog(args.catalog)
    if catalog.digest != args.digest:
        raise ValueError("Compiler catalog identity mismatch")
    cache: OrderedDict[str, tuple[dict[str, Any], int]] = OrderedDict()
    hits, misses, evictions = 0, 0, 0
    args.directory.mkdir(parents=True, exist_ok=True)
    try:
        base = collision_base(UsdModel.from_file(args.scene), args.dt_ns, tuple(args.gravity))
        for line in sys.stdin:
            request = json.loads(line)
            try:
                center = np.asarray(request["center"], dtype=np.float64)
                pinned = request.get("pinned", {})
                key_data = {
                    "center": center.tolist(),
                    "half_extent": request["half_extent"],
                    "generation": ObjectStateStore.generation(args.state),
                    "pinned": sorted(
                        (
                            identity,
                            (np.floor(np.asarray(state["position"]) / 4 + 0.5) * 4).tolist(),
                            state["radius"],
                        )
                        for identity, state in pinned.items()
                    ),
                }
                key = hashlib.sha256(json.dumps(key_data, sort_keys=True).encode()).hexdigest()[:24]
                hit = key in cache
                if hit:
                    result, size = cache.pop(key)
                    hits += 1
                else:
                    if args.delay_s:
                        time.sleep(args.delay_s)
                    model, _data, indices, metadata = compile_world_window(
                        base, catalog, center, request["half_extent"], args.state, pinned
                    )
                    path = args.directory / (key + ".mjb")
                    index_path = path.with_suffix(".npy")
                    mujoco.mj_saveModel(model, str(path), None)
                    np.save(index_path, indices)
                    result = {
                        "window": str(path),
                        "indices_path": str(index_path),
                        "metadata": metadata,
                    }
                    size = path.stat().st_size + index_path.stat().st_size
                    if size > 512 << 20:
                        path.unlink(missing_ok=True)
                        index_path.unlink(missing_ok=True)
                        raise MemoryError("Compiled model exceeds cache budget")
                    while cache and (
                        len(cache) >= 8
                        or sum(entry[1] for entry in cache.values()) + size > 512 << 20
                    ):
                        _, (old, _) = cache.popitem(last=False)
                        Path(old["window"]).unlink(missing_ok=True)
                        Path(old["indices_path"]).unlink(missing_ok=True)
                        evictions += 1
                    misses += 1
                cache[key] = (result, size)
                states = ObjectStateStore.read(args.state, result["metadata"].get("actors", {}))
                states.update(pinned)
                # MJB caches geometry; actor coordinates are restored from current persistent state.
                response = {
                    **result,
                    "states": states,
                    "cache_hit": hit,
                    "compiled_cache": {
                        "hits": hits,
                        "misses": misses,
                        "evictions": evictions,
                        "entries": len(cache),
                        "bytes": sum(entry[1] for entry in cache.values()),
                    },
                }
                print(json.dumps(response), flush=True)
            except Exception as error:
                print(json.dumps({"window": "", "error": repr(error)}), flush=True)
    finally:
        catalog.close()


if __name__ == "__main__":
    main()
