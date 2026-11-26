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

"""Private, lifecycle-owned compiler process; JSON pipes carry no robot actions."""

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

from dimos.experimental.gta_sim2.streaming import CollisionCatalog, collision_base, compile_window
from dimos.robot.assets.model import UsdModel


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--catalog-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dt-ns", type=int, required=True)
    parser.add_argument("--gravity", type=float, nargs=3, required=True)
    args = parser.parse_args()
    try:
        base = collision_base(UsdModel.from_file(args.scene), args.dt_ns, tuple(args.gravity))
        catalog = CollisionCatalog(args.catalog)
        if catalog.digest != args.catalog_sha256:
            raise ValueError("Collision catalog changed after physics initialization")
        for line in sys.stdin:
            request = json.loads(line)
            ready = compile_window(
                base,
                catalog,
                np.asarray(request["center"], dtype=np.float64),
                request["half_extent"],
            )
            mujoco.mj_saveModel(ready.model, str(args.output), None)
            print(
                json.dumps({"window": str(args.output), "indices": ready.indices.tolist()}),
                flush=True,
            )
    except Exception as error:
        print(json.dumps({"window": "", "error": repr(error)}), flush=True)
        raise


if __name__ == "__main__":
    main()
