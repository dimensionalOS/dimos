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

"""Command-line entrypoint for the four-question DimSim smoke corpus."""

import argparse
from pathlib import Path

from dimos.benchmark.dimsim.bundle import generate_smoke_release
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.oracle import (
    SceneClientOracleProvider,
    get_stable_scene_oracle_view,
)
from dimos.simulation.dimsim.scene_client import SceneClient


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--fixture",
        action="store_true",
        help="Use hermetic contract data instead of a running DimSim instance.",
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--snapshot-timeout-s", type=float, default=300.0)
    parser.add_argument("--stability-delay-s", type=float, default=1.0)
    args = parser.parse_args()

    if args.fixture:
        view = apartment_oracle_fixture()
    else:
        if args.snapshot_timeout_s <= 0:
            parser.error("--snapshot-timeout-s must be positive")
        if args.stability_delay_s < 0:
            parser.error("--stability-delay-s must be non-negative")
        client = SceneClient(
            host=args.host,
            port=args.port,
            timeout=args.snapshot_timeout_s,
        )
        client.start()
        try:
            view = get_stable_scene_oracle_view(
                SceneClientOracleProvider(client),
                stability_delay_s=args.stability_delay_s,
            )
        finally:
            client.stop()
    manifest = generate_smoke_release(view, args.output)
    print(f"Wrote {manifest.task_count} tasks to {args.output}")


if __name__ == "__main__":
    main()
