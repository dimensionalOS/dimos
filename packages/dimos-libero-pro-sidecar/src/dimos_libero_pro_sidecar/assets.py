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

"""Explicit LIBERO-PRO asset bootstrap command.

This module intentionally does not run during sidecar import or health checks.
Call it directly when local benchmark assets should be downloaded/staged.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos_libero_pro_sidecar.server import (
    LiberoProRuntimeConfig,
    bootstrap_assets,
    validate_assets,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    bootstrap = subparsers.add_parser("bootstrap", help="Download and validate LIBERO-PRO assets.")
    bootstrap.add_argument("--benchmark-name", required=True)
    bootstrap.add_argument("--bddl-root", type=Path, required=True)
    bootstrap.add_argument("--init-states-root", type=Path, required=True)
    bootstrap.add_argument("--task-index", type=int, default=0)
    args = parser.parse_args()

    config = LiberoProRuntimeConfig(
        host="127.0.0.1",
        port=0,
        benchmark_name=args.benchmark_name,
        bddl_root=args.bddl_root,
        init_states_root=args.init_states_root,
        task_index=args.task_index,
    )
    bootstrap_assets(config)
    validate_assets(config)


if __name__ == "__main__":
    main()
