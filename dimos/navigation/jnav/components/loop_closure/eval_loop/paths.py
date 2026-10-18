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

"""Shared paths for the eval_loop scripts."""

import os
from pathlib import Path

REPO = Path(__file__).resolve().parents[6]
# results, prepared dbs and maps all land here
OUT = Path(os.environ.get("EVAL_LOOP_OUT", "eval_loop_results")).resolve()
OUT.mkdir(parents=True, exist_ok=True)
PGO_EVAL = Path(
    os.environ.get(
        "PGO_EVAL",
        REPO / "dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust/target/release/pgo-eval",
    )
)


def import_ivan():
    """Ivan's go2 PGO (dimos/navigation/go2/loop_closure), merged in from main."""
    try:
        from dimos.navigation.go2.loop_closure import module, pgo, pgo_map
    except ImportError as error:
        raise SystemExit(
            "can't import dimos.navigation.go2.loop_closure; is the venv for this checkout active?\n"
            f"({error})"
        ) from None
    return module, pgo, pgo_map
