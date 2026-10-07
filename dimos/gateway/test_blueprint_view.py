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

"""The blueprint view's graph layouts (blueprint_view/layout.js): no two nodes overlap, in every layout, for a small
and a crowded blueprint, with and without 20 extra live topics drawn. Runs layout_check.js under dimos's pinned Deno."""

import json
from pathlib import Path
import subprocess

from dimos.utils.deno import ensure_deno

CHECK = Path(__file__).parent / "blueprint_view" / "layout_check.js"


def test_no_layout_overlaps_nodes() -> None:
    out = subprocess.run(
        [ensure_deno(), "run", "--no-prompt", CHECK.name],
        cwd=CHECK.parent,
        capture_output=True,
        text=True,
        timeout=120,
        check=True,
    ).stdout
    results = json.loads(out)
    assert len(results) == 40
    assert any(r["vertical"] for r in results)
    assert [r for r in results if r["overlaps"]] == []
