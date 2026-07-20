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

import pytest

from dimos.utils.cli.dtop import ResourceSpyApp, _compute_ranges


@pytest.mark.parametrize(
    ("panel_pss", "expected"),
    [
        pytest.param((0, 32 * 1048576), "PSS 0.0 MB", id="pss"),
        pytest.param((0, 0), "RSS 64.0 MB", id="rss-fallback"),
    ],
)
def test_make_lines_selects_one_memory_metric_for_panel(
    panel_pss: tuple[int, int], expected: str
) -> None:
    data = [{"pss": pss, "rss": 64 * 1048576} for pss in panel_pss]
    line1, _ = ResourceSpyApp._make_lines(data[0], stale=False, ranges=_compute_ranges(data))
    assert expected in line1.plain
