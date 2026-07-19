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

from dimos.utils.cli.dtop import select_memory_metric


@pytest.mark.parametrize(
    ("pss", "expected"),
    [
        pytest.param(32 * 1048576, ("PSS", "pss"), id="pss"),
        pytest.param(0, ("RSS", "rss"), id="rss-fallback"),
    ],
)
def test_select_memory_metric_uses_available_measure(pss: int, expected: tuple[str, str]) -> None:
    assert select_memory_metric({"pss": pss}) == expected
