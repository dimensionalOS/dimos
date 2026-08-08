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

"""Reproducible row sampling for suites whose rows come in fixed-size groups."""

from __future__ import annotations

import random

DEFAULT_GROUP_SIZE = 4


def sample_group_rows(
    total_rows: int,
    *,
    group_size: int = DEFAULT_GROUP_SIZE,
    groups: int,
    seed: int,
) -> list[int]:
    """Draw whole groups of consecutive rows and return their upstream row indices.

    Suites repeat one stimulus across `group_size` consecutive rows to balance
    where the right answer sits, so a subset that splits a group grades a
    stimulus on some of its variants and inherits that placement bias. Groups
    come back in the order they were drawn; the rows within each group stay in
    upstream order.
    """
    if group_size < 1:
        raise ValueError(f"group_size must be at least 1, got {group_size}")
    if total_rows < 1:
        raise ValueError(f"total_rows must be at least 1, got {total_rows}")
    if total_rows % group_size:
        raise ValueError(f"total_rows {total_rows} is not a multiple of group_size {group_size}")
    available = total_rows // group_size
    if groups < 1:
        raise ValueError(f"groups must be at least 1, got {groups}")
    if groups > available:
        raise ValueError(f"cannot draw {groups} groups from the {available} available")
    drawn = random.Random(seed).sample(range(available), groups)
    return [group * group_size + offset for group in drawn for offset in range(group_size)]
