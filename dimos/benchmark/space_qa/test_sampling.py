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

"""What a sampled subset owes the suite it was drawn from."""

import pytest

from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE, sample_group_rows

GOLDEN = [40, 41, 42, 43, 16, 17, 18, 19, 48, 49, 50, 51]


def test_a_seed_pins_the_exact_rows_it_draws() -> None:
    assert sample_group_rows(80, groups=3, seed=7) == GOLDEN


def test_the_same_arguments_always_draw_the_same_rows() -> None:
    assert sample_group_rows(80, groups=3, seed=7) == sample_group_rows(80, groups=3, seed=7)


def test_a_different_seed_draws_different_rows() -> None:
    assert sample_group_rows(20, groups=2, seed=0) != sample_group_rows(20, groups=2, seed=1)


def test_every_drawn_group_arrives_whole_and_in_upstream_order() -> None:
    rows = sample_group_rows(400, groups=25, seed=11)
    assert len(rows) == 25 * DEFAULT_GROUP_SIZE
    assert len(set(rows)) == len(rows)
    assert all(0 <= row < 400 for row in rows)
    for start in range(0, len(rows), DEFAULT_GROUP_SIZE):
        group = rows[start : start + DEFAULT_GROUP_SIZE]
        assert group[0] % DEFAULT_GROUP_SIZE == 0
        assert group == [group[0] + offset for offset in range(DEFAULT_GROUP_SIZE)]


def test_drawing_every_group_returns_the_whole_suite() -> None:
    assert sorted(sample_group_rows(40, groups=10, seed=5)) == list(range(40))


def test_a_group_size_of_one_degrades_to_plain_row_sampling() -> None:
    rows = sample_group_rows(10, group_size=1, groups=4, seed=2)
    assert len(rows) == 4
    assert len(set(rows)) == 4


def test_rows_that_do_not_divide_into_groups_are_refused() -> None:
    with pytest.raises(ValueError, match="not a multiple of group_size"):
        sample_group_rows(81, groups=3, seed=7)


def test_asking_for_more_groups_than_exist_is_refused() -> None:
    with pytest.raises(ValueError, match="cannot draw 21 groups from the 20 available"):
        sample_group_rows(80, groups=21, seed=7)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"total_rows": 0, "groups": 1, "seed": 0}, "total_rows must be at least 1"),
        ({"total_rows": 8, "groups": 0, "seed": 0}, "groups must be at least 1"),
        ({"total_rows": 8, "groups": 1, "seed": 0, "group_size": 0}, "group_size must be at least"),
    ],
)
def test_degenerate_sizes_are_refused(kwargs: dict[str, int], message: str) -> None:
    total_rows = kwargs.pop("total_rows")
    with pytest.raises(ValueError, match=message):
        sample_group_rows(total_rows, **kwargs)
