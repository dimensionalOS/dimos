# Copyright 2025-2026 Dimensional Inc.
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

"""Ordering within an epoch, and staleness against a caller-supplied clock."""

from __future__ import annotations

import pickle

import pytest

from dimos.control.contract.sequence import Freshness, is_newer


def test_anything_is_newer_than_nothing() -> None:
    """The first frame from a source is always accepted."""
    assert is_newer(0, 0, None)
    assert is_newer(-5, -99, None)


@pytest.mark.parametrize(
    ("sequence", "newer"),
    [(6, True), (5, False), (4, False), (1000, True)],
)
def test_same_epoch_needs_a_strictly_greater_sequence(sequence: int, newer: bool) -> None:
    """A replayed or duplicated frame loses; only a later one wins."""
    assert is_newer(1, sequence, (1, 5)) is newer


def test_equal_sequence_is_not_newer() -> None:
    """The duplicate case, called out because it is the one that bites."""
    assert not is_newer(3, 77, (3, 77))


def test_a_higher_epoch_always_wins() -> None:
    """Sequences restart per epoch, so a new epoch beats any old sequence."""
    assert is_newer(2, 0, (1, 10_000))


def test_a_lower_epoch_never_wins() -> None:
    """A frame from a retired epoch loses even with a huge sequence."""
    assert not is_newer(1, 10_000, (2, 0))


def test_freshness_starts_stale() -> None:
    """A source that has never reported is stale, not fresh by default."""
    assert Freshness().is_stale(now=100.0, timeout_s=1.0)
    assert Freshness().age(now=100.0) is None


def test_freshness_after_a_receipt() -> None:
    """Marking a receipt makes a source fresh and gives it an age."""
    fresh = Freshness()
    fresh.mark(10.0)

    assert not fresh.is_stale(now=10.5, timeout_s=1.0)
    assert fresh.age(now=10.5) == pytest.approx(0.5)


def test_exactly_at_the_timeout_is_not_stale() -> None:
    """The boundary is inclusive, so a source running at its rate does not flap."""
    fresh = Freshness()
    fresh.mark(10.0)

    assert not fresh.is_stale(now=11.0, timeout_s=1.0)
    assert fresh.is_stale(now=11.000001, timeout_s=1.0)


def test_marking_again_refreshes() -> None:
    """A later receipt clears staleness."""
    fresh = Freshness()
    fresh.mark(10.0)
    assert fresh.is_stale(now=20.0, timeout_s=1.0)

    fresh.mark(19.8)

    assert not fresh.is_stale(now=20.0, timeout_s=1.0)


def test_freshness_pickles() -> None:
    """Freshness travels with a source snapshot over RPC."""
    fresh = Freshness()
    fresh.mark(42.0)

    assert pickle.loads(pickle.dumps(fresh)).last_receipt == 42.0
