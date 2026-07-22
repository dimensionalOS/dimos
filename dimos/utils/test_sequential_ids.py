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
from concurrent.futures import ThreadPoolExecutor

from dimos.utils.sequential_ids import SequentialIds


def test_starts_at_zero() -> None:
    ids = SequentialIds()
    assert ids.next() == 0


def test_increments_by_one() -> None:
    ids = SequentialIds()
    seen = [ids.next() for _ in range(5)]
    assert seen == [0, 1, 2, 3, 4]


def test_independent_instances() -> None:
    a = SequentialIds()
    b = SequentialIds()
    assert a.next() == 0
    assert b.next() == 0
    assert a.next() == 1


def test_concurrent_next_is_unique() -> None:
    ids = SequentialIds()
    n = 50

    def worker(_: int) -> int:
        return ids.next()

    with ThreadPoolExecutor(max_workers=n) as ex:
        out = list(ex.map(worker, range(n)))

    # thread-safe: every id in [0, n) appears exactly once
    assert sorted(out) == list(range(n))
    assert len(set(out)) == n
