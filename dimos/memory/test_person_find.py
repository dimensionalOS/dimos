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

from __future__ import annotations

from types import SimpleNamespace
from typing import cast

from dimos.memory.person_find import _CandidateManager
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


def _candidate(ts: float) -> Detection2DBBox:
    return cast("Detection2DBBox", SimpleNamespace(ts=ts))


def test_candidate_manager_preserves_external_similarity_order() -> None:
    ranked = sorted(
        [
            (0.6, _candidate(2.0)),
            (0.9, _candidate(1.0)),
            (0.8, _candidate(3.0)),
        ],
        key=lambda item: item[0],
        reverse=True,
    )
    manager = _CandidateManager([candidate for _, candidate in ranked])

    ordered = [candidate.ts for candidate in manager.candidates]

    assert ordered == [1.0, 3.0, 2.0]


def test_candidate_manager_pops_next_candidate() -> None:
    manager = _CandidateManager(
        [
            _candidate(1.0),
            _candidate(2.0),
        ]
    )

    candidate = manager.pop_candidate()

    assert candidate.ts == 1.0
    assert [candidate.ts for candidate in manager.candidates] == [2.0]


def test_candidate_manager_drops_each_covered_window_in_order() -> None:
    manager = _CandidateManager(
        [
            _candidate(9.9),
            _candidate(4.9),
            _candidate(7.0),
            _candidate(10.0),
            _candidate(2.0),
            _candidate(5.0),
            _candidate(6.2),
        ]
    )

    # Apply the first accepted window.
    assert manager.pop_candidate().ts == 9.9
    manager.drop_by_window(7.0, 10.0)
    after_first_window = [candidate.ts for candidate in manager.candidates]

    # Apply the next accepted window.
    assert manager.pop_candidate().ts == 4.9
    manager.drop_by_window(2.0, 5.0)
    after_second_window = [candidate.ts for candidate in manager.candidates]

    assert after_first_window == [4.9, 2.0, 5.0, 6.2]
    assert after_second_window == [6.2]
