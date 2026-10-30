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

from dimos.evals.vqa.diversity import (
    DiversityCandidate,
    difficulty_for,
    select_diverse_case_ids,
)


def test_difficulty_prefers_harder_count_and_behind() -> None:
    assert difficulty_for("object_count", "four or more", ("chair",)) > difficulty_for(
        "object_count", "one", ("chair",)
    )
    assert difficulty_for("ego_fore_aft", "behind", ("nearest chair",)) > difficulty_for(
        "ego_fore_aft", "ahead", ("nearest chair",)
    )


def test_select_diverse_caps_category_and_balances_families() -> None:
    candidates = [
        DiversityCandidate("a1", "presence", "chair", 0.2, "yes"),
        DiversityCandidate("a2", "presence", "chair", 0.2, "yes"),
        DiversityCandidate("a3", "presence", "plant", 0.2, "yes"),
        DiversityCandidate("b1", "object_count", "chair", 1.0, "four or more"),
        DiversityCandidate("b2", "object_count", "desk", 0.5, "two"),
        DiversityCandidate("c1", "ego_fore_aft", "person", 1.0, "behind"),
        DiversityCandidate("c2", "ego_fore_aft", "person", 0.4, "ahead"),
    ]
    kept = select_diverse_case_ids(candidates, max_per_category=1, max_per_family_category=1)
    # chair appears in presence and object_count — category cap 1 keeps only one chair total
    chair_ids = {cid for cid in ("a1", "a2", "b1") if cid in kept}
    assert len(chair_ids) == 1
    assert "c1" in kept  # harder behind over ahead
    assert "c2" not in kept
    assert "b2" in kept
    assert "a3" in kept
