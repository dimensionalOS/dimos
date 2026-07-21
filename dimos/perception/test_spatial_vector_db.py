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

import numpy as np
import pytest

from dimos.perception.spatial_vector_db import SpatialVectorDB

NEAR_FRAME_ID = "frame_near"
FAR_FRAME_ID = "frame_far"
NO_POS_FRAME_ID = "frame_no_pos"
NEAR_POS = (1.5, -2.0, 0.3)
FAR_POS = (30.0, 40.0, 0.0)


def _frame_metadata(frame_id: str, pos: tuple[float, float, float]) -> dict[str, float | str]:
    """Metadata with the keys SpatialMemory._process_frame stores for each frame."""
    return {
        "pos_x": pos[0],
        "pos_y": pos[1],
        "pos_z": pos[2],
        "rot_x": 0.0,
        "rot_y": 0.0,
        "rot_z": 0.0,
        "timestamp": 1700000000.0,
        "frame_id": frame_id,
    }


@pytest.fixture(scope="module")
def db() -> SpatialVectorDB:
    db = SpatialVectorDB(collection_name="test_spatial_vector_db")
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    db.add_image_vector(
        NEAR_FRAME_ID,
        image,
        np.array([1.0, 0.0]),
        _frame_metadata(NEAR_FRAME_ID, NEAR_POS),
    )
    db.add_image_vector(
        FAR_FRAME_ID,
        image,
        np.array([0.0, 1.0]),
        _frame_metadata(FAR_FRAME_ID, FAR_POS),
    )
    # Entry without position metadata — location queries must skip it, not crash.
    db.add_image_vector(
        NO_POS_FRAME_ID,
        image,
        np.array([1.0, 1.0]),
        {"frame_id": NO_POS_FRAME_ID, "timestamp": 1700000000.0},
    )
    return db


def test_query_by_location_returns_frames_within_radius(db: SpatialVectorDB) -> None:
    results = db.query_by_location(1.0, -2.0, radius=2.0)

    assert [r["id"] for r in results] == [NEAR_FRAME_ID]
    assert results[0]["metadata"]["pos_x"] == NEAR_POS[0]
    assert results[0]["metadata"]["pos_y"] == NEAR_POS[1]
    assert results[0]["distance"] == pytest.approx(0.5)


def test_query_by_location_skips_frames_without_position(db: SpatialVectorDB) -> None:
    results = db.query_by_location(1.0, -2.0, radius=1e9)

    assert [r["id"] for r in results] == [NEAR_FRAME_ID, FAR_FRAME_ID]


def test_get_all_locations_returns_stored_coordinates(db: SpatialVectorDB) -> None:
    assert sorted(db.get_all_locations()) == sorted([NEAR_POS, FAR_POS])
