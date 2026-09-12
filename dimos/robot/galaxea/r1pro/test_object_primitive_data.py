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

"""Primitive views must not leak placement actions into pick training."""

import json

import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.object_packing import OBJECT_PACKING_IO
from dimos.robot.galaxea.r1pro.object_primitive_data import (
    episode_frame_slice,
    segment_demonstrations,
)


def test_primitive_views_share_unchanged_recordings_with_disjoint_actions(tmp_path):
    source = tmp_path / "original"
    source.mkdir()
    phases = np.array(
        [
            "above",
            "approach",
            "grasp",
            "lift",
            "lift",
            "clear_sources",
            "release",
            "retreat",
            "settle",
        ]
    )
    recording = source / "episode.npz"
    np.savez(recording, phase=phases, action=np.arange(len(phases)))
    manifest = {
        "profile": OBJECT_PACKING_IO.name,
        "images": True,
        "episodes": [{"file": recording.name, "frames": len(phases), "success": True}],
    }
    (source / "manifest.json").write_text(json.dumps(manifest))
    original = recording.read_bytes()
    output = tmp_path / "views"
    report = segment_demonstrations(source, output)
    pick = json.loads((output / "pick/manifest.json").read_text())["episodes"][0]
    place = json.loads((output / "place/manifest.json").read_text())["episodes"][0]
    with np.load(recording) as data:
        np.testing.assert_array_equal(
            data["action"][episode_frame_slice(pick, len(phases))], [0, 1, 2, 3, 4]
        )
        np.testing.assert_array_equal(
            data["action"][episode_frame_slice(place, len(phases))], [5, 6, 7, 8]
        )
    assert pick["file"] == place["file"] == str(recording)
    assert recording.read_bytes() == original
    assert report["new_demonstrations"] == 0


@pytest.mark.parametrize(
    "row",
    [
        {"frames": 3, "frame_start": -1, "frame_stop": 2},
        {"frames": 3, "frame_start": 1, "frame_stop": 5},
        {"frames": 2, "frame_start": 3, "frame_stop": 5},
    ],
)
def test_invalid_frame_views_are_rejected(row):
    with pytest.raises(ValueError):
        episode_frame_slice(row, 4)
