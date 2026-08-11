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

import hashlib
import json
from pathlib import Path


def test_pinned_public_fixture_contains_selected_episode_layout() -> None:
    manifest_path = Path(__file__).with_name("upstream-manifest.v1.json")
    manifest = json.loads(manifest_path.read_text())
    episode = manifest["episode"]
    dataset = manifest["assets"]["r2r_vlnce_v1_3"]
    scene = manifest["assets"]["mp3d_example_v1_1"]

    assert episode["episode_id"] == "515"
    assert episode["scene_id"] == "mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"
    assert dataset["train_file"] == "R2R_VLNCE_v1-3/train/train.json.gz"
    assert scene["required_files"] == {
        "17DRP5sb8fy/17DRP5sb8fy.glb": (
            "334456925e056c83a9a7a5c768b3d37cdd23425d8ca20743bfce015be3f56b04"
        ),
        "17DRP5sb8fy/17DRP5sb8fy.navmesh": (
            "0f36abb98ee3545d7e1f5103882269206feaaea05dcb54014333290b1d2b00c8"
        ),
        "mp3d.scene_dataset_config.json": (
            "b56050135d429839ca7066f335de51897d27e9741810bff8e13972fe539b0f5f"
        ),
    }
    assert (
        hashlib.sha256(episode["instruction"].encode()).hexdigest() == episode["instruction_sha256"]
    )
    assert (
        episode["canonical_episode_sha256"]
        == "d5934ea7576ecbe96093aa6b7cc8c097081b651382898f7c7ce1410fbfd3740a"
    )


def test_manifest_records_exact_archive_sizes_and_source_revisions() -> None:
    manifest = json.loads(Path(__file__).with_name("upstream-manifest.v1.json").read_text())

    assert manifest["assets"]["r2r_vlnce_v1_3"]["archive_bytes"] == 2_608_469
    assert manifest["assets"]["mp3d_example_v1_1"]["archive_bytes"] == 67_765_004
    assert manifest["upstream"] == {
        "vlnce": {
            "repository": "https://github.com/jacobkrantz/VLN-CE",
            "revision": "729d141b2ee10628061ada74dd3a5b9f70faeba5",
        },
        "habitat_sim": {
            "repository": "https://github.com/facebookresearch/habitat-sim",
            "revision": "856d4b08c1a2632626bf0d205bf46471a99502b7",
            "release": "v0.1.7",
        },
        "habitat_lab": {
            "repository": "https://github.com/facebookresearch/habitat-lab",
            "revision": "d6ed1c0a0e786f16f261de2beafe347f4186d0d8",
            "release": "v0.1.7",
        },
    }
