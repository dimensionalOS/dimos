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

"""Pinned upstream identities for the bounded SPACE text evaluation."""

SPACE_REVISION = "eec58a24516bfcd4554807b0a2b9d08b04eafd1c"
SPACE_REPOSITORY = "https://github.com/apple-aiml-research/ml-space-benchmark.git"
DATA_URL = "https://ml-site.cdn-apple.com/datasets/space/space.tar.gz"
DATA_MEMBER = "SPACE_data_release/MapSketchingBEVText/qas.json"
DATA_SHA256 = "664e4e30b92017903de39997a6a2bda2f26763419d8b28ca653b63b94f1c9dc2"
DATA_BYTES = 776_160
MAX_DOWNLOAD_BYTES = 20 * 1024 * 1024
DOWNLOAD_TIMEOUT_S = 120.0
TASK = "MapSketchingBEVText"

# Frozen before model execution: seed 3399, two variants from each base layout.
SELECTED_INDICES = (
    19,
    73,
    97,
    51,
    13,
    84,
    42,
    0,
    112,
    118,
    33,
    62,
    93,
    68,
    78,
    47,
    100,
    55,
    11,
    30,
)
SMOKE_INDEX = 4

SOURCE_SHA256 = {
    "space/evaluate_qas.py": "3f3903178a4c3046f59e04f5a497cb8917fd7b15bcd23d7d7e8d6c5133ff9615",
    "space/agents/qa_agent.py": "2534c4b698342148530403863fcff90ddff886f5c849fbc2549bbfc3222b32ae",
    "space/registry.py": "befd23d0d21a9ae6202b401720e391dcb4dbadee818c9e1c59b6152775228039",
    "space/agents/__init__.py": "577daa88a7718791f21699d0b45038618c43193b7f5292ce1123e37d64e8834d",
    "space/configs/__init__.py": "37fd61c4a875d75f0df57b9fea452ded2d1713081fdea73d767bac490bed6a73",
}
