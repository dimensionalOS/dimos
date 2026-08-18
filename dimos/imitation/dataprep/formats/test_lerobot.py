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

"""Read-back coverage independent of the isolated LeRobot writer."""

import json
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

from dimos.imitation.dataprep.formats.lerobot.reader import inspect


def test_inspect_native_lerobot_metadata(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    episodes = root / "meta" / "episodes" / "chunk-000"
    episodes.mkdir(parents=True)
    (root / "meta" / "info.json").write_text(
        json.dumps(
            {
                "codebase_version": "v3.0",
                "total_episodes": 2,
                "total_frames": 7,
                "fps": 15,
                "robot_type": "openyam",
                "features": {
                    "observation.images.wrist": {"dtype": "video", "shape": [8, 8, 3]},
                    "observation.state": {"dtype": "float32", "shape": [7]},
                    "action": {"dtype": "float32", "shape": [7]},
                },
            }
        )
    )
    (root / "meta" / "stats.json").write_text("{}")
    pq.write_table(pa.table({"length": [3, 4]}), episodes / "file-000.parquet")

    result = inspect(root)

    assert result["robot"] == "openyam"
    assert result["episodes"] == 2
    assert result["frames"] == 7
    assert result["episode_lengths"] == {
        "min": 3,
        "max": 4,
        "mean": 3.5,
        "uniform": False,
    }
    assert result["observation"]["observation.images.wrist"]["dtype"] == "video"
