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

from pathlib import Path

import pytest

from dimos.learning.dataprep.cli import _load_config


@pytest.mark.parametrize(
    ("filename", "contents"),
    [
        (
            "config.yaml",
            "source: data/recordings/demo.db\nobservation:\n  image:\n    stream: color_image # RGB input\n",
        ),
        (
            "config.json",
            '{"source": "data/recordings/demo.db", "observation": {"image": {"stream": "color_image"}}}',
        ),
    ],
)
def test_load_config_supports_yaml_and_json(tmp_path: Path, filename: str, contents: str) -> None:
    config_path = tmp_path / filename
    config_path.write_text(contents)

    config = _load_config(config_path, source=None, output=None, output_format=None)

    assert config.source == "data/recordings/demo.db"
    assert config.observation["image"].stream == "color_image"
