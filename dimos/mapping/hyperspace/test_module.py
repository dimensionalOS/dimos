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

from pathlib import Path

import numpy as np
import tomllib

from dimos.mapping.hyperspace.cli import heat_colors
from dimos.mapping.hyperspace.module import Hyperspace, HyperspaceConfig

CRATE = Path(__file__).parent / "rust" / "Cargo.toml"


def test_ports_match_the_rust_registry() -> None:
    """The python ports and the crate's baked registry entry must agree."""
    manifest = tomllib.loads(CRATE.read_text())
    entry = manifest["package"]["metadata"]["dimos"]["module"]["hyperspace"]
    assert entry["python"] == "dimos.mapping.hyperspace.module:Hyperspace"
    annotations = Hyperspace.__annotations__
    for port in entry["inputs"]:
        assert port in annotations, f"rust declares input {port}, python does not"
    for port in entry["outputs"]:
        assert port in annotations, f"rust declares output {port}, python does not"


def test_executable_is_the_workspace_binary() -> None:
    config = HyperspaceConfig()
    assert config.executable.endswith("target/release/hyperspace")
    assert config.build_command == "cargo build --release"


def test_defaults_keep_the_models_off() -> None:
    """A default launch must not need model files or a GPU."""
    config = HyperspaceConfig()
    assert config.model_dir == ""
    assert config.depth_weights_dir == ""
    assert config.cuda is False


def test_heat_colors_run_dark_to_light() -> None:
    colors = heat_colors(np.array([0.0, 0.5, 1.0]))
    assert colors.shape == (3, 3)
    brightness = colors.astype(int).sum(axis=1)
    assert brightness[0] < brightness[1] < brightness[2]


def test_parse_answer_reads_the_json_the_module_publishes() -> None:
    from dimos.mapping.hyperspace.module import parse_answer

    data = (
        '{"id": 9, "text": "a cone", "frame": "odom", "voxel_size": 0.1, "voxels": 2, '
        '"best": [{"xyz": [0.15, 0.25, 0.35], "score": 1.0}, {"xyz": [0.45, 0.55, 0.65], "score": 0.25}], '
        '"stats": {}}'
    )
    answer = parse_answer(data)
    assert answer["id"] == 9
    assert answer["frame"] == "odom"
    assert answer["voxels"] == 2
    assert answer["best"][0] == {"xyz": [0.15, 0.25, 0.35], "score": 1.0}


def test_find_is_an_instant_skill() -> None:
    assert Hyperspace.find.__skill__ is True
    assert Hyperspace.find.__skill_lifecycle__ == "instant"
