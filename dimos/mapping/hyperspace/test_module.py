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


def test_summarize_answer_orders_by_score() -> None:
    from dimos.mapping.hyperspace.module import summarize_answer
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    cloud = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32),
        frame_id="odom",
        timestamp=1.0,
        intensities=np.array([0.2, 1.0, 0.5], dtype=np.float32),
    )
    cloud.seq = 7
    decoded = PointCloud2.lcm_decode(cloud.lcm_encode())
    assert decoded.seq == 7, "the query id must survive the wire"
    summary = summarize_answer(decoded, top=2)
    assert summary["frame"] == "odom"
    assert summary["voxels"] == 3
    assert [b["xyz"][0] for b in summary["best"]] == [1.0, 2.0]
    assert summary["best"][0]["score"] == 1.0


def test_summarize_answer_handles_an_empty_cloud() -> None:
    from dimos.mapping.hyperspace.module import summarize_answer
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    empty = PointCloud2.lcm_decode(PointCloud2(frame_id="odom", ts=1.0).lcm_encode())
    assert summarize_answer(empty) == {"frame": "odom", "voxels": 0, "best": []}


def test_find_is_an_instant_skill() -> None:
    assert Hyperspace.find.__skill__ is True
    assert Hyperspace.find.__skill_lifecycle__ == "instant"
