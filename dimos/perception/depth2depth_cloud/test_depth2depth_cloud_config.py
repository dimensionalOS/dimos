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

"""Depth2DepthCloudConfig: it must be exactly what the Rust config accepts, since that refuses anything else at launch."""

from __future__ import annotations

import re

from pydantic import ValidationError
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.perception.depth2depth_cloud.module import Depth2DepthCloudConfig

WEIGHTS = dict(weights_dir="/weights")
RUST_MODULE = DIMOS_PROJECT_ROOT / "dimos/perception/depth2depth_cloud/rust/src/module.rs"


def test_every_field_the_rust_config_requires_is_sent():
    # The Rust config has no defaults and rejects unknown keys, so the two field lists must match exactly.
    rust_config = RUST_MODULE.read_text().split("pub struct Config {")[1].split("\n}")[0]
    rust_fields = set(re.findall(r"^\s+(\w+):", rust_config, re.MULTILINE))
    assert set(Depth2DepthCloudConfig(**WEIGHTS).to_config_dict()) == rust_fields


@pytest.mark.parametrize("size", [100, 50, 1050])
def test_a_model_size_off_the_patch_grid_is_refused(size):
    with pytest.raises(ValidationError, match="multiple of 14"):
        Depth2DepthCloudConfig(**WEIGHTS, model_width=size)


def test_the_python_bounds_are_the_rust_bounds():
    # A value Python lets through and Rust refuses fails only when the robot starts the module.
    rust = RUST_MODULE.read_text().split("pub struct Config {")[1].split("\n}")[0]
    ranges = re.findall(r"range\(min = ([\d.]+), max = ([\d.]+)\)\)\]\n\s+(\w+):", rust)
    assert ranges
    for low, high, name in ranges:
        bounds = {type(m).__name__: m for m in Depth2DepthCloudConfig.model_fields[name].metadata}
        if not bounds:  # the model size, checked by its own validator in module.py
            continue
        assert (bounds["Ge"].ge, bounds["Le"].le) == (float(low), float(high)), name


def test_a_jpeg_scale_the_decoder_cannot_do_is_refused():
    # The Rust decoder would silently fall back to full size, three times the undistort work.
    with pytest.raises(ValidationError, match="1, 2, 4 or 8"):
        Depth2DepthCloudConfig(**WEIGHTS, decode_scale=3)
