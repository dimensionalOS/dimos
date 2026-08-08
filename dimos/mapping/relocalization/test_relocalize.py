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

from pytest import MonkeyPatch

from dimos.mapping.relocalization.module import Config
from dimos.mapping.relocalization.relocalize import FINE_VOXEL, RERANK_DIST, relocalize_scales
from dimos.mapping.voxels.lidar_defaults import (
    DEFAULT_FINE_VOXEL,
    DEFAULT_RERANK_DIST,
    MID360_FINE_VOXEL,
    MID360_RERANK_DIST,
)


def test_relocalize_scales_keep_defaults_without_mid360() -> None:
    assert relocalize_scales(None) == (FINE_VOXEL, RERANK_DIST)
    assert relocalize_scales("default") == (FINE_VOXEL, RERANK_DIST)
    assert FINE_VOXEL == DEFAULT_FINE_VOXEL == 0.1
    assert RERANK_DIST == DEFAULT_RERANK_DIST == DEFAULT_FINE_VOXEL * 1.5


def test_relocalize_scales_mid360() -> None:
    assert relocalize_scales("mid360") == (0.06, 0.12)
    assert relocalize_scales("mid360") == (MID360_FINE_VOXEL, MID360_RERANK_DIST)


def test_relocalize_scales_explicit_override_wins() -> None:
    assert relocalize_scales("mid360", fine_voxel=0.2, rerank_dist=0.4) == (0.2, 0.4)
    assert relocalize_scales("default", fine_voxel=0.08) == (0.08, RERANK_DIST)


def test_relocalization_config_inherits_global_lidar_config(monkeypatch: MonkeyPatch) -> None:
    cfg = Config()
    monkeypatch.setattr(cfg.g, "lidar_config", "default")
    assert cfg.resolved_lidar_config() == "default"
    assert cfg.resolved_relocalize_scales() == (FINE_VOXEL, RERANK_DIST)

    monkeypatch.setattr(cfg.g, "lidar_config", "mid360")
    assert cfg.resolved_lidar_config() == "mid360"
    assert cfg.resolved_relocalize_scales() == (MID360_FINE_VOXEL, MID360_RERANK_DIST)


def test_relocalization_config_module_lidar_config_overrides_global(
    monkeypatch: MonkeyPatch,
) -> None:
    cfg = Config(lidar_config="mid360")
    monkeypatch.setattr(cfg.g, "lidar_config", "default")
    assert cfg.resolved_lidar_config() == "mid360"
    assert cfg.resolved_relocalize_scales() == (MID360_FINE_VOXEL, MID360_RERANK_DIST)
