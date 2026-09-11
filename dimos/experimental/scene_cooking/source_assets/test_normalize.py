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

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.experimental.scene_cooking.source_assets import normalize as source_asset


def test_prepare_scene_source_passes_through_direct_mesh_source(tmp_path: Path) -> None:
    source = tmp_path / "scene.glb"
    source.write_bytes(b"glb")

    prepared = source_asset.prepare_scene_source(source)

    assert prepared.original_path == source
    assert prepared.cook_path == source
    assert not prepared.normalized
    assert prepared.normalizer is None


def test_prepare_scene_source_normalizes_blend_with_blender(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "market.blend"
    source.write_bytes(b"blend")
    cache_root = tmp_path / "cache"
    calls: list[list[str]] = []

    def fake_run_command(args: list[str], label: str) -> str:
        calls.append(args)
        assert label == "blender source normalization"
        Path(args[-1]).write_bytes(b"glb")
        return (
            "DIMOS_BLEND_NORMALIZER base_objects=2 instances=10 "
            "realized_objects=12 unique_meshes=4 skipped_empty=1"
        )

    monkeypatch.setattr(source_asset.shutil, "which", lambda command: f"/usr/bin/{command}")
    monkeypatch.setattr(source_asset, "_source_cache_key", lambda path, version: "abc123")
    monkeypatch.setattr(source_asset, "_run_command", fake_run_command)

    prepared = source_asset.prepare_scene_source(source, cache_root=cache_root)

    assert prepared.original_path == source
    assert prepared.cook_path == cache_root / "market-abc123.glb"
    assert prepared.cook_path.read_bytes() == b"glb"
    assert prepared.normalized
    assert prepared.normalizer == "blend-evaluated-depsgraph-v1"
    assert prepared.stats["instances"] == 10
    assert prepared.stats["unique_meshes"] == 4
    assert calls == [
        [
            "/usr/bin/blender",
            "--background",
            str(source),
            "--python",
            calls[0][4],
            "--",
            str(cache_root / "market-abc123.glb"),
        ]
    ]


def test_prepare_scene_source_uses_cached_blend_normalization(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "market.blend"
    source.write_bytes(b"blend")
    cache_root = tmp_path / "cache"
    cache_root.mkdir()
    normalized = cache_root / "market-abc123.glb"
    normalized.write_bytes(b"cached")

    monkeypatch.setattr(source_asset, "_source_cache_key", lambda path, version: "abc123")
    monkeypatch.setattr(
        source_asset,
        "_run_command",
        lambda args, label: pytest.fail("cache hit should not invoke Blender"),
    )

    prepared = source_asset.prepare_scene_source(source, cache_root=cache_root)

    assert prepared.cook_path == normalized
    assert prepared.stats == {"cache_hit": True}


def test_prepare_scene_source_rejects_unsupported_suffix(tmp_path: Path) -> None:
    source = tmp_path / "scene.fbx"
    source.write_bytes(b"fbx")

    with pytest.raises(RuntimeError, match="unsupported scene source suffix"):
        source_asset.prepare_scene_source(source)
