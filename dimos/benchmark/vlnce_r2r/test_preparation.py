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

from collections.abc import Callable
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

import pytest

from dimos.benchmark.vlnce_r2r.models import (
    BenchmarkInstruction,
    ExternalAssetFileRef,
    ExternalAssetRef,
    ExternalBenchmarkPreparationRef,
    ExternalOciImageRef,
    VlnceEpisodeSource,
)
from dimos.benchmark.vlnce_r2r.preparation import (
    PreparationError,
    prepare_public_assets,
    resolve_oci_image,
)

INSTRUCTION = "Exit the bedroom, enter the bathroom, wait at the toilet. "


def _canonical(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()


def _make_archive(root: Path) -> tuple[Path, dict[str, bytes], dict[str, object]]:
    episode = {
        "episode_id": "515",
        "scene_id": "mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        "instruction": {"instruction_text": INSTRUCTION},
        "start_position": [1.0, 2.0, 3.0],
        "goals": [{"position": [4.0, 5.0, 6.0]}],
    }
    dataset = json.dumps({"episodes": [episode]}).encode()
    dataset_path = root / "train.json.gz"
    with dataset_path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as handle:
            handle.write(dataset)
    files = {
        "R2R_VLNCE_v1-3/train/train.json.gz": dataset_path.read_bytes(),
        "17DRP5sb8fy/17DRP5sb8fy.glb": b"fake-public-scene",
        "17DRP5sb8fy/17DRP5sb8fy.navmesh": b"fake-public-navmesh",
    }
    archive = root / "assets.zip"
    with zipfile.ZipFile(archive, "w") as package:
        for name, content in files.items():
            package.writestr(name, content)
    return archive, files, episode


def _case_inputs(
    archive: Path, files: dict[str, bytes], episode: dict[str, object]
) -> tuple[VlnceEpisodeSource, BenchmarkInstruction]:
    asset = ExternalAssetRef(
        asset_id="public",
        url="https://example.com/assets.zip",
        archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
        archive_bytes=archive.stat().st_size,
        cache_subdir="assets/public",
        required_files=tuple(
            ExternalAssetFileRef(path=path, sha256=hashlib.sha256(content).hexdigest())
            for path, content in files.items()
        ),
    )
    source = VlnceEpisodeSource(
        upstream_revision="f" * 40,
        dataset_revision="R2R_VLNCE_v1-3",
        split="train",
        episode_id="515",
        episode_sha256=hashlib.sha256(_canonical(episode)).hexdigest(),
        scene_id="mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        episode_asset_id="public",
        episode_path="R2R_VLNCE_v1-3/train/train.json.gz",
        scene_asset_id="public",
        scene_path="17DRP5sb8fy/17DRP5sb8fy.glb",
        navmesh_path="17DRP5sb8fy/17DRP5sb8fy.navmesh",
        condition_label="dimos_geometry_training_scene_development",
        preparation=ExternalBenchmarkPreparationRef(
            revision="v1",
            cache_namespace="agent_eval/vlnce_r2r",
            assets=(asset,),
            image=ExternalOciImageRef(
                image_name="localhost/dimos-vlnce-r2r",
                image_digest="a" * 64,
                build_context="container",
                build_recipe_sha256="b" * 64,
                base_image="example/base",
                base_image_digest="c" * 64,
            ),
        ),
    )
    task = BenchmarkInstruction(
        prompt=INSTRUCTION,
        instruction_sha256=hashlib.sha256(INSTRUCTION.encode()).hexdigest(),
        submission_guidance="Call submit_route() exactly once after completing the route.",
    )
    return source, task


def _copy_downloader(archive: Path, calls: list[str] | None = None) -> Callable[[str, Path], None]:
    def download(url: str, destination: Path) -> None:
        if calls is not None:
            calls.append(url)
        shutil.copyfile(archive, destination)

    return download


def test_cold_then_warm_offline_cache_and_exact_episode_binding(tmp_path: Path) -> None:
    archive, files, episode = _make_archive(tmp_path)
    source, task = _case_inputs(archive, files, episode)
    calls: list[str] = []

    cold = prepare_public_assets(
        source, task, cache_root=tmp_path / "cache", download=_copy_downloader(archive, calls)
    )
    warm = prepare_public_assets(
        source,
        task,
        cache_root=tmp_path / "cache",
        download=lambda _url, _path: pytest.fail("warm cache attempted a download"),
    )

    assert calls == ["https://example.com/assets.zip"]
    assert not cold.assets["public"].cache_hit
    assert warm.assets["public"].cache_hit
    assert warm.episode.episode == episode
    assert (
        warm.assets["public"].root.joinpath(source.scene_path).read_bytes()
        == files[source.scene_path]
    )


def test_episode_preflight_rejects_case_instruction_mismatch(tmp_path: Path) -> None:
    archive, files, episode = _make_archive(tmp_path)
    source, task = _case_inputs(archive, files, episode)
    changed = task.model_copy(
        update={
            "prompt": "A different instruction",
            "instruction_sha256": hashlib.sha256(b"A different instruction").hexdigest(),
        }
    )

    with pytest.raises(PreparationError, match="instruction"):
        prepare_public_assets(
            source, changed, cache_root=tmp_path / "cache", download=_copy_downloader(archive)
        )


def test_oci_resolver_accepts_only_expected_digest(tmp_path: Path) -> None:
    image = _image_with_recipe(tmp_path)

    def inspect(command: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(command, 0, f"sha256:{image.image_digest}\n", "")

    assert resolve_oci_image(image, project_root=tmp_path, runner=inspect) == (
        f"sha256:{image.image_digest}"
    )


def _image_with_recipe(root: Path) -> ExternalOciImageRef:
    context = root / "container"
    context.mkdir()
    recipe = context / "Containerfile"
    recipe.write_text("FROM scratch\n")
    return ExternalOciImageRef(
        image_name="localhost/dimos-vlnce-r2r",
        image_digest="a" * 64,
        build_context="container",
        build_recipe_sha256=hashlib.sha256(recipe.read_bytes()).hexdigest(),
        base_image="scratch",
        base_image_digest="b" * 64,
    )
