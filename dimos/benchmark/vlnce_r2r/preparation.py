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

"""Content-addressed preparation for the public VLN-CE development case."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
from typing import Any
import zipfile

from filelock import FileLock
import requests

from dimos.benchmark.vlnce_r2r.models import (
    BenchmarkInstruction,
    ExternalAssetRef,
    ExternalOciImageRef,
    VlnceEpisodeSource,
)
from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT

Download = Callable[[str, Path], None]
CommandRunner = Callable[[list[str]], subprocess.CompletedProcess[str]]


@dataclass(frozen=True)
class PreparedAsset:
    """A verified, immutable-by-address extracted asset tree."""

    asset_id: str
    root: Path
    archive_sha256: str
    required_file_sha256: Mapping[str, str]
    cache_hit: bool


@dataclass(frozen=True)
class EpisodeBinding:
    """Private episode selected from the verified upstream dataset."""

    episode: Mapping[str, Any]
    episode_sha256: str


@dataclass(frozen=True)
class PreparationReceipt:
    """Verified inputs required to start one benchmark attempt."""

    assets: Mapping[str, PreparedAsset]
    episode: EpisodeBinding
    image_id: str | None = None


class PreparationError(RuntimeError):
    """A required external input could not be prepared safely."""


def prepare_public_assets(
    source: VlnceEpisodeSource,
    task: BenchmarkInstruction,
    *,
    cache_root: Path = CACHE_DIR,
    download: Download | None = None,
) -> PreparationReceipt:
    """Download, verify, and bind all external inputs declared by ``source``."""

    downloader = download or _download
    namespace = cache_root / source.preparation.cache_namespace
    namespace.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=min(4, len(source.preparation.assets))) as executor:
        futures = {
            asset.asset_id: executor.submit(_prepare_asset, asset, namespace, downloader)
            for asset in source.preparation.assets
        }
        assets = {asset_id: future.result() for asset_id, future in futures.items()}

    episode_root = assets[source.episode_asset_id].root
    episode = verify_episode_binding(source, task, episode_root / source.episode_path)
    return PreparationReceipt(assets=assets, episode=episode)


def resolve_oci_image(
    image: ExternalOciImageRef,
    *,
    project_root: Path = DIMOS_PROJECT_ROOT,
    runner: CommandRunner | None = None,
) -> str:
    """Resolve or build the exact case-bound OCI image and return its image ID."""

    run = runner or _run_command
    context = (project_root / image.build_context).resolve()
    if not context.is_relative_to(project_root.resolve()) or not context.is_dir():
        raise PreparationError(f"OCI build context is missing or unsafe: {image.build_context}")
    recipe = context / "Containerfile"
    if _sha256_file(recipe) != image.build_recipe_sha256:
        raise PreparationError("OCI build recipe does not match the case-bound digest")

    resolved = _inspect_image(image.image_name, run)
    if resolved is None:
        try:
            result = run(
                [
                    "podman",
                    "build",
                    "--pull=never",
                    "--timestamp=0",
                    "--tag",
                    image.image_name,
                    str(context),
                ]
            )
        except FileNotFoundError as error:
            raise PreparationError("podman is required to prepare the VLN-CE runtime") from error
        if result.returncode != 0:
            raise PreparationError(f"OCI image build failed: {result.stderr.strip()}")
        resolved = _inspect_image(image.image_name, run)
    if resolved is None:
        raise PreparationError("OCI image remained unavailable after preparation")
    expected = f"sha256:{image.image_digest}"
    normalized = resolved if resolved.startswith("sha256:") else f"sha256:{resolved}"
    if normalized != expected:
        raise PreparationError(f"OCI image digest mismatch: expected {expected}, got {resolved}")
    return normalized


def verify_episode_binding(
    source: VlnceEpisodeSource,
    task: BenchmarkInstruction,
    dataset_path: Path,
) -> EpisodeBinding:
    """Select exactly one case-bound episode without exposing it to DimOS runtime."""

    try:
        with gzip.open(dataset_path, "rt", encoding="utf-8") as handle:
            document = json.load(handle)
    except (OSError, json.JSONDecodeError) as error:
        raise PreparationError(f"could not read episode dataset: {dataset_path}") from error
    candidates = [
        episode
        for episode in document.get("episodes", [])
        if str(episode.get("episode_id")) == source.episode_id
    ]
    if len(candidates) != 1:
        raise PreparationError(
            f"expected exactly one episode {source.episode_id!r}, found {len(candidates)}"
        )
    episode = candidates[0]
    if episode.get("scene_id") != source.scene_id:
        raise PreparationError("episode scene does not match the case")
    instruction = episode.get("instruction", {}).get("instruction_text")
    if instruction != task.prompt:
        raise PreparationError("episode instruction does not match the case task")
    instruction_sha256 = hashlib.sha256(instruction.encode()).hexdigest()
    if instruction_sha256 != task.instruction_sha256:
        raise PreparationError("episode instruction digest does not match the case")
    episode_sha256 = hashlib.sha256(_canonical_json(episode)).hexdigest()
    if episode_sha256 != source.episode_sha256:
        raise PreparationError("episode payload does not match the case-bound digest")
    return EpisodeBinding(episode=episode, episode_sha256=episode_sha256)


def _prepare_asset(
    asset: ExternalAssetRef,
    namespace: Path,
    download: Download,
) -> PreparedAsset:
    target = namespace / asset.cache_subdir / asset.archive_sha256
    lock_path = target.parent / f"{asset.archive_sha256}.lock"
    target.parent.mkdir(parents=True, exist_ok=True)
    with FileLock(lock_path):
        verified = _verify_asset_tree(asset, target)
        if verified is not None:
            return PreparedAsset(
                asset_id=asset.asset_id,
                root=target,
                archive_sha256=asset.archive_sha256,
                required_file_sha256=verified,
                cache_hit=True,
            )

        with tempfile.TemporaryDirectory(prefix=f".{asset.asset_id}-", dir=target.parent) as raw:
            staging_parent = Path(raw)
            archive = staging_parent / "download.zip"
            extracted = staging_parent / "extracted"
            download(asset.url, archive)
            if archive.stat().st_size != asset.archive_bytes:
                raise PreparationError(f"download size mismatch for asset {asset.asset_id!r}")
            if _sha256_file(archive) != asset.archive_sha256:
                raise PreparationError(f"download checksum mismatch for asset {asset.asset_id!r}")
            _safe_extract_zip(archive, extracted)
            verified = _verify_asset_tree(asset, extracted)
            if verified is None:
                raise PreparationError(f"required files failed verification for {asset.asset_id!r}")
            receipt = {
                "asset_id": asset.asset_id,
                "archive_sha256": asset.archive_sha256,
                "required_files": verified,
            }
            (extracted / ".dimos-verified.json").write_text(
                json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n"
            )
            if target.exists():
                shutil.rmtree(target)
            os.replace(extracted, target)
        return PreparedAsset(
            asset_id=asset.asset_id,
            root=target,
            archive_sha256=asset.archive_sha256,
            required_file_sha256=verified,
            cache_hit=False,
        )


def _verify_asset_tree(asset: ExternalAssetRef, root: Path) -> dict[str, str] | None:
    if not root.is_dir():
        return None
    verified: dict[str, str] = {}
    for required in asset.required_files:
        path = root / required.path
        if not path.is_file() or path.is_symlink():
            return None
        digest = _sha256_file(path)
        if digest != required.sha256:
            return None
        verified[required.path] = digest
    return verified


def _safe_extract_zip(archive: Path, destination: Path) -> None:
    destination.mkdir()
    root = destination.resolve()
    try:
        with zipfile.ZipFile(archive) as package:
            for member in package.infolist():
                relative = Path(member.filename)
                target = (destination / relative).resolve()
                mode = member.external_attr >> 16
                if (
                    relative.is_absolute()
                    or ".." in relative.parts
                    or not target.is_relative_to(root)
                    or stat.S_ISLNK(mode)
                ):
                    raise PreparationError(f"unsafe ZIP member: {member.filename!r}")
                if member.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                    continue
                target.parent.mkdir(parents=True, exist_ok=True)
                with package.open(member) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output)
    except (OSError, zipfile.BadZipFile) as error:
        raise PreparationError(f"could not extract asset archive: {archive}") from error


def _download(url: str, destination: Path) -> None:
    try:
        with requests.get(url, stream=True, timeout=(10, 120)) as response:
            response.raise_for_status()
            with destination.open("xb") as output:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    if chunk:
                        output.write(chunk)
    except requests.RequestException as error:
        raise PreparationError(f"download failed for {url}") from error


def _inspect_image(image_name: str, run: CommandRunner) -> str | None:
    try:
        result = run(["podman", "image", "inspect", "--format", "{{.Id}}", image_name])
    except FileNotFoundError:
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def _run_command(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, capture_output=True, check=False, text=True)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise PreparationError(f"could not hash required file: {path}") from error
    return digest.hexdigest()


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()
