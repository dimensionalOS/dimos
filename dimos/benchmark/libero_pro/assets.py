"""Download and verify task assets before a LIBERO-PRO trial starts."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path, PurePosixPath

import requests

from dimos.benchmark.libero_pro.models import AssetReference, LiberoTaskManifest
from dimos.constants import CACHE_DIR


class LiberoAssetError(RuntimeError):
    """A pinned LIBERO-PRO task asset is missing or invalid."""


@dataclass(frozen=True)
class PreparedAssets:
    bddl: Path
    init_states: Path


def prepare_assets(
    manifest: LiberoTaskManifest,
    *,
    cache_root: Path = CACHE_DIR / "evaluation" / "libero_pro",
) -> PreparedAssets:
    root = cache_root / manifest.source.dataset_revision
    return PreparedAssets(
        bddl=_materialize(manifest, manifest.assets.bddl, root),
        init_states=_materialize(manifest, manifest.assets.init_states, root),
    )


def _materialize(
    manifest: LiberoTaskManifest,
    reference: AssetReference,
    root: Path,
) -> Path:
    relative = PurePosixPath(reference.repository_path)
    if relative.is_absolute() or ".." in relative.parts:
        raise LiberoAssetError(f"unsafe asset path: {reference.repository_path}")
    target = root.joinpath(*relative.parts)
    if target.is_file():
        _verify(target, reference)
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    url = (
        f"https://huggingface.co/datasets/{manifest.source.dataset_repository}/resolve/"
        f"{manifest.source.dataset_revision}/{reference.repository_path}"
    )
    response = requests.get(url, timeout=120)
    response.raise_for_status()
    temporary = target.with_suffix(target.suffix + ".partial")
    temporary.write_bytes(response.content)
    try:
        _verify(temporary, reference)
        temporary.replace(target)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return target


def _verify(path: Path, reference: AssetReference) -> None:
    if path.stat().st_size != reference.size_bytes:
        raise LiberoAssetError(f"asset size mismatch: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != reference.sha256:
        raise LiberoAssetError(f"asset digest mismatch: {path}")
