# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Verify Git LFS objects through the Batch API without downloading them."""

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path
import subprocess

import requests

BATCH_SIZE = 100
POINTER_VERSION = "https://git-lfs.github.com/spec/v1"
REMEDIATION = "Upload the object with bin/lfs_push, then retry this check."


@dataclass(frozen=True)
class Pointer:
    path: str
    oid: str
    size: int


class LFSVerificationError(RuntimeError):
    """A remote LFS object could not be established as available."""


def _git(args: list[str], repo: Path) -> bytes:
    result = subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True)
    return result.stdout


def _parse_pointer(path: str, contents: bytes) -> Pointer | None:
    try:
        fields = dict(
            line.split(" ", 1) for line in contents.decode("utf-8").splitlines() if " " in line
        )
    except (UnicodeDecodeError, ValueError):
        return None
    if fields.get("version") != POINTER_VERSION:
        return None
    oid = fields.get("oid", "")
    size_text = fields.get("size", "")
    if not oid.startswith("sha256:") or not size_text.isdigit():
        return None
    return Pointer(path, oid.removeprefix("sha256:"), int(size_text))


def discover_pointers(target: str = "HEAD", repo: Path = Path(".")) -> list[Pointer]:
    """Read all pointer blobs tracked by *target*, without invoking Git LFS."""
    listing = _git(["ls-tree", "-r", "-l", "-z", "--full-tree", target], repo)
    pointers: list[Pointer] = []
    for entry in listing.split(b"\0"):
        if not entry:
            continue
        metadata, path_bytes = entry.split(b"\t", 1)
        # `git ls-tree -l` right-aligns the size, so the separator is a run of
        # spaces rather than one literal space.
        mode, kind, oid, _size = metadata.decode("ascii").split()
        if kind != "blob" or mode == "160000":
            continue
        contents = _git(["cat-file", "blob", oid], repo)
        pointer = _parse_pointer(path_bytes.decode("utf-8"), contents)
        if pointer is not None:
            pointers.append(pointer)
    return pointers


def configured_lfs_url(repo: Path = Path(".")) -> str:
    """Return the endpoint configured in the repository's .lfsconfig."""
    try:
        value = _git(["config", "--file", ".lfsconfig", "--get", "lfs.url"], repo)
    except subprocess.CalledProcessError as exc:
        raise LFSVerificationError(
            "Could not read lfs.url from .lfsconfig; refusing to guess an endpoint."
        ) from exc
    url = value.decode().strip()
    if not url:
        raise LFSVerificationError(
            "The repository .lfsconfig has no lfs.url; refusing to guess an endpoint."
        )
    return url.rstrip("/")


def _diagnostic(items: Iterable[Pointer], reason: str) -> str:
    lines = [f"{reason} Remote LFS availability check failed."]
    lines.extend(f"- {item.path} (OID {item.oid})" for item in items)
    lines.append(REMEDIATION)
    return "\n".join(lines)


def _download_href(obj: object) -> str | None:
    if not isinstance(obj, dict) or not isinstance(obj.get("actions"), dict):
        return None
    download = obj["actions"].get("download")
    if not isinstance(download, dict):
        return None
    href = download.get("href")
    return href if isinstance(href, str) and href else None


def verify(
    target: str = "HEAD",
    repo: Path = Path("."),
    *,
    batch_size: int = BATCH_SIZE,
    post: Callable[..., requests.Response] = requests.post,
) -> None:
    """Verify every pointer in *target* using bounded metadata-only requests."""
    if not 1 <= batch_size <= BATCH_SIZE:
        raise ValueError(f"batch_size must be between 1 and {BATCH_SIZE}")
    pointers = discover_pointers(target, repo)
    if not pointers:
        return
    endpoint = configured_lfs_url(repo) + "/objects/batch"
    for start in range(0, len(pointers), batch_size):
        batch = pointers[start : start + batch_size]
        payload = {
            "operation": "download",
            "transfers": ["basic"],
            "objects": [{"oid": p.oid, "size": p.size} for p in batch],
        }
        try:
            response = post(
                endpoint,
                json=payload,
                headers={
                    "Accept": "application/vnd.git-lfs+json",
                    "Content-Type": "application/vnd.git-lfs+json",
                },
                timeout=15,
            )
            response.raise_for_status()
            body = response.json()
            objects = body["objects"]
            if not isinstance(objects, list):
                raise ValueError("objects is not a list")
            by_oid = {
                obj["oid"]: obj
                for obj in objects
                if isinstance(obj, dict) and isinstance(obj.get("oid"), str)
            }
            unavailable = [p for p in batch if _download_href(by_oid.get(p.oid)) is None]
            if unavailable:
                raise LFSVerificationError(
                    _diagnostic(unavailable, "Missing remote object or download action.")
                )
        except LFSVerificationError:
            raise
        except (requests.RequestException, ValueError, KeyError, TypeError) as exc:
            raise LFSVerificationError(
                _diagnostic(batch, f"LFS service/protocol failure ({exc}).")
            ) from exc
