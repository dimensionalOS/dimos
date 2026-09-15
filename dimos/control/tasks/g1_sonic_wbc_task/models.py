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

"""Download pinned NVIDIA SONIC models and example motion clips.

The policy bundle pins come from CC's SONIC setup. The shared planner is also
downloaded from Hugging Face; clips come directly from NVIDIA's pinned sources.
No model download runs during blueprint discovery or a control tick.
"""

import argparse
import hashlib
from pathlib import Path

from huggingface_hub import hf_hub_download
import requests

from dimos.control.tasks.g1_sonic_wbc_task.model_sources import (
    MODEL_FILES,
    MODEL_REPOSITORY,
    MODEL_REVISION,
    MOTION_FILES,
    MOTION_REVISION,
    sonic_model_directory,
)

_PROFILE_PREFIXES = {"sonic-v1.1": "sonic_v1_1/", "sonic-low-latency": "low_latency/"}
_MOTION_URL = "https://media.githubusercontent.com/media/NVlabs/GR00T-WholeBodyControl"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(path: Path, expected: str) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"missing SONIC asset: {path}; run dimos-sonic-models")
    actual = sha256(path)
    if actual != expected:
        raise ValueError(f"SONIC SHA-256 mismatch for {path}: expected {expected}, found {actual}")


def _verified(path: Path, expected: str, check: bool) -> bool:
    try:
        verify(path, expected)
    except (FileNotFoundError, ValueError):
        if check:
            raise
        return False
    return True


def _download_motion(name: str, path: Path, expected: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".download")
    url = f"{_MOTION_URL}/{MOTION_REVISION}/gear_sonic_deploy/reference/example/{name}"
    try:
        with requests.get(url, stream=True, timeout=(10, 120)) as response:
            response.raise_for_status()
            with temporary.open("wb") as stream:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    stream.write(chunk)
        verify(temporary, expected)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def setup_models(
    destination: Path,
    *,
    profile: str = "sonic-v1.1",
    check: bool = False,
    motions: bool = True,
) -> list[Path]:
    """Fetch missing/corrupt assets, or verify the installation without network IO."""
    if profile != "all" and profile not in _PROFILE_PREFIXES:
        raise ValueError(f"unknown SONIC profile: {profile}")
    prefix = _PROFILE_PREFIXES.get(profile)
    selected = {
        name: digest
        for name, digest in MODEL_FILES.items()
        if name == "planner_sonic.onnx" or prefix is None or name.startswith(prefix)
    }
    paths = []
    for name, expected in selected.items():
        path = destination / name
        if not _verified(path, expected, check):
            hf_hub_download(
                repo_id=MODEL_REPOSITORY,
                filename=name,
                revision=MODEL_REVISION,
                local_dir=destination,
                force_download=path.exists(),
            )
            verify(path, expected)
        paths.append(path)
    if motions:
        for name, expected in MOTION_FILES.items():
            path = destination / "motions" / name
            if not _verified(path, expected, check):
                _download_motion(name, path, expected)
            paths.append(path)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=sonic_model_directory())
    parser.add_argument("--profile", choices=[*_PROFILE_PREFIXES, "all"], default="sonic-v1.1")
    parser.add_argument(
        "--check", action="store_true", help="Verify existing files without downloads"
    )
    parser.add_argument("--skip-motions", action="store_true")
    args = parser.parse_args()
    paths = setup_models(
        args.destination, profile=args.profile, check=args.check, motions=not args.skip_motions
    )
    print(f"Verified {len(paths)} SONIC assets in {args.destination}")


if __name__ == "__main__":
    main()
