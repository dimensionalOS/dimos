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

"""Download-on-first-use assets for the Microduck simulation.

The Microduck MJCF models and meshes (~25 MB of STLs) live in
`pollen-robotics/microduck_rl`, and the pretrained walking policy ships in
`pollen-robotics/microduck`. Neither belongs in this git repo, so this module
pulls both from GitHub at pinned commits into a local cache the first time a
Microduck simulation starts.

Cache layout (``$DIMOS_MICRODUCK_ASSETS`` overrides the default root
``~/.cache/dimos/microduck``)::

    <root>/robot/robot_walk.xml       # MJCF + joints_properties.xml + assets/*.stl
    <root>/policies/alpha_walking.onnx
    <root>/.complete-<version>        # marker; bump _CACHE_VERSION to refetch
"""

from __future__ import annotations

import io
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
import urllib.request

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Pinned upstream commits (Apache-2.0 code, CC BY-NC-SA meshes).
_MICRODUCK_RL_COMMIT = "d424a0c899f6b33cbd3daeb279913134349c0b63"
_MICRODUCK_COMMIT = "590b986bd8c0d50ae02cb3ea2f59c463b6828168"

_RL_TARBALL = (
    f"https://codeload.github.com/pollen-robotics/microduck_rl/tar.gz/{_MICRODUCK_RL_COMMIT}"
)
_RL_ROBOT_SUBDIR = "src/mjlab_microduck/robot/microduck"
_POLICY_URL = (
    "https://raw.githubusercontent.com/pollen-robotics/microduck/"
    f"{_MICRODUCK_COMMIT}/policies/alpha_walking.onnx"
)

_CACHE_VERSION = "1"

ROBOT_MJCF_NAME = "robot_walk.xml"
WALKING_POLICY_NAME = "alpha_walking.onnx"


def assets_root() -> Path:
    override = os.environ.get("DIMOS_MICRODUCK_ASSETS")
    if override:
        return Path(override).expanduser()
    return Path.home() / ".cache" / "dimos" / "microduck"


def robot_mjcf_path() -> Path:
    return assets_root() / "robot" / ROBOT_MJCF_NAME


def walking_policy_path() -> Path:
    return assets_root() / "policies" / WALKING_POLICY_NAME


def _marker() -> Path:
    return assets_root() / f".complete-{_CACHE_VERSION}"


def ensure_assets() -> Path:
    """Fetch the Microduck MJCF/meshes and walking policy if not cached.

    Returns the cache root. Raises on download failure with a hint about
    the manual fallback (cloning the repos and pointing
    ``DIMOS_MICRODUCK_ASSETS`` at a prepared directory).
    """
    root = assets_root()
    if _marker().exists() and robot_mjcf_path().exists() and walking_policy_path().exists():
        return root

    logger.info("Fetching Microduck assets", root=str(root))
    root.mkdir(parents=True, exist_ok=True)

    try:
        _fetch_robot_dir(root / "robot")
        _fetch_policy(root / "policies" / WALKING_POLICY_NAME)
    except Exception as exc:
        raise RuntimeError(
            "Failed to download Microduck assets from GitHub. To prepare them "
            "manually, copy microduck_rl's src/mjlab_microduck/robot/microduck/ "
            "to <dir>/robot/ and microduck's policies/alpha_walking.onnx to "
            "<dir>/policies/, then set DIMOS_MICRODUCK_ASSETS=<dir>."
        ) from exc

    _marker().touch()
    logger.info("Microduck assets ready", root=str(root))
    return root


def _fetch_robot_dir(dest: Path) -> None:
    if (dest / ROBOT_MJCF_NAME).exists():
        return
    logger.info("Downloading microduck_rl robot models", url=_RL_TARBALL)
    with urllib.request.urlopen(_RL_TARBALL, timeout=120) as resp:
        payload = resp.read()

    prefix = None
    with tempfile.TemporaryDirectory() as tmp:
        with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as tar:
            members = []
            for member in tar.getmembers():
                if prefix is None:
                    prefix = member.name.split("/", 1)[0]
                rel = member.name.split("/", 1)[-1]
                if rel.startswith(_RL_ROBOT_SUBDIR) and member.isfile():
                    members.append(member)
            tar.extractall(tmp, members=members, filter="data")
        src = Path(tmp) / (prefix or "") / _RL_ROBOT_SUBDIR
        if not (src / ROBOT_MJCF_NAME).exists():
            raise RuntimeError(f"robot dir not found in tarball: {src}")
        if dest.exists():
            shutil.rmtree(dest)
        shutil.copytree(src, dest)


def _fetch_policy(dest: Path) -> None:
    if dest.exists():
        return
    logger.info("Downloading Microduck walking policy", url=_POLICY_URL)
    dest.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(_POLICY_URL, timeout=120) as resp:
        payload = resp.read()
    if len(payload) < 100_000:
        raise RuntimeError(f"policy download looks truncated ({len(payload)} bytes)")
    dest.write_bytes(payload)
