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
`pollen-robotics/microduck_rl`, and the pretrained policies ship in
`pollen-robotics/microduck`. Neither belongs in this git repo, so this module
pulls both from GitHub at pinned commits into a local cache the first time a
Microduck simulation starts.

Cache layout (``$DIMOS_MICRODUCK_ASSETS`` overrides the default root
``~/.cache/dimos/microduck``)::

    <root>/robot/robot_walk.xml                   # MJCF + joints_properties.xml + assets/*.stl
    <root>/robot/robot_allcollisions.xml          # "default" variant (ROBOT_MJCF_BY_VARIANT)
    <root>/robot/robot_allcollisions_rollers.xml  # "rollers" variant
    <root>/policies/<name>.onnx                   # one per policy (POLICY_FILES)
    <root>/.complete-<version>                    # informational marker

Fetching is per file and additive: whatever is already cached is neither
re-downloaded nor deleted, only missing files are fetched, and each policy
download is preceded by a HEAD request confirming the pinned source serves
that file. A policy that cannot be obtained is reported in
``MicroduckAssets.missing`` (the cockpit greys it out) instead of failing the
start; only the walking policy and the robot model are mandatory. Callers
that only run the walking policy can pass ``ensure_assets(policies=("walk",))``
to skip the probes for the other eight.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
import io
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
import urllib.error
import urllib.request

from dimos.robot.microduck.policies import DEFAULT_VARIANT, POLICY_SPECS, PolicyName
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Pinned upstream commits (Apache-2.0 code, CC BY-NC-SA meshes).
_MICRODUCK_RL_COMMIT = "d424a0c899f6b33cbd3daeb279913134349c0b63"
_MICRODUCK_COMMIT = "590b986bd8c0d50ae02cb3ea2f59c463b6828168"

_RL_TARBALL = (
    f"https://codeload.github.com/pollen-robotics/microduck_rl/tar.gz/{_MICRODUCK_RL_COMMIT}"
)
_RL_ROBOT_SUBDIR = "src/mjlab_microduck/robot/microduck"
_POLICY_BASE_URL = (
    f"https://raw.githubusercontent.com/pollen-robotics/microduck/{_MICRODUCK_COMMIT}/policies"
)

_CACHE_VERSION = "1"
_MIN_POLICY_BYTES = 100_000
# The HEAD probe is the only network call a start with a complete-but-for-one
# cache makes; keep it short so an offline machine is not held up for long.
_HEAD_TIMEOUT_S = 5
_GET_TIMEOUT_S = 120

ROBOT_MJCF_NAME = "robot_walk.xml"
WALKING_POLICY_NAME = "alpha_walking.onnx"

# Robot MJCF per variant: the wheeled model is needed for the roller policies,
# the plain all-collisions model for everything else (see policies.py).
ROBOT_MJCF_BY_VARIANT: dict[str, str] = {
    "default": "robot_allcollisions.xml",
    "rollers": "robot_allcollisions_rollers.xml",
}

# Policy -> ONNX file name under <root>/policies/.
POLICY_FILES: dict[PolicyName, str] = {name: spec.onnx for name, spec in POLICY_SPECS.items()}


@dataclass(frozen=True)
class MicroduckAssets:
    """What ``ensure_assets`` found or fetched."""

    robot_dir: Path
    policy_dir: Path
    missing: tuple[PolicyName, ...]  # policies whose ONNX could not be obtained

    def robot_mjcf(self, variant: str = DEFAULT_VARIANT) -> Path:
        return self.robot_dir / ROBOT_MJCF_BY_VARIANT[variant]

    def policy_path(self, name: PolicyName | str) -> Path:
        return self.policy_dir / POLICY_FILES[PolicyName(name)]


def assets_root() -> Path:
    override = os.environ.get("DIMOS_MICRODUCK_ASSETS")
    if override:
        return Path(override).expanduser()
    return Path.home() / ".cache" / "dimos" / "microduck"


def robot_dir() -> Path:
    return assets_root() / "robot"


def policy_dir() -> Path:
    return assets_root() / "policies"


def robot_mjcf_path() -> Path:
    """The walk-only model the plain ``microduck-sim`` blueprint uses."""
    return robot_dir() / ROBOT_MJCF_NAME


def variant_mjcf_path(variant: str = DEFAULT_VARIANT) -> Path:
    if variant not in ROBOT_MJCF_BY_VARIANT:
        raise ValueError(
            f"unknown Microduck variant {variant!r}; expected one of {tuple(ROBOT_MJCF_BY_VARIANT)}"
        )
    return robot_dir() / ROBOT_MJCF_BY_VARIANT[variant]


def walking_policy_path() -> Path:
    return policy_dir() / WALKING_POLICY_NAME


def policy_path(name: PolicyName | str) -> Path:
    return policy_dir() / POLICY_FILES[PolicyName(name)]


def _marker() -> Path:
    return assets_root() / f".complete-{_CACHE_VERSION}"


def ensure_assets(
    variant: str = DEFAULT_VARIANT,
    *,
    policies: Iterable[PolicyName | str] | None = None,
) -> MicroduckAssets:
    """Fetch whatever Microduck files are missing for ``variant``; never deletes.

    Mandatory: the robot models (one tarball holds every variant) and the
    walking policy - failing to get them raises with a hint about the manual
    fallback (cloning the repos and pointing ``DIMOS_MICRODUCK_ASSETS`` at a
    prepared directory). Every other policy is best effort and ends up in
    ``MicroduckAssets.missing`` when it cannot be downloaded (offline, or the
    pinned source does not serve it).

    ``policies`` restricts which policies may be downloaded (default: all
    nine); a walk-only caller passes ``policies=("walk",)`` to avoid any
    network probe for the others. Policies not in the cache are always
    reported in ``missing``, whether or not they were requested.
    """
    variant_mjcf = variant_mjcf_path(variant)  # validates the variant
    root = assets_root()
    robots = robot_dir()
    policy_root = policy_dir()
    wanted = {PolicyName(p) for p in policies} if policies is not None else set(POLICY_FILES)
    wanted.add(PolicyName.WALK)

    required_mjcf = {ROBOT_MJCF_NAME, variant_mjcf.name}
    if not all((robots / name).exists() for name in required_mjcf):
        logger.info("Fetching Microduck robot models", root=str(root))
        root.mkdir(parents=True, exist_ok=True)
        try:
            _fetch_robot_dir(robots, required_mjcf)
        except Exception as exc:
            raise RuntimeError(_MANUAL_HINT.format(what="robot models")) from exc

    missing: list[PolicyName] = []
    unreachable = False
    for name, filename in POLICY_FILES.items():
        dest = policy_root / filename
        if dest.exists():
            continue
        if unreachable or name not in wanted:
            missing.append(name)
            continue
        try:
            _fetch_policy(dest)
        except _SourceUnreachableError as exc:
            logger.warning(
                "Microduck policy source unreachable; skipping remaining downloads",
                error=str(exc),
            )
            unreachable = True
            missing.append(name)
        except Exception as exc:
            logger.warning("Microduck policy unavailable", policy=str(name), error=str(exc))
            missing.append(name)

    if PolicyName.WALK in missing:
        raise RuntimeError(_MANUAL_HINT.format(what="walking policy"))

    if not _marker().exists():
        root.mkdir(parents=True, exist_ok=True)
        _marker().touch()
    missing_wanted = [str(m) for m in missing if m in wanted]
    if missing_wanted:
        logger.warning(
            "Microduck assets ready with missing policies", root=str(root), missing=missing_wanted
        )
    else:
        logger.info("Microduck assets ready", root=str(root))
    return MicroduckAssets(robot_dir=robots, policy_dir=policy_root, missing=tuple(missing))


_MANUAL_HINT = (
    "Failed to download the Microduck {what} from GitHub. To prepare the assets "
    "manually, copy microduck_rl's src/mjlab_microduck/robot/microduck/ to "
    "<dir>/robot/ and microduck's policies/*.onnx to <dir>/policies/, then set "
    "DIMOS_MICRODUCK_ASSETS=<dir>."
)


class _SourceUnreachableError(RuntimeError):
    """The download host could not be reached at all (offline / DNS / timeout)."""


def _fetch_robot_dir(dest: Path, required: Iterable[str] = (ROBOT_MJCF_NAME,)) -> None:
    """Download the microduck_rl robot directory into ``dest`` (additive copy)."""
    logger.info("Downloading microduck_rl robot models", url=_RL_TARBALL)
    with urllib.request.urlopen(_RL_TARBALL, timeout=_GET_TIMEOUT_S) as resp:
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
        for name in required:
            if not (src / name).exists():
                raise RuntimeError(f"{name} not found in tarball dir {src}")
        # Never wipe an existing cache: overlay the tarball on top of it.
        shutil.copytree(src, dest, dirs_exist_ok=True)


def _source_serves(url: str) -> bool:
    """HEAD ``url``: True if served, False if the source lacks it; raises when offline."""
    request = urllib.request.Request(url, method="HEAD")
    try:
        with urllib.request.urlopen(request, timeout=_HEAD_TIMEOUT_S) as resp:
            return 200 <= resp.status < 300
    except urllib.error.HTTPError as exc:
        if exc.code == 404 or exc.code == 410:
            return False
        raise
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise _SourceUnreachableError(f"{url}: {exc}") from exc


def _fetch_policy(dest: Path) -> None:
    """Download one policy file, first confirming the pinned source serves it."""
    if dest.exists():
        return
    url = f"{_POLICY_BASE_URL}/{dest.name}"
    if not _source_serves(url):
        raise RuntimeError(f"pinned source does not serve {dest.name}: {url}")
    logger.info("Downloading Microduck policy", url=url)
    dest.parent.mkdir(parents=True, exist_ok=True)
    try:
        with urllib.request.urlopen(url, timeout=_GET_TIMEOUT_S) as resp:
            payload = resp.read()
    except urllib.error.HTTPError:
        raise
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise _SourceUnreachableError(f"{url}: {exc}") from exc
    if len(payload) < _MIN_POLICY_BYTES:
        raise RuntimeError(f"policy download looks truncated ({len(payload)} bytes)")
    partial = dest.with_name(dest.name + ".part")
    partial.write_bytes(payload)
    partial.replace(dest)
