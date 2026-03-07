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

"""
Docker image building and Dockerfile conversion utilities.
Converts any Dockerfile into a DimOS module container by appending a footer
that installs DimOS and creates the module entrypoint.
"""

from __future__ import annotations

import hashlib
import subprocess
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.core.docker_runner import DockerModuleConfig

logger = setup_logger()

_BUILD_HASH_LABEL = "dimos.build.hash"

DOCKER_CMD_TIMEOUT = 20

# the way of detecting already-converted Dockerfiles (UUID ensures uniqueness)
DIMOS_SENTINEL = "DIMOS-MODULE-CONVERSION-427593ae-c6e8-4cf1-9b2d-ee81a420a5dc"

# Footer appended to Dockerfiles for DimOS module conversion
DIMOS_FOOTER = f"""
# ==== {DIMOS_SENTINEL} ====
# Copy DimOS source from build context
COPY dimos /dimos/source/dimos/
COPY pyproject.toml /dimos/source/
COPY docker/python/module-install.sh /tmp/module-install.sh

# Install DimOS and create entrypoint
RUN bash /tmp/module-install.sh /dimos/source && rm /tmp/module-install.sh

ENTRYPOINT ["/dimos/entrypoint.sh"]
"""


def _convert_dockerfile(dockerfile: Path) -> Path:
    """Append DimOS footer to Dockerfile. Returns path to converted file."""
    content = dockerfile.read_text()

    # Already converted?
    if DIMOS_SENTINEL in content:
        return dockerfile

    logger.info(f"Converting {dockerfile.name} to DimOS format")

    converted = dockerfile.parent / f".{dockerfile.name}.dimos"
    converted.write_text(content.rstrip() + "\n" + DIMOS_FOOTER.lstrip("\n"))
    return converted


def _compute_build_hash(cfg: DockerModuleConfig) -> str:
    """Hash Dockerfile contents, build args, and build context path."""
    assert cfg.docker_file is not None
    digest = hashlib.sha256()
    digest.update(cfg.docker_file.read_bytes())
    for key, val in sorted(cfg.docker_build_args.items()):
        digest.update(f"{key}={val}".encode())
    return digest.hexdigest()


def _get_image_build_hash(docker_bin: str, image_name: str) -> str | None:
    """Read the build hash label from an existing Docker image."""
    r = subprocess.run(
        [
            docker_bin,
            "image",
            "inspect",
            "-f",
            '{{index .Config.Labels "' + _BUILD_HASH_LABEL + '"}}',
            image_name,
        ],
        capture_output=True,
        text=True,
        timeout=DOCKER_CMD_TIMEOUT,
        check=False,
    )
    if r.returncode != 0:
        return None
    value = r.stdout.strip()
    # docker prints "<no value>" when the label is missing
    return value if value and value != "<no value>" else None


def build_image(cfg: DockerModuleConfig) -> None:
    """Build Docker image using footer mode conversion."""
    if cfg.docker_file is None:
        raise ValueError("docker_file is required for building Docker images")

    build_hash = _compute_build_hash(cfg)
    dockerfile = _convert_dockerfile(cfg.docker_file)

    context = cfg.docker_build_context or cfg.docker_file.parent
    cmd = [cfg.docker_bin, "build", "-t", cfg.docker_image, "-f", str(dockerfile)]
    cmd.extend(["--label", f"{_BUILD_HASH_LABEL}={build_hash}"])
    if cfg.docker_build_ssh:
        cmd.extend(["--ssh", "default"])
    for k, v in cfg.docker_build_args.items():
        cmd.extend(["--build-arg", f"{k}={v}"])
    cmd.append(str(context))

    logger.info(f"Building Docker image: {cfg.docker_image}")
    result = subprocess.run(cmd, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Docker build failed with exit code {result.returncode}")


def image_exists(cfg: DockerModuleConfig) -> bool:
    """Check if the configured Docker image exists locally."""
    r = subprocess.run(
        [cfg.docker_bin, "image", "inspect", cfg.docker_image],
        capture_output=True,
        text=True,
        timeout=DOCKER_CMD_TIMEOUT,
        check=False,
    )
    return r.returncode == 0


__all__ = [
    "DIMOS_FOOTER",
    "_compute_build_hash",
    "_get_image_build_hash",
    "build_image",
    "image_exists",
]
