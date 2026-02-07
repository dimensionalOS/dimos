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

import subprocess
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.core.docker_runner import DockerModuleConfig

logger = setup_logger()

# Timeout for quick Docker commands
DOCKER_CMD_TIMEOUT = 20

# Sentinel value to detect already-converted Dockerfiles (UUID ensures uniqueness)
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


def _run(cmd: list[str], *, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    """Run a command and return the result."""
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)


def _run_streaming(cmd: list[str]) -> int:
    """Run command and stream output to terminal. Returns exit code."""
    result = subprocess.run(cmd, text=True)
    return result.returncode


def _docker_bin(cfg: DockerModuleConfig) -> str:
    """Get docker binary path."""
    return cfg.docker_bin or "docker"


def _image_exists(docker_bin: str, image_name: str) -> bool:
    """Check if a Docker image exists locally."""
    r = _run([docker_bin, "image", "inspect", image_name], timeout=DOCKER_CMD_TIMEOUT)
    return r.returncode == 0


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


def build_image(cfg: DockerModuleConfig) -> None:
    """Build Docker image using footer mode conversion."""
    if cfg.docker_file is None:
        raise ValueError("docker_file is required for building Docker images")
    dockerfile = _convert_dockerfile(cfg.docker_file)

    context = cfg.docker_build_context or cfg.docker_file.parent
    cmd = [_docker_bin(cfg), "build", "-t", cfg.docker_image, "-f", str(dockerfile)]
    for k, v in cfg.docker_build_args.items():
        cmd.extend(["--build-arg", f"{k}={v}"])
    cmd.append(str(context))

    logger.info(f"Building Docker image: {cfg.docker_image}")
    exit_code = _run_streaming(cmd)
    if exit_code != 0:
        raise RuntimeError(f"Docker build failed with exit code {exit_code}")


def image_exists(cfg: DockerModuleConfig) -> bool:
    """Check if the configured Docker image exists locally."""
    return _image_exists(_docker_bin(cfg), cfg.docker_image)


__all__ = [
    "DIMOS_FOOTER",
    "build_image",
    "image_exists",
]
