#!/usr/bin/env python3
# Helper for generating docker-based dev environment assets.
from __future__ import annotations

from pathlib import Path
from typing import Iterable

from . import prompt_tools as p
from .shell_tooling import run_command


DOCKERFILE_TEMPLATE = r"""FFROM ubuntu:22.04

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install basic requirements
RUN apt-get update && apt-get install -y \
    python-is-python3 \
    python3-venv \
    curl \
    lsb-release \
    gnupg2 \
    bash \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/root/.local/bin:$PATH"

# Run your installer (now "python"/"pip" point at the venv by default because of PATH)
# RUN bash -lc 'sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/refs/heads/master/install") --non-interactive --no-env-setup'
ARG DIMOS_FEATURES_FOR_DOCKER=""
ENV DIMOS_REF_FOR_DOCKER="refs/heads/master"
RUN bash -lc 'sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/$DIMOS_REF_FOR_DOCKER/install") --just-system-install --non-interactive --features "$DIMOS_FEATURES_FOR_DOCKER"'

RUN echo '# Dimos auto-setup'                                                                                                                                                                                                >> "$HOME/.bashrc" && \
    echo 'if [ ! -d "$PWD/.dimos" ]; then'                                                                                                                                                                                   >> "$HOME/.bashrc" && \
    echo '    sh <(curl -fsSL "https://raw.githubusercontent.com/jeff-hykin/mystery_test_1/$DIMOS_REF_FOR_DOCKER/install") --no-system-install --non-interactive --features "$DIMOS_FEATURES_FOR_DOCKER"' >> "$HOME/.bashrc" && \
    echo '    touch "$PWD/.dimos"'                                                                                                                                                                                           >> "$HOME/.bashrc" && \
    echo 'fi'                                                                                                                                                                                                                >> "$HOME/.bashrc" && \
    echo ''                                                                                                                                                                                                                  >> "$HOME/.bashrc" && \
    echo '# Activate virtualenv if present'                                                                                                                                                                                  >> "$HOME/.bashrc" && \
    echo 'if [ -f "$PWD/venv/bin/activate" ]; then'                                                                                                                                                                          >> "$HOME/.bashrc" && \
    echo '    . "$PWD/venv/bin/activate"'                                                                                                                                                                                    >> "$HOME/.bashrc" && \
    echo 'fi'                                                                                                                                                                                                                >> "$HOME/.bashrc" && \
    :

COPY ./ /app/
WORKDIR /app/

# Start an interactive login shell by default (loads /etc/profile.d/activate-venv.sh)
CMD ["bash", "-l"]
"""


def _write_file(path: Path, content: str) -> None:
    path.write_text(content)


def _maybe_write(path: Path, content: str) -> bool:
    if path.exists():
        if not p.ask_yes_no(f"{path.name} already exists. Overwrite?"):
            return False
    _write_file(path, content)
    return True


def _env_block(features: Iterable[str]) -> str:
    feature_str = ",".join(features)
    return f"DIMOS_FEATURES_FOR_DOCKER=\"{feature_str}\"\n"


def _script_export_env() -> str:
    return 'if [ -f ".env" ]; then export $(grep -v "^#" .env | xargs); fi\nexport DIMOS_FEATURES_FOR_DOCKER="${DIMOS_FEATURES_FOR_DOCKER:-}"\n'


def _build_script() -> str:
    return (
        "#!/usr/bin/env bash\n"
        "set -e\n"
        + _script_export_env()
        + 'docker build --build-arg DIMOS_FEATURES_FOR_DOCKER="$DIMOS_FEATURES_FOR_DOCKER" -t dimos-dev .\n'
    )


def _exec_script() -> str:
    return (
        "#!/usr/bin/env bash\n"
        "set -e\n"
        + _script_export_env()
        + 'docker run --rm -it -v "$PWD:/workspace" -w /workspace dimos-dev bash -l\n'
    )


def setup_docker_env(project_dir: str | Path, features: Iterable[str]) -> dict[str, Path]:
    """Generate Dockerfile, run_build.sh, run_exec.sh, and .env with features."""
    project_dir = Path(project_dir)
    dockerfile_path = project_dir / "Dockerfile"
    env_path = project_dir / ".env"
    build_script_path = project_dir / "run" / "docker_build"
    exec_script_path = project_dir / "run" / "docker_exec"

    _maybe_write(dockerfile_path, DOCKERFILE_TEMPLATE)
    if not env_path.exists():
        _write_file(env_path, _env_block(features))
    else:
        if p.ask_yes_no(f"{env_path.name} exists. Overwrite with current features?"):
            _write_file(env_path, _env_block(features))

    _maybe_write(build_script_path, _build_script())
    _maybe_write(exec_script_path, _exec_script())

    for script in (build_script_path, exec_script_path):
        try:
            script.chmod(script.stat().st_mode | 0o111)
        except Exception:
            pass

    return {
        "dockerfile": dockerfile_path,
        "env": env_path,
        "build_script": build_script_path,
        "exec_script": exec_script_path,
    }


__all__ = ["setup_docker_env", "DOCKERFILE_TEMPLATE"]
