#!/usr/bin/env python3
# Helper for generating docker-based dev environment assets.
from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from . import prompt_tools as p
from .bundled_data import DOCKERFILE_TEMPLATE
from .shell_tooling import run_command

if TYPE_CHECKING:
    from collections.abc import Iterable


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
    return f"DIMOS_ENABLED_FEATURES=\"{feature_str}\"\n"


def _script_export_env() -> str:
    return 'if [ -f ".env" ]; then export $(grep -v "^#" .env | xargs); fi\nexport DIMOS_ENABLED_FEATURES="${DIMOS_ENABLED_FEATURES:-}"\n'


def _build_script() -> str:
    return (
        "#!/usr/bin/env bash\n"
        "set -e\n"
        + _script_export_env()
        + 'docker build --build-arg DIMOS_ENABLED_FEATURES="$DIMOS_ENABLED_FEATURES" -t dimos-dev .\n'
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


__all__ = ["DOCKERFILE_TEMPLATE", "setup_docker_env"]
