"""Opt-in, real rootless acceptance for the spatial runner image."""

import os
from pathlib import Path
import subprocess
from zipfile import ZIP_DEFLATED, ZipFile

import pytest


def _wheel(directory: Path, name: str) -> Path:
    path = directory / f"{name}-1.0-py3-none-any.whl"
    info = f"{name}-1.0.dist-info"
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr(f"{name}/__init__.py", f'VALUE = "{name}"\n')
        archive.writestr(f"{info}/METADATA", f"Metadata-Version: 2.1\nName: {name}\nVersion: 1.0\n")
        archive.writestr(
            f"{info}/WHEEL", "Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: py3-none-any\n"
        )
        archive.writestr(f"{info}/RECORD", "")
    return path


@pytest.mark.skipif(
    os.environ.get("DIMOS_SPATIAL_RUNNER_INTEGRATION") != "1",
    reason="opt-in Podman integration smoke test",
)
def test_rootless_image_contract(tmp_path: Path) -> None:
    base = os.environ.get("DIMOS_SPATIAL_RUNNER_BASE_IMAGE")
    if not base:
        pytest.skip("set DIMOS_SPATIAL_RUNNER_BASE_IMAGE to an immutable base")
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    _wheel(input_dir, "pkg_a")
    _wheel(input_dir, "pkg_b")
    tag = f"dimos-spatial-runner-smoke:{os.getpid()}"
    env = {**os.environ, "DIMOS_SPATIAL_RUNNER_ALLOW_DIRTY": "1"}
    subprocess.run(
        [
            "python",
            "docker/pi-spatial-runner/build.py",
            "--base-image",
            base,
            "--tag",
            tag,
            "--allow-dirty",
        ],
        check=True,
        env=env,
    )
    containers: list[str] = []
    try:
        for attempt in (1, 2):
            name = f"dimos-spatial-accept-{os.getpid()}-{attempt}"
            containers.append(name)
            work = tmp_path / f"work-{attempt}"
            agent = tmp_path / f"agent-{attempt}"
            work.mkdir()
            agent.mkdir()
            os.chown(work, 1000, 1000)
            os.chown(agent, 1000, 1000)
            agent.chmod(0o700)
            # Exact equivalent of the repository RootlessPodman persistent case:
            # read-only root, constrained resources, all caps dropped, and the
            # three explicitly scoped mounts.
            command = (
                (
                    'set -eux; test "$(id -u)" != 0; '
                    "grep ' / .* ro,' /proc/self/mountinfo; grep ' /work .* rw,' /proc/self/mountinfo; "
                    "python -m venv --system-site-packages /agent/venv; export PATH=/agent/venv/bin:$PATH; "
                    "uv pip install /input/pkg_a-1.0-py3-none-any.whl; "
                    "python -m pip install /input/pkg_b-1.0-py3-none-any.whl; "
                    "python -c 'import importlib.util,sys,pkg_a,pkg_b,dimos; "
                    'assert sys.prefix == "/agent/venv" and pkg_a.__file__.startswith("/agent/venv") '
                    'and pkg_b.__file__.startswith("/agent/venv") and dimos.__file__.startswith("/usr/local"); '
                    'assert importlib.util.find_spec("pkg_a") and importlib.util.find_spec("pkg_b")\'; '
                    "touch /work/probe; rm /work/probe; command -v bash file find git grep gzip jq ps sed tar"
                )
                if attempt == 1
                else (
                    "set -eux; python -m venv --system-site-packages /agent/venv; export PATH=/agent/venv/bin:$PATH; "
                    'python -c \'import importlib.util,dimos; assert importlib.util.find_spec("pkg_a") is None '
                    'and importlib.util.find_spec("pkg_b") is None and dimos.__file__.startswith("/usr/local")\''
                )
            )
            subprocess.run(
                [
                    "podman",
                    "run",
                    "--name",
                    name,
                    "--rm",
                    "--read-only",
                    "--user",
                    "1000:1000",
                    "--userns=keep-id",
                    "--cap-drop",
                    "ALL",
                    "--security-opt",
                    "no-new-privileges",
                    "--http-proxy=false",
                    "--pids-limit",
                    "128",
                    "--memory",
                    "1g",
                    "--cpus",
                    "2",
                    "--mount",
                    f"type=bind,src={input_dir},dst=/input,ro",
                    "--mount",
                    f"type=bind,src={work},dst=/work,rw",
                    "--mount",
                    f"type=bind,src={agent},dst=/agent,rw,exec,nosuid,nodev",
                    tag,
                    "bash",
                    "-lc",
                    command,
                ],
                check=True,
            )
    finally:
        for name in containers:
            subprocess.run(["podman", "rm", "-f", name], check=False, capture_output=True)
        subprocess.run(["podman", "rmi", "-f", tag], check=False, capture_output=True)
