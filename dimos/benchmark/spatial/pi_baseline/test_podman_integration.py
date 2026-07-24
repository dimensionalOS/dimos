"""Opt-in rootless Podman contract test (never pulls or builds an image)."""

from __future__ import annotations

import os
from pathlib import Path
from threading import Event
from zipfile import ZIP_DEFLATED, ZipFile

import pytest

from .podman import _DIGEST, PodmanRun, RootlessPodman
from .topology import PinnedRuntimeTopology, pin_runtime_topology


def _write_wheel(directory: Path, package: str) -> Path:
    """Write a minimal offline wheel without invoking a build backend."""
    normalized = package.replace("-", "_")
    filename = f"{normalized}-1.0-py3-none-any.whl"
    dist_info = f"{normalized}-1.0.dist-info"
    path = directory / filename
    with ZipFile(path, "w", ZIP_DEFLATED) as wheel:
        wheel.writestr(f"{normalized}/__init__.py", f'ORIGIN = "{package}"\n')
        wheel.writestr(
            f"{dist_info}/METADATA",
            f"Metadata-Version: 2.1\nName: {package}\nVersion: 1.0\n",
        )
        wheel.writestr(
            f"{dist_info}/WHEEL",
            "Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: py3-none-any\n",
        )
        wheel.writestr(f"{dist_info}/RECORD", "")
    return path


def _runtime_topology(root: Path, attempt: str) -> PinnedRuntimeTopology:
    paths = {name: root / f"{attempt}-{name}" for name in ("input", "work", "output", "private")}
    for path in paths.values():
        path.mkdir()
    _write_wheel(paths["input"], "pi_marker_a")
    _write_wheel(paths["input"], "pi_marker_b")
    return pin_runtime_topology(
        input_dir=paths["input"],
        workspace_dir=paths["work"],
        output_dir=paths["output"],
        private_dir=paths["private"],
    )


def test_real_rootless_podman_contract(tmp_path: Path) -> None:
    """Exercise the same persistent lifecycle used by the runner.

    Supply DIMOS_SPATIAL_RUNNER_IMAGE with a named immutable digest, for example
    after a deterministic local registry publish:
    ``podman tag <local-image> localhost:5000/pi-runner:contract`` followed by
    ``podman push localhost:5000/pi-runner:contract`` and use the resulting
    ``localhost:5000/pi-runner@sha256:<RepoDigest>`` value here.
    """
    image = os.environ.get("DIMOS_SPATIAL_RUNNER_IMAGE")
    if image is None or _DIGEST.fullmatch(image) is None:
        pytest.skip("DIMOS_SPATIAL_RUNNER_IMAGE is not a named immutable digest")

    podman = RootlessPodman()
    if not podman.is_rootless(Event()):
        pytest.skip("Podman is not rootless")

    def exercise_case(attempt: str, install: bool) -> None:
        topology = _runtime_topology(tmp_path, attempt)
        request = PodmanRun(image, f"integration-{attempt}", topology)
        try:
            with podman.persistent(request, Event()) as case:
                command = (
                    "export PATH=/agent/venv/bin:$PATH; "
                    'test "$(id -u):$(id -g)" = 1000:1000; '
                    "command -v sh && command -v python && command -v uv; "
                )
                if install:
                    command += (
                        "uv pip install /input/pi_marker_a-1.0-py3-none-any.whl; "
                        "python -m pip install /input/pi_marker_b-1.0-py3-none-any.whl; "
                        "python -c 'import pathlib,sys,pi_marker_a,pi_marker_b; "
                        'assert sys.prefix == "/agent/venv"; '
                        'assert pathlib.Path(pi_marker_a.__file__).is_relative_to("/agent/venv"); '
                        'assert pathlib.Path(pi_marker_b.__file__).is_relative_to("/agent/venv")\'; '
                    )
                else:
                    command += (
                        "python -c 'import importlib.util; "
                        'assert importlib.util.find_spec("pi_marker_a") is None; '
                        'assert importlib.util.find_spec("pi_marker_b") is None\'; '
                    )
                command += (
                    "python -c 'import dimos.core.module,pathlib,sysconfig; "
                    "origin=pathlib.Path(dimos.core.module.__file__); "
                    'global_site=pathlib.Path(sysconfig.get_path("purelib", '
                    'vars={"base":"/usr/local","platbase":"/usr/local"})); '
                    "assert origin.is_relative_to(global_site)'"
                )
                result = case.exec(command)
                assert result.returncode == 0, result.stderr
                clean = case.exec(
                    "python -c 'from pathlib import Path; "
                    'bad=(".venv",".cache","cache","site-packages"); '
                    'assert not any(p.name in bad or p.suffix == ".whl" for p in Path("/work").rglob("*"))\''
                )
                assert clean.returncode == 0, clean.stderr
        finally:
            topology.close()
        assert podman.verify_removed(request.run_id)

    exercise_case("a", install=True)
    exercise_case("b", install=False)
