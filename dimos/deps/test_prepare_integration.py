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

"""Clean-install coverage: fresh virtualenv, core dimos, ``dimos prepare``, then use it.

Slow and network-bound (first run downloads every bundle), so these tests carry the
``clean_install`` marker and run in the install workflow. Locally::

    uv run pytest -m clean_install dimos/deps/test_prepare_integration.py

``DIMOS_CLEAN_INSTALL_BACKEND`` (cpu, the default, or cuda) selects the backend and
``DIMOS_CLEAN_INSTALL_BUNDLES`` (comma separated) narrows the bundles; by default every
bundle the host supports is covered.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.backend import UnsupportedError, check_supported
from dimos.deps.bundles import load_assignments, lock_path
from dimos.deps.export_locks import BACKENDS

pytestmark = [pytest.mark.clean_install, pytest.mark.timeout(3600)]

BACKEND = os.environ.get("DIMOS_CLEAN_INSTALL_BACKEND", "cpu")
REPRESENTATIVE = {
    "runtime-common": "replay",
    "runtime-unitree": "unitree-go2",
    "runtime-manipulation": "xarm7-planner-coordinator",
    "runtime-unitree-dds": "unitree-g1-teleop",
    "runtime-drone": "drone-basic",
    "runtime-spot": "spot",
}
# A tiny pure-Python package outside uv.lock: it must survive repeated preparation.
SENTINEL = ("termcolor", "termcolor")


def _supported_bundles() -> list[str]:
    requested = os.environ.get("DIMOS_CLEAN_INSTALL_BUNDLES")
    bundles = requested.split(",") if requested else sorted(set(load_assignments().values()))
    supported = []
    for bundle in bundles:
        try:
            check_supported([bundle], BACKEND)  # type: ignore[arg-type]
        except UnsupportedError:
            continue
        supported.append(bundle)
    return supported


BUNDLES = _supported_bundles()
UV = shutil.which("uv")
assert UV, "uv is required for clean-install tests"


@dataclass
class Venv:
    root: Path
    home: Path
    cache: Path

    @property
    def python(self) -> Path:
        return self.root / "bin" / "python"

    @property
    def dimos(self) -> Path:
        return self.root / "bin" / "dimos"

    def env(self) -> dict[str, str]:
        # PYTHONPATH may point at system site-packages (ROS), which must not leak in.
        env = {
            key: value
            for key, value in os.environ.items()
            if key not in ("VIRTUAL_ENV", "PYTHONPATH", "PYTHONHOME") and not key.startswith("UV_")
        }
        env.update(
            HOME=str(self.home),
            XDG_STATE_HOME=str(self.home / "state"),
            XDG_CACHE_HOME=str(self.home / "cache"),
            XDG_CONFIG_HOME=str(self.home / "config"),
            UV_CACHE_DIR=str(self.cache),
            UV_PROJECT_ENVIRONMENT=str(self.root),
            LCM_DEFAULT_URL="memq://",
        )
        return env

    def run(self, *command: str | Path, check: bool = True) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            [str(part) for part in command], env=self.env(), capture_output=True, text=True
        )
        if check and result.returncode:
            pytest.fail(
                f"{command[0]} failed ({result.returncode}):\n{result.stdout}\n{result.stderr}"
            )
        return result

    def prepare(self, *names: str, offline: bool = False, check: bool = True):  # type: ignore[no-untyped-def]
        command = [self.dimos, "prepare", *names, "--backend", BACKEND]
        if offline:
            command.append("--offline")
        return self.run(*command, check=check)

    def verify(self, bundle: str) -> str:
        result = self.run(
            self.python, "-m", "dimos.deps.verify_bundle", bundle, "--backend", BACKEND
        )
        assert f"verified {bundle}" in result.stdout, result.stdout
        return result.stdout


def _shared_cache() -> Path:
    return Path(os.environ.get("UV_CACHE_DIR") or Path.home() / ".cache" / "uv")


def _new_venv(tmp_path: Path, cache: Path) -> Venv:
    venv = Venv(root=tmp_path / "venv", home=tmp_path / "home", cache=cache)
    venv.home.mkdir()
    venv.run(UV, "venv", "--python", sys.executable, venv.root)
    return venv


def _install_core_from_checkout(venv: Venv) -> None:
    venv.run(
        UV,
        "sync",
        "--locked",
        "--no-default-groups",
        "--project",
        DIMOS_PROJECT_ROOT,
        "--python",
        venv.python,
    )


@pytest.fixture(scope="module")
def wheel(tmp_path_factory: pytest.TempPathFactory) -> Path:
    out = tmp_path_factory.mktemp("wheel")
    env = {**os.environ, "DIMOS_ALLOW_MISSING_COCKPIT": "1"}
    subprocess.run(
        [UV, "build", "--wheel", "--out-dir", str(out)], cwd=DIMOS_PROJECT_ROOT, env=env, check=True
    )
    return next(out.glob("dimos-*.whl"))


@pytest.fixture(scope="module", params=BUNDLES)
def prepared(
    request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory
) -> Iterator[Venv]:
    """A fresh checkout-mode venv with one bundle prepared, verified, and prepared again."""
    bundle = request.param
    venv = _new_venv(tmp_path_factory.mktemp(bundle), _shared_cache())
    _install_core_from_checkout(venv)
    venv.bundle = bundle  # type: ignore[attr-defined]
    yield venv


def test_prepare_installs_and_the_bundle_works(prepared: Venv) -> None:
    bundle: str = prepared.bundle  # type: ignore[attr-defined]
    first = prepared.prepare(REPRESENTATIVE[bundle])
    assert "Installed Python dependencies for" in first.stdout
    assert "not verified" in first.stdout
    assert "verified: opencv-contrib and onnxruntime" in first.stdout

    prepared.verify(bundle)


def test_repeated_prepare_keeps_unrelated_packages(prepared: Venv) -> None:
    bundle: str = prepared.bundle  # type: ignore[attr-defined]
    distribution, module = SENTINEL
    prepared.run(UV, "pip", "install", "--python", prepared.python, distribution)

    prepared.prepare(REPRESENTATIVE[bundle])

    prepared.run(prepared.python, "-c", f"import {module}")
    prepared.verify(bundle)


def test_offline_prepare_succeeds_from_the_warm_cache(prepared: Venv) -> None:
    bundle: str = prepared.bundle  # type: ignore[attr-defined]

    result = prepared.prepare(REPRESENTATIVE[bundle], offline=True)

    assert "Installed Python dependencies for" in result.stdout


def test_offline_prepare_fails_with_an_empty_cache(tmp_path: Path) -> None:
    venv = _new_venv(tmp_path, _shared_cache())
    _install_core_from_checkout(venv)
    venv.cache = tmp_path / "empty-cache"
    venv.cache.mkdir()

    result = venv.prepare("drone-basic", offline=True, check=False)

    assert result.returncode != 0
    assert "installer exited with status" in result.stdout + result.stderr


@pytest.mark.skipif(
    not {"runtime-unitree", "runtime-drone"} <= set(BUNDLES), reason="pair not selected"
)
@pytest.mark.parametrize("order", [("unitree-go2", "drone-basic"), ("drone-basic", "unitree-go2")])
def test_bundles_accumulate_in_either_order(tmp_path: Path, order: tuple[str, str]) -> None:
    venv = _new_venv(tmp_path, _shared_cache())
    _install_core_from_checkout(venv)

    for name in order:
        venv.prepare(name)

    venv.verify("runtime-unitree")
    venv.verify("runtime-drone")


def test_wheel_prepares_from_packaged_locks_outside_the_checkout(
    tmp_path: Path, wheel: Path
) -> None:
    venv = _new_venv(tmp_path, _shared_cache())
    venv.run(UV, "pip", "install", "--python", venv.python, wheel)
    decoy = tmp_path / "decoy"
    decoy.mkdir()
    (decoy / "pyproject.toml").write_text('[project]\nname = "dimos"\nversion = "0"\n')

    shown = subprocess.run(
        [venv.dimos, "deps", "drone-basic", "--backend", BACKEND],
        cwd=decoy,
        env=venv.env(),
        capture_output=True,
        text=True,
        check=True,
    )
    assert "packaged lock artifacts of dimos" in shown.stdout
    installed = subprocess.run(
        [venv.dimos, "prepare", "drone-basic", "--backend", BACKEND],
        cwd=decoy,
        env=venv.env(),
        capture_output=True,
        text=True,
        check=True,
    )
    assert "pylock.runtime-drone" in installed.stdout

    venv.verify("runtime-drone")


def test_sdist_to_wheel_keeps_the_dependency_artifacts(tmp_path: Path) -> None:
    env = {**os.environ, "DIMOS_ALLOW_MISSING_COCKPIT": "1"}
    subprocess.run(
        [UV, "build", "--sdist", "--out-dir", str(tmp_path)],
        cwd=DIMOS_PROJECT_ROOT,
        env=env,
        check=True,
    )
    sdist = next(tmp_path.glob("dimos-*.tar.gz"))
    subprocess.run(
        [UV, "build", "--wheel", "--out-dir", str(tmp_path / "wheel"), str(sdist)],
        cwd=tmp_path,
        env=env,
        check=True,
    )
    wheel = next((tmp_path / "wheel").glob("dimos-*.whl"))

    names = set(zipfile.ZipFile(wheel).namelist())

    assert "dimos/deps/bundles.json" in names
    assert "dimos/deps/verify_bundle.py" in names
    expected_locks = {
        f"dimos/deps/locks/{lock_path(bundle, backend).name}"
        for bundle in set(load_assignments().values())
        for backend in BACKENDS
    }
    assert expected_locks <= names
