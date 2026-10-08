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

from collections.abc import Callable
from pathlib import Path
import re
import sys

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps import backend as backend_module
from dimos.deps.backend import (
    DDS_BUNDLES,
    PLANNING_BUNDLES,
    UnsupportedError,
    check_supported,
    limitations,
    resolve_backend,
)
from dimos.deps.bundles import load_assignments

if sys.version_info >= (3, 11):
    import tomllib
else:  # pytest depends on tomli below 3.11
    import tomli as tomllib

FakeHost = Callable[..., None]


@pytest.fixture
def fake_host(monkeypatch: pytest.MonkeyPatch) -> FakeHost:
    def configure(
        platform_name: str, machine: str, cuda_major: int = 0, python: str = "3.12"
    ) -> None:
        monkeypatch.setattr(backend_module, "host", lambda: (platform_name, machine))
        monkeypatch.setattr(backend_module, "detect_cuda_major", lambda: cuda_major)
        monkeypatch.setattr(backend_module, "python_version", lambda: python)

    return configure


def test_auto_picks_cuda_with_a_cuda_12_driver(fake_host: FakeHost) -> None:
    fake_host("linux", "x86_64", cuda_major=12)

    assert resolve_backend("auto") == ("cuda", "auto: the NVIDIA driver supports CUDA 12.x")


def test_auto_picks_cpu_without_a_driver(fake_host: FakeHost) -> None:
    fake_host("linux", "x86_64", cuda_major=0)

    assert resolve_backend("auto") == ("cpu", "auto: no NVIDIA driver detected")


def test_auto_picks_cpu_with_an_old_driver(fake_host: FakeHost) -> None:
    fake_host("linux", "x86_64", cuda_major=11)

    backend, reason = resolve_backend("auto")

    assert backend == "cpu"
    assert "CUDA 11.x" in reason


def test_auto_is_cpu_off_linux_x86_64(fake_host: FakeHost) -> None:
    fake_host("darwin", "arm64")

    assert resolve_backend("auto") == ("cpu", "auto: the cuda backend is Linux x86_64 only")


def test_explicit_cpu_is_honoured_even_with_a_gpu(fake_host: FakeHost) -> None:
    fake_host("linux", "x86_64", cuda_major=12)

    assert resolve_backend("cpu") == ("cpu", "requested")


def test_explicit_cuda_off_linux_x86_64_is_unsupported(fake_host: FakeHost) -> None:
    fake_host("linux", "aarch64", cuda_major=12)

    with pytest.raises(UnsupportedError, match="Linux x86_64 only"):
        resolve_backend("cuda")


def test_explicit_cuda_without_a_driver_proceeds(fake_host: FakeHost) -> None:
    fake_host("linux", "x86_64", cuda_major=0)

    backend, reason = resolve_backend("cuda")

    assert backend == "cuda"
    assert "no NVIDIA driver" in reason


def test_invalid_backend_choice_is_rejected(fake_host: FakeHost) -> None:
    fake_host("linux", "x86_64")

    with pytest.raises(ValueError, match="must be one of"):
        resolve_backend("tpu")


def test_unknown_hosts_are_unsupported(fake_host: FakeHost) -> None:
    fake_host("win32", "AMD64")

    with pytest.raises(UnsupportedError, match="not a supported host"):
        check_supported(["runtime-common"], "cpu")


def test_cuda_backend_is_rejected_off_linux_x86_64(fake_host: FakeHost) -> None:
    fake_host("darwin", "arm64")

    with pytest.raises(UnsupportedError, match="Linux x86_64 only"):
        check_supported(["runtime-common"], "cuda")


def test_planning_bundles_need_python_312_on_apple_silicon(fake_host: FakeHost) -> None:
    fake_host("darwin", "arm64", python="3.11")

    with pytest.raises(UnsupportedError, match="Drake 1.45.0"):
        check_supported(["runtime-manipulation"], "cpu")
    check_supported(["runtime-unitree"], "cpu")


def test_planning_bundles_pass_with_python_312_on_apple_silicon(fake_host: FakeHost) -> None:
    fake_host("darwin", "arm64", python="3.12")

    check_supported(["runtime-manipulation"], "cpu")


def test_dds_bundle_needs_cyclonedds_home_without_a_wheel(
    fake_host: FakeHost, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    fake_host("linux", "x86_64", python="3.12")
    monkeypatch.delenv("CYCLONEDDS_HOME", raising=False)

    with pytest.raises(UnsupportedError, match="CycloneDDS C library"):
        check_supported(["runtime-unitree-dds"], "cpu")

    monkeypatch.setenv("CYCLONEDDS_HOME", str(tmp_path))
    check_supported(["runtime-unitree-dds"], "cpu")


def test_dds_bundle_installs_from_wheels_on_python_310_linux_x86_64(
    fake_host: FakeHost, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake_host("linux", "x86_64", python="3.10")
    monkeypatch.delenv("CYCLONEDDS_HOME", raising=False)

    check_supported(["runtime-unitree-dds"], "cpu")


def test_cyclonedds_home_must_be_an_existing_directory(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setenv("CYCLONEDDS_HOME", str(tmp_path / "absent"))

    assert backend_module.cyclonedds_home_configured() is False


def test_limitations_mention_drake_on_linux_aarch64(fake_host: FakeHost) -> None:
    fake_host("linux", "aarch64")

    notes = limitations(["runtime-manipulation"], "cpu")

    assert any("Drake has no Linux aarch64 wheel" in note for note in notes)
    assert any("Jetson CUDA is not supported" in note for note in notes)


def test_limitations_for_common_bundle_do_not_mention_arms(fake_host: FakeHost) -> None:
    fake_host("linux", "x86_64", python="3.12")

    notes = limitations(["runtime-common"], "cpu")

    assert not any("A750" in note or "A1Z" in note for note in notes)
    assert notes[0] == "Linux x86_64 installs CUDA-capable PyTorch wheels even with --backend cpu"


def _expanded_extras() -> dict[str, set[str]]:
    """Every extra in pyproject.toml with its self-referenced extras expanded."""
    with (DIMOS_PROJECT_ROOT / "pyproject.toml").open("rb") as f:
        declared = tomllib.load(f)["project"]["optional-dependencies"]

    def expand(extra: str) -> set[str]:
        result = {extra}
        for requirement in declared[extra]:
            match = re.fullmatch(r"dimos\[([^\]]+)\]", requirement)
            if match:
                for included in match.group(1).split(","):
                    result |= expand(included.strip())
        return result

    return {extra: expand(extra) for extra in declared}


def test_restriction_tables_match_pyproject_bundle_contents() -> None:
    extras = _expanded_extras()
    bundles = sorted(set(load_assignments().values()))

    planning = {bundle for bundle in bundles if "planning" in extras[bundle]}
    dds = {bundle for bundle in bundles if "unitree-dds" in extras[bundle]}

    assert planning == set(PLANNING_BUNDLES)
    assert dds == set(DDS_BUNDLES)
    assert not any({"cpu", "cuda"} & extras[bundle] for bundle in bundles)
