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

from pathlib import Path
import sys

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps import policy as policy_module
from dimos.deps.policy import (
    UvPolicy,
    find_project_file,
    load_constraints,
    load_uv_policy,
    packages_outside,
    render_wheel_project,
)
from dimos.deps.profiles import PROFILES

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

POLICY = UvPolicy(
    required_version=">=0.9.25",
    override_dependencies=("opencv-python; sys_platform == 'never'", "torch>=2.1,<2.8"),
    constraint_dependencies=("onnxruntime<1.24; python_full_version < '3.11'",),
    sources={
        "graspgenx": {"git": "https://github.com/NVlabs/GraspGenX.git", "rev": "b942909"},
        "torch": [{"index": "pytorch-cu128", "marker": "sys_platform == 'linux'"}],
    },
    indexes=(
        {
            "name": "pytorch-cu128",
            "url": "https://download.pytorch.org/whl/cu128",
            "explicit": True,
        },
    ),
)


def test_render_wheel_project_applies_the_policy() -> None:
    text = render_wheel_project(
        "0.0.14", ["web", "unitree"], "3.12", PROFILES["linux-x86_64-cpu"], POLICY
    )
    document = tomllib.loads(text)
    assert document["project"]["dependencies"] == ["dimos[unitree,web]==0.0.14"]
    assert document["project"]["requires-python"] == "==3.12.*"
    uv = document["tool"]["uv"]
    assert uv["package"] is False and uv["required-version"] == ">=0.9.25"
    assert uv["environments"] == ["sys_platform == 'linux' and platform_machine == 'x86_64'"]
    # The git source is not an override in the repository, so it becomes one here
    # to bind the transitive requirement; torch already is one.
    assert uv["override-dependencies"] == [
        "opencv-python; sys_platform == 'never'",
        "torch>=2.1,<2.8",
        "graspgenx @ git+https://github.com/NVlabs/GraspGenX.git@b942909",
    ]
    assert uv["constraint-dependencies"] == ["onnxruntime<1.24; python_full_version < '3.11'"]
    assert uv["sources"]["torch"] == [
        {"index": "pytorch-cu128", "marker": "sys_platform == 'linux'"}
    ]
    assert uv["index"] == [
        {"name": "pytorch-cu128", "url": "https://download.pytorch.org/whl/cu128", "explicit": True}
    ]
    core_only = render_wheel_project("0.0.14", [], "3.10", PROFILES["macos-arm64-cpu"], POLICY)
    assert tomllib.loads(core_only)["project"]["dependencies"] == ["dimos==0.0.14"]


def test_render_wheel_project_applies_constraints() -> None:
    constraints = (
        "numpy==2.3.5",
        "torch==2.7.1+cu128 ; sys_platform == 'linux' and platform_machine == 'x86_64'",
    )
    text = render_wheel_project(
        "0.0.14", [], "3.12", PROFILES["linux-x86_64-cpu"], POLICY, constraints
    )
    uv = tomllib.loads(text)["tool"]["uv"]
    assert uv["constraint-dependencies"] == [
        "onnxruntime<1.24; python_full_version < '3.11'",
        *constraints,
    ]


def test_load_constraints(tmp_path: Path) -> None:
    path = tmp_path / "constraints.txt"
    path.write_text("# header\n\nnumpy==2.3.5\ntorch==2.7.1 ; sys_platform == 'linux'\n")
    assert load_constraints(path) == ("numpy==2.3.5", "torch==2.7.1 ; sys_platform == 'linux'")
    path.write_text("pkg @ git+https://example.com/pkg.git\n")
    with pytest.raises(ValueError, match="unsupported constraint line"):
        load_constraints(path)


def test_packages_outside_the_tested_set(tmp_path: Path) -> None:
    lock = tmp_path / "uv.lock"
    lock.write_text(
        "version = 1\n"
        '[[package]]\nname = "dimos-managed-env"\nversion = "0"\n'
        '[[package]]\nname = "dimos"\nversion = "0.0.14"\n'
        '[[package]]\nname = "NumPy"\nversion = "2.3.5"\n'
        '[[package]]\nname = "surprise"\nversion = "1"\n'
    )
    assert packages_outside(lock, ("numpy==2.3.5",)) == ["surprise"]
    assert packages_outside(lock, ("numpy==2.3.5", "surprise==1")) == []


def test_repository_policy_loads() -> None:
    path = find_project_file(DIMOS_PROJECT_ROOT)
    assert path == DIMOS_PROJECT_ROOT / "pyproject.toml"
    policy = load_uv_policy(path)
    assert "opencv-python; sys_platform == 'never'" in policy.override_dependencies
    assert any(index.get("name") == "pytorch-cu128" for index in policy.indexes)
    assert "torch" in policy.sources


def test_find_project_file_falls_back_to_the_shipped_copy(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "pyproject.toml").write_text('[project]\nname = "other"\n')
    monkeypatch.setattr(policy_module, "SHIPPED_PROJECT_FILE", tmp_path / "missing.toml")
    assert find_project_file(tmp_path) is None
    shipped = tmp_path / "shipped.toml"
    shipped.write_text('[project]\nname = "dimos"\n')
    monkeypatch.setattr(policy_module, "SHIPPED_PROJECT_FILE", shipped)
    assert find_project_file(tmp_path) == shipped
