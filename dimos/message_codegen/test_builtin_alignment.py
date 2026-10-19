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

import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from dimos_cdrgen.source_package import prepare
import pytest


@pytest.fixture(scope="module")
def aligned_checkout(tmp_path_factory):
    root = Path(__file__).resolve().parents[2]
    checkout = tmp_path_factory.mktemp("aligned-source")
    ignore = shutil.ignore_patterns("build", "dist", "*.egg-info", "__pycache__", "target")
    for name in ["dimos/message_codegen", "packages/dimos-generated"]:
        shutil.copytree(root / name, checkout / name, ignore=ignore)
    (checkout / "dimos/__init__.py").write_text("")
    (checkout / "scripts").mkdir()
    shutil.copyfile(
        root / "scripts/generate_builtin_messages.py",
        checkout / "scripts/generate_builtin_messages.py",
    )
    shutil.copyfile(root / "pyproject.toml", checkout / "pyproject.toml")
    result = run_generation(checkout, check=False)
    assert result.returncode == 0, result.stderr
    result = run_generation(checkout, check=True)
    assert result.returncode == 0, result.stderr
    return checkout


def run_generation(checkout, *, check):
    environment = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "scripts.generate_builtin_messages",
            *(["--check"] if check else []),
        ],
        cwd=checkout,
        env=environment,
        text=True,
        capture_output=True,
    )


@pytest.mark.parametrize(
    "change",
    [
        "field",
        "schema",
        "new_type",
        "deleted_type",
        "renamed_type",
        "version",
        "extra_output",
        "missing_python",
        "missing_cpp",
        "missing_rust",
        "missing_rust_generator",
        "missing_rust_definition",
        "missing_schema",
    ],
)
def test_independent_regeneration_rejects_and_repairs_drift(aligned_checkout, tmp_path, change):
    checkout = tmp_path / "checkout"
    shutil.copytree(aligned_checkout, checkout)
    definitions = checkout / "packages/dimos-generated/interfaces"
    output = checkout / "packages/dimos-generated/build/message-source/python"
    point = definitions / "geometry_msgs/msg/Point.msg"
    if change == "field":
        point.write_text(point.read_text().replace("float64 x", "float32 x"))
    elif change == "schema":
        point.write_text(point.read_text() + "\n# Changed source schema comment\n")
    elif change == "new_type":
        (definitions / "alignment_msgs/msg").mkdir(parents=True)
        (definitions / "alignment_msgs/msg/AlignmentProbe.msg").write_text(
            "std_msgs/Header header\nfloat64 value\n"
        )
        catalog = checkout / "packages/dimos-generated/catalog.json"
        data = json.loads(catalog.read_text())
        data["roots"]["alignment_msgs/msg/AlignmentProbe"] = "Explicit test contract"
        catalog.write_text(json.dumps(data))
    elif change == "deleted_type":
        (definitions / "std_msgs/msg/Empty.msg").unlink()
        catalog = checkout / "packages/dimos-generated/catalog.json"
        data = json.loads(catalog.read_text())
        del data["roots"]["std_msgs/msg/Empty"]
        catalog.write_text(json.dumps(data))
    elif change == "renamed_type":
        previous = definitions / "dimos_msgs/msg/VideoStats.msg"
        previous.rename(previous.with_name("RenamedStats.msg"))
        catalog = checkout / "packages/dimos-generated/catalog.json"
        data = json.loads(catalog.read_text())
        data["roots"]["dimos_msgs/msg/RenamedStats"] = data["roots"].pop(
            "dimos_msgs/msg/VideoStats"
        )
        catalog.write_text(json.dumps(data))
    elif change == "version":
        metadata = checkout / "packages/dimos-generated/pyproject.toml"
        metadata.write_text(metadata.read_text().replace('version = "0.1.0"', 'version = "0.1.1"'))
    elif change == "extra_output":
        # This file was never tracked by Git. The gate compares inventories,
        # rather than relying on git diff (which would omit it).
        (output / "dimos_generated/unexpected.py").write_text("STALE = True\n")
    else:
        paths = {
            "missing_python": "dimos_generated/_types.py",
            "missing_cpp": "dimos_generated_schemas/package/cpp/CMakeLists.txt",
            "missing_rust": "dimos_generated_schemas/package/rust/src/lib.rs",
            "missing_schema": "dimos_generated_schemas/schemas/geometry_msgs/msg/Point.msg",
            "missing_rust_generator": "dimos_generated_schemas/package/rust/build.rs",
            "missing_rust_definition": "dimos_generated_schemas/package/rust/interfaces/geometry_msgs/msg/Point.msg",
        }
        (output / paths[change]).unlink()
    before_check = {
        str(path.relative_to(output)): path.read_bytes()
        for path in output.rglob("*")
        if path.is_file()
    }
    failed = run_generation(checkout, check=True)
    after_check = {
        str(path.relative_to(output)): path.read_bytes()
        for path in output.rglob("*")
        if path.is_file()
    }
    assert after_check == before_check, "--check must not repair or rewrite build output"
    assert failed.returncode != 0
    assert "Generated source drift" in failed.stderr, failed.stderr
    if change == "field":
        for language_output in [
            "_types.pyi",
            "cpp/geometry_msgs/msg/Point.msg",
            "rust/interfaces/geometry_msgs/msg/Point.msg",
        ]:
            assert language_output in failed.stderr
    repaired = run_generation(checkout, check=False)
    assert repaired.returncode == 0, repaired.stderr
    aligned = run_generation(checkout, check=True)
    assert aligned.returncode == 0, aligned.stderr
    if change == "new_type":
        assert (output / "dimos_generated/alignment_msgs/msg/__init__.py").read_text().find(
            "AlignmentProbe"
        ) >= 0
    elif change == "deleted_type":
        assert not (output / "dimos_generated_schemas/schemas/std_msgs/msg/Empty.msg").exists()
    elif change == "renamed_type":
        text = (output / "dimos_generated/dimos_msgs/msg/__init__.py").read_text()
        assert "RenamedStats" in text
        assert "VideoStats" not in text
    elif change == "version":
        statements = ast.parse((output / "dimos_generated/__init__.py").read_text()).body
        version = next(
            ast.literal_eval(statement.value)
            for statement in statements
            if isinstance(statement, ast.Assign)
            and isinstance(statement.targets[0], ast.Name)
            and statement.targets[0].id == "__dimos_version__"
        )
        assert version == "0.1.1"
        manifest_path = output / "dimos_generated_schemas/package/message-package.json"
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text())
            assert manifest["version"] == "0.1.1"
        assert (
            'version = "0.1.1"'
            in (output / "dimos_generated_schemas/package/rust/Cargo.toml").read_text()
        )
        assert (
            "VERSION 0.1.1"
            in (output / "dimos_generated_schemas/package/cpp/CMakeLists.txt").read_text()
        )


def test_source_package_preparation_keeps_alignment_check_valid(aligned_checkout):
    project = aligned_checkout / "packages/dimos-generated"
    prepare(project)
    assert (project / "build/message-source/python/setup.py").is_file()
    result = run_generation(aligned_checkout, check=True)
    assert result.returncode == 0, result.stderr
