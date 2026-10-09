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
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


@pytest.fixture(scope="module")
def aligned_checkout(tmp_path_factory):
    missing = None
    if importlib.util.find_spec("ruff") is None:
        missing = "Install the locked message-codegen dependency group for alignment tests"
    elif shutil.which("rustup") is None:
        missing = "Alignment tests require the explicitly installed Rust 1.92.0 formatter"
    else:
        formatter = subprocess.run(
            ["rustup", "run", "1.92.0", "rustfmt", "--version"], capture_output=True, text=True
        )
        if formatter.returncode:
            missing = "Alignment tests require rustup toolchain install 1.92.0 --component rustfmt"
    if missing:
        if os.environ.get("DIMOS_MESSAGE_ALIGNMENT_REQUIRED") == "1":
            pytest.fail(missing)
        pytest.skip(missing)
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
        "generator",
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
    definitions = checkout / "dimos/message_codegen/schemas"
    output = checkout / "packages/dimos-generated/src"
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
    elif change == "deleted_type":
        (definitions / "std_msgs/msg/Empty.msg").unlink()
    elif change == "generator":
        runtime = checkout / "dimos/message_codegen/templates/runtime.py"
        runtime.write_text("# Deliberately changed generator output\n" + runtime.read_text())
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
            "missing_cpp": "dimos_generated_schemas/package/cpp/messages.hpp",
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
    assert after_check == before_check, "--check must not repair or rewrite checked-in output"
    assert failed.returncode != 0
    assert "Generated source drift" in failed.stderr, failed.stderr
    if change == "field":
        for language_output in [
            "_types.py",
            "messages.hpp",
            "rust/build.rs",
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
