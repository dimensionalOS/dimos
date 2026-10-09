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

"""Regenerate checked-in built-in source; --check rejects drift without writing it."""

import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib

from dimos.message_codegen.distribution import write_distribution
from dimos.message_codegen.generate import generate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    project = tomllib.loads((root / "packages/dimos-generated/pyproject.toml").read_text())[
        "project"
    ]
    version = project["version"]
    target = root / "packages/dimos-generated/src"
    with tempfile.TemporaryDirectory(prefix="dimos-source-") as temporary:
        output = Path(temporary)
        names = generate([], output, shared=True, version=version)
        write_distribution(output, "dimos_generated", names, version=version)
        source = output / "python"
        packages = [source / "dimos_generated", source / "dimos_generated_schemas"]
        # Formatting is a maintainer codegen prerequisite, never an install/import step.
        # Maintainer-only license for our registry alias, not upstream declarations.
        alias = source / "dimos_generated/_types.py"
        header = "\n".join(Path(__file__).read_text().splitlines()[:13]) + "\n\n"
        alias.write_text(header + alias.read_text())
        python_files = [
            str(path)
            for package in packages
            for path in package.rglob("*")
            if path.suffix in {".py", ".pyi"}
        ]
        subprocess.run(
            [
                sys.executable,
                "-m",
                "ruff",
                "check",
                "--fix",
                "--config",
                str(root / "pyproject.toml"),
                *python_files,
            ],
            check=True,
        )
        subprocess.run(
            [
                sys.executable,
                "-m",
                "ruff",
                "format",
                "--config",
                str(root / "pyproject.toml"),
                *python_files,
            ],
            check=True,
        )
        rust_root = source / "dimos_generated_schemas/package/rust/src"
        license_text = (
            "\n".join(
                (root / "dimos/message_codegen/templates/message_build.rs")
                .read_text()
                .splitlines()[:13]
            )
            + "\n\n"
        )
        library = rust_root / "lib.rs"
        library.write_text(license_text + library.read_text())
        subprocess.run(
            [
                "rustup",
                "run",
                "1.92.0",
                "rustfmt",
                "--edition",
                "2024",
                str(library),
                str(rust_root.parent / "build.rs"),
            ],
            check=True,
        )
        expected = {
            str(path.relative_to(source)): path.read_bytes()
            for package in packages
            for path in package.rglob("*")
            if path.is_file()
        }
        actual = {
            str(path.relative_to(target)): path.read_bytes()
            for path in target.rglob("*")
            if path.is_file()
            and "__pycache__" not in path.parts
            and path.suffix != ".pyc"
            and not any(part.endswith(".egg-info") for part in path.parts)
        }
        if args.check:
            if actual != expected:
                missing = sorted(expected.keys() - actual.keys())
                stale = sorted(actual.keys() - expected.keys())
                changed = sorted(
                    name
                    for name in actual.keys() & expected.keys()
                    if actual[name] != expected[name]
                )
                parser.error(
                    f"Generated source drift: missing={missing}, stale={stale}, changed={changed}. Run python -m scripts.generate_builtin_messages"
                )
            print("Built-in generated Python/C++/Rust sources match canonical definitions")
            return
        target.mkdir(parents=True, exist_ok=True)
        for package in packages:
            destination = target / package.name
            if destination.exists():
                shutil.rmtree(destination)
            shutil.copytree(package, destination)
        print(f"Generated built-in source in {target}")


if __name__ == "__main__":
    main()
