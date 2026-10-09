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

"""Strictly check literal URL filenames using temporary valid Python module names."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import tempfile


def main() -> None:
    endpoints = Path(__file__).parents[1] / "endpoints"
    files = [path for path in sorted(endpoints.rglob("*.py")) if path.name != "__init__.py"]
    if not files:
        raise RuntimeError("gateway endpoint sources are missing")
    config = Path(__file__).parents[3] / "pyproject.toml"
    with tempfile.TemporaryDirectory(prefix="gateway-mypy-") as directory:
        temporary = Path(directory)
        checked_config = temporary / "pyproject.toml"
        checked_config.write_text(
            "\n".join(
                line for line in config.read_text().splitlines() if not line.startswith("files = ")
            )
        )
        sources = []
        originals = {}
        for index, path in enumerate(files):
            source = temporary / f"endpoint_{index}.py"
            source.write_text(path.read_text())
            sources.append(str(source))
            originals[str(source)] = str(path.relative_to(endpoints))
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "mypy",
                "--config-file",
                str(checked_config),
                "--follow-imports=silent",
                *sources,
            ],
            capture_output=True,
            text=True,
        )
        output = result.stdout + result.stderr
        for source_name, original in originals.items():
            output = output.replace(source_name, original)
        print(output, end="")
        print(f"Endpoint types: {len(files)} checked")
        raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
