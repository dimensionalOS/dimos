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

"""Build this external message package against the installed DimOS release.

The checked-in ``pyproject.toml.in`` deliberately carries no version literal:
the owner version is read from the installed ``dimos`` distribution (or its
generated message manifest) and substituted into a temporary copy. The build
runs with ``--no-build-isolation --no-deps`` so it uses the already-installed
DimOS and cannot silently fetch a different PyPI release.
"""

from importlib.metadata import PackageNotFoundError, distribution
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory

HERE = Path(__file__).resolve().parent


def owner_version() -> str:
    try:
        return distribution("dimos").version
    except PackageNotFoundError:
        try:
            from dimos_messages_schemas.provider import package_root  # type: ignore[import-untyped]
        except ImportError as error:  # pragma: no cover - example guard
            raise SystemExit("Install DimOS (pip install .) before building this example") from error
        return json.loads((package_root() / "message-package.json").read_text())["version"]


def main() -> None:
    version = owner_version()
    with TemporaryDirectory(prefix="dimos-external-messages-") as temporary:
        project = Path(temporary) / "acme-telemetry"
        shutil.copytree(HERE, project)
        (project / "pyproject.toml").write_text(
            (project / "pyproject.toml.in").read_text().replace("@DIMOS_VERSION@", version)
        )
        (project / "pyproject.toml.in").unlink()
        wheelhouse = Path(temporary) / "dist"
        site = Path(temporary) / "site"
        subprocess.run(
            [
                sys.executable,
                "-m",
                "pip",
                "wheel",
                "--no-build-isolation",
                "--no-deps",
                "--wheel-dir",
                str(wheelhouse),
                str(project),
            ],
            check=True,
        )
        subprocess.run(
            [
                sys.executable,
                "-m",
                "pip",
                "install",
                "--no-index",
                "--no-deps",
                "--target",
                str(site),
                str(next(wheelhouse.glob("acme_telemetry-*.whl"))),
            ],
            check=True,
        )
        subprocess.run(
            [sys.executable, str(HERE / "demo_external.py")],
            check=True,
            env={**os.environ, "PYTHONPATH": str(site)},
        )


if __name__ == "__main__":
    main()
