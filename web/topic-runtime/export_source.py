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

"""Create the small, pinned source archive consumed by Desktop's offline Nix build."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

from package_runtime import normalized
import tomllib

PROJECT = Path(__file__).resolve().parent
ROOT = PROJECT.parents[1]


def export(output: Path) -> None:
    if subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT, text=True
    ).strip():
        raise ValueError("commit tracked changes before exporting a pinned runtime")
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    output = output.absolute()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="dimos-topic-source-") as scratch:
        checkout = Path(scratch) / "checkout"
        checkout.mkdir()
        # Export committed inputs, so ignored SDK bundles, pycache, local files
        # and filesystem permissions cannot affect the source build.
        project = tomllib.loads((PROJECT / "pyproject.toml").read_text())
        includes = project["tool"]["hatch"]["build"]["targets"]["wheel"]["force-include"]
        paths = {"web", "dimos/utils/deno.py"}
        paths.update(str((PROJECT / name).resolve().relative_to(ROOT)) for name in includes)
        archived = Path(scratch) / "checkout.tar"
        with archived.open("wb") as destination:
            subprocess.run(
                ["git", "archive", revision, "--", *sorted(paths)],
                cwd=ROOT,
                stdout=destination,
                check=True,
            )
        with tarfile.open(archived) as archive:
            archive.extractall(checkout, filter="data")
        source = Path(scratch) / "source"
        subprocess.run(
            [
                "deno",
                "run",
                "-A",
                "relay/build_runtime.ts",
                "--output",
                str(source),
            ],
            cwd=checkout / "web",
            check=True,
        )
        subprocess.run(
            [
                "uv",
                "build",
                "--project",
                str(checkout / "web/topic-runtime"),
                "--wheel",
                "--out-dir",
                str(source),
            ],
            check=True,
        )
        for name in ("assemble_runtime.py", "package_runtime.py"):
            shutil.copy2(checkout / "web/topic-runtime" / name, source / name)
        shutil.copy2(checkout / "LICENSE", source / "LICENSE")
        with (
            output.open("wb") as raw,
            gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as compressed,
            tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive,
        ):
            for path in sorted(source.rglob("*")):
                if not path.resolve().is_relative_to(source):
                    raise ValueError("source archive has an external symlink")
                archive.add(
                    path,
                    arcname="dimos-topic-source/" + str(path.relative_to(source)),
                    recursive=False,
                    filter=normalized,
                )
    with output.open("rb") as content:
        digest = hashlib.file_digest(content, "sha256").hexdigest()
    output.with_suffix(".json").write_text(
        json.dumps(
            {
                "format": 1,
                "file": output.name,
                "sha256": digest,
                "dimosBase": revision,
                "protocol": 8,
                "topics": 1,
                "runtimeVersion": "0.1.0",
            },
            indent=4,
        )
        + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    export(parser.parse_args().output)
