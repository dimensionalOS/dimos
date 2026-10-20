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

"""DimOS source-build inputs; imported only by packaging and preparation tools."""

from hashlib import sha256
import json
import os
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory

from dimos_cdrgen.source_inputs import prepare_sources

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / "packages/dimos-generated"


def _matches(directory: Path, files: dict[str, str]) -> bool:
    return all(
        (directory / name).is_file()
        and sha256((directory / name).read_bytes()).hexdigest() == digest
        for name, digest in files.items()
    )


def prepare_builtin_sources(
    cache: Path | None = None, *, offline: bool = False, project: Path = PROJECT
) -> Path:
    cache = cache or Path(
        os.environ.get("DIMOS_MESSAGE_SOURCE_CACHE", ROOT / "build/builtin-source-cache")
    )
    lock = project / "schema-sources.json"
    data = json.loads(lock.read_text())
    if data.get("format") != 1:
        raise ValueError("Unsupported source lock format")
    upstream = {
        name: entry["sha256"]
        for source in data["sources"]
        for name, entry in source["files"].items()
    }
    if any(Path(name).is_absolute() or ".." in Path(name).parts for name in upstream):
        raise ValueError("Unsafe source input path")
    custom_root = project / "custom/dimos_msgs"
    custom = {
        "dimos_msgs/msg/" + path.name: sha256(path.read_bytes()).hexdigest()
        for path in (custom_root / "msg").glob("*.msg")
    }
    custom["licenses/dimos_msgs-LICENSE"] = sha256(
        (custom_root / "LICENSE").read_bytes()
    ).hexdigest()
    expected = {**upstream, **custom}
    target = project / "interfaces"
    actual = {str(path.relative_to(target)) for path in target.rglob("*") if path.is_file()}
    if actual == expected.keys() and _matches(target, expected):
        return target
    # Stage both downloaded and local definitions before replacing previous inputs.
    with TemporaryDirectory(prefix=".interfaces-", dir=project) as directory:
        stage = Path(directory) / "inputs"
        if _matches(target, upstream):
            stage.mkdir()
            for name in upstream:
                output = stage / name
                output.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(target / name, output)
        else:
            prepare_sources(lock, stage, cache / "archives", offline=offline)
        shutil.copytree(custom_root / "msg", stage / "dimos_msgs/msg")
        (stage / "licenses").mkdir(exist_ok=True)
        shutil.copyfile(custom_root / "LICENSE", stage / "licenses/dimos_msgs-LICENSE")
        backup = Path(directory) / "previous"
        if target.exists():
            target.rename(backup)
        try:
            stage.rename(target)
        except OSError:
            if backup.exists():
                backup.rename(target)
            raise
    return target


def prepare_web_dist(web: Path, *, offline: bool = False) -> None:
    required = (web / "sdk/dist/sdk.js", web / "cockpit/dist/index.html")
    if all(path.is_file() for path in required):
        return
    deno = shutil.which("deno")
    if deno is None:
        raise RuntimeError(
            "Deno build dependency is missing. Use a normal isolated pip install, "
            "or install the build-system requirements before --no-build-isolation."
        )
    frozen = ["--frozen", *(["--cached-only"] if offline else [])]
    subprocess.run([deno, "install", *frozen], cwd=web, check=True, timeout=600)
    with TemporaryDirectory(prefix=".wheel-web-", dir=web) as directory:
        outputs = []
        for name, filename in (("sdk", "sdk.js"), ("cockpit", "index.html")):
            output = Path(directory) / name
            subprocess.run(
                [deno, "run", *frozen, "-A", "npm:vite", "build", "--outDir", str(output)],
                cwd=web / name,
                check=True,
                timeout=600,
            )
            if not (output / filename).is_file():
                raise RuntimeError(f"Web build did not produce {name}/{filename}")
            outputs.append((output, web / name / "dist"))
        # A failed build leaves both previous distributions intact.
        for output, target in outputs:
            if target.exists():
                shutil.rmtree(target)
            output.rename(target)


def copy_native_sdk(root: Path, target: Path) -> None:
    """Ship owned SDK source integration, never compile it during Python installation."""
    sources = [root / "native/cpp/CMakeLists.txt"]
    sources.extend((root / "native/cpp/include").rglob("*.hpp"))
    for crate in ("dimos-module", "dimos-module-macros"):
        source = root / "native/rust" / crate
        sources.append(source / "Cargo.toml")
        if (source / "Cargo.lock").is_file():
            sources.append(source / "Cargo.lock")
        sources.extend((source / "src").rglob("*.rs"))
        sources.extend((source / "examples").rglob("*.rs"))
    if not (root / "native/cpp/include/dimos/native/module.hpp").is_file():
        raise RuntimeError("Native SDK sources missing from source distribution")
    with TemporaryDirectory(prefix=".native-sdk-", dir=target.parent) as directory:
        staged = Path(directory) / "sdk"
        for source in sources:
            output = staged / source.relative_to(root / "native")
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, output)
        previous = Path(directory) / "previous"
        if target.exists():
            target.rename(previous)
        try:
            staged.rename(target)
        except OSError:
            if previous.exists():
                previous.rename(target)
            raise
