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
import os
from pathlib import Path, PurePosixPath
import shutil
from tempfile import TemporaryDirectory
from typing import Annotated, Literal

from dimos_cdrgen.source_inputs import prepare_sources
from pydantic import AfterValidator, BaseModel, ConfigDict, Field

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / "packages/dimos-generated"


def _relative_path(value: str) -> str:
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or ".." in path.parts:
        raise ValueError("Unsafe source input path")
    return value


RelativePath = Annotated[str, AfterValidator(_relative_path)]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]


class SourceFile(BaseModel):
    model_config = ConfigDict(strict=True, extra="forbid")

    path: RelativePath
    sha256: Sha256


class SourceArchive(BaseModel):
    model_config = ConfigDict(strict=True, extra="forbid")

    repository: str | None = None
    revision: str | None = None
    url: str
    sha256: Sha256
    prefix: RelativePath
    files: dict[RelativePath, SourceFile]


class SourceLock(BaseModel):
    model_config = ConfigDict(strict=True, extra="forbid")

    format: Literal[1]
    sources: list[SourceArchive]


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
    data = SourceLock.model_validate_json(lock.read_text())
    upstream = {
        name: entry.sha256 for source in data.sources for name, entry in source.files.items()
    }
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
