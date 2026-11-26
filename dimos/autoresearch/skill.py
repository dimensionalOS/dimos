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

"""Publish a frozen artifact and its measured performance as an agent skill."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import json
from pathlib import Path
import re
import shutil
import tempfile

import yaml

from dimos.autoresearch.session import Session
from dimos.constants import CONFIG_DIR

USER_SKILLS_DIR = CONFIG_DIR / "agent" / "skills"
NAME_MAX = 64
_NAME = re.compile(r"[a-z0-9]+(-[a-z0-9]+)*")


def skill_name(text: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")
    return slug[:NAME_MAX].rstrip("-") or "autoresearch-program"


@contextmanager
def skill_directory(root: Path, name: str) -> Iterator[Path]:
    """Publish the populated folder only after the exporter completes."""
    if len(name) > NAME_MAX or not _NAME.fullmatch(name):
        raise ValueError(f"invalid skill name {name!r}: use lowercase words joined by hyphens")
    root.mkdir(parents=True, exist_ok=True)
    target = root / name
    if target.exists():
        raise FileExistsError(f"{target} exists; choose another skill name")
    with tempfile.TemporaryDirectory(dir=root, prefix=".export-") as staging:
        folder = Path(staging) / name
        folder.mkdir()
        yield folder
        folder.rename(target)


def export_artifact(
    session: Session, root: Path, instructions: str, name: str | None = None
) -> Path:
    """Bundle a scored winner with the author's directions for using that artifact."""
    info = session.info()
    if info.status != "scored" or info.frozen_candidate is None:
        raise ValueError("complete held-out scoring before exporting the artifact")
    if not instructions.strip():
        raise ValueError("provide instructions for using the winning artifact")
    candidate = info.frozen_candidate
    artifact = session.artifact(candidate)
    name = name or skill_name(info.task)
    with skill_directory(root, name) as folder:
        asset = Path("assets") / "candidate" / artifact.name
        (folder / asset).parent.mkdir(parents=True)
        shutil.copyfile(artifact, folder / asset)
        validation = dict(
            task=info.task,
            metric=info.metric,
            direction=info.direction,
            candidate=candidate,
            heldout_score=info.heldout_score,
            trials=[r.model_dump(mode="json") for r in session.results(candidate)],
        )
        (folder / "assets/validation.json").write_text(json.dumps(validation, indent=2))
        front = yaml.safe_dump(
            {"name": name, "description": " ".join(info.task.split())[:1024]}, sort_keys=False
        )
        (folder / "SKILL.md").write_text(
            f"---\n{front}---\n\n{instructions.strip()}\n\n"
            f"The evaluated artifact is `{asset.as_posix()}`. "
            f"Its held-out mean {info.metric} was {info.heldout_score}. "
            "Trial measurements are in `assets/validation.json`.\n"
        )
    return root / name
