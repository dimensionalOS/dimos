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

"""Create a separate renderer project from the saved office's native content."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from dimos.constants import DIMOS_PROJECT_ROOT


def prepare(source: Path, destination: Path, engine: Path) -> None:
    original = source / "unreal/SF555Office"
    project = destination / "DimosRenderer"
    if project.resolve().is_relative_to(source.resolve()):
        raise ValueError("renderer output must be separate from the source office project")
    template = DIMOS_PROJECT_ROOT / "native/unreal/DimosRenderer"
    for name in ("Source", "Config", "Scripts", "Shaders"):
        shutil.copytree(template / name, project / name, dirs_exist_ok=True)
    shutil.copy2(template / "DimosRenderer.uproject", project / "DimosRenderer.uproject")
    shutil.copytree(original / "Content", project / "Content", dirs_exist_ok=True)
    # Preserve the office's rendering configuration. Do not copy unrelated
    # project service settings or credentials into this derived project.
    settings = (original / "Config/DefaultEngine.ini").read_text()
    settings = settings.split("[/Script/Engine.RendererSettings]\n", 1)[1].split("\n[", 1)[0]
    config = project / "Config/DefaultEngine.ini"
    value = config.read_text()
    start = value.index("[/Script/Engine.RendererSettings]")
    end = value.index("\n[", start + 1)
    config.write_text(
        value[:start] + "[/Script/Engine.RendererSettings]\n" + settings + "\n" + value[end:]
    )
    manifest = json.loads((source / "exports/unreal-v24/scene.json").read_text())
    report = json.loads((destination / "collision-report.json").read_text())
    if report["source_sha256"] != manifest["source_sha256"]:
        raise ValueError("physics export and native map come from different Blender sources")
    provenance = {
        "source_sha256": manifest["source_sha256"],
        "glb_sha256": hashlib.sha256(
            (source / "exports/unreal-v24/SF555Office.glb").read_bytes()
        ).hexdigest(),
        "level": "/Game/SF555/Maps/Office",
        "source_project": str(original.resolve()),
    }
    (destination / "provenance.json").write_text(json.dumps(provenance, indent=2))
    shutil.copy2(
        source / "exports/unreal-v24/reference_contacts.json",
        destination / "reference_contacts.json",
    )
    uproject = project.resolve() / "DimosRenderer.uproject"
    subprocess.run(
        [
            str(engine / "Engine/Build/BatchFiles/Mac/Build.sh"),
            "DimosRendererEditor",
            "Mac",
            "Development",
            str(uproject),
            "-WaitMutex",
        ],
        check=True,
    )
    editor = engine / "Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"
    subprocess.run(
        [
            str(editor),
            str(uproject),
            "-run=pythonscript",
            f"-script={project.resolve()}/Scripts/prepare_assets.py",
            "-unattended",
            "-NullRHI",
            "-nosound",
        ],
        check=True,
    )
    print(f"Renderer project ready: {uproject}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--engine", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source, args.output, args.engine)
