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

"""Assemble a relocatable runtime from already verified, offline release inputs.

This path needs no uv, pip, network access or execution of the portable
interpreter inside Nix's filesystem sandbox. All wheels must
come from the frozen per-platform manifest. Runtime entrypoints use -m, so
wheel console-script installation and absolute build-prefix shebangs are avoided.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import zipfile


def checksum(path: Path) -> str:
    with path.open("rb") as content:
        return hashlib.file_digest(content, "sha256").hexdigest()


def assemble(inputs: Path, source: Path, relay: Path, output: Path) -> None:
    if output.exists():
        raise ValueError("runtime output already exists")
    spec = json.loads(inputs.read_text())
    output.mkdir(parents=True)
    with tarfile.open(spec["python"]) as archive:
        archive.extractall(output, filter="data")
    site = output / "python/lib/python3.12/site-packages"
    site.mkdir(parents=True, exist_ok=True)
    (own_wheel,) = source.glob("dimos_topic_runtime-*.whl")
    for wheel in [*(Path(path) for path in spec["wheels"]), own_wheel]:
        with zipfile.ZipFile(wheel) as archive:
            for entry in archive.infolist():
                if entry.is_dir():
                    continue
                relative = Path(entry.filename)
                if relative.is_absolute() or ".." in relative.parts:
                    raise ValueError("wheel contains an escaping path")
                if relative.parts[0].endswith(".data"):
                    # Reject new installation schemes for review rather than
                    # silently omit a dependency's runtime files.
                    if relative.parts[1] not in ("purelib", "platlib"):
                        raise ValueError(f"unsupported wheel installation scheme: {entry.filename}")
                    relative = Path(*relative.parts[2:])
                target = site / relative
                if entry.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(archive.read(entry))
    (output / "relay/sdk").mkdir(parents=True)
    shutil.copy2(relay, output / "relay/dimos-relay")
    (output / "relay/dimos-relay").chmod(0o755)
    shutil.copy2(source / "sdk/sdk.js", output / "relay/sdk/sdk.js")
    licenses = output / "licenses"
    licenses.mkdir()
    shutil.copy2(source / "LICENSE", licenses / "DimOS-LICENSE")
    shutil.copy2(spec["denoLicense"], licenses / "Deno-LICENSE.md")
    for path in (source / "node_modules").rglob("*"):
        if path.is_file() and path.name.lower().startswith(("license", "copying", "notice")):
            target = licenses / "npm" / path.relative_to(source / "node_modules")
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    files = {}
    for path in sorted(output.rglob("*")):
        if path.is_symlink() and not path.resolve().is_relative_to(output.resolve()):
            raise ValueError(f"runtime symlink escapes its bundle: {path}")
        if path.is_file() and not path.is_symlink() and path.suffix != ".pyc":
            files[str(path.relative_to(output))] = checksum(path)
    manifest = {
        "format": 1,
        "version": "0.1.0",
        "protocol": 8,
        "topics": 1,
        "pythonVersion": "3.12.13",
        "platform": {"os": spec["os"], "arch": spec["arch"]},
        "python": "python/bin/python3",
        "relay": "relay/dimos-relay",
        "sdk": "relay/sdk/sdk.js",
        "wheelSha256": checksum(own_wheel),
        "files": files,
    }
    (output / "runtime.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--relay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    assemble(args.inputs, args.source, args.relay, args.output)
