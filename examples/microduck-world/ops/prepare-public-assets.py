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

"""Stage the existing browser build and exported scene for Cloudflare assets."""

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    build = ROOT / "web/dist"
    scene = ROOT / "state/viewer"
    if not (build / "index.html").is_file() or not any(scene.glob("scene-*.json")):
        raise SystemExit("Build web/dist and start the world once to export state/viewer first")
    target = ROOT / "edge/public"
    target.mkdir(parents=True, exist_ok=True)
    for directory in ("client", "world-assets"):
        shutil.rmtree(target / directory, ignore_errors=True)
    shutil.copytree(build, target / "client")
    shutil.copy2(build / "index.html", target / "index.html")
    (target / "world-assets").mkdir()
    for path in scene.glob("scene-*.json*"):
        if path.suffix not in {".json", ".gz"}:
            continue
        shutil.copy2(path, target / "world-assets" / path.name)
    for path in target.rglob("*"):
        if path.is_file() and path.stat().st_size > 25 * 1024 * 1024:
            raise SystemExit(f"Asset exceeds Cloudflare's per-file limit: {path.name}")
    print("Staged browser and scene assets in edge/public")


if __name__ == "__main__":
    main()
