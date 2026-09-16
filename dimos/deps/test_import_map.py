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

from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.import_map import IMPORT_TO_DISTRIBUTION, is_stdlib, provider_for
from dimos.deps.imports import ImportKind, is_excluded_from_check, iter_source_files, scan_file
from dimos.deps.lock import LockIndex

DIMOS_DIR = DIMOS_PROJECT_ROOT / "dimos"


def test_provider_kinds() -> None:
    assert provider_for("cv2") is not None and provider_for("cv2").names == (
        "opencv-contrib-python",
    )
    assert provider_for("open3d") is not None and provider_for("open3d").kind == "dist"
    assert provider_for("rclpy") is not None and provider_for("rclpy").kind == "system"
    assert provider_for("dimos_mls_planner") is not None
    assert provider_for("dimos_mls_planner").kind == "native"
    assert provider_for("pytest") is not None and provider_for("pytest").kind == "dev"
    assert provider_for("redis") is not None and provider_for("redis").kind == "undeclared"
    assert provider_for("no_such_package") is None
    assert is_stdlib("json") and is_stdlib("__future__") and not is_stdlib("numpy")


def test_every_mapped_distribution_is_locked() -> None:
    locked = LockIndex.load(DIMOS_PROJECT_ROOT).packages
    missing = {
        name: dists
        for name, dists in IMPORT_TO_DISTRIBUTION.items()
        if not any(dist in locked for dist in dists)
    }
    assert not missing, f"import map names distributions absent from uv.lock: {missing}"


def _excluded(path: Path) -> bool:
    return is_excluded_from_check(str(path.relative_to(DIMOS_DIR)))


def test_every_third_party_import_is_mapped() -> None:
    unknown: dict[str, str] = {}
    for path in iter_source_files(DIMOS_DIR, exclude=_excluded):
        scan = scan_file(path, project_root=DIMOS_PROJECT_ROOT)
        for site in scan.imports:
            if site.kind is ImportKind.MAIN_ONLY:
                continue
            top = site.top_level
            if not top or top == "dimos" or is_stdlib(top) or provider_for(top) is not None:
                continue
            unknown.setdefault(top, f"{scan.module}:{site.lineno}")
    assert not unknown, f"add these import names to dimos/deps/import_map.py: {unknown}"
