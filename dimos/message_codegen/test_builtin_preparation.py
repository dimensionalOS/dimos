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

"""Product preparation uses pinned caches, never a source-build network hook."""

import json
import os
from pathlib import Path
import shutil

from dimos_cdrgen.source_package import configuration, prepare as generate
import pytest

from scripts.prepare_builtin_sources import prepare


def test_unprepared_product_fails_and_cached_preparation_is_complete(tmp_path, monkeypatch):
    available = os.environ.get("DIMOS_BUILTIN_SOURCE_CACHE")
    if not available:
        if os.environ.get("DIMOS_MESSAGE_INSTALL_REQUIRED") == "1":
            pytest.fail("Explicit schema preparation cache is required")
        pytest.skip("Prepare the pinned builtin source cache for offline acceptance")
    project = tmp_path / "product"
    source = Path(__file__).resolve().parents[2] / "packages/dimos-generated"
    shutil.copytree(
        source,
        project,
        ignore=shutil.ignore_patterns("interfaces", "build", "dist", "*.egg-info", "__pycache__"),
    )
    with pytest.raises(ValueError, match="not prepared"):
        configuration(project)
    cache = tmp_path / "cache"
    shutil.copytree(Path(available) / "archives", cache / "archives")

    def no_network(*args, **kwargs):
        raise AssertionError("Offline builtin preparation attempted a download")

    monkeypatch.setattr("dimos_cdrgen.source_inputs.requests.get", no_network)
    inputs = prepare(cache, offline=True, project=project)
    assert len(list(inputs.rglob("*.msg"))) == 78
    output = generate(project)
    resources = output / "dimos_generated_schemas/package"
    manifest = json.loads((resources / "message-package.json").read_text())
    assert len(manifest["owned"]) == 78
    assert len(list(project.glob("custom/dimos_msgs/msg/*.msg"))) == 19
    lock = json.loads((project / "schema-sources.json").read_text())
    assert all(len(item["sha256"]) == 64 for item in lock["sources"])
    assert (resources / "dimos_msgs-LICENSE").is_file()
    assert all((resources / path.name).is_file() for path in (inputs / "licenses").iterdir())
