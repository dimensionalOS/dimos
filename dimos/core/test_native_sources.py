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

import tarfile

from dimos.core.native_sources import unpack_sources


def test_native_sources_are_reusable_writable_snapshots(tmp_path):
    source = tmp_path / "Cargo.toml"
    source.write_text("[workspace]\nmembers = []\n")
    bundle = tmp_path / "sources.tar"
    with tarfile.open(bundle, "w") as archive:
        archive.add(source, arcname="Cargo.toml")
    root = unpack_sources(bundle, tmp_path / "cache")
    assert (root / "Cargo.toml").read_text() == source.read_text()
    assert (root / ".git").is_dir()
    (root / "target").mkdir()
    assert unpack_sources(bundle, tmp_path / "cache") == root
    assert (root / "target").is_dir()


def test_source_content_selects_separate_build_roots(tmp_path):
    source = tmp_path / "Cargo.toml"
    bundle = tmp_path / "sources.tar"
    roots = []
    for version in ("one", "two"):
        source.write_text(version)
        with tarfile.open(bundle, "w") as archive:
            archive.add(source, arcname="Cargo.toml")
        roots.append(unpack_sources(bundle, tmp_path / "cache"))
    assert roots[0] != roots[1]
    assert (roots[0] / "Cargo.toml").read_text() == "one"
