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

import pytest

from dimos.deps.bundles import (
    BundleMetadataError,
    UnknownNameError,
    builtin_names,
    bundles_for,
    load_assignments,
    lock_path,
)


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "bundles.json"
    path.write_text(text)
    return path


def test_load_assignments_reverses_the_bundle_mapping(tmp_path: Path) -> None:
    path = _write(tmp_path, '{"runtime-a": ["x", "y"], "runtime-b": ["z"]}')

    assert load_assignments(path) == {"x": "runtime-a", "y": "runtime-a", "z": "runtime-b"}


@pytest.mark.parametrize(
    ("text", "match"),
    [
        ('{"runtime-a": ["x"], "runtime-a": ["y"]}', "duplicate key"),
        ('{"runtime-a": ["x"], "runtime-b": ["x"]}', "more than one bundle"),
        ('{"runtime-a": ["x", "x"]}', "more than one bundle"),
        ('{"runtime-a": "x"}', "list of names"),
        ('{"runtime-a": ["x", 3]}', "non-string"),
        ('{"runtime-a": [""]}', "empty or non-string"),
        ('{"": ["x"]}', "empty bundle"),
        ("[]", "non-empty object"),
        ("{}", "non-empty object"),
        ("{not json", "not valid JSON"),
    ],
)
def test_load_assignments_rejects_malformed_metadata(tmp_path: Path, text: str, match: str) -> None:
    with pytest.raises(BundleMetadataError, match=match):
        load_assignments(_write(tmp_path, text))


def test_load_assignments_missing_file_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(BundleMetadataError, match="missing"):
        load_assignments(tmp_path / "absent.json")


def test_bundles_for_unions_and_sorts() -> None:
    assignments = {"a": "runtime-z", "b": "runtime-a", "c": "runtime-z"}

    assert bundles_for(["a", "b", "c", "a"], assignments) == ["runtime-a", "runtime-z"]


def test_bundles_for_unknown_name_suggests_close_matches() -> None:
    with pytest.raises(UnknownNameError, match="Did you mean: unitree-go2"):
        bundles_for(["unitree-go3"], load_assignments())


def test_bundles_for_registered_but_unassigned_name_is_a_metadata_error() -> None:
    with pytest.raises(BundleMetadataError, match="has no bundle"):
        bundles_for(["unitree-go2"], {})


def test_shipped_metadata_covers_exactly_the_registry() -> None:
    assignments = load_assignments()

    assert set(assignments) == builtin_names()
    assert all(bundle.startswith("runtime-") for bundle in assignments.values())


def test_lock_path_uses_the_name_pattern_uv_requires() -> None:
    assert lock_path("runtime-unitree", "cpu").name == "pylock.runtime-unitree-cpu.toml"
