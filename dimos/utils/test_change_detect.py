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

"""Tests for the change detection utility."""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.utils.change_detect import Glob, clear_cache, did_change, update_cache


@pytest.fixture(autouse=True)
def _use_tmp_cache(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Redirect the change-detection cache to a temp dir for every test."""
    monkeypatch.setattr(
        "dimos.utils.change_detect._get_cache_dir",
        lambda: tmp_path / "cache",
    )


@pytest.fixture()
def src_dir(tmp_path: Path) -> Path:
    """A temp directory with two source files for testing."""
    d = tmp_path / "src"
    d.mkdir()
    (d / "a.c").write_text("int main() { return 0; }")
    (d / "b.c").write_text("void helper() {}")
    return d


def test_first_call_returns_true(src_dir: Path) -> None:
    assert did_change("test_cache", [str(src_dir)]) is True


def test_second_call_no_change_returns_false(src_dir: Path) -> None:
    did_change("test_cache", [str(src_dir)])
    assert did_change("test_cache", [str(src_dir)]) is False


def test_file_modified_returns_true(src_dir: Path) -> None:
    did_change("test_cache", [str(src_dir)])
    (src_dir / "a.c").write_text("int main() { return 1; }")
    assert did_change("test_cache", [str(src_dir)]) is True


def test_file_added_to_dir_returns_true(src_dir: Path) -> None:
    did_change("test_cache", [str(src_dir)])
    (src_dir / "c.c").write_text("void new_func() {}")
    assert did_change("test_cache", [str(src_dir)]) is True


def test_file_deleted_returns_true(src_dir: Path) -> None:
    did_change("test_cache", [str(src_dir)])
    (src_dir / "b.c").unlink()
    assert did_change("test_cache", [str(src_dir)]) is True


def test_glob_pattern(src_dir: Path) -> None:
    pattern = Glob(str(src_dir / "*.c"))
    assert did_change("glob_cache", [pattern]) is True
    assert did_change("glob_cache", [pattern]) is False
    (src_dir / "a.c").write_text("changed!")
    assert did_change("glob_cache", [pattern]) is True


def test_str_with_glob_chars_is_literal(tmp_path: Path) -> None:
    """A plain str containing '*' must NOT be glob-expanded."""
    weird_name = tmp_path / "file[1].txt"
    weird_name.write_text("content")
    # str path — treated literally, should find the file
    assert did_change("literal_test", [str(weird_name)]) is True
    assert did_change("literal_test", [str(weird_name)]) is False


def test_separate_cache_names_independent(src_dir: Path) -> None:
    paths = [str(src_dir)]
    did_change("cache_a", paths)
    did_change("cache_b", paths)
    # Both caches are now up-to-date
    assert did_change("cache_a", paths) is False
    assert did_change("cache_b", paths) is False
    # Modify a file — both caches should report changed independently
    (src_dir / "a.c").write_text("changed")
    assert did_change("cache_a", paths) is True
    # cache_b hasn't been checked since the change
    assert did_change("cache_b", paths) is True


def test_clear_cache(src_dir: Path) -> None:
    paths = [str(src_dir)]
    did_change("clear_test", paths)
    assert did_change("clear_test", paths) is False
    assert clear_cache("clear_test") is True
    assert did_change("clear_test", paths) is True


def test_clear_cache_nonexistent() -> None:
    assert clear_cache("does_not_exist") is False


def test_empty_paths_returns_false() -> None:
    assert did_change("empty_test", []) is False


def test_nonexistent_path_warns(monkeypatch: pytest.MonkeyPatch) -> None:
    """A non-existent absolute path logs a warning and returns False (no files → skip rebuild)."""
    warnings: list[tuple[str, dict]] = []

    def fake_warning(msg: str, **kwargs: object) -> None:
        warnings.append((msg, dict(kwargs)))

    monkeypatch.setattr("dimos.utils.change_detect.logger.warning", fake_warning)
    result = did_change("missing_test", ["/nonexistent/path/to/file.c"])
    assert result is False
    assert any(
        "does not exist" in msg.lower() and kw.get("path") == "/nonexistent/path/to/file.c"
        for msg, kw in warnings
    ), f"expected 'Path does not exist' warning, got: {warnings}"


def test_relative_path_without_cwd_raises() -> None:
    """Relative paths without cwd= should raise ValueError."""
    with pytest.raises(ValueError, match="Relative path.*without a cwd"):
        did_change("rel_test", ["some/relative/path.c"])


def test_relative_path_with_cwd(src_dir: Path) -> None:
    """Relative paths should resolve against the provided cwd."""
    assert did_change("cwd_test", ["src/a.c"], cwd=src_dir.parent) is True
    assert did_change("cwd_test", ["src/a.c"], cwd=src_dir.parent) is False


def test_update_false_does_not_write_cache(src_dir: Path) -> None:
    """With update=False, repeated calls keep returning True (cache not updated)."""
    paths = [str(src_dir)]
    assert did_change("no_update", paths, update=False) is True
    # Cache was not written, so still reports changed
    assert did_change("no_update", paths, update=False) is True
    # Now update explicitly
    update_cache("no_update", paths)
    # Cache is current, no change
    assert did_change("no_update", paths, update=False) is False


def test_update_cache_after_build(src_dir: Path) -> None:
    """Simulates the build workflow: check without update, build, then update."""
    paths = [str(src_dir)]
    # First check — no cache yet
    assert did_change("build_test", paths, update=False) is True
    # Simulate successful build → update cache
    update_cache("build_test", paths)
    # No changes since update
    assert did_change("build_test", paths, update=False) is False
    # Modify a file
    (src_dir / "a.c").write_text("int main() { return 42; }")
    # Now detects the change
    assert did_change("build_test", paths, update=False) is True
    # Simulate failed build — don't call update_cache
    # Next check still sees the change
    assert did_change("build_test", paths, update=False) is True


def test_max_file_size_fingerprints_large_files(tmp_path: Path) -> None:
    """Files over max_file_size use (size, mtime_ns), not content."""
    import os

    big = tmp_path / "big.bin"
    big.write_bytes(b"a" * 2000)  # 2000 bytes

    paths = [str(big)]
    # Seed the cache with a 1000-byte threshold — big.bin is over it, so
    # only its size+mtime are hashed.
    assert did_change("large", paths, max_file_size=1000) is True
    assert did_change("large", paths, max_file_size=1000) is False

    # Replace the content but preserve size and mtime: should NOT be detected
    # (this is the known precision loss of fingerprint-mode).
    stat_before = big.stat()
    big.write_bytes(b"b" * 2000)
    os.utime(big, ns=(stat_before.st_atime_ns, stat_before.st_mtime_ns))
    assert did_change("large", paths, max_file_size=1000) is False

    # Change the size → detected.
    big.write_bytes(b"b" * 2001)
    assert did_change("large", paths, max_file_size=1000) is True


def test_max_file_size_none_hashes_content(tmp_path: Path) -> None:
    """Without max_file_size, content changes are always detected even if
    mtime is preserved."""
    import os

    f = tmp_path / "src.bin"
    f.write_bytes(b"a" * 2000)

    paths = [str(f)]
    assert did_change("content", paths) is True
    assert did_change("content", paths) is False

    # Rewrite with same size and same mtime but different content.
    st = f.stat()
    f.write_bytes(b"b" * 2000)
    os.utime(f, ns=(st.st_atime_ns, st.st_mtime_ns))
    assert did_change("content", paths) is True, (
        "content-mode should catch content changes regardless of mtime"
    )


def test_max_file_size_mode_switch_invalidates_cache(tmp_path: Path) -> None:
    """A file crossing the size threshold should invalidate the cached hash
    because the per-file mode marker (content vs stat) differs."""
    f = tmp_path / "grows.bin"
    f.write_bytes(b"a" * 500)  # under threshold

    paths = [str(f)]
    # First call: content-mode (file is under 1000 bytes).
    assert did_change("grow", paths, max_file_size=1000) is True
    assert did_change("grow", paths, max_file_size=1000) is False

    # Grow past the threshold → switches to stat-mode → different digest.
    f.write_bytes(b"a" * 1500)
    assert did_change("grow", paths, max_file_size=1000) is True
