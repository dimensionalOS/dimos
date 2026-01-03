# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for doclinks - using virtual markdown content against actual repo."""

from pathlib import Path

from doclinks import (
    build_doc_index,
    build_file_index,
    extract_other_backticks,
    find_symbol_line,
    process_markdown,
    split_by_ignore_regions,
)
import pytest

# Use the actual repo root
REPO_ROOT = Path(__file__).parent.parent.parent.parent


@pytest.fixture(scope="module")
def file_index():
    """Build file index once for all tests."""
    return build_file_index(REPO_ROOT)


@pytest.fixture(scope="module")
def doc_index():
    """Build doc index once for all tests."""
    return build_doc_index(REPO_ROOT)


class TestFileIndex:
    def test_finds_spec_files(self, file_index):
        """Should find spec.py files with various path suffixes."""
        # Exact match with path
        assert "protocol/service/spec.py" in file_index
        candidates = file_index["protocol/service/spec.py"]
        assert len(candidates) == 1
        assert candidates[0] == Path("dimos/protocol/service/spec.py")

    def test_service_spec_unique(self, file_index):
        """service/spec.py should uniquely match one file."""
        candidates = file_index.get("service/spec.py", [])
        assert len(candidates) == 1
        assert "protocol/service/spec.py" in str(candidates[0])

    def test_spec_ambiguous(self, file_index):
        """spec.py alone should match multiple files."""
        candidates = file_index.get("spec.py", [])
        assert len(candidates) > 1  # Multiple spec.py files exist

    def test_excludes_venv(self, file_index):
        """Should not include files from .venv directory."""
        for paths in file_index.values():
            for p in paths:
                # Check for .venv as a path component, not just substring
                assert ".venv" not in p.parts


class TestSymbolLookup:
    def test_find_configurable_in_spec(self):
        """Should find Configurable class in service/spec.py."""
        spec_path = REPO_ROOT / "dimos/protocol/service/spec.py"
        line = find_symbol_line(spec_path, "Configurable")
        assert line is not None
        assert line > 0

        # Verify it's the class definition line
        with open(spec_path) as f:
            lines = f.readlines()
            assert "class Configurable" in lines[line - 1]

    def test_find_nonexistent_symbol(self):
        """Should return None for symbols that don't exist."""
        spec_path = REPO_ROOT / "dimos/protocol/service/spec.py"
        line = find_symbol_line(spec_path, "NonExistentSymbol12345")
        assert line is None


class TestExtractBackticks:
    def test_extracts_symbols(self):
        """Should extract backticked terms excluding file refs."""
        line = "See [`service/spec.py`]() for `Configurable` and `Service`"
        symbols = extract_other_backticks(line, "service/spec.py")
        assert "Configurable" in symbols
        assert "Service" in symbols
        assert "service/spec.py" not in symbols

    def test_excludes_file_paths(self):
        """Should exclude things that look like file paths."""
        line = "See [`foo.py`]() and `bar.py` and `Symbol`"
        symbols = extract_other_backticks(line, "foo.py")
        assert "Symbol" in symbols
        assert "bar.py" not in symbols  # Has .py extension
        assert "foo.py" not in symbols


class TestProcessMarkdown:
    def test_resolves_service_spec(self, file_index):
        """Should resolve service/spec.py to full path."""
        content = "See [`service/spec.py`]() for details"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert len(errors) == 0
        assert len(changes) == 1
        assert "/dimos/protocol/service/spec.py" in new_content

    def test_auto_links_symbol(self, file_index):
        """Should auto-add line number for symbol on same line."""
        content = "The `Configurable` class is in [`service/spec.py`]()"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert len(errors) == 0
        assert "#L" in new_content  # Should have line number

    def test_preserves_existing_line_fragment(self, file_index):
        """Should preserve existing #L fragments."""
        content = "See [`service/spec.py`](#L99)"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert "#L99" in new_content

    def test_skips_anchor_links(self, file_index):
        """Should skip anchor-only links like [`Symbol`](#section)."""
        content = "See [`SomeClass`](#some-section) for details"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert len(errors) == 0
        assert len(changes) == 0
        assert new_content == content  # Unchanged

    def test_skips_non_file_refs(self, file_index):
        """Should skip refs that don't look like files."""
        content = "The `MyClass` is documented at [`MyClass`]()"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert len(errors) == 0
        assert len(changes) == 0

    def test_errors_on_ambiguous(self, file_index):
        """Should error when file reference is ambiguous."""
        content = "See [`spec.py`]() for details"  # Multiple spec.py files
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert len(errors) == 1
        assert "matches multiple files" in errors[0]

    def test_errors_on_not_found(self, file_index):
        """Should error when file doesn't exist."""
        content = "See [`nonexistent/file.py`]() for details"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert len(errors) == 1
        assert "No file matching" in errors[0]

    def test_github_mode(self, file_index):
        """Should generate GitHub URLs in github mode."""
        content = "See [`service/spec.py`]()"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="github",
            github_url="https://github.com/org/repo",
            github_ref="main",
        )

        assert "https://github.com/org/repo/blob/main/dimos/protocol/service/spec.py" in new_content

    def test_relative_mode(self, file_index):
        """Should generate relative paths in relative mode."""
        content = "See [`service/spec.py`]()"
        doc_path = REPO_ROOT / "docs/concepts/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="relative",
            github_url=None,
            github_ref="main",
        )

        assert new_content.startswith("See [`service/spec.py`](../../")
        assert "dimos/protocol/service/spec.py" in new_content


class TestDocIndex:
    def test_indexes_by_stem(self, doc_index):
        """Should index docs by lowercase stem."""
        assert "configuration" in doc_index
        assert "modules" in doc_index
        assert "development" in doc_index

    def test_case_insensitive(self, doc_index):
        """Should use lowercase keys."""
        # All keys should be lowercase
        for key in doc_index:
            assert key == key.lower()


class TestDocLinking:
    def test_resolves_doc_link(self, file_index, doc_index):
        """Should resolve [Text](.md) to doc path."""
        content = "See [Configuration](.md) for details"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
            doc_index=doc_index,
        )

        assert len(errors) == 0
        assert len(changes) == 1
        assert "[Configuration](/docs/" in new_content
        assert ".md)" in new_content

    def test_case_insensitive_lookup(self, file_index, doc_index):
        """Should match case-insensitively."""
        content = "See [CONFIGURATION](.md) for details"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
            doc_index=doc_index,
        )

        assert len(errors) == 0
        assert "[CONFIGURATION](" in new_content  # Preserves original text
        assert ".md)" in new_content

    def test_doc_link_github_mode(self, file_index, doc_index):
        """Should generate GitHub URLs for doc links."""
        content = "See [Configuration](.md)"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="github",
            github_url="https://github.com/org/repo",
            github_ref="main",
            doc_index=doc_index,
        )

        assert "https://github.com/org/repo/blob/main/docs/" in new_content
        assert ".md)" in new_content

    def test_doc_link_relative_mode(self, file_index, doc_index):
        """Should generate relative paths for doc links."""
        content = "See [Development](.md)"
        doc_path = REPO_ROOT / "docs/concepts/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="relative",
            github_url=None,
            github_ref="main",
            doc_index=doc_index,
        )

        assert len(errors) == 0
        # Should be relative path from docs/concepts/ to docs/
        assert "../" in new_content

    def test_doc_not_found_error(self, file_index, doc_index):
        """Should error when doc doesn't exist."""
        content = "See [NonexistentDoc](.md)"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
            doc_index=doc_index,
        )

        assert len(errors) == 1
        assert "No doc matching" in errors[0]

    def test_skips_regular_links(self, file_index, doc_index):
        """Should not affect regular markdown links."""
        content = "See [regular link](https://example.com) here"
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
            doc_index=doc_index,
        )

        assert new_content == content  # Unchanged


class TestIgnoreRegions:
    def test_split_no_ignore(self):
        """Content without ignore markers should be fully processed."""
        content = "Hello world"
        regions = split_by_ignore_regions(content)
        assert len(regions) == 1
        assert regions[0] == ("Hello world", True)

    def test_split_single_ignore(self):
        """Should correctly split around a single ignore region."""
        content = "before<!-- doclinks-ignore-start -->ignored<!-- doclinks-ignore-end -->after"
        regions = split_by_ignore_regions(content)

        # Should have: before (process), marker (no), ignored+end (no), after (process)
        assert len(regions) == 4
        assert regions[0] == ("before", True)
        assert regions[1][1] is False  # Start marker
        assert regions[2][1] is False  # Ignored content + end marker
        assert regions[3] == ("after", True)

    def test_split_multiple_ignores(self):
        """Should handle multiple ignore regions."""
        content = (
            "a<!-- doclinks-ignore-start -->x<!-- doclinks-ignore-end -->"
            "b<!-- doclinks-ignore-start -->y<!-- doclinks-ignore-end -->c"
        )
        regions = split_by_ignore_regions(content)

        # Check that processable regions are correctly identified
        processable = [r[0] for r in regions if r[1]]
        assert "a" in processable
        assert "b" in processable
        assert "c" in processable

    def test_split_case_insensitive(self):
        """Should handle different case in markers."""
        content = "before<!-- DOCLINKS-IGNORE-START -->ignored<!-- DOCLINKS-IGNORE-END -->after"
        regions = split_by_ignore_regions(content)

        processable = [r[0] for r in regions if r[1]]
        assert "before" in processable
        assert "after" in processable
        assert "ignored" not in processable

    def test_split_unclosed_ignore(self):
        """Unclosed ignore region should ignore rest of content."""
        content = "before<!-- doclinks-ignore-start -->rest of file"
        regions = split_by_ignore_regions(content)

        processable = [r[0] for r in regions if r[1]]
        assert "before" in processable
        assert "rest of file" not in processable

    def test_ignores_links_in_region(self, file_index):
        """Links inside ignore region should not be processed."""
        content = (
            "Process [`service/spec.py`]() here\n"
            "<!-- doclinks-ignore-start -->\n"
            "Skip [`service/spec.py`]() here\n"
            "<!-- doclinks-ignore-end -->\n"
            "Process [`service/spec.py`]() again"
        )
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
        )

        assert len(errors) == 0
        # Should have 2 changes (before and after ignore region)
        assert len(changes) == 2

        # Verify the ignored region is untouched
        assert "Skip [`service/spec.py`]() here" in new_content

        # Verify the processed regions have resolved links
        lines = new_content.split("\n")
        assert "/dimos/protocol/service/spec.py" in lines[0]
        assert "/dimos/protocol/service/spec.py" in lines[-1]

    def test_ignores_doc_links_in_region(self, file_index, doc_index):
        """Doc links inside ignore region should not be processed."""
        content = (
            "[Configuration](.md)\n"
            "<!-- doclinks-ignore-start -->\n"
            "[Configuration](.md) example\n"
            "<!-- doclinks-ignore-end -->\n"
            "[Configuration](.md)"
        )
        doc_path = REPO_ROOT / "docs/test.md"

        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
            doc_index=doc_index,
        )

        assert len(errors) == 0
        assert len(changes) == 2  # Only 2 links processed

        # Verify the ignored region still has .md placeholder
        assert "[Configuration](.md) example" in new_content


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
