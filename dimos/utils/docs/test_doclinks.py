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

import pytest

from dimos.utils.docs.doclinks import (
    build_doc_index,
    build_file_index,
    extract_other_backticks,
    find_symbol_line,
    pick_best_candidate,
    process_markdown,
    resolve_candidates,
    score_path_similarity,
    split_by_ignore_regions,
)

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

        new_content, _changes, errors = process_markdown(
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

        new_content, _changes, _errors = process_markdown(
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

        _new_content, changes, errors = process_markdown(
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

        _new_content, _changes, errors = process_markdown(
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

        _new_content, _changes, errors = process_markdown(
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

        new_content, _changes, _errors = process_markdown(
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
        doc_path = REPO_ROOT / "docs/usage/test.md"

        new_content, _changes, _errors = process_markdown(
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
        assert "blueprints" in doc_index

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

        new_content, _changes, errors = process_markdown(
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

        new_content, _changes, _errors = process_markdown(
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
        content = "See [Blueprints](.md)"
        doc_path = REPO_ROOT / "docs/usage/test.md"

        new_content, _changes, errors = process_markdown(
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
        # Should be relative path from docs/usage/ to target doc
        assert "[Blueprints](blueprints.md)" in new_content

    def test_doc_not_found_error(self, file_index, doc_index):
        """Should error when doc doesn't exist."""
        content = "See [NonexistentDoc](.md)"
        doc_path = REPO_ROOT / "docs/test.md"

        _new_content, _changes, errors = process_markdown(
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

        new_content, _changes, _errors = process_markdown(
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


class TestPathSimilarity:
    def test_exact_dir_match(self):
        """Same directory components should give high score."""
        candidate = Path("docs/agents/docs/codeblocks.md")
        score = score_path_similarity(candidate, "docs/agents/docs_agent/codeblocks.md")
        assert score >= 2  # docs, agents

    def test_partial_match(self):
        """Some shared dirs should give partial score."""
        candidate = Path("docs/other/codeblocks.md")
        score = score_path_similarity(candidate, "docs/agents/docs_agent/codeblocks.md")
        assert score == 2  # docs dir + filename match

    def test_no_match(self):
        """Unrelated dirs should give filename-only score."""
        candidate = Path("src/lib/codeblocks.md")
        score = score_path_similarity(candidate, "docs/agents/docs_agent/codeblocks.md")
        assert score == 1  # filename match only, no dir overlap

    def test_pick_best_single(self):
        """Single candidate always wins."""
        candidates = [Path("docs/agents/docs/codeblocks.md")]
        best = pick_best_candidate(candidates, "docs/agents/docs_agent/codeblocks.md")
        assert best == candidates[0]

    def test_pick_best_disambiguates(self):
        """Should pick candidate with more directory overlap."""
        candidates = [
            Path("docs/other/codeblocks.md"),
            Path("docs/agents/docs/codeblocks.md"),
        ]
        best = pick_best_candidate(candidates, "docs/agents/docs_agent/codeblocks.md")
        assert best == Path("docs/agents/docs/codeblocks.md")

    def test_pick_best_tie_returns_none(self):
        """Tied scores should return None."""
        candidates = [
            Path("a/x/file.md"),
            Path("b/x/file.md"),
        ]
        best = pick_best_candidate(candidates, "c/x/file.md")
        assert best is None


class TestResolveCandidates:
    def test_single_candidate(self):
        candidates = [Path("docs/usage/modules.md")]
        assert resolve_candidates(candidates, "modules.md") == candidates[0]

    def test_empty_candidates(self):
        assert resolve_candidates([], "modules.md") is None

    def test_disambiguates(self):
        candidates = [
            Path("docs/other/codeblocks.md"),
            Path("docs/agents/docs/codeblocks.md"),
        ]
        result = resolve_candidates(candidates, "docs/agents/docs_agent/codeblocks.md")
        assert result == Path("docs/agents/docs/codeblocks.md")

    def test_tie_returns_none(self):
        candidates = [Path("a/x/file.md"), Path("b/x/file.md")]
        assert resolve_candidates(candidates, "c/x/file.md") is None


class TestLinkResolution:
    def _process(self, content, file_index, doc_index, doc_path=None, link_mode="absolute"):
        if doc_path is None:
            doc_path = REPO_ROOT / "docs/usage/test.md"
        return process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode=link_mode,
            github_url=None,
            github_ref="main",
            doc_index=doc_index,
        )

    def test_resolves_relative_md_link(self, file_index, doc_index):
        """Should resolve a valid relative .md link to absolute path."""
        # docs/usage/configuration.md exists — link from docs/usage/test.md
        content = "[Configuration](configuration.md)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert len(errors) == 0
        assert "configuration.md" in new_content

    def test_validates_absolute_md_link(self, file_index, doc_index):
        """Valid absolute .md link should be left unchanged."""
        content = "[Configuration](/docs/usage/configuration.md)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert len(errors) == 0
        assert new_content == content

    def test_reports_broken_absolute_md_link(self, file_index, doc_index):
        """Broken absolute .md link with no match should error."""
        content = "[Foo](/docs/nonexistent/xyzzy_no_match.md)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert len(errors) == 1
        assert "Broken link" in errors[0] or "does not exist" in errors[0]

    def test_searches_broken_relative_link(self, file_index, doc_index):
        """Broken relative .md link should be resolved by name search if unique."""
        # Link to a non-existent relative path, but stem matches a known doc
        content = "[Configuration](../nonexistent/configuration.md)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        # Should resolve via search fallback (configuration.md exists)
        if "configuration" in doc_index and len(doc_index["configuration"]) == 1:
            assert len(errors) == 0
            assert len(changes) == 1
            assert "found by search" in changes[0]
        else:
            # Multiple matches — disambiguation should kick in
            assert len(errors) <= 1

    def test_disambiguates_by_path_similarity(self, file_index, doc_index):
        """Multiple candidates should be disambiguated by directory overlap."""
        # Build a custom doc_index with multiple candidates
        from collections import defaultdict

        custom_doc_index: dict[str, list[Path]] = defaultdict(list)
        custom_doc_index["testdoc"] = [
            Path("docs/other/testdoc.md"),
            Path("docs/agents/docs/testdoc.md"),
        ]

        content = "[TestDoc](../agents/docs_agent/testdoc.md)"
        doc_path = REPO_ROOT / "docs/usage/test.md"
        new_content, changes, errors = process_markdown(
            content,
            REPO_ROOT,
            doc_path,
            file_index,
            link_mode="absolute",
            github_url=None,
            github_ref="main",
            doc_index=custom_doc_index,
        )

        # Should pick docs/agents/docs/testdoc.md (shares "docs", "agents")
        assert len(errors) == 0
        assert len(changes) == 1
        assert "agents/docs/testdoc.md" in new_content

    def test_skips_url_md_links(self, file_index, doc_index):
        """HTTP(S) .md links should be left untouched."""
        content = "[External](https://example.com/guide.md)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert len(errors) == 0
        assert len(changes) == 0
        assert new_content == content

    def test_preserves_fragment(self, file_index, doc_index):
        """Fragment (#section) should be preserved in resolved link."""
        content = "[Config](configuration.md#advanced)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert "#advanced" in new_content

    def test_skips_backtick_wrapped(self, file_index, doc_index):
        """Backtick-wrapped .md link text should be skipped by md_link_pattern."""
        content = "[`configuration.md`](configuration.md)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        # The code_pattern handles backtick links; md_link_pattern sees backticks and skips
        # No double-processing should occur
        assert "configuration.md" in new_content

    def test_md_links_in_ignore_region(self, file_index, doc_index):
        """Links in ignore regions should not be processed."""
        content = (
            "[Configuration](configuration.md)\n"
            "<!-- doclinks-ignore-start -->\n"
            "[Configuration](broken_nonexistent.md)\n"
            "<!-- doclinks-ignore-end -->\n"
            "[Configuration](configuration.md)"
        )
        new_content, changes, errors = self._process(content, file_index, doc_index)

        # The broken link in ignore region should not produce errors
        assert "broken_nonexistent.md" in new_content  # Preserved as-is

    def test_validates_absolute_py_link(self, file_index, doc_index):
        """Valid absolute .py link (without backticks) should be left unchanged."""
        content = "[spec](/dimos/protocol/service/spec.py)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert len(errors) == 0
        assert new_content == content

    def test_broken_py_link_searches_file_index(self, file_index, doc_index):
        """Broken .py link should fall back to file_index search."""
        content = "[spec](/nonexistent/path/service/spec.py)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        # service/spec.py is unique in file_index — should resolve
        # But spec.py alone is ambiguous, so it depends on disambiguation
        # The fallback searches by filename (spec.py) which has multiple matches
        # pick_best_candidate should resolve via path similarity
        if len(errors) == 0:
            assert "fixed broken link" in changes[0]
        # If ambiguous, at least we get an error not a silent pass
        else:
            assert "Broken link" in errors[0]

    def test_validates_directory_link(self, file_index, doc_index):
        """Valid directory link should be left unchanged."""
        content = "[examples](/examples/)"
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

        if (REPO_ROOT / "examples").exists():
            assert len(errors) == 0
            assert new_content == content
        else:
            # Directory doesn't exist — should error
            assert len(errors) == 1

    def test_skips_image_links(self, file_index, doc_index):
        """Image links ![alt](path) should not be processed."""
        content = "![screenshot](assets/screenshot.png)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert len(errors) == 0
        assert len(changes) == 0
        assert new_content == content

    def test_skips_mailto_links(self, file_index, doc_index):
        """mailto: links should be left untouched."""
        content = "[Email](mailto:test@example.com)"
        new_content, changes, errors = self._process(content, file_index, doc_index)

        assert len(errors) == 0
        assert len(changes) == 0
        assert new_content == content
