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

"""Tests for pattern matching utilities."""

import re

from dimos.protocol.pubsub.patterns import Glob, pattern_matches


class TestPatternMatchesString:
    """Tests for exact string matching."""

    def test_exact_match(self) -> None:
        assert pattern_matches("/sensor/temp", "/sensor/temp") is True

    def test_no_match(self) -> None:
        assert pattern_matches("/sensor/temp", "/sensor/humidity") is False

    def test_empty_string(self) -> None:
        assert pattern_matches("", "") is True
        assert pattern_matches("", "/sensor") is False

    def test_partial_match_fails(self) -> None:
        assert pattern_matches("/sensor", "/sensor/temp") is False
        assert pattern_matches("/sensor/temp", "/sensor") is False


class TestPatternMatchesGlob:
    """Tests for Glob pattern matching."""

    def test_single_wildcard(self) -> None:
        glob = Glob("/sensor/*")
        assert pattern_matches(glob, "/sensor/temp") is True
        assert pattern_matches(glob, "/sensor/humidity") is True
        assert pattern_matches(glob, "/sensor/") is True

    def test_single_wildcard_no_slash(self) -> None:
        glob = Glob("/sensor/*")
        assert pattern_matches(glob, "/sensor/nested/path") is False

    def test_double_wildcard(self) -> None:
        glob = Glob("/robot/**")
        assert pattern_matches(glob, "/robot/arm") is True
        assert pattern_matches(glob, "/robot/arm/joint1") is True
        assert pattern_matches(glob, "/robot/leg/motor/encoder") is True

    def test_question_mark(self) -> None:
        glob = Glob("/sensor/?")
        assert pattern_matches(glob, "/sensor/a") is True
        assert pattern_matches(glob, "/sensor/1") is True
        assert pattern_matches(glob, "/sensor/ab") is False

    def test_mixed_patterns(self) -> None:
        glob = Glob("/robot/*/joint?")
        assert pattern_matches(glob, "/robot/arm/joint1") is True
        assert pattern_matches(glob, "/robot/leg/joint2") is True
        assert pattern_matches(glob, "/robot/arm/joint12") is False
        assert pattern_matches(glob, "/robot/arm/nested/joint1") is False

    def test_no_wildcards(self) -> None:
        glob = Glob("/exact/path")
        assert pattern_matches(glob, "/exact/path") is True
        assert pattern_matches(glob, "/exact/other") is False

    def test_double_wildcard_middle(self) -> None:
        glob = Glob("/start/**/end")
        # Note: ** becomes .* so /start/**/end requires a / before end
        assert pattern_matches(glob, "/start//end") is True
        assert pattern_matches(glob, "/start/middle/end") is True
        assert pattern_matches(glob, "/start/a/b/c/end") is True


class TestPatternMatchesRegex:
    """Tests for compiled regex pattern matching."""

    def test_simple_regex(self) -> None:
        pattern = re.compile(r"/sensor/\w+")
        assert pattern_matches(pattern, "/sensor/temp") is True
        assert pattern_matches(pattern, "/sensor/123") is True

    def test_regex_anchored(self) -> None:
        pattern = re.compile(r"/sensor/temp")
        assert pattern_matches(pattern, "/sensor/temp") is True
        assert pattern_matches(pattern, "/sensor/temperature") is False

    def test_regex_groups(self) -> None:
        pattern = re.compile(r"/robot/(arm|leg)/joint(\d+)")
        assert pattern_matches(pattern, "/robot/arm/joint1") is True
        assert pattern_matches(pattern, "/robot/leg/joint42") is True
        assert pattern_matches(pattern, "/robot/head/joint1") is False

    def test_regex_optional(self) -> None:
        pattern = re.compile(r"/sensor/temp/?")
        assert pattern_matches(pattern, "/sensor/temp") is True
        assert pattern_matches(pattern, "/sensor/temp/") is True


class TestGlobClass:
    """Tests for the Glob class itself."""

    def test_pattern_property(self) -> None:
        glob = Glob("/sensor/*")
        assert glob.pattern == "/sensor/[^/]*"

    def test_glob_property(self) -> None:
        glob = Glob("/sensor/*")
        assert glob.glob == "/sensor/*"

    def test_repr(self) -> None:
        glob = Glob("/sensor/*")
        assert repr(glob) == "Glob('/sensor/*')"

    def test_double_star_regex(self) -> None:
        glob = Glob("/robot/**")
        assert glob.pattern == "/robot/.*"

    def test_special_chars_escaped(self) -> None:
        glob = Glob("/path.with.dots")
        assert pattern_matches(glob, "/path.with.dots") is True
        assert pattern_matches(glob, "/pathXwithXdots") is False
