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

"""CLI arg handling for `dimos spy`: --transport parsing and validation."""

import pytest

from dimos.utils.cli.spy.run_spy import _parse_transports


def test_parse_transports_default_is_none():
    assert _parse_transports([]) is None
    assert _parse_transports(["web"]) is None


def test_parse_transports_collects_repeated_flags():
    assert _parse_transports(["--transport", "lcm"]) == ["lcm"]
    assert _parse_transports(["--transport=zenoh"]) == ["zenoh"]
    assert _parse_transports(["--transport", "lcm", "--transport", "zenoh"]) == ["lcm", "zenoh"]


def test_parse_transports_rejects_unknown_transport():
    with pytest.raises(SystemExit, match="zneoh"):
        _parse_transports(["--transport", "zneoh"])


def test_parse_transports_error_lists_valid_choices():
    with pytest.raises(SystemExit, match="lcm"):
        _parse_transports(["--transport=nope"])
