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
from dimos.utils.colors import (
    HEAT_GRADIENT_ANSI256,
    blue,
    cyan,
    green,
    orange,
    red,
    yellow,
)

RESET = "\033[0m"


def test_green_wraps_and_resets() -> None:
    assert green("hi") == "\033[92mhi\033[0m"


def test_blue() -> None:
    assert blue("x") == "\033[94mx\033[0m"


def test_red() -> None:
    assert red("x") == "\033[91mx\033[0m"


def test_yellow() -> None:
    assert yellow("x") == "\033[93mx\033[0m"


def test_cyan() -> None:
    assert cyan("x") == "\033[96mx\033[0m"


def test_orange_uses_256() -> None:
    assert orange("x") == "\033[38;5;208mx\033[0m"


def test_preserves_text() -> None:
    for fn in (green, blue, red, yellow, cyan, orange):
        wrapped = fn("abc")
        assert wrapped.count("abc") == 1
        assert wrapped.endswith(RESET)


def test_heat_gradient_shape() -> None:
    assert HEAT_GRADIENT_ANSI256[0] == 52
    assert HEAT_GRADIENT_ANSI256[-1] == 34
    assert len(HEAT_GRADIENT_ANSI256) == 18
