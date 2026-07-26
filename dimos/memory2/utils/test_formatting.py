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
from rich.text import Text

from dimos.memory2.utils.formatting import (
    FilterRepr,
    _colorize,
    render_text,
)


def test_render_text_keeps_plain_content() -> None:
    out = render_text(Text("hello"))
    assert "hello" in out
    assert isinstance(out, str)


def test_colorize_preserves_plain_and_colors() -> None:
    t = _colorize("a | b")
    # plain text is preserved, and the names are wrapped with the cyan style
    assert t.plain == "a | b"
    styles = [span.style for span in t.spans]
    assert "cyan" in styles


def test_colorize_pipeline_with_arrow() -> None:
    t = _colorize("a -> b")
    assert t.plain == "a -> b"
    assert "cyan" in [span.style for span in t.spans]


def test_filter_repr_mixin_renders_colored_str() -> None:
    import re

    class _Demo(FilterRepr):
        def __str__(self) -> str:
            return "foo(a | b)"

    rendered = repr(_Demo())
    # ANSI codes wrap the styled segments; strip them to recover the
    # underlying plain text.
    plain = re.sub(r"\x1b\[[0-9;]*m", "", rendered)
    assert plain == "foo(a | b)"
    assert isinstance(rendered, str)
