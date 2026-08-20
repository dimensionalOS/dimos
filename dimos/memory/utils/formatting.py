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

"""Rich rendering helpers for memory types.

All rich/ANSI logic lives here. Other modules import the mixin and
``render_text`` — nothing else needs to touch ``rich`` directly.
"""

from __future__ import annotations

from rich.console import Console
from rich.text import Text

_console = Console(force_terminal=True, highlight=False)


def render_text(text: Text) -> str:
    """Render rich Text to a terminal string with ANSI codes."""
    with _console.capture() as cap:
        _console.print(text, end="", soft_wrap=True)
    return cap.get()


def _colorize(plain: str) -> Text:
    """Turn ``'name(args)'``, ``'a | b'``, or ``'a -> b'`` into rich Text with cyan names."""
    t = Text()
    pipe = Text(" | ", style="dim")
    arrow = Text(" -> ", style="dim")
    for i, seg in enumerate(plain.split(" | ")):
        if i > 0:
            t.append_text(pipe)
        for j, part in enumerate(seg.split(" -> ")):
            if j > 0:
                t.append_text(arrow)
            name, _, rest = part.partition("(")
            t.append(name, style="cyan")
            if rest:
                t.append(f"({rest}")
    return t


class FilterRepr:
    """Mixin for filters: subclass defines ``__str__``, gets colored ``__repr__`` free."""

    def __repr__(self) -> str:
        return render_text(_colorize(str(self)))
