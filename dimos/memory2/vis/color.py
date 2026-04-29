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

"""Color type and utilities for memory2 visualization.

Canonical storage is RGBA float 0-1 (matplotlib-native, easy to blend). All
consumers convert at the boundary:

    matplotlib: Color.rgba_f()     → (r, g, b, a)
    SVG:        Color.hex() + a    → "#rrggbb" + opacity="..."
    rerun:      Color.rgb_u8()     → (r, g, b) u8
    PIL:        Color.rgba_u8()    → (r, g, b, a) u8

Use ``Color.from_hex("#3498db")`` or a palette name (``"red"``, ``"blue"``, …)
to construct. Use ``ColorRange(cmap)`` + ``range(value)`` for cmap-deferred
colors whose min/max is learned as you add elements.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
import colorsys
from dataclasses import dataclass
import functools
from typing import Any


@functools.lru_cache(maxsize=16)
def _cmap(name: str) -> Any:
    import matplotlib.pyplot as plt

    return plt.get_cmap(name)


@dataclass(frozen=True, eq=False)
class Color:
    """Concrete RGBA color, stored as floats in [0, 1].

    Immutable — all manipulation methods return a new instance.
    """

    r: float
    g: float
    b: float
    a: float = 1.0

    @classmethod
    def from_hex(cls, s: str) -> Color:
        """Parse ``#rrggbb`` / ``#rgb`` or a palette name (``"red"``, …)."""
        if not s.startswith("#"):
            try:
                return _PALETTE_NAMES[s.lower()]
            except KeyError:
                raise ValueError(
                    f"Unknown color name {s!r}. Use a hex string (#rrggbb) "
                    f"or one of the palette names: {sorted(_PALETTE_NAMES)}"
                ) from None
        h = s.lstrip("#")
        if len(h) == 3:
            h = "".join(c * 2 for c in h)
        if len(h) != 6:
            raise ValueError(f"Invalid hex color {s!r}")
        return cls(int(h[0:2], 16) / 255, int(h[2:4], 16) / 255, int(h[4:6], 16) / 255)

    @classmethod
    def from_cmap(cls, cmap: str, t: float) -> Color:
        """Sample a matplotlib colormap at ``t`` (clamped to [0, 1])."""
        r, g, b, a = _cmap(cmap)(max(0.0, min(1.0, t)))
        return cls(r, g, b, a)

    @classmethod
    def coerce(cls, x: Color | DeferredColor | str) -> Color:
        """Normalize any accepted color form to a concrete ``Color``."""
        if isinstance(x, Color):
            return x
        if isinstance(x, DeferredColor):
            return x.resolve()
        return cls.from_hex(x)

    def hex(self) -> str:
        """``#rrggbb`` — alpha is dropped; SVG pairs this with an ``opacity`` attribute."""
        return f"#{round(self.r * 255):02x}{round(self.g * 255):02x}{round(self.b * 255):02x}"

    def rgb_u8(self) -> tuple[int, int, int]:
        return (round(self.r * 255), round(self.g * 255), round(self.b * 255))

    def rgba_u8(self) -> tuple[int, int, int, int]:
        return (*self.rgb_u8(), round(self.a * 255))

    def rgba_f(self) -> tuple[float, float, float, float]:
        return (self.r, self.g, self.b, self.a)

    def with_alpha(self, a: float) -> Color:
        return Color(self.r, self.g, self.b, a)

    def blend(self, other: Color, t: float) -> Color:
        """Linear RGB blend: ``t=0`` → self, ``t=1`` → other."""
        t = max(0.0, min(1.0, t))
        return Color(
            self.r + (other.r - self.r) * t,
            self.g + (other.g - self.g) * t,
            self.b + (other.b - self.b) * t,
            self.a + (other.a - self.a) * t,
        )

    def __str__(self) -> str:
        return self.hex()

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Color):
            return (self.r, self.g, self.b, self.a) == (other.r, other.g, other.b, other.a)
        if isinstance(other, str):
            try:
                return self.hex() == Color.from_hex(other).hex()
            except ValueError:
                return False
        return NotImplemented

    def __hash__(self) -> int:
        return hash((self.r, self.g, self.b, self.a))


class ColorRange:
    """Tracks value min/max as you call it; returns :class:`DeferredColor` instances.

    Each ``ColorRange`` is its own aggregator — no cross-plot bleed, no global
    registry. The returned ``DeferredColor`` resolves to a cmap-sampled
    :class:`Color` at render time using the final min/max.
    """

    def __init__(self, cmap: str = "turbo") -> None:
        self.cmap = cmap
        self._lo: float | None = None
        self._hi: float | None = None

    def __call__(self, value: float) -> DeferredColor:
        self._lo = value if self._lo is None else min(self._lo, value)
        self._hi = value if self._hi is None else max(self._hi, value)
        return DeferredColor(self, value)


@dataclass(frozen=True)
class DeferredColor:
    """A value tagged with a :class:`ColorRange`; resolves lazily to :class:`Color`."""

    range: ColorRange
    value: float

    def resolve(self) -> Color:
        lo, hi = self.range._lo, self.range._hi
        t = 0.5 if lo is None or hi is None or lo == hi else (self.value - lo) / (hi - lo)
        return Color.from_cmap(self.range.cmap, t)

    def __str__(self) -> str:
        return self.resolve().hex()


def resolve_deferred(elements: Iterable[Any]) -> None:
    """Mutate ``el.color`` from :class:`DeferredColor` → :class:`Color` for each element."""
    for el in elements:
        c = getattr(el, "color", None)
        if isinstance(c, DeferredColor):
            el.color = c.resolve()


# Named palette: 12 visually-distinct colors that share visual weight.
#
# Indices 0..5 are hand-curated flat-UI colors that match the defaults in
# vis/space/elements.py so a Plot embedded next to a Space drawing reads as
# the same family. Indices 6..11 are generated by gap-subdivision in HSL
# space (preserving the curated set's average L≈0.51 / S≈0.72) so they fill
# the largest hue gaps between the curated colors. Together they form 12
# maximally-distinct hues. Beyond 12, `palette_iter` continues with a
# golden-angle hue walk that uses the same average L/S.

blue = Color.from_hex("#3498db")
red = Color.from_hex("#e74c3c")
yellow = Color.from_hex("#f1c40f")
teal = Color.from_hex("#1abc9c")
purple = Color.from_hex("#9b59b6")
orange = Color.from_hex("#e67e22")
green = Color.from_hex("#4cdc29")
magenta = Color.from_hex("#dc2994")
indigo = Color.from_hex("#3329dc")
cyan = Color.from_hex("#29c9dc")
vermilion = Color.from_hex("#dc5b29")
amber = Color.from_hex("#dc9a29")

PALETTE: list[Color] = [
    blue,
    red,
    yellow,
    teal,
    purple,
    orange,
    green,
    magenta,
    indigo,
    cyan,
    vermilion,
    amber,
]

_PALETTE_NAMES: dict[str, Color] = {
    "blue": blue,
    "red": red,
    "yellow": yellow,
    "teal": teal,
    "purple": purple,
    "orange": orange,
    "green": green,
    "magenta": magenta,
    "indigo": indigo,
    "cyan": cyan,
    "vermilion": vermilion,
    "amber": amber,
}


def palette_iter(
    palette: list[Color] = PALETTE,
    exclude: Iterable[Color | str] | None = None,
) -> Iterator[Color]:
    """Yield colors forever for auto-assigning Series/Markers.

    Yields ``palette`` in order, then continues indefinitely via a
    golden-angle (137.5°) hue walk anchored at the average L/S of ``palette``
    so generated colors share visual weight with the named ones.

    ``exclude`` skips already-pinned colors; accepts ``Color`` or hex/name strings.
    """
    excluded: set[str] = set()
    for x in exclude or ():
        try:
            excluded.add(Color.coerce(x).hex())
        except ValueError:
            pass  # unknown string — nothing to exclude

    def emit(c: Color) -> bool:
        return c.hex() not in excluded

    for c in palette:
        if emit(c):
            yield c

    if not palette:
        return

    hls = [colorsys.rgb_to_hls(c.r, c.g, c.b) for c in palette]
    avg_l = sum(p[1] for p in hls) / len(hls)
    avg_s = sum(p[2] for p in hls) / len(hls)
    # Anchor the walk on the last palette color's hue so the first
    # generated color is offset 137.5° from the end of the named set.
    hue = hls[-1][0]
    while True:
        hue = (hue + 137.5 / 360.0) % 1.0
        r, g, b = colorsys.hls_to_rgb(hue, avg_l, avg_s)
        c = Color(r, g, b)
        if emit(c):
            yield c
