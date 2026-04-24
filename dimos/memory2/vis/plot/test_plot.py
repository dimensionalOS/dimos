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

"""Tests for Plot builder and SVG rendering."""

import math

import pytest

from dimos.memory2.type.observation import Observation
from dimos.memory2.vis.plot.elements import HLine, Markers, Series, Style, VLine
from dimos.memory2.vis.plot.plot import Plot, TimeAxis


class TestPlotAdd:
    """Plot.add() smart dispatch."""

    def test_add_series(self):
        p = Plot()
        s = Series(ts=[1, 2, 3], values=[10, 20, 30], label="speed")
        p.add(s)
        assert len(p) == 1
        assert p.elements[0] is s

    def test_add_markers(self):
        p = Plot()
        m = Markers(ts=[1, 2], values=[5, 10], color="red")
        p.add(m)
        assert len(p) == 1
        assert isinstance(p.elements[0], Markers)

    def test_add_hline(self):
        p = Plot()
        p.add(HLine(y=0.5, label="threshold"))
        assert len(p) == 1
        assert p.elements[0].y == 0.5

    def test_add_from_observation_list(self):
        obs_list = [
            Observation(id=i, ts=float(i), pose=(i, 0, 0, 0, 0, 0, 1), _data=float(i * 10))
            for i in range(5)
        ]
        p = Plot()
        p.add(obs_list, label="test", color="blue")
        assert len(p) == 1
        el = p.elements[0]
        assert isinstance(el, Series)
        assert el.ts == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert el.values == [0.0, 10.0, 20.0, 30.0, 40.0]
        assert el.label == "test"
        assert el.color == "blue"

    def test_add_chaining(self):
        p = Plot().add(Series(ts=[1, 2], values=[10, 20])).add(HLine(y=15))
        assert len(p) == 2

    def test_add_unknown_type_raises(self):
        p = Plot()
        with pytest.raises(TypeError, match="does not know how to handle"):
            p.add(42)


class TestPlotSVG:
    """SVG rendering via matplotlib."""

    def test_empty_plot(self):
        svg = Plot().to_svg()
        assert "<svg" in svg
        assert "</svg>" in svg

    def test_series_renders(self):
        p = Plot()
        p.add(Series(ts=[0, 1, 2, 3], values=[0, 1, 4, 9], label="y=x²"))
        svg = p.to_svg()
        assert "<svg" in svg

    def test_mixed_elements(self):
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[10, 20, 30], label="speed"))
        p.add(Markers(ts=[0.5, 1.5], values=[15, 25], label="events"))
        p.add(HLine(y=20, label="limit"))
        svg = p.to_svg()
        assert "<svg" in svg

    def test_to_svg_writes_file(self, tmp_path):
        p = Plot()
        p.add(Series(ts=[0, 1], values=[0, 1]))
        out = tmp_path / "test.svg"
        p.to_svg(str(out))
        assert out.exists()
        assert "<svg" in out.read_text()

    def test_default_axis_is_shared(self):
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[10, 20, 30], label="a"))
        p.add(Series(ts=[0, 1, 2], values=[15, 25, 35], label="b"))
        svg = p.to_svg()
        assert "<svg" in svg

    def test_named_axis_creates_twin(self):
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[10, 20, 30], label="speed (m/s)"))
        p.add(
            Series(
                ts=[0, 1, 2],
                values=[100, 200, 300],
                label="elapsed (s)",
                axis="right",
            )
        )
        svg = p.to_svg()
        assert "<svg" in svg
        # Both labels should appear in the merged legend
        assert "speed" in svg
        assert "elapsed" in svg

    def test_three_axes_offset_spines(self):
        # Primary + two named twins. Renderer should ladder the spines so they
        # don't collide on the right side. Just confirm it renders all three
        # series and doesn't crash.
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[10, 20, 30], label="speed"))
        p.add(Series(ts=[0, 1, 2], values=[100, 200, 300], label="time", axis="time"))
        p.add(Series(ts=[0, 1, 2], values=[0.1, 0.5, 0.9], label="sim", axis="semantics"))
        svg = p.to_svg()
        assert "<svg" in svg
        for label in ("speed", "time", "sim"):
            assert label in svg

    def test_auto_color_cycle_uses_theme_palette(self):
        # First two theme colors are blue then red. Without our shared cycle,
        # the twin-axis series would reuse the primary's first color.
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[10, 20, 30]))
        p.add(Series(ts=[0, 1, 2], values=[100, 200, 300], axis="right"))
        svg = p.to_svg()
        assert "#3498db" in svg  # blue (first theme color, primary axis)
        assert "#e74c3c" in svg  # red  (second theme color, twin axis)

    def test_series_default_style_is_solid(self):
        s = Series(ts=[0, 1], values=[0, 1])
        assert s.style == Style.solid

    def test_series_dashed_style_renders_as_dashed(self):
        # matplotlib emits stroke-dasharray for dashed lines
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[0, 1, 2], style=Style.dashed))
        svg = p.to_svg()
        assert "stroke-dasharray" in svg

    def test_series_dotted_style_renders_as_dotted(self):
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[0, 1, 2], style=Style.dotted))
        svg = p.to_svg()
        assert "stroke-dasharray" in svg

    def test_series_connect_breaks_on_gap(self):
        # Three samples at t=0, 1, 5. Gap 0→1 is 1s (kept), gap 1→5 is 4s
        # (broken). The renderer should produce SVG without crashing.
        from dimos.memory2.vis.plot.svg import _break_on_gaps

        ts, values = _break_on_gaps([0.0, 1.0, 5.0], [10.0, 20.0, 30.0], 2.0)
        # The 4s gap should have inserted a NaN between samples 1 and 5.
        assert len(ts) == 4
        assert ts[0] == 0.0
        assert ts[1] == 1.0
        assert math.isnan(ts[2])
        assert ts[3] == 5.0

        # End-to-end through the renderer.
        p = Plot()
        p.add(Series(ts=[0.0, 1.0, 5.0], values=[10.0, 20.0, 30.0], connect=2.0))
        svg = p.to_svg()
        assert "<svg" in svg

    def test_series_connect_none_keeps_all(self):
        from dimos.memory2.vis.plot.svg import _break_on_gaps

        ts, values = _break_on_gaps([0.0, 1.0, 5.0], [10.0, 20.0, 30.0], None)
        assert ts == [0.0, 1.0, 5.0]
        assert values == [10.0, 20.0, 30.0]

    def test_series_gap_fill_zero_creates_valley(self):
        # Drops to zero at the gap boundaries instead of breaking the line.
        from dimos.memory2.vis.plot.svg import _break_on_gaps

        ts, values = _break_on_gaps([0.0, 1.0, 5.0], [10.0, 20.0, 30.0], 2.0, fill=0.0)
        # At the gap (between t=1 and t=5), inject (1, 0) and (5, 0).
        assert ts == [0.0, 1.0, 1.0, 5.0, 5.0]
        assert values == [10.0, 20.0, 0.0, 0.0, 30.0]

    def test_series_gap_fill_arbitrary_value(self):
        from dimos.memory2.vis.plot.svg import _break_on_gaps

        ts, values = _break_on_gaps([0.0, 1.0, 5.0], [10.0, 20.0, 30.0], 2.0, fill=42.0)
        assert ts == [0.0, 1.0, 1.0, 5.0, 5.0]
        assert values == [10.0, 20.0, 42.0, 42.0, 30.0]

    def test_vline_renders(self):
        from dimos.memory2.vis import color

        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[0, 1, 2]))
        p.add(VLine(x=1.0, color=color.red, label="marker"))
        svg = p.to_svg()
        assert "<svg" in svg
        assert "marker" in svg
        assert color.red.hex() in svg

    def test_vline_add_dispatch(self):
        # VLine should be storable via Plot.add() like the other element types.
        p = Plot()
        p.add(VLine(x=5.0))
        assert len(p) == 1
        assert isinstance(p.elements[0], VLine)

    def test_hline_uses_style_enum(self):
        # Default is Style.dashed; renderer should still produce dasharray
        p = Plot()
        p.add(Series(ts=[0, 1], values=[0, 1]))  # solid baseline so we have data
        p.add(HLine(y=0.5, style=Style.dotted))
        svg = p.to_svg()
        assert "stroke-dasharray" in svg

    def test_time_axis_default_is_relative(self):
        assert Plot().time_axis == TimeAxis.relative

    def test_time_axis_relative_shows_seconds_from_zero(self):
        # Use unix-ish timestamps. With relative mode the tick labels should
        # be small second counts ("0s", "60s", ...), not huge unix numbers.
        p = Plot()  # default relative
        p.add(Series(ts=[1_700_000_000, 1_700_000_060, 1_700_000_120], values=[1, 2, 3]))
        svg = p.to_svg()
        assert "0s" in svg
        assert "120s" in svg or "60s" in svg
        # The raw unix number should not be rendered as a tick label.
        assert "1700000000" not in svg

    def test_time_axis_absolute_shows_hhmmss(self):
        # Pick a timestamp whose local HH:MM is determinate enough to check.
        # Use 1_700_000_000 as an arbitrary reference; we can't assume a timezone,
        # but the format HH:MM:SS should still be present as ":" separators.
        p = Plot(time_axis=TimeAxis.absolute)
        p.add(Series(ts=[1_700_000_000, 1_700_000_030], values=[1, 2]))
        svg = p.to_svg()
        # HH:MM:SS format means colons should appear in at least one tick label.
        import re

        # matplotlib emits tick labels as <text>HH:MM:SS</text>
        assert re.search(r"\d\d:\d\d:\d\d", svg)

    def test_time_axis_raw_preserves_default_matplotlib_format(self):
        # In raw mode we do nothing, so matplotlib's default numeric formatter
        # runs. Big unix timestamps get rendered in scientific form.
        p = Plot(time_axis=TimeAxis.raw)
        p.add(Series(ts=[1_700_000_000, 1_700_000_060], values=[1, 2]))
        svg = p.to_svg()
        # Raw mode should not produce "0s" style labels.
        assert "0s" not in svg

    def test_opacity_appears_in_svg(self):
        # opacity=0.4 should land as opacity="0.4" on the matplotlib-rendered
        # path. matplotlib emits it as either `opacity` or `stroke-opacity`
        # depending on the artist; we just need to see the value in the output.
        p = Plot()
        p.add(Series(ts=[0, 1, 2], values=[0, 1, 2], opacity=0.4))
        svg = p.to_svg()
        assert "0.4" in svg

    def test_explicit_color_excluded_from_auto_cycle(self):
        # If the user pins a Series to color.red, the auto-cycle for the next
        # series should skip red and yield yellow (the third color) instead
        # of red — otherwise we'd get two red lines.
        from dimos.memory2.vis import color

        p = Plot()
        p.add(Series(ts=[0, 1], values=[0, 1]))  # auto → blue (first)
        p.add(Series(ts=[0, 1], values=[2, 3], color=color.red))  # explicit red
        p.add(Series(ts=[0, 1], values=[4, 5]))  # auto → yellow (red is excluded)
        svg = p.to_svg()
        # Both blue and yellow should appear, plus the explicit red.
        assert color.blue.hex() in svg
        assert color.red.hex() in svg
        assert color.yellow.hex() in svg
        # Red should appear exactly once (the explicit one, not from the cycle).
        assert svg.count(color.red.hex()) == 1


class TestPlotRepr:
    def test_repr_empty(self):
        assert repr(Plot()) == "Plot()"

    def test_repr_with_elements(self):
        p = Plot()
        p.add(Series(ts=[0], values=[0]))
        p.add(Series(ts=[0], values=[0]))
        p.add(HLine(y=1))
        assert repr(p) == "Plot(HLine=1, Series=2)"


class TestPlotRerunStub:
    """Plot.to_rerun() is currently a no-op placeholder — must not raise."""

    def test_to_rerun_does_not_raise(self):
        Plot().to_rerun()


class TestPalette:
    """The named palette and palette_iter live in vis/color.py."""

    def test_named_constants_exist(self):
        from dimos.memory2.vis import color

        assert color.blue == "#3498db"
        assert color.red == "#e74c3c"
        assert color.green.hex().startswith("#")
        assert color.amber.hex().startswith("#")
        assert len(color.PALETTE) == 12
        assert color.PALETTE[0] == color.blue
        assert color.PALETTE[6] == color.green

    def test_palette_iter_yields_palette_first(self):
        from dimos.memory2.vis import color

        it = color.palette_iter()
        assert [next(it) for _ in range(12)] == color.PALETTE

    def test_palette_iter_continues_past_palette(self):
        from dimos.memory2.vis import color

        it = color.palette_iter()
        first_thirteen = [next(it) for _ in range(13)]
        # 13th color is generated, must be a valid Color and distinct from all 12 named.
        assert first_thirteen[12].hex().startswith("#")
        assert first_thirteen[12] not in color.PALETTE

    def test_palette_iter_excludes(self):
        from dimos.memory2.vis import color

        it = color.palette_iter(exclude={color.red, color.yellow})
        first_three = [next(it) for _ in range(3)]
        # Skipped red and yellow, so the first three are blue, teal, purple.
        assert first_three == [color.blue, color.teal, color.purple]
