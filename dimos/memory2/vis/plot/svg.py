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

"""Matplotlib-based SVG renderer for Plot."""

from __future__ import annotations

import io
from typing import TYPE_CHECKING, Any

import matplotlib
import matplotlib.pyplot as plt

from dimos.memory2.vis.color import Color, palette_iter
from dimos.memory2.vis.plot.elements import HLine, Markers, Series, VLine
from dimos.memory2.vis.plot.plot import TimeAxis

if TYPE_CHECKING:
    from dimos.memory2.vis.plot.plot import Plot

matplotlib.use("Agg")


def _apply_time_axis(ax: Any, plot: Plot) -> None:
    """Install an x-axis tick formatter based on plot.time_axis."""
    if plot.time_axis == TimeAxis.raw:
        return

    # Reference point: earliest sample across all Series/Markers.
    all_ts: list[float] = []
    for el in plot.elements:
        if isinstance(el, (Series, Markers)) and el.ts:
            all_ts.append(el.ts[0])
    if not all_ts:
        return
    t0 = min(all_ts)

    from matplotlib.ticker import FuncFormatter

    if plot.time_axis == TimeAxis.relative:

        def fmt(ts: float, _: int = 0) -> str:
            return f"{ts - t0:.0f}s"
    elif plot.time_axis == TimeAxis.absolute:
        from datetime import datetime

        def fmt(ts: float, _: int = 0) -> str:
            return datetime.fromtimestamp(ts).strftime("%H:%M:%S")
    else:
        return

    ax.xaxis.set_major_formatter(FuncFormatter(fmt))


def _break_on_gaps(
    ts: list[float],
    values: list[float],
    max_gap: float | None,
    fill: float | None = None,
) -> tuple[list[float], list[float]]:
    """Handle gaps in a series. Returns (ts', values') ready to plot.

    When two consecutive samples are more than ``max_gap`` apart in x:

    - ``fill is None``: insert ``(NaN, NaN)`` between them. matplotlib's
      ``plot`` skips line segments touching a NaN endpoint, so the line
      visually breaks across the gap.
    - ``fill is not None``: insert ``(prev_t, fill)`` and ``(next_t, fill)``,
      so the line drops vertically at the prev sample, runs flat at ``fill``
      across the gap, then rises vertically at the next sample.

    Returns the arrays unchanged when ``max_gap`` is ``None``.
    """
    if max_gap is None or len(ts) < 2:
        return list(ts), list(values)
    out_ts: list[float] = [ts[0]]
    out_v: list[float] = [values[0]]
    nan = float("nan")
    for i in range(1, len(ts)):
        if ts[i] - ts[i - 1] > max_gap:
            if fill is None:
                out_ts.append(nan)
                out_v.append(nan)
            else:
                out_ts.append(ts[i - 1])
                out_v.append(fill)
                out_ts.append(ts[i])
                out_v.append(fill)
        out_ts.append(ts[i])
        out_v.append(values[i])
    return out_ts, out_v


def render(plot: Plot, width: float = 10, height: float = 3.5) -> str:
    """Render a Plot to an SVG string via matplotlib."""
    with plt.style.context("dark_background"):
        fig, ax = plt.subplots(figsize=(width, height))
        fig.patch.set_alpha(0.0)
        ax.set_facecolor("#16213e")
        ax.grid(True, color="#2a2a4a", linewidth=0.5)

        # Lazily create twin y-axes for any element with axis != None.
        # All twins share the primary x-axis (matplotlib `ax.twinx()`).
        # The first twin sits at the default right edge; each additional twin
        # gets its right spine pushed outward in axes-relative coordinates so
        # their tick labels form a ladder instead of stacking on top of each
        # other. The figure's right margin grows below to make room.
        axes: dict[str | None, Any] = {None: ax}
        twin_offset_step = 0.10

        def axis_for(name: str | None) -> Any:
            if name not in axes:
                twin = ax.twinx()
                twin.set_facecolor("none")
                # Index among twins: 0 = first (no offset), 1 = second, ...
                twin_index = sum(1 for k in axes if k is not None)
                if twin_index > 0:
                    twin.spines["right"].set_position(("axes", 1.0 + twin_offset_step * twin_index))
                axes[name] = twin
            return axes[name]

        # Drive a single shared color cycle across all axes (primary + twins)
        # so series on a twin don't reuse the primary's first color. Excludes
        # any color the user has already pinned to a specific element so the
        # auto-cycle won't double-up on it.
        explicit_colors = {
            el.color
            for el in plot.elements
            if isinstance(el, (Series, Markers)) and el.color is not None
        }
        color_iter = palette_iter(exclude=explicit_colors)

        # Track the dominant color of each twin axis (the color of the first
        # Series/Markers landed on it) so we can color-code its spine and tick
        # labels after the plot loop. Primary axis stays neutral so its ticks
        # read as the baseline.
        axis_colors: dict[str, tuple[float, float, float, float]] = {}

        def mpl_color(c: Color, opacity: float) -> tuple[float, float, float, float]:
            return c.with_alpha(c.a * opacity).rgba_f()

        for el in plot.elements:
            # VLine has no axis field — it always draws on the primary.
            target = ax if isinstance(el, VLine) else axis_for(el.axis)
            raw: str | Color | None = el.color
            if raw is None and isinstance(el, (Series, Markers)):
                raw = next(color_iter)
            color = Color.coerce(raw) if raw is not None else None
            rgba = mpl_color(color, el.opacity) if color is not None else None
            if (
                not isinstance(el, VLine)
                and el.axis is not None
                and el.axis not in axis_colors
                and isinstance(el, (Series, Markers))
                and rgba is not None
            ):
                axis_colors[el.axis] = rgba
            if isinstance(el, Series):
                ts, values = _break_on_gaps(el.ts, el.values, el.connect, el.gap_fill)
                target.plot(
                    ts,
                    values,
                    color=rgba,
                    linewidth=el.width,
                    label=el.label,
                    linestyle=el.style.value,
                )
            elif isinstance(el, Markers):
                target.scatter(
                    el.ts,
                    el.values,
                    color=rgba,
                    s=el.radius**2 * 10,
                    label=el.label,
                )
            elif isinstance(el, HLine):
                target.axhline(
                    el.y,
                    color=rgba,
                    linestyle=el.style.value,
                    linewidth=1,
                    label=el.label,
                )
            elif isinstance(el, VLine):
                # Always on the primary — twins share x, so visually identical.
                ax.axvline(
                    el.x,
                    color=rgba,
                    linestyle=el.style.value,
                    linewidth=1,
                    label=el.label,
                )

        # Thin spine + tick borders (1px) on every axes — primary and twins.
        # Color-code each twin's right spine and y tick labels with the color
        # of its first Series/Markers, so users can tell which numbers belong
        # to which series. Primary axis stays neutral.
        for name, axes_obj in axes.items():
            for spine in axes_obj.spines.values():
                spine.set_linewidth(1)
            axes_obj.tick_params(width=1)
            if name is not None and name in axis_colors:
                c = axis_colors[name]
                axes_obj.spines["right"].set_color(c)
                axes_obj.spines["right"].set_linewidth(1)
                axes_obj.tick_params(axis="y", colors=c, width=1)

        # Combine handles from all axes into a single legend. Attach it to the
        # *last* axes created (the most recent twin, or the primary if there
        # are no twins) so the legend paints last and isn't covered by twin
        # tick labels / spines drawn afterward in matplotlib's axes draw order.
        all_handles: list[Any] = []
        all_labels: list[str] = []
        for axes_obj in axes.values():
            h, l = axes_obj.get_legend_handles_labels()
            all_handles.extend(h)
            all_labels.extend(l)
        if all_handles:
            legend_host = next(reversed(axes.values()))
            legend_host.legend(
                all_handles,
                all_labels,
                facecolor="#1a1a2e",
                edgecolor="#2a2a4a",
                framealpha=0.9,
                loc="center left",
                bbox_to_anchor=(1.02, 0.5),
            )

        ax.set_xlabel("time (s)")
        _apply_time_axis(ax, plot)

        # Make room on the right for offset twin spines (each extra twin past
        # the first needs about `twin_offset_step` of axes-relative width).
        # `tight_layout` doesn't know about offset spines and will clip them,
        # so for 2+ twins we use explicit margins instead.
        n_twins = sum(1 for k in axes if k is not None)
        if n_twins >= 2:
            extras = n_twins - 1
            right_margin = max(0.6, 0.95 - twin_offset_step * extras)
            fig.subplots_adjust(left=0.08, right=right_margin, top=0.95, bottom=0.18)
        else:
            fig.tight_layout()

        buf = io.StringIO()
        fig.savefig(buf, format="svg", bbox_inches="tight")
        plt.close(fig)

        return buf.getvalue()
