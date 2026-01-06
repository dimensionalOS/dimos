# Copyright 2025 Dimensional Inc.
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

"""Terminal plotting utilities using plotext."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import plotext as plt  # type: ignore[import-untyped]

if TYPE_CHECKING:
    from collections.abc import Sequence


def _default_size() -> tuple[int, int]:
    """Return default plot size (terminal width, half terminal height)."""
    tw, th = plt.terminal_size()
    return tw, th // 2


@dataclass
class Series:
    """A data series for plotting."""

    y: Sequence[float]
    x: Sequence[float] | None = None
    label: str | None = None
    color: tuple[int, int, int] | None = None
    marker: str = "braille"  # braille, dot, hd, fhd, sd


@dataclass
class Plot:
    """Terminal plot."""

    title: str | None = None
    xlabel: str | None = None
    ylabel: str | None = None
    width: int | None = None
    height: int | None = None
    series: list[Series] = field(default_factory=list)

    def add(
        self,
        y: Sequence[float],
        x: Sequence[float] | None = None,
        label: str | None = None,
        color: tuple[int, int, int] | None = None,
        marker: str = "braille",
    ) -> Plot:
        """Add a data series to the plot.

        Args:
            y: Y values
            x: X values (optional, defaults to 0, 1, 2, ...)
            label: Series label for legend
            color: RGB tuple (optional, auto-assigned from theme)
            marker: Marker style (braille, dot, hd, fhd, sd)

        Returns:
            Self for chaining
        """
        self.series.append(Series(y=y, x=x, label=label, color=color, marker=marker))
        return self

    def build(self) -> str:
        """Build the plot and return as string."""
        plt.clf()
        plt.theme("dark")

        # Set size (default to terminal width, half terminal height)
        dw, dh = _default_size()
        plt.plotsize(self.width or dw, self.height or dh)

        # Plot each series
        for _i, s in enumerate(self.series):
            x = list(s.x) if s.x is not None else list(range(len(s.y)))
            y = list(s.y)
            if s.color:
                plt.plot(x, y, label=s.label, marker=s.marker, color=s.color)
            else:
                plt.plot(x, y, label=s.label, marker=s.marker)

        # Set labels and title
        if self.title:
            plt.title(self.title)
        if self.xlabel:
            plt.xlabel(self.xlabel)
        if self.ylabel:
            plt.ylabel(self.ylabel)

        result: str = plt.build()
        return result

    def show(self) -> None:
        """Print the plot to stdout."""
        print(self.build())


def plot(
    y: Sequence[float],
    x: Sequence[float] | None = None,
    title: str | None = None,
    xlabel: str | None = None,
    ylabel: str | None = None,
    label: str | None = None,
    width: int | None = None,
    height: int | None = None,
) -> None:
    """Quick single-series plot.

    Args:
        y: Y values
        x: X values (optional)
        title: Plot title
        xlabel: X-axis label
        ylabel: Y-axis label
        label: Series label
        width: Plot width in characters
        height: Plot height in characters
    """
    p = Plot(title=title, xlabel=xlabel, ylabel=ylabel, width=width, height=height)
    p.add(y, x, label=label)
    p.show()


def bar(
    labels: Sequence[str],
    values: Sequence[float],
    title: str | None = None,
    xlabel: str | None = None,
    ylabel: str | None = None,
    width: int | None = None,
    height: int | None = None,
    horizontal: bool = False,
) -> None:
    """Quick bar chart.

    Args:
        labels: Category labels
        values: Values for each category
        title: Plot title
        xlabel: X-axis label
        ylabel: Y-axis label
        width: Plot width in characters
        height: Plot height in characters
        horizontal: If True, draw horizontal bars
    """
    plt.clf()
    plt.theme("dark")
    dw, dh = _default_size()
    plt.plotsize(width or dw, height or dh)

    if horizontal:
        plt.bar(list(labels), list(values), orientation="h")
    else:
        plt.bar(list(labels), list(values))

    if title:
        plt.title(title)
    if xlabel:
        plt.xlabel(xlabel)
    if ylabel:
        plt.ylabel(ylabel)

    print(plt.build())


def scatter(
    x: Sequence[float],
    y: Sequence[float],
    title: str | None = None,
    xlabel: str | None = None,
    ylabel: str | None = None,
    width: int | None = None,
    height: int | None = None,
) -> None:
    """Quick scatter plot.

    Args:
        x: X values
        y: Y values
        title: Plot title
        xlabel: X-axis label
        ylabel: Y-axis label
        width: Plot width in characters
        height: Plot height in characters
    """
    plt.clf()
    plt.theme("dark")
    dw, dh = _default_size()
    plt.plotsize(width or dw, height or dh)

    plt.scatter(list(x), list(y), marker="dot")

    if title:
        plt.title(title)
    if xlabel:
        plt.xlabel(xlabel)
    if ylabel:
        plt.ylabel(ylabel)

    print(plt.build())


def compare_bars(
    labels: Sequence[str],
    data: dict[str, Sequence[float]],
    title: str | None = None,
    xlabel: str | None = None,
    ylabel: str | None = None,
    width: int | None = None,
    height: int | None = None,
) -> None:
    """Compare multiple series as grouped bars.

    Args:
        labels: Category labels (x-axis)
        data: Dict mapping series name to values
        title: Plot title
        xlabel: X-axis label
        ylabel: Y-axis label
        width: Plot width in characters
        height: Plot height in characters

    Example:
        compare_bars(
            ["moondream-full", "moondream-512", "moondream-256"],
            {"query_time": [2.1, 1.5, 0.8], "accuracy": [95, 92, 85]},
            title="Model Performance"
        )
    """
    plt.clf()
    plt.theme("dark")
    dw, dh = _default_size()
    plt.plotsize(width or dw, height or dh)

    for name, values in data.items():
        plt.bar(list(labels), list(values), label=name)

    if title:
        plt.title(title)
    if xlabel:
        plt.xlabel(xlabel)
    if ylabel:
        plt.ylabel(ylabel)

    print(plt.build())


if __name__ == "__main__":
    # Demo
    print("Line plot:")
    plot([1, 4, 9, 16, 25], title="Squares", xlabel="n", ylabel="n²")

    print("\nBar chart:")
    bar(
        ["moondream-full", "moondream-512", "moondream-256"],
        [2.1, 1.5, 0.8],
        title="Query Time (s)",
        ylabel="seconds",
    )

    print("\nMulti-series plot:")
    p = Plot(title="Model Performance", xlabel="resize", ylabel="time (s)")
    p.add([2.1, 1.5, 0.8], label="moondream")
    p.add([1.8, 1.2, 0.6], label="qwen")
    p.show()
