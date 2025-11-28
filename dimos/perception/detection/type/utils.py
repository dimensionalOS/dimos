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

import hashlib

from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.types.timestamped import to_timestamp


def _hash_to_color(name: str) -> str:
    """Generate a consistent color for a given name using hash."""
    # List of rich colors to choose from
    colors = [
        "cyan",
        "magenta",
        "yellow",
        "blue",
        "green",
        "red",
        "bright_cyan",
        "bright_magenta",
        "bright_yellow",
        "bright_blue",
        "bright_green",
        "bright_red",
        "purple",
        "white",
        "pink",
    ]

    # Hash the name and pick a color
    hash_value = hashlib.md5(name.encode()).digest()[0]
    return colors[hash_value % len(colors)]


class TableStr:
    """Mixin class that provides table-based string representation for detection collections."""

    def __str__(self) -> str:
        console = Console(force_terminal=True, legacy_windows=False)

        # Create a table for detections
        table = Table(
            title=f"{self.__class__.__name__} [{len(self.detections)} detections @ {to_timestamp(self.image.ts):.3f}]",  # type: ignore[attr-defined]
            show_header=True,
            show_edge=True,
        )

        # Dynamically build columns based on the first detection's dict keys
        if not self.detections:  # type: ignore[attr-defined]
            return (
                f"   {self.__class__.__name__} [0 detections @ {to_timestamp(self.image.ts):.3f}]"  # type: ignore[attr-defined]
            )

        # Cache all repr_dicts to avoid double computation
        detection_dicts = [det.to_repr_dict() for det in self]  # type: ignore[attr-defined]

        first_dict = detection_dicts[0]
        table.add_column("#", style="dim")
        for col in first_dict.keys():
            color = _hash_to_color(col)
            table.add_column(col.title(), style=color)

        # Add each detection to the table
        for i, d in enumerate(detection_dicts):
            row = [str(i)]

            for key in first_dict.keys():
                if key == "conf":
                    # Color-code confidence
                    conf_color = (
                        "green"
                        if float(d[key]) > 0.8
                        else "yellow"
                        if float(d[key]) > 0.5
                        else "red"
                    )
                    row.append(Text(f"{d[key]}", style=conf_color))  # type: ignore[arg-type]
                elif key == "points" and d.get(key) == "None":
                    row.append(Text(d.get(key, ""), style="dim"))  # type: ignore[arg-type]
                else:
                    row.append(str(d.get(key, "")))
            table.add_row(*row)

        with console.capture() as capture:
            console.print(table)
        return capture.get().strip()
