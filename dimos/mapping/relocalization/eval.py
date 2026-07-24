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

"""Compact per-source accept/reject summary for --eval, formatted from the tally the RelocalizationModule fills as it fires -- pure formatting, no I/O, no log parsing."""

from dataclasses import dataclass, field
from statistics import median


@dataclass
class SourceTally:
    """One source's running accept/reject counts and its accepted fitnesses."""

    accepts: int = 0
    rejects: int = 0
    fitnesses: list[float] = field(default_factory=list)  # accepted fixes only


def format_eval_summary(tally: dict[str, SourceTally]) -> str:
    """Render `tally` as an aligned `source acc rej med_fit` table with a TOTAL row."""
    header = ["source", "acc", "rej", "med_fit"]
    body: list[list[str]] = [header]
    for source in sorted(tally):
        t = tally[source]
        med = f"{median(t.fitnesses):.3f}" if t.fitnesses else "-"
        body.append([source, str(t.accepts), str(t.rejects), med])
    # TOTAL: counts sum; med_fit does not aggregate across sources, so it blanks.
    total = [
        "TOTAL",
        str(sum(t.accepts for t in tally.values())),
        str(sum(t.rejects for t in tally.values())),
        "",
    ]
    widths = [max(len(row[i]) for row in (*body, total)) for i in range(len(header))]
    sep = "  ".join("-" * widths[j] for j in range(len(header)))
    lines: list[str] = []
    for i, row in enumerate(body):
        lines.append("  ".join(c.ljust(widths[j]) for j, c in enumerate(row)))
        if i == 0:
            lines.append(sep)
    lines.append(sep)
    lines.append("  ".join(c.ljust(widths[j]) for j, c in enumerate(total)))
    return "\n".join(lines)
