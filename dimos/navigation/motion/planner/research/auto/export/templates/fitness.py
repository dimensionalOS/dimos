#!/usr/bin/env python3
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

"""The optimization fitness -- the ONE place its weights are defined.

Two numbers come out of every battery run, and they are not the same number:

  referee_score  gate * (100*gold + 10*consist + 1*speed), max 111.
                 The shipping number -- what review judges a candidate on.
                 Defined by the referee (referee/score.py), not here.

  fitness        0 if dq else GOLD_W*gold + CONSIST_W*consist + SPEED_W*speed.
                 What evo climbs. Same pillars, same referee measurements,
                 reweighted so the research problem actually produces a
                 gradient.

The weights have been retuned twice in this benchmark's history, both times
for good reason -- treat them as a tunable, not scripture:

  epoch 1-2: equal thirds (10, 10, 10), max 30. The referee's own (100, 10,
  1) left no gradient: at the original baseline speed was 0.16 and a
  doubling moved the shipping score by 0.5 out of 111, so every non-gold
  experiment looked like noise. Equal thirds fixed that and carried the run
  from 18.4 to 27.9.

  epoch 3: (20, 5, 1), max 26. Equal thirds stopped being right once gold
  and speed both approached their ceilings -- measured on two real
  candidates it ranked them in the OPPOSITE order to the referee. (20, 5, 1)
  is roughly the geometric middle of (1, 1, 1) and (100, 10, 1): it orders
  like the referee while keeping consistency and speed visible above the
  ~0.007 noise floor.

RETUNE PROCEDURE. Changing this file changes what every score means:
  1. edit the weights here;
  2. update `.evo/harness.lock` (this file is hash-pinned);
  3. update any fitness-denominated thresholds in gates.json;
  4. re-run the baseline and record the new numbers;
  5. say what you did and why in project.md.
Skipping any step silently invalidates comparisons across the boundary.
"""

from __future__ import annotations

from typing import Any

GOLD_W, CONSIST_W, SPEED_W = 20.0, 5.0, 1.0
FITNESS_MAX = GOLD_W + CONSIST_W + SPEED_W


def fitness_of(w: dict[str, Any]) -> float:
    """Per-world optimization fitness from the referee's own pillar values."""
    if w["dq"]:
        return 0.0
    return GOLD_W * w["gold"] + CONSIST_W * w["consistency"] + SPEED_W * w["speed"]
